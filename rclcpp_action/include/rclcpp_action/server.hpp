// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP_ACTION__SERVER_HPP_
#define RCLCPP_ACTION__SERVER_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "rcl/event_callback.h"
#include "rcl_action/action_server.h"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/waitable.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_action/types.hpp"
#include "rclcpp_action/visibility_control.hpp"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"

namespace rclcpp_action {
// 前向声明 (Forward declaration)
class ServerBaseImpl;

/**
 * @brief 一个由操作服务器回调返回的响应，当请求目标时。 (A response returned by an action server
 * callback when a goal is requested.)
 */
enum class GoalResponse : int8_t {
  /// 目标被拒绝，不会执行。 (The goal is rejected and will not be executed.)
  REJECT = 1,
  /// 服务器接受目标，并立即开始执行。 (The server accepts the goal, and is going to begin execution
  /// immediately.)
  ACCEPT_AND_EXECUTE = 2,
  /// 服务器接受目标，并稍后执行。 (The server accepts the goal, and is going to execute it later.)
  ACCEPT_AND_DEFER = 3,
};

/**
 * @brief 一个由操作服务器回调返回的响应，当要求取消目标时。 (A response returned by an action
 * server callback when a goal has been asked to be canceled.)
 */
enum class CancelResponse : int8_t {
  /// 服务器不会尝试取消目标。 (The server will not try to cancel the goal.)
  REJECT = 1,
  /// 服务器已同意尝试取消目标。 (The server has agreed to try to cancel the goal.)
  ACCEPT = 2,
};

/// Base Action Server implementation
/// \internal
/**
 * 这个类不应该被直接用于编写动作服务器的用户。
 * 用户应该使用 `rclcpp_action::Server`。
 * 在内部，这个类负责与 `rcl_action` API 接口。
 *
 * This class should not be used directly by users writing an action server.
 * Instead users should use `rclcpp_action::Server`.
 *
 * Internally, this class is responsible for interfacing with the `rcl_action` API.
 */
class ServerBase : public rclcpp::Waitable {
public:
  /// 枚举类型，用于标识属于动作服务器的实体
  /// Enum to identify entities belonging to the action server
  enum class EntityType : std::size_t {
    GoalService,    ///< 目标服务实体 (Goal service entity)
    ResultService,  ///< 结果服务实体 (Result service entity)
    CancelService,  ///< 取消服务实体 (Cancel service entity)
  };

  /// \brief 虚拟析构函数
  /// \details 释放 ServerBase 类的资源
  ///
  /// Virtual destructor
  /// Releases resources of the ServerBase class
  RCLCPP_ACTION_PUBLIC
  virtual ~ServerBase();

  // ------------- Waitables API -------------

  /// \brief 获取实现 action server 所需的订阅数
  /// \details 返回用于实现 action server 的订阅数量
  /// \return 订阅数量
  ///
  /// Get the number of subscriptions required for implementing an action server
  /// Returns the number of subscriptions used to implement an action server
  /// \return Number of subscriptions
  RCLCPP_ACTION_PUBLIC
  size_t get_number_of_ready_subscriptions() override;

  /// \brief 获取实现 action server 所需的定时器数
  /// \details 返回用于实现 action server 的定时器数量
  /// \return 定时器数量
  ///
  /// Get the number of timers required for implementing an action server
  /// Returns the number of timers used to implement an action server
  /// \return Number of timers
  RCLCPP_ACTION_PUBLIC
  size_t get_number_of_ready_timers() override;

  /// \brief 获取实现 action server 所需的服务客户端数
  /// \details 返回用于实现 action server 的服务客户端数量
  /// \return 服务客户端数量
  ///
  /// Get the number of service clients required for implementing an action server
  /// Returns the number of service clients used to implement an action server
  /// \return Number of service clients
  RCLCPP_ACTION_PUBLIC
  size_t get_number_of_ready_clients() override;

  /// \brief 获取实现 action server 所需的服务服务器数
  /// \details 返回用于实现 action server 的服务服务器数量
  /// \return 服务服务器数量
  ///
  /// Get the number of service servers required for implementing an action server
  /// Returns the number of service servers used to implement an action server
  /// \return Number of service servers
  RCLCPP_ACTION_PUBLIC
  size_t get_number_of_ready_services() override;

  /// \brief 获取实现 action server 所需的保护条件数
  /// \details 返回用于实现 action server 的保护条件数量
  /// \return 保护条件数量
  ///
  /// Get the number of guard conditions required for implementing an action server
  /// Returns the number of guard conditions used to implement an action server
  /// \return Number of guard conditions
  RCLCPP_ACTION_PUBLIC
  size_t get_number_of_ready_guard_conditions() override;

  /// \brief 将所有实体添加到等待集
  /// \details 将用于实现 action server 的所有实体添加到指定的等待集中
  /// \param wait_set 等待集指针
  ///
  /// Add all entities to a wait set
  /// Adds all entities used to implement an action server to the specified wait set
  /// \param wait_set Pointer to the wait set
  RCLCPP_ACTION_PUBLIC
  void add_to_wait_set(rcl_wait_set_t *wait_set) override;

  /// 返回 true，如果操作服务器中的任何实体都准备好执行。
  /// Return true if any entity belonging to the action server is ready to be executed.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  bool is_ready(rcl_wait_set_t *) override;

  /// 获取共享指针类型的数据
  /// Get shared pointer type data
  RCLCPP_ACTION_PUBLIC
  std::shared_ptr<void> take_data() override;

  /// 通过实体 ID 获取共享指针类型的数据
  /// Get shared pointer type data by entity id
  RCLCPP_ACTION_PUBLIC
  std::shared_ptr<void> take_data_by_entity_id(size_t id) override;

  /// 对 wait set 中准备好的实体进行操作。
  /// Act on entities in the wait set which are ready to be acted upon.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void execute(std::shared_ptr<void> &data) override;

  /// 设置操作服务器实体事件发生时调用的回调函数
  /// Set a callback to be called when action server entities have an event
  /**
   * 回调接收一个 size_t 类型，表示自上次调用此回调以来接收到的消息数量。
   * The callback receives a size_t which is the number of messages received
   * since the last time this callback was called.
   * 通常为 1，但如果在设置任何回调之前收到了消息，则可以为 > 1。
   * Normally this is 1, but can be > 1 if messages were received before any
   * callback was set.
   *
   * 回调还接收一个 int 标识符参数，该参数标识准备好的操作服务器实体。
   * The callback also receives an int identifier argument, which identifies
   * the action server entity which is ready.
   * 这意味着提供的回调可以根据触发 waitable 准备就绪的实体来使用标识符进行不同的操作。
   * This implies that the provided callback can use the identifier to behave
   * differently depending on which entity triggered the waitable to become ready.
   *
   * 再次调用它将清除之前设置的任何回调。
   * Calling it again will clear any previously set callback.
   *
   * 如果回调不可调用，将抛出异常。
   * An exception will be thrown if the callback is not callable.
   *
   * 此函数是线程安全的。
   * This function is thread-safe.
   *
   * 如果您希望在回调中获得更多信息，如订阅或其他信息，可以使用 lambda 或 std::bind。
   * If you want more information available in the callback, like the subscription
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \param[in] 当收到新消息时调用的回调函数。
   * \param[in] callback functor to be called when a new message is received.
   */
  RCLCPP_ACTION_PUBLIC
  void set_on_ready_callback(std::function<void(size_t, int)> callback) override;

  /**
   * @brief 取消设置当 waitable 准备好时调用的回调函数 (Unset the callback to be called whenever the
   * waitable becomes ready)
   *
   * @details 这个函数用于清除之前设置的当 waitable 对象准备好时调用的回调函数。
   *          当不再需要执行特定操作时，可以使用此函数取消回调。
   *          (This function is used to clear the previously set callback to be called when the
   * waitable object is ready. When you no longer need to perform a specific operation, you can use
   * this function to cancel the callback.)
   */
  RCLCPP_ACTION_PUBLIC
  void clear_on_ready_callback() override;
  // 结束 Waitables API (End Waitables API)
  // -----------------

protected:
  /**
   * @brief 构造函数 (Constructor)
   *
   * @param[in] node_base 节点基本接口的共享指针 (Shared pointer to the NodeBaseInterface)
   * @param[in] node_clock 节点时钟接口的共享指针 (Shared pointer to the NodeClockInterface)
   * @param[in] node_logging 节点日志接口的共享指针 (Shared pointer to the NodeLoggingInterface)
   * @param[in] name action 服务器的名称 (Name of the action server)
   * @param[in] type_support ROSIDL 动作类型支持结构体 (ROSIDL action type support struct)
   * @param[in] options rcl_action_server 的配置选项 (Configuration options for the
   * rcl_action_server)
   */
  RCLCPP_ACTION_PUBLIC
  ServerBase(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
      const std::string &name,
      const rosidl_action_type_support_t *type_support,
      const rcl_action_server_options_t &options);

  // -----------------------------------------------------
  // API for communication between ServerBase and Server<>

  /**
   * @brief 当收到目标请求时，ServerBase 将调用此函数。子类应转换为实际类型并调用用户回调。
   * (ServerBase will call this function when a goal request is received. The subclass should
   * convert to the real type and call a user's callback.)
   *
   * @return std::pair<GoalResponse, std::shared_ptr<void>> 包含 GoalResponse 和请求的共享指针的
   * pair 对象 (A pair object containing GoalResponse and a shared pointer to the request)
   */
  RCLCPP_ACTION_PUBLIC
  virtual std::pair<GoalResponse, std::shared_ptr<void>> call_handle_goal_callback(
      GoalUUID &, std::shared_ptr<void> request) = 0;

  /**
   * @brief ServerBase 将确定哪些目标 ID 正在取消，然后为每个目标 ID
   * 调用此函数。子类应查找目标句柄并调用用户回调。 (ServerBase will determine which goal ids are
   * being cancelled, and then call this function for each goal id. The subclass should look up a
   * goal handle and call the user's callback.)
   *
   * @param[in] uuid 要取消的目标 UUID (The GoalUUID to be cancelled)
   * @return CancelResponse 取消操作的响应 (The response of the cancel operation)
   */
  RCLCPP_ACTION_PUBLIC
  virtual CancelResponse call_handle_cancel_callback(const GoalUUID &uuid) = 0;

  /**
   * @brief 给定一个目标请求消息，返回其中包含的 UUID。 (Given a goal request message, return the
   * UUID contained within.)
   *
   * @param[in] message 目标请求消息 (Goal request message)
   * @return GoalUUID 消息中包含的目标 UUID (The GoalUUID contained in the message)
   */
  RCLCPP_ACTION_PUBLIC
  virtual GoalUUID get_goal_id_from_goal_request(void *message) = 0;

  /**
   * @brief 创建一个空的目标请求消息，以便可以从较低层获取。 (Create an empty goal request message
   * so it can be taken from a lower layer.)
   *
   * @return std::shared_ptr<void> 空目标请求消息的共享指针 (Shared pointer to the empty goal
   * request message)
   */
  RCLCPP_ACTION_PUBLIC
  virtual std::shared_ptr<void> create_goal_request() = 0;

  /// 通知用户回调，一个目标已被接受。
  /// Call user callback to inform them a goal has been accepted.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual void call_goal_accepted_callback(
      std::shared_ptr<rcl_action_goal_handle_t>
          rcl_goal_handle,  ///< [in] ROS2 action 目标句柄 / ROS2 action goal handle
      GoalUUID uuid,        ///< [in] 目标的唯一标识符 / Unique identifier of the goal
      std::shared_ptr<void>
          goal_request_message) = 0;  ///< [in] 目标请求消息 / Goal request message

  /// 根据结果请求消息返回其中包含的 UUID。
  /// Given a result request message, return the UUID contained within.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual GoalUUID get_goal_id_from_result_request(
      void *message) = 0;  ///< [in] 结果请求消息 / Result request message

  /// 创建一个空的目标请求消息，以便从较低层获取。
  /// Create an empty goal request message so it can be taken from a lower layer.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual std::shared_ptr<void> create_result_request() = 0;

  /// 创建一个空的目标结果消息，以便作为较低层的回复发送。
  /// Create an empty goal result message so it can be sent as a reply in a lower layer.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual std::shared_ptr<void> create_result_response(
      decltype(action_msgs::msg::GoalStatus::status) status) = 0;  ///< [in] 目标状态 / Goal status

  /// 发布状态信息。
  /// Publish status information.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void publish_status();

  /// 通知目标终止状态。
  /// Notify goal terminal state.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void notify_goal_terminal_state();

  /// 发布结果消息。
  /// Publish result message.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void publish_result(
      const GoalUUID &uuid,  ///< [in] 目标的唯一标识符 / Unique identifier of the goal
      std::shared_ptr<void> result_msg);  ///< [in] 结果消息 / Result message

  /// 发布反馈消息。
  /// Publish feedback message.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void publish_feedback(std::shared_ptr<void> feedback_msg);  ///< [in] 反馈消息 / Feedback message

  // 结束用于 ServerBase 和 Server<> 之间通信的 API
  // End API for communication between ServerBase and Server<>
  // ---------------------------------------------------------

private:
  /// 处理添加新目标到服务器的请求
  /// Handle a request to add a new goal to the server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void execute_goal_request_received(std::shared_ptr<void> &data);

  /// 处理取消服务器上目标的请求
  /// Handle a request to cancel goals on the server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void execute_cancel_request_received(std::shared_ptr<void> &data);

  /// 处理获取操作结果的请求
  /// Handle a request to get the result of an action
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void execute_result_request_received(std::shared_ptr<void> &data);

  /// 处理超时，表示已完成的目标应该被服务器遗忘
  /// Handle a timeout indicating a completed goal should be forgotten by the server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void execute_check_expired_goals();

  /// 私有实现
  /// Private implementation
  /// \internal
  std::unique_ptr<ServerBaseImpl> pimpl_;

  /// 设置一个 std::function 回调，当指定的实体准备好时调用它
  /// Set a std::function callback to be called when the specified entity is ready
  RCLCPP_ACTION_PUBLIC
  void set_callback_to_entity(EntityType entity_type, std::function<void(size_t, int)> callback);

protected:
  // Mutex 用于保护回调存储的访问。
  // Mutex to protect the access of callbacks storage.
  std::recursive_mutex listener_mutex_;

  // 存储 std::function 回调以保持它们在作用域中
  // Storage for std::function callbacks to keep them in scope
  std::unordered_map<EntityType, std::function<void(size_t)>> entity_type_to_on_ready_callback_;

  /**
   * @brief 设置一个回调，当指定的实体准备好时调用
   *        Set a callback to be called when the specified entity is ready
   *
   * @param[in] entity_type 实体类型（例如：服务器、客户端等）
   *                        Entity type (e.g., server, client, etc.)
   * @param[in] callback    当实体准备好时要调用的回调函数
   *                        Callback function to be called when the entity is ready
   * @param[in] user_data   用户数据，将传递给回调函数
   *                        User data that will be passed to the callback function
   */
  RCLCPP_ACTION_PUBLIC
  void set_on_ready_callback(
      EntityType entity_type, rcl_event_callback_t callback, const void *user_data);

  // 用于标识是否已设置 on_ready 回调
  // Flag to indicate if the on_ready callback has been set
  bool on_ready_callback_set_{false};
};

/// Action Server
/**
 * 这个类创建一个动作服务器。
 * This class creates an action server.
 *
 * 使用 `rclcpp_action::create_server()` 创建这个服务器的实例。
 * Create an instance of this server using `rclcpp_action::create_server()`.
 *
 * 在内部，这个类负责：
 * Internally, this class is responsible for:
 *  - 将 C++ 动作类型转换为 `rclcpp_action::ServerBase` 的通用类型，和
 *  - coverting between the C++ action type and generic types for `rclcpp_action::ServerBase`, and
 *  - 调用用户回调。
 *  - calling user callbacks.
 */
template <typename ActionT>
class Server : public ServerBase, public std::enable_shared_from_this<Server<ActionT>> {
public:
  // 定义智能指针，但不可复制
  // Define smart pointer, but not copyable
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Server)

  /// 接受或拒绝目标请求的回调函数签名。
  /// Signature of a callback that accepts or rejects goal requests.
  using GoalCallback =
      std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;

  /// 接受或拒绝取消目标请求的回调函数签名。
  /// Signature of a callback that accepts or rejects requests to cancel a goal.
  using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;

  /// 当目标被接受时用于通知的回调函数签名。
  /// Signature of a callback that is used to notify when the goal has been accepted.
  using AcceptedCallback = std::function<void(std::shared_ptr<ServerGoalHandle<ActionT>>)>;

  /// 构造一个动作服务器。
  /// Construct an action server.
  /**
   * 这将构建一个动作服务器，但在将其添加到节点之前，它将无法工作。
   * 使用 `rclcpp_action::create_server()` 同时构造并添加到节点。
   * This constructs an action server, but it will not work until it has been added to a node.
   * Use `rclcpp_action::create_server()` to both construct and add to a node.
   *
   * 必须提供三个回调：
   *  - 一个用于接受或拒绝发送到服务器的目标，
   *  - 一个用于接受或拒绝取消目标的请求，
   *  - 一个在目标被接受后接收目标句柄的回调。
   * 所有回调必须是非阻塞的。
   * Three callbacks must be provided:
   *  - one to accept or reject goals sent to the server,
   *  - one to accept or reject requests to cancel a goal,
   *  - one to receive a goal handle after a goal has been accepted.
   * All callbacks must be non-blocking.
   * 使用 `rclcpp_action::ServerGoalHandle` 上的方法设置目标结果。
   * The result of a goal should be set using methods on `rclcpp_action::ServerGoalHandle`.
   *
   * \param[in] node_base 指向节点基本接口的指针。
   * \param[in] node_clock 允许获取节点时钟的接口指针。
   * \param[in] node_logging 允许获取节点记录器的接口指针。
   * \param[in] name 动作的名称。动作客户端和动作服务器必须使用相同的名称和类型进行通信。
   * \param[in] options 传递给底层 `rcl_action_server_t` 的选项。
   * \param[in] handle_goal 决定目标是否应被接受或拒绝的回调。
   * \param[in] handle_cancel
   * 决定是否应尝试取消目标的回调。此回调的返回仅表示服务器是否会尝试取消目标。它并不表示目标实际上已被取消。
   * \param[in] handle_accepted 调用以向用户提供目标句柄的回调。执行。
   *
   * \param[in] node_base a pointer to the base interface of a node.
   * \param[in] node_clock a pointer to an interface that allows getting a node's clock.
   * \param[in] node_logging a pointer to an interface that allows getting a node's logger.
   * \param[in] name the name of an action.
   *  The same name and type must be used by both the action client and action server to
   *  communicate.
   * \param[in] options Options to pass to the underlying `rcl_action_server_t`.
   * \param[in] handle_goal a callback that decides if a goal should be accepted or rejected.
   * \param[in] handle_cancel a callback that decides if a goal should be attemted to be canceled.
   *  The return from this callback only indicates if the server will try to cancel a goal.
   *  It does not indicate if the goal was actually canceled.
   * \param[in] handle_accepted a callback that is called to give the user a handle to the goal.
   *  execution.
   */
  Server(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
      const std::string &name,
      const rcl_action_server_options_t &options,
      GoalCallback handle_goal,
      CancelCallback handle_cancel,
      AcceptedCallback handle_accepted)
      : ServerBase(
            node_base,
            node_clock,
            node_logging,
            name,
            rosidl_typesupport_cpp::get_action_type_support_handle<ActionT>(),
            options),
        // 初始化 handle_goal_ 成员变量
        // Initialize the handle_goal_ member variable
        handle_goal_(handle_goal),
        // 初始化 handle_cancel_ 成员变量
        // Initialize the handle_cancel_ member variable
        handle_cancel_(handle_cancel),
        // 初始化 handle_accepted_ 成员变量
        // Initialize the handle_accepted_ member variable
        handle_accepted_(handle_accepted) {}

  virtual ~Server() = default;

protected:
  // -----------------------------------------------------
  // API for communication between ServerBase and Server<>

  /// \internal
  /**
   * @brief 调用处理目标回调函数，实现服务器基类和服务器之间的通信。
   *        Call the handle goal callback function to implement communication between server base
   * and server.
   *
   * @tparam ActionT 动作类型。Action type.
   * @param uuid 目标UUID。Goal UUID.
   * @param message 通信消息，包含目标请求。Communication message containing the goal request.
   * @return std::pair<GoalResponse, std::shared_ptr<void>> 返回用户响应和ROS响应。Return user
   * response and ROS response.
   */
  std::pair<GoalResponse, std::shared_ptr<void>> call_handle_goal_callback(
      GoalUUID &uuid, std::shared_ptr<void> message) override {
    // 将void指针转换为发送目标服务请求类型的智能指针。
    // Convert void pointer to a smart pointer of SendGoalService request type.
    auto request =
        std::static_pointer_cast<typename ActionT::Impl::SendGoalService::Request>(message);

    // 创建一个目标对象，并将其与请求关联。
    // Create a goal object and associate it with the request.
    auto goal = std::shared_ptr<typename ActionT::Goal>(request, &request->goal);

    // 调用处理目标回调函数，并获取用户响应。
    // Call the handle goal callback function and get the user response.
    GoalResponse user_response = handle_goal_(uuid, goal);

    // 创建一个ROS响应对象。
    // Create a ROS response object.
    auto ros_response = std::make_shared<typename ActionT::Impl::SendGoalService::Response>();

    // 根据用户响应设置ROS响应的accepted字段。
    // Set the accepted field of the ROS response based on the user response.
    ros_response->accepted = GoalResponse::ACCEPT_AND_EXECUTE == user_response ||
                             GoalResponse::ACCEPT_AND_DEFER == user_response;

    // 返回用户响应和ROS响应作为一个pair对象。
    // Return the user response and ROS response as a pair object.
    return std::make_pair(user_response, ros_response);
  }

  /// \internal
  /// \brief 处理取消目标回调的函数
  /// \details 当客户端请求取消一个目标时，此函数将被调用
  /// \param[in] uuid 要取消的目标的UUID
  /// \return 返回一个CancelResponse类型的值，表示是否接受或拒绝取消请求
  ///
  /// Handle the cancel goal callback function
  /// This function will be called when a client requests to cancel a goal
  /// \param[in] uuid The UUID of the goal to be canceled
  /// \return Returns a value of type CancelResponse, indicating whether to accept or reject the
  /// cancel request
  CancelResponse call_handle_cancel_callback(const GoalUUID &uuid) override {
    // 定义一个共享指针，用于存储找到的目标句柄
    // Define a shared pointer to store the found goal handle
    std::shared_ptr<ServerGoalHandle<ActionT>> goal_handle;
    {
      // 使用互斥锁保护对goal_handles_的访问
      // Protect access to goal_handles_ with a mutex lock
      std::lock_guard<std::mutex> lock(goal_handles_mutex_);
      auto element = goal_handles_.find(uuid);
      if (element != goal_handles_.end()) {
        // 如果找到了与给定UUID匹配的目标句柄，则将其存储在goal_handle中
        // Store the found goal handle that matches the given UUID in goal_handle
        goal_handle = element->second.lock();
      }
    }

    // 初始化一个CancelResponse变量为REJECT
    // Initialize a CancelResponse variable as REJECT
    CancelResponse resp = CancelResponse::REJECT;
    if (goal_handle) {
      // 调用handle_cancel_函数处理取消请求
      // Call the handle_cancel_ function to process the cancel request
      resp = handle_cancel_(goal_handle);
      if (CancelResponse::ACCEPT == resp) {
        try {
          // 如果接受取消请求，则调用_cancel_goal()函数取消目标
          // If the cancel request is accepted, call the _cancel_goal() function to cancel the goal
          goal_handle->_cancel_goal();
        } catch (const rclcpp::exceptions::RCLError &ex) {
          // 如果在取消过程中出现异常，记录调试信息并返回拒绝取消的响应
          // If an exception occurs during cancellation, log debug information and return a reject
          // cancel response
          RCLCPP_DEBUG(
              rclcpp::get_logger("rclcpp_action"),
              "Failed to cancel goal in call_handle_cancel_callback: %s", ex.what());
          return CancelResponse::REJECT;
        }
      }
    }
    // 返回最终的取消响应结果
    // Return the final cancel response result
    return resp;
  }

  /// \internal
  /// \brief 调用目标接受回调函数
  /// \param rcl_goal_handle RCL动作目标句柄的共享指针
  /// \param uuid 目标的UUID
  /// \param goal_request_message 目标请求消息的共享指针
  ///
  /// Call the goal accepted callback function
  /// \param rcl_goal_handle Shared pointer to RCL action goal handle
  /// \param uuid UUID of the goal
  /// \param goal_request_message Shared pointer to goal request message
  void call_goal_accepted_callback(
      std::shared_ptr<rcl_action_goal_handle_t> rcl_goal_handle,
      GoalUUID uuid,
      std::shared_ptr<void> goal_request_message) override {
    // 创建一个服务器目标句柄的共享指针
    // Create a shared pointer to a server goal handle
    std::shared_ptr<ServerGoalHandle<ActionT>> goal_handle;

    // 创建一个弱指针，指向当前对象
    // Create a weak pointer pointing to the current object
    std::weak_ptr<Server<ActionT>> weak_this = this->shared_from_this();

    // 定义一个终止状态回调函数，将在目标达到终止状态时调用
    // Define a terminal state callback function that will be called when the goal reaches a
    // terminal state
    std::function<void(const GoalUUID &, std::shared_ptr<void>)> on_terminal_state =
        [weak_this](const GoalUUID &goal_uuid, std::shared_ptr<void> result_message) {
          // 尝试获取当前对象的共享指针
          // Attempt to get a shared pointer to the current object
          std::shared_ptr<Server<ActionT>> shared_this = weak_this.lock();
          if (!shared_this) {
            return;
          }
          // 向请求结果的节点发送结果消息
          // Send result message to any node that requested the result
          shared_this->publish_result(goal_uuid, result_message);
          // 在目标句柄状态改变时发布一个状态消息
          // Publish a status message whenever a goal handle changes state
          shared_this->publish_status();
          // 通知基类，以便重新计算过期目标定时器
          // Notify the base class so it can recalculate the expired goal timer
          shared_this->notify_goal_terminal_state();
          // 删除现有数据（ServerBase 和 rcl_action_server_t 会保留数据，直到目标句柄过期）
          // Delete data now (ServerBase and rcl_action_server_t keep data until goal handle
          // expires)
          std::lock_guard<std::mutex> lock(shared_this->goal_handles_mutex_);
          shared_this->goal_handles_.erase(goal_uuid);
        };

    // 定义一个执行中回调函数，将在目标处于执行状态时调用
    // Define an executing callback function that will be called when the goal is in an executing
    // state
    std::function<void(const GoalUUID &)> on_executing = [weak_this](const GoalUUID &goal_uuid) {
      std::shared_ptr<Server<ActionT>> shared_this = weak_this.lock();
      if (!shared_this) {
        return;
      }
      (void)goal_uuid;
      // 在目标句柄状态改变时发布一个状态消息
      // Publish a status message whenever a goal handle changes state
      shared_this->publish_status();
    };

    // 定义一个发布反馈的函数
    // Define a function to publish feedback
    std::function<void(std::shared_ptr<typename ActionT::Impl::FeedbackMessage>)> publish_feedback =
        [weak_this](std::shared_ptr<typename ActionT::Impl::FeedbackMessage> feedback_msg) {
          std::shared_ptr<Server<ActionT>> shared_this = weak_this.lock();
          if (!shared_this) {
            return;
          }
          shared_this->publish_feedback(std::static_pointer_cast<void>(feedback_msg));
        };

    // 将目标请求消息转换为特定类型的请求对象
    // Cast the goal request message to a specific type of request object
    auto request = std::static_pointer_cast<const typename ActionT::Impl::SendGoalService::Request>(
        goal_request_message);
    auto goal = std::shared_ptr<const typename ActionT::Goal>(request, &request->goal);

    // 创建一个新的服务器目标句柄，包含回调函数和发布反馈的函数
    // Create a new server goal handle with callback functions and a function to publish feedback
    goal_handle.reset(new ServerGoalHandle<ActionT>(
        rcl_goal_handle, uuid, goal, on_terminal_state, on_executing, publish_feedback));

    // 将新创建的目标句柄添加到目标句柄集合中
    // Add the newly created goal handle to the goal handles collection
    {
      std::lock_guard<std::mutex> lock(goal_handles_mutex_);
      goal_handles_[uuid] = goal_handle;
    }

    // 调用目标已接受处理函数
    // Call the goal accepted handling function
    handle_accepted_(goal_handle);
  }

  /// \internal
  /// \brief 从目标请求消息中提取目标ID (Extract GoalUUID from goal request message)
  /// \param[in] message 目标请求消息的指针 (Pointer to the goal request message)
  /// \return GoalUUID 返回目标ID (Returns the GoalUUID)
  GoalUUID get_goal_id_from_goal_request(void *message) override {
    // 将message转换为ActionT::Impl::SendGoalService::Request类型并获取goal_id.uuid
    // (Cast message to ActionT::Impl::SendGoalService::Request type and access goal_id.uuid)
    return static_cast<typename ActionT::Impl::SendGoalService::Request *>(message)->goal_id.uuid;
  }

  /// \internal
  /// \brief 创建一个新的目标请求实例 (Create a new instance of goal request)
  /// \return std::shared_ptr<void> 返回新创建的目标请求实例的共享指针 (Returns a shared pointer to
  /// the newly created goal request instance)
  std::shared_ptr<void> create_goal_request() override {
    // 创建一个新的ActionT::Impl::SendGoalService::Request实例并返回其共享指针
    // (Create a new ActionT::Impl::SendGoalService::Request instance and return its shared pointer)
    return std::shared_ptr<void>(new typename ActionT::Impl::SendGoalService::Request());
  }

  /// \internal
  /// \brief 从结果请求消息中提取目标ID (Extract GoalUUID from result request message)
  /// \param[in] message 结果请求消息的指针 (Pointer to the result request message)
  /// \return GoalUUID 返回目标ID (Returns the GoalUUID)
  GoalUUID get_goal_id_from_result_request(void *message) override {
    // 将message转换为ActionT::Impl::GetResultService::Request类型并获取goal_id.uuid
    // (Cast message to ActionT::Impl::GetResultService::Request type and access goal_id.uuid)
    return static_cast<typename ActionT::Impl::GetResultService::Request *>(message)->goal_id.uuid;
  }

  /// \internal
  /// \brief 创建一个新的结果请求实例 (Create a new instance of result request)
  /// \return std::shared_ptr<void> 返回新创建的结果请求实例的共享指针 (Returns a shared pointer to
  /// the newly created result request instance)
  std::shared_ptr<void> create_result_request() override {
    // 创建一个新的ActionT::Impl::GetResultService::Request实例并返回其共享指针
    // (Create a new ActionT::Impl::GetResultService::Request instance and return its shared
    // pointer)
    return std::shared_ptr<void>(new typename ActionT::Impl::GetResultService::Request());
  }

  /// \internal
  /// \brief 根据状态创建一个新的结果响应实例 (Create a new instance of result response based on
  /// status) \param[in] status action_msgs::msg::GoalStatus中的状态值 (The status value from
  /// action_msgs::msg::GoalStatus) \return std::shared_ptr<void> 返回新创建的结果响应实例的共享指针
  /// (Returns a shared pointer to the newly created result response instance)
  std::shared_ptr<void> create_result_response(
      decltype(action_msgs::msg::GoalStatus::status) status) override {
    // 创建一个新的ActionT::Impl::GetResultService::Response实例
    // (Create a new ActionT::Impl::GetResultService::Response instance)
    auto result = std::make_shared<typename ActionT::Impl::GetResultService::Response>();
    // 设置结果响应的状态 (Set the status of the result response)
    result->status = status;
    // 返回结果响应实例的共享指针 (Return the shared pointer to the result response instance)
    return std::static_pointer_cast<void>(result);
  }

  // 结束ServerBase与Server<>之间的通信API (End API for communication between ServerBase and
  // Server<>)
  // ---------------------------------------------------------

private:
  /**
   * @brief GoalCallback 类型定义，用于处理目标回调。
   * @brief Type definition for GoalCallback, used for handling goal callbacks.
   */
  GoalCallback handle_goal_;

  /**
   * @brief CancelCallback 类型定义，用于处理取消回调。
   * @brief Type definition for CancelCallback, used for handling cancel callbacks.
   */
  CancelCallback handle_cancel_;

  /**
   * @brief AcceptedCallback 类型定义，用于处理已接受的回调。
   * @brief Type definition for AcceptedCallback, used for handling accepted callbacks.
   */
  AcceptedCallback handle_accepted_;

  /**
   * @brief GoalHandleWeakPtr 类型定义，表示一个弱指针，指向 ServerGoalHandle<ActionT> 对象。
   * @brief Type definition for GoalHandleWeakPtr, representing a weak pointer to a
   * ServerGoalHandle<ActionT> object.
   */
  using GoalHandleWeakPtr = std::weak_ptr<ServerGoalHandle<ActionT>>;

  /**
   * @brief 一个从目标 ID 到目标处理弱指针的映射。这用于为 handle_cancel 提供目标处理。
   * @brief A map of goal id to goal handle weak pointers. This is used to provide a goal handle to
   * handle_cancel.
   */
  std::unordered_map<GoalUUID, GoalHandleWeakPtr> goal_handles_;

  /**
   * @brief 保护 goal_handles_ 的互斥锁。
   * @brief Mutex protecting goal_handles_.
   */
  std::mutex goal_handles_mutex_;
#endif  // RCLCPP_ACTION__SERVER_HPP_

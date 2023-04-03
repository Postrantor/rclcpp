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

#ifndef RCLCPP_ACTION__CLIENT_HPP_
#define RCLCPP_ACTION__CLIENT_HPP_

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "rcl/event_callback.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/waitable.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/exceptions.hpp"
#include "rclcpp_action/types.hpp"
#include "rclcpp_action/visibility_control.hpp"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"

namespace rclcpp_action {
// Forward declaration
class ClientBaseImpl;

/// \brief 基本的 Action 客户端实现 (Base Action Client implementation)
/// \internal
/**
 * 本类不应该被希望创建动作客户端的用户直接使用。
 * 相反，用户应该使用 `rclcpp_action::Client<>`。
 * 在内部，这个类负责与 `rcl_action` API 进行交互。
 *
 * This class should not be used directly by users wanting to create an action client.
 * Instead, users should use `rclcpp_action::Client<>`.
 * Internally, this class is responsible for interfacing with the `rcl_action` API.
 */
class ClientBase : public rclcpp::Waitable {
public:
  // 公共成员函数
  // Public member function

  // \brief 析构函数 (Destructor)
  RCLCPP_ACTION_PUBLIC
  virtual ~ClientBase();

  /// \brief 枚举类，用于标识与动作客户端相关的实体 (Enum class for identifying entities belonging
  /// to the action client)
  enum class EntityType : std::size_t {
    GoalClient,            ///< 目标客户端 (Goal client)
    ResultClient,          ///< 结果客户端 (Result client)
    CancelClient,          ///< 取消客户端 (Cancel client)
    FeedbackSubscription,  ///< 反馈订阅 (Feedback subscription)
    StatusSubscription,    ///< 状态订阅 (Status subscription)
  };

  /// \brief 检查是否有一个准备好接收目标请求的动作服务器 (Check if there is an action server that
  /// is ready to take goal requests) \return 如果有一个准备好接收目标请求的动作服务器，则返回
  /// true；否则返回 false (Return true if there is an action server that is ready to take goal
  /// requests; otherwise return false)
  RCLCPP_ACTION_PUBLIC
  bool action_server_is_ready() const;

  /// \brief 等待action_server_is_ready()变为true，或直到达到给定的超时时间 (Wait for
  /// action_server_is_ready() to become true, or until the given timeout is reached) \tparam RepT
  /// 超时时间的表示类型，默认为 int64_t (Representation type for the timeout duration, default is
  /// int64_t) \tparam RatioT 超时时间的单位，默认为毫秒 (Unit of the timeout duration, default is
  /// std::milli) \param[in] timeout 超时时间，默认为 -1，表示无限等待 (Timeout duration, default is
  /// -1, which means waiting indefinitely) \return 如果在超时时间内动作服务器准备好，则返回
  /// true；否则返回 false (Return true if the action server is ready within the timeout; otherwise
  /// return false)
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_action_server(
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    // 将超时时间从给定的单位转换为纳秒，并调用wait_for_action_server_nanoseconds()函数 (Convert the
    // timeout duration from the given unit to nanoseconds and call the
    // wait_for_action_server_nanoseconds() function)
    return wait_for_action_server_nanoseconds(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  // -------------
  // Waitables API

  /// \internal
  /// \brief 获取准备好的订阅数量 (Get the number of ready subscriptions)
  /// \return 准备好的订阅数量 (Number of ready subscriptions)
  RCLCPP_ACTION_PUBLIC
  size_t get_number_of_ready_subscriptions() override;

  /// \internal
  /// \brief 获取准备好的定时器数量 (Get the number of ready timers)
  /// \return 准备好的定时器数量 (Number of ready timers)
  RCLCPP_ACTION_PUBLIC
  size_t get_number_of_ready_timers() override;

  /// \internal
  /// \brief 获取准备好的客户端数量 (Get the number of ready clients)
  /// \return 准备好的客户端数量 (Number of ready clients)
  RCLCPP_ACTION_PUBLIC
  size_t get_number_of_ready_clients() override;

  /// \internal
  /// \brief 获取准备好的服务数量 (Get the number of ready services)
  /// \return 准备好的服务数量 (Number of ready services)
  RCLCPP_ACTION_PUBLIC
  size_t get_number_of_ready_services() override;

  /// \internal
  /// \brief 获取准备好的保护条件数量 (Get the number of ready guard conditions)
  /// \return 准备好的保护条件数量 (Number of ready guard conditions)
  RCLCPP_ACTION_PUBLIC
  size_t get_number_of_ready_guard_conditions() override;

  /// \internal
  /// \brief 将可等待对象添加到等待集合中 (Add the waitable object to the wait set)
  /// \param wait_set 等待集合指针 (Pointer to the wait set)
  RCLCPP_ACTION_PUBLIC
  void add_to_wait_set(rcl_wait_set_t* wait_set) override;

  /// \internal
  /// \brief 检查等待集合中的对象是否准备好 (Check if the object in the wait set is ready)
  /// \param wait_set 等待集合指针 (Pointer to the wait set)
  /// \return 对象是否准备好 (Whether the object is ready or not)
  RCLCPP_ACTION_PUBLIC
  bool is_ready(rcl_wait_set_t* wait_set) override;

  /// \internal
  /// \brief 获取可执行数据 (Get the executable data)
  /// \return 可执行数据的共享指针 (Shared pointer to the executable data)
  RCLCPP_ACTION_PUBLIC
  std::shared_ptr<void> take_data() override;

  /// \internal
  /// \brief 根据实体 ID 获取可执行数据 (Get the executable data by entity ID)
  /// \param id 实体 ID (Entity ID)
  /// \return 可执行数据的共享指针 (Shared pointer to the executable data)
  RCLCPP_ACTION_PUBLIC
  std::shared_ptr<void> take_data_by_entity_id(size_t id) override;

  /// \internal
  /// \brief 执行给定的数据 (Execute the given data)
  /// \param data 要执行的数据的共享指针 (Shared pointer to the data to be executed)
  RCLCPP_ACTION_PUBLIC
  void execute(std::shared_ptr<void>& data) override;

  /// \internal
  /// 设置一个回调，当 action client 实体有事件时调用 (Set a callback to be called when action
  /// client entities have an event)
  /**
   * 回调接收一个 size_t 类型参数，表示自上次调用此回调以来接收到的消息数量。
   * (The callback receives a size_t which is the number of messages received
   * since the last time this callback was called.)
   * 通常为1，但如果在设置任何回调之前接收到了消息，则可能 > 1。
   * (Normally this is 1, but can be > 1 if messages were received before any
   * callback was set.)
   *
   * 回调还接收一个 int 标识符参数，该参数标识哪个 action client 实体已准备好。
   * (The callback also receives an int identifier argument, which identifies
   * the action client entity which is ready.)
   * 这意味着提供的回调可以使用标识符根据触发 waitable 变为可用的实体来采取不同的行为。
   * (This implies that the provided callback can use the identifier to behave
   * differently depending on which entity triggered the waitable to become ready.)
   *
   * 再次调用它将清除之前设置的任何回调。
   * (Calling it again will clear any previously set callback.)
   *
   * 如果回调无法调用，将抛出异常。
   * (An exception will be thrown if the callback is not callable.)
   *
   * 此功能是线程安全的。
   * (This function is thread-safe.)
   *
   * 如果您希望在回调中获取更多信息，例如订阅或其他信息，您可以使用带有捕获的 lambda 或 std::bind。
   * (If you want more information available in the callback, like the subscription
   * or other information, you may use a lambda with captures or std::bind.)
   *
   * \param[in] 回调，当收到新消息时调用。 (callback functor to be called when a new message is
   * received.)
   */
  RCLCPP_ACTION_PUBLIC
  void set_on_ready_callback(std::function<void(size_t, int)> callback) override;

  /// 取消注册新事件的回调（如果有）。 (Unset the callback registered for new events, if any.)
  RCLCPP_ACTION_PUBLIC
  void clear_on_ready_callback() override;

  // End Waitables API
  // -----------------

protected:
  /**
   * @brief 构造一个ClientBase对象 (Construct a ClientBase object)
   *
   * @param node_base 共享指针，指向NodeBaseInterface类型的节点基础接口 (Shared pointer to a
   * NodeBaseInterface type node base interface)
   * @param node_graph 共享指针，指向NodeGraphInterface类型的节点图形接口 (Shared pointer to a
   * NodeGraphInterface type node graph interface)
   * @param node_logging 共享指针，指向NodeLoggingInterface类型的节点日志接口 (Shared pointer to a
   * NodeLoggingInterface type node logging interface)
   * @param action_name 字符串，表示操作名称 (String representing the action name)
   * @param type_support 指向rosidl_action_type_support_t类型的操作类型支持 (Pointer to a
   * rosidl_action_type_support_t type action type support)
   * @param options rcl_action_client_options_t类型的操作客户端选项 (rcl_action_client_options_t
   * type action client options)
   */
  RCLCPP_ACTION_PUBLIC
  ClientBase(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
      const std::string& action_name,
      const rosidl_action_type_support_t* type_support,
      const rcl_action_client_options_t& options);

  /**
   * @brief 等待action_server_is_ready()变为true，或者直到给定的超时时间到达 (Wait for
   * action_server_is_ready() to become true, or until the given timeout is reached)
   *
   * @param timeout 超时时间，以纳秒为单位 (Timeout duration in nanoseconds)
   * @return 如果操作服务器在超时时间内准备就绪，则返回true，否则返回false (Returns true if the
   * action server is ready within the timeout, otherwise returns false)
   */
  RCLCPP_ACTION_PUBLIC
  bool wait_for_action_server_nanoseconds(std::chrono::nanoseconds timeout);

  // -----------------------------------------------------
  // 用于 ClientBase 和 Client<> 之间通信的 API (API for communication between ClientBase and
  // Client<>)
  using ResponseCallback = std::function<void(std::shared_ptr<void> response)>;

  /// \internal
  /// 获取日志器 (Get a logger)
  RCLCPP_ACTION_PUBLIC
  rclcpp::Logger get_logger();

  /// \internal
  /// 生成目标 ID (Generate a goal ID)
  RCLCPP_ACTION_PUBLIC
  virtual GoalUUID generate_goal_id();

  /// \internal
  /// 发送目标请求 (Send a goal request)
  RCLCPP_ACTION_PUBLIC
  virtual void send_goal_request(std::shared_ptr<void> request, ResponseCallback callback);

  /// \internal
  /// 发送结果请求 (Send a result request)
  RCLCPP_ACTION_PUBLIC
  virtual void send_result_request(std::shared_ptr<void> request, ResponseCallback callback);

  /// \internal
  /// 发送取消请求 (Send a cancel request)
  RCLCPP_ACTION_PUBLIC
  virtual void send_cancel_request(std::shared_ptr<void> request, ResponseCallback callback);

  /// \internal
  /// 创建目标响应 (Create a goal response)
  virtual std::shared_ptr<void> create_goal_response() const = 0;

  /// \internal
  /// 处理目标响应 (Handle a goal response)
  RCLCPP_ACTION_PUBLIC
  virtual void handle_goal_response(
      const rmw_request_id_t& response_header, std::shared_ptr<void> goal_response);

  /// \internal
  /// 创建结果响应 (Create a result response)
  virtual std::shared_ptr<void> create_result_response() const = 0;

  /// \internal
  /// 处理结果响应 (Handle a result response)
  RCLCPP_ACTION_PUBLIC
  virtual void handle_result_response(
      const rmw_request_id_t& response_header, std::shared_ptr<void> result_response);

  /// \internal
  /// 创建取消响应 (Create a cancel response)
  virtual std::shared_ptr<void> create_cancel_response() const = 0;

  /// \internal
  /// 处理取消响应 (Handle a cancel response)
  RCLCPP_ACTION_PUBLIC
  virtual void handle_cancel_response(
      const rmw_request_id_t& response_header, std::shared_ptr<void> cancel_response);

  /// \internal
  /// \brief 创建一个反馈消息对象（Create a feedback message object）
  /// \return 返回一个指向反馈消息对象的共享指针（Return a shared pointer to the feedback message
  /// object）
  virtual std::shared_ptr<void> create_feedback_message() const = 0;

  /// \internal
  /// \brief 处理收到的反馈消息（Handle received feedback message）
  /// \param[in] message 指向反馈消息对象的共享指针（Shared pointer to the feedback message object）
  virtual void handle_feedback_message(std::shared_ptr<void> message) = 0;

  /// \internal
  /// \brief 创建一个状态消息对象（Create a status message object）
  /// \return 返回一个指向状态消息对象的共享指针（Return a shared pointer to the status message
  /// object）
  virtual std::shared_ptr<void> create_status_message() const = 0;

  /// \internal
  /// \brief 处理收到的状态消息（Handle received status message）
  /// \param[in] message 指向状态消息对象的共享指针（Shared pointer to the status message object）
  virtual void handle_status_message(std::shared_ptr<void> message) = 0;

  // 结束 ClientBase 和 Client<> 之间的通信 API（End API for communication between ClientBase and
  // Client<>）
  // ---------------------------------------------------------

  /// \internal
  /// \brief 设置实体准备就绪时要调用的回调函数（Set a callback to be called when the specified
  /// entity is ready） \param[in] entity_type 要设置回调的实体类型（Entity type for which the
  /// callback should be set） \param[in] callback 当实体准备就绪时要调用的回调函数（Callback to be
  /// called when the entity is ready） \param[in] user_data 用户数据，将传递给回调函数（User data
  /// to be passed to the callback function）
  RCLCPP_ACTION_PUBLIC
  void set_on_ready_callback(
      EntityType entity_type, rcl_event_callback_t callback, const void* user_data);

  // 保护回调存储的互斥锁（Mutex to protect the callbacks storage）
  std::recursive_mutex listener_mutex_;
  // 用于保存 std::function 回调的存储，以便在作用域内保留它们（Storage for std::function callbacks
  // to keep them in scope）
  std::unordered_map<EntityType, std::function<void(size_t)>> entity_type_to_on_ready_callback_;

private:
  // 实现客户端基类的私有实现指针（Private implementation pointer for the ClientBase class）
  std::unique_ptr<ClientBaseImpl> pimpl_;

  /// \brief 设置实体准备就绪时要调用的 std::function 类型的回调函数（Set a std::function callback
  /// to be called when the specified entity is ready） \param[in] entity_type
  /// 要设置回调的实体类型（Entity type for which the callback should be set） \param[in] callback
  /// 当实体准备就绪时要调用的 std::function 类型的回调函数（std::function type callback to be
  /// called when the entity is ready）
  RCLCPP_ACTION_PUBLIC
  void set_callback_to_entity(EntityType entity_type, std::function<void(size_t, int)> callback);

  // 标记是否已设置准备就绪回调函数（Flag indicating whether the on-ready callback has been set）
  bool on_ready_callback_set_{false};
};

/// 动作客户端 (Action Client)
/**
 * 这个类创建一个动作客户端。 (This class creates an action client.)
 *
 * 要创建一个动作客户端的实例，请使用 `rclcpp_action::create_client()`。 (To create an instance of
 * an action client use `rclcpp_action::create_client()`.)
 *
 * 在内部，这个类负责： (Internally, this class is responsible for:)
 *  - 将 C++ 动作类型和 `rclcpp_action::ClientBase` 的通用类型之间进行转换， (converting between the
 * C++ action type and generic types for `rclcpp_action::ClientBase`, and)
 *  - 调用用户回调。 (calling user callbacks.)
 */
template <typename ActionT>
class Client : public ClientBase {
public:
  // 不可复制的智能指针定义 (Smart pointer definitions not copyable)
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Client<ActionT>)

  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using GoalHandle = ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using GoalResponseCallback = std::function<void(typename GoalHandle::SharedPtr)>;
  using FeedbackCallback = typename GoalHandle::FeedbackCallback;
  using ResultCallback = typename GoalHandle::ResultCallback;
  using CancelRequest = typename ActionT::Impl::CancelGoalService::Request;
  using CancelResponse = typename ActionT::Impl::CancelGoalService::Response;
  using CancelCallback = std::function<void(typename CancelResponse::SharedPtr)>;

  /// 发送目标的选项 (Options for sending a goal)
  /**
   * 该结构体用于将参数传递给函数 `async_send_goal`。
   * This struct is used to pass parameters to the function `async_send_goal`.
   */
  struct SendGoalOptions {
    // 默认构造函数 (Default constructor)
    SendGoalOptions()
        : goal_response_callback(nullptr), feedback_callback(nullptr), result_callback(nullptr) {}

    /// 当目标被接受或拒绝时调用的函数 (Function called when the goal is accepted or rejected)
    /**
     * 接受一个目标句柄共享指针作为唯一参数。
     * Takes a single argument that is a goal handle shared pointer.
     * 如果目标被接受，则指针指向一个有效的目标句柄。
     * If the goal is accepted, then the pointer points to a valid goal handle.
     * 如果目标被拒绝，则指针值为 `nullptr`。
     * If the goal is rejected, then pointer has the value `nullptr`.
     */
    GoalResponseCallback goal_response_callback;

    /// 当收到目标反馈时调用的函数 (Function called whenever feedback is received for the goal)
    FeedbackCallback feedback_callback;

    /// 当收到目标结果时调用的函数 (Function called when the result for the goal is received)
    ResultCallback result_callback;
  };

  /// 构造一个动作客户端。 (Construct an action client.)
  /**
   * 这将构造一个动作客户端，但在将其添加到节点之前，它将不起作用。
   * 使用 `rclcpp_action::create_client()` 同时构造和添加到节点。
   * (This constructs an action client, but it will not work until it has been added to a node.
   * Use `rclcpp_action::create_client()` to both construct and add to a node.)
   *
   * \param[in] node_base 指向节点基本接口的指针。 (A pointer to the base interface of a node.)
   * \param[in] node_graph 允许获取关于节点的图形信息的接口的指针。 (A pointer to an interface that
   * allows getting graph information about a node.) \param[in] node_logging
   * 允许获取节点日志记录器的接口的指针。 (A pointer to an interface that allows getting a node's
   * logger.) \param[in] action_name 动作名称。 (The action name.) \param[in] client_options
   * 传递给底层 `rcl_action::rcl_action_client_t` 的选项。 (Options to pass to the underlying
   * `rcl_action::rcl_action_client_t`.)
   */
  Client(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
      const std::string& action_name,
      const rcl_action_client_options_t& client_options = rcl_action_client_get_default_options())
      : ClientBase(
            node_base,
            node_graph,
            node_logging,
            action_name,
            rosidl_typesupport_cpp::get_action_type_support_handle<ActionT>(),
            client_options) {}

  /// 发送一个动作目标并异步获取结果。
  /// Send an action goal and asynchronously get the result.
  /**
   * 如果动作服务器接受了目标，则返回的future设置为`ClientGoalHandle`。
   * If the goal is accepted by an action server, the returned future is set to a
   * `ClientGoalHandle`. 如果动作服务器拒绝了目标，则future设置为`nullptr`。 If the goal is rejected
   * by an action server, then the future is set to a `nullptr`.
   *
   * 返回的目标句柄用于监视目标的状态并获得最终结果。
   * The returned goal handle is used to monitor the status of the goal and get the final result.
   * 只要您持有对共享指针的引用，或者直到rclcpp_action::Client被销毁，此时目标状态将变为UNKNOWN，它就是有效的。
   * It is valid as long as you hold a reference to the shared pointer or until the
   * rclcpp_action::Client is destroyed at which point the goal status will become UNKNOWN.
   *
   * \param[in] goal 目标请求。
   * \param[in] goal The goal request.
   * \param[in] options 发送目标请求的选项。包含对目标响应（接受/拒绝）、反馈和最终结果回调的引用。
   * \param[in] options Options for sending the goal request. Contains references to callbacks for
   *   the goal response (accepted/rejected), feedback, and the final result.
   * \return 当目标被接受或拒绝时完成的future。
   * \return A future that completes when the goal has been accepted or rejected.
   *   如果目标被拒绝，则结果将是一个`nullptr`。
   *   If the goal is rejected, then the result will be a `nullptr`.
   */
  std::shared_future<typename GoalHandle::SharedPtr> async_send_goal(
      const Goal& goal, const SendGoalOptions& options = SendGoalOptions()) {
    // 将promise放入堆中以便移动。
    // Put promise in the heap to move it around.
    auto promise = std::make_shared<std::promise<typename GoalHandle::SharedPtr>>();
    std::shared_future<typename GoalHandle::SharedPtr> future(promise->get_future());
    using GoalRequest = typename ActionT::Impl::SendGoalService::Request;
    auto goal_request = std::make_shared<GoalRequest>();
    goal_request->goal_id.uuid = this->generate_goal_id();
    goal_request->goal = goal;
    this->send_goal_request(
        std::static_pointer_cast<void>(goal_request),
        [this, goal_request, options, promise](std::shared_ptr<void> response) mutable {
          using GoalResponse = typename ActionT::Impl::SendGoalService::Response;
          auto goal_response = std::static_pointer_cast<GoalResponse>(response);
          if (!goal_response->accepted) {
            promise->set_value(nullptr);
            if (options.goal_response_callback) {
              options.goal_response_callback(nullptr);
            }
            return;
          }
          GoalInfo goal_info;
          goal_info.goal_id.uuid = goal_request->goal_id.uuid;
          goal_info.stamp = goal_response->stamp;
          // 不要使用std::make_shared，因为友元关系无法转发。
          // Do not use std::make_shared as friendship cannot be forwarded.
          std::shared_ptr<GoalHandle> goal_handle(
              new GoalHandle(goal_info, options.feedback_callback, options.result_callback));
          {
            std::lock_guard<std::mutex> guard(goal_handles_mutex_);
            goal_handles_[goal_handle->get_goal_id()] = goal_handle;
          }
          promise->set_value(goal_handle);
          if (options.goal_response_callback) {
            options.goal_response_callback(goal_handle);
          }

          if (options.result_callback) {
            this->make_result_aware(goal_handle);
          }
        });

    // TODO(jacobperron): 将其封装到自己的函数中
    //                   并考虑暴露一个选项来禁用此清理
    // TODO(jacobperron): Encapsulate into it's own function and
    //                    consider exposing an option to disable this cleanup
    // 为了防止列表失控，忘记没有更多用户引用的任何目标
    // To prevent the list from growing out of control, forget about any goals
    // with no more user references
    {
      std::lock_guard<std::mutex> guard(goal_handles_mutex_);
      auto goal_handle_it = goal_handles_.begin();
      while (goal_handle_it != goal_handles_.end()) {
        if (!goal_handle_it->second.lock()) {
          RCLCPP_DEBUG(this->get_logger(), "在send_goal()期间删除对目标句柄的弱引用");
          // Dropping weak reference to goal handle during send_goal()
          goal_handle_it = goal_handles_.erase(goal_handle_it);
        } else {
          ++goal_handle_it;
        }
      }
    }

    return future;
  }

  /// 异步获取活动目标的结果 (Asynchronously get the result for an active goal)
  /**
   * \throws exceptions::UnknownGoalHandleError 如果目标未知或已达到终止状态，或者请求结果时出错 (If
   * the goal is unknown or already reached a terminal state, or if there was an error requesting
   * the result) \param[in] goal_handle 要获取结果的目标句柄 (The goal handle for which to get the
   * result) \param[in] result_callback 当收到结果时调用的可选回调 (Optional callback that is called
   * when the result is received) \return 当目标完成时设置为目标结果的 future (A future that is set
   * to the goal result when the goal is finished)
   */
  std::shared_future<WrappedResult> async_get_result(
      typename GoalHandle::SharedPtr goal_handle, ResultCallback result_callback = nullptr) {
    // 锁定目标句柄互斥体 (Lock the goal handles mutex)
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);

    // 如果目标句柄不存在，则抛出异常 (If the goal handle does not exist, throw an exception)
    if (goal_handles_.count(goal_handle->get_goal_id()) == 0) {
      throw exceptions::UnknownGoalHandleError();
    }

    // 如果目标句柄已失效，则抛出异常 (If the goal handle is invalidated, throw an exception)
    if (goal_handle->is_invalidated()) {
      // 如果在目标响应回调期间发送结果请求失败，可能会发生此情况 (This case can happen if there was
      // a failure to send the result request during the goal response callback)
      throw goal_handle->invalidate_exception_;
    }

    // 如果提供了结果回调，则设置目标句柄的结果回调 (If a result callback is provided, set the goal
    // handle's result callback)
    if (result_callback) {
      // 这将覆盖之前注册的回调 (This will override any previously registered callback)
      goal_handle->set_result_callback(result_callback);
    }

    // 使目标句柄意识到结果 (Make the goal handle aware of the result)
    this->make_result_aware(goal_handle);

    // 返回一个异步获取结果的 future (Return a future for asynchronously getting the result)
    return goal_handle->async_get_result();
  }

  /// 异步请求取消目标。 (Asynchronously request a goal be canceled.)
  /**
   * \throws exceptions::UnknownGoalHandleError 如果目标未知或已达到终止状态。 (If the goal is
   * unknown or already reached a terminal state.) \param[in] goal_handle 请求取消的目标句柄。 (The
   * goal handle requesting to be canceled.) \param[in] cancel_callback
   * 可选的回调函数，当收到响应时调用。 (Optional callback that is called when the response is
   * received.) 回调函数接受一个参数：一个指向 CancelResponse 消息的共享指针。 (The callback takes
   * one parameter: a shared pointer to the CancelResponse message.) \return
   * 当请求被动作服务器确认时，设置为 CancelResponse 消息的 future。 (A future to a CancelResponse
   * message that is set when the request has been acknowledged by an action server.) 参见 (See) <a
   * href="https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv">
   * action_msgs/CancelGoal.srv</a>.
   */
  std::shared_future<typename CancelResponse::SharedPtr> async_cancel_goal(
      typename GoalHandle::SharedPtr goal_handle, CancelCallback cancel_callback = nullptr) {
    // 对 goal_handles_mutex_ 上锁 (Locking the goal_handles_mutex_)
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    // 检查 goal_handles_ 中是否存在此目标句柄 (Checking if the goal handle exists in goal_handles_)
    if (goal_handles_.count(goal_handle->get_goal_id()) == 0) {
      // 抛出异常：未知的目标句柄错误 (Throwing exception: Unknown Goal Handle Error)
      throw exceptions::UnknownGoalHandleError();
    }
    // 创建取消请求 (Creating the cancel request)
    auto cancel_request = std::make_shared<CancelRequest>();
    // 设置取消请求的目标 ID (Setting the goal ID for the cancel request)
    cancel_request->goal_info.goal_id.uuid = goal_handle->get_goal_id();
    // 调用 async_cancel 函数并返回结果 (Calling the async_cancel function and returning the result)
    return async_cancel(cancel_request, cancel_callback);
  }

  /// 异步请求取消所有目标。 (Asynchronously request for all goals to be canceled.)
  /**
   * \param[in] cancel_callback 可选的回调函数，当收到响应时调用。 (Optional callback that is called
   * when the response is received.) 回调函数接受一个参数：一个指向 CancelResponse 消息的共享指针。
   * (The callback takes one parameter: a shared pointer to the CancelResponse message.) \return
   * 当请求被动作服务器确认时，设置为 CancelResponse 消息的 future。 (A future to a CancelResponse
   * message that is set when the request has been acknowledged by an action server.) 参见 (See) <a
   * href="https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv">
   * action_msgs/CancelGoal.srv</a>.
   */
  std::shared_future<typename CancelResponse::SharedPtr> async_cancel_all_goals(
      CancelCallback cancel_callback = nullptr) {
    // 创建取消请求 (Creating the cancel request)
    auto cancel_request = std::make_shared<CancelRequest>();
    // 将取消请求的目标 ID 的 UUID 全部设置为 0，表示取消所有目标 (Setting all UUIDs of the goal ID
    // in the cancel request to 0, indicating canceling all goals)
    std::fill(
        cancel_request->goal_info.goal_id.uuid.begin(),
        cancel_request->goal_info.goal_id.uuid.end(), 0u);
    // 调用 async_cancel 函数并返回结果 (Calling the async_cancel function and returning the result)
    return async_cancel(cancel_request, cancel_callback);
  }

  /// 异步请求在指定时间或之前的所有目标被取消。
  /// Asynchronously request all goals at or before a specified time be canceled.
  /**
   * \param[in] stamp 取消目标请求的时间戳。
   * \param[in] stamp The timestamp for the cancel goal request.
   * \param[in] cancel_callback 当收到响应时调用的可选回调。
   * \param[in] cancel_callback Optional callback that is called when the response is received.
   *   回调接受一个参数：指向CancelResponse消息的共享指针。
   *   The callback takes one parameter: a shared pointer to the CancelResponse message.
   * \return 当请求已被动作服务器确认时设置的指向CancelResponse消息的future。
   * \return A future to a CancelResponse message that is set when the request has been
   * acknowledged by an action server.
   * 参见
   * See
   * <a href="https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv">
   * action_msgs/CancelGoal.srv</a>.
   */
  std::shared_future<typename CancelResponse::SharedPtr> async_cancel_goals_before(
      const rclcpp::Time& stamp, CancelCallback cancel_callback = nullptr) {
    // 创建一个CancelRequest共享指针
    // Create a shared pointer to a CancelRequest
    auto cancel_request = std::make_shared<CancelRequest>();
    // 使用0u填充cancel_request->goal_info.goal_id.uuid的范围
    // Fill the range of cancel_request->goal_info.goal_id.uuid with 0u
    std::fill(
        cancel_request->goal_info.goal_id.uuid.begin(),
        cancel_request->goal_info.goal_id.uuid.end(), 0u);
    // 设置取消请求的时间戳
    // Set the timestamp for the cancel request
    cancel_request->goal_info.stamp = stamp;
    // 调用async_cancel并返回结果
    // Call async_cancel and return the result
    return async_cancel(cancel_request, cancel_callback);
  }

  // 客户端虚拟析构函数
  // Virtual destructor for the Client
  virtual ~Client() {
    // 使用互斥锁保护goal_handles_数据结构
    // Protect the goal_handles_ data structure with a mutex lock
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);
    // 初始化迭代器指向goal_handles_的开始
    // Initialize an iterator pointing to the beginning of goal_handles_
    auto it = goal_handles_.begin();
    // 当迭代器不等于goal_handles_的结束时，进行循环
    // Loop while the iterator is not equal to the end of goal_handles_
    while (it != goal_handles_.end()) {
      // 获取一个指向GoalHandle的弱指针，并尝试将其升级为共享指针
      // Get a weak pointer to a GoalHandle and attempt to upgrade it to a shared pointer
      typename GoalHandle::SharedPtr goal_handle = it->second.lock();
      // 如果goal_handle有效
      // If goal_handle is valid
      if (goal_handle) {
        // 使目标句柄无效，并传递UnawareGoalHandleError异常
        // Invalidate the goal handle and pass the UnawareGoalHandleError exception
        goal_handle->invalidate(exceptions::UnawareGoalHandleError());
      }
      // 从goal_handles_中擦除当前迭代器指向的元素，并将迭代器更新为指向下一个元素
      // Erase the element pointed to by the current iterator from goal_handles_ and update the
      // iterator to point to the next element
      it = goal_handles_.erase(it);
    }
  }

private:
  /// \internal
  /// \brief 创建一个目标响应对象 (Create a goal response object)
  /// \return 返回一个 std::shared_ptr<void> 类型的指针，指向一个新创建的 GoalResponse 对象 (Returns
  /// a std::shared_ptr<void> pointing to a newly created GoalResponse object)
  std::shared_ptr<void> create_goal_response() const override {
    // 使用 ActionT::Impl::SendGoalService::Response 类型定义 GoalResponse (Define GoalResponse
    // using the ActionT::Impl::SendGoalService::Response type)
    using GoalResponse = typename ActionT::Impl::SendGoalService::Response;

    // 创建一个新的 GoalResponse 对象，并用 std::shared_ptr<void> 包装 (Create a new GoalResponse
    // object and wrap it with a std::shared_ptr<void>)
    return std::shared_ptr<void>(new GoalResponse());
  }

  /// \internal
  /// \brief 创建一个结果响应对象 (Create a result response object)
  /// \return 返回一个 std::shared_ptr<void> 类型的指针，指向一个新创建的 GoalResultResponse 对象
  /// (Returns a std::shared_ptr<void> pointing to a newly created GoalResultResponse object)
  std::shared_ptr<void> create_result_response() const override {
    // 使用 ActionT::Impl::GetResultService::Response 类型定义 GoalResultResponse (Define
    // GoalResultResponse using the ActionT::Impl::GetResultService::Response type)
    using GoalResultResponse = typename ActionT::Impl::GetResultService::Response;

    // 创建一个新的 GoalResultResponse 对象，并用 std::shared_ptr<void> 包装 (Create a new
    // GoalResultResponse object and wrap it with a std::shared_ptr<void>)
    return std::shared_ptr<void>(new GoalResultResponse());
  }

  /// \internal
  /// \brief 创建一个取消响应对象 (Create a cancel response object)
  /// \return 返回一个 std::shared_ptr<void> 类型的指针，指向一个新创建的 CancelResponse 对象
  /// (Returns a std::shared_ptr<void> pointing to a newly created CancelResponse object)
  std::shared_ptr<void> create_cancel_response() const override {
    // 创建一个新的 CancelResponse 对象，并用 std::shared_ptr<void> 包装 (Create a new
    // CancelResponse object and wrap it with a std::shared_ptr<void>)
    return std::shared_ptr<void>(new CancelResponse());
  }

  /// \internal
  /// \brief 创建一个反馈消息对象 (Create a feedback message object)
  /// \return 返回一个 std::shared_ptr<void> 类型的指针，指向一个新创建的 FeedbackMessage 对象
  /// (Returns a std::shared_ptr<void> pointing to a newly created FeedbackMessage object)
  std::shared_ptr<void> create_feedback_message() const override {
    // 使用 ActionT::Impl::FeedbackMessage 类型定义 FeedbackMessage (Define FeedbackMessage using
    // the ActionT::Impl::FeedbackMessage type)
    using FeedbackMessage = typename ActionT::Impl::FeedbackMessage;

    // 创建一个新的 FeedbackMessage 对象，并用 std::shared_ptr<void> 包装 (Create a new
    // FeedbackMessage object and wrap it with a std::shared_ptr<void>)
    return std::shared_ptr<void>(new FeedbackMessage());
  }

  /// \internal
  /// \brief 处理接收到的反馈消息
  /// \param message 反馈消息的共享指针
  ///
  /// Handle the received feedback message.
  /// \param message Shared pointer to the feedback message
  void handle_feedback_message(std::shared_ptr<void> message) override {
    // 使用 std::mutex 保护 goal_handles_ 的访问
    // Protect access to goal_handles_ using std::mutex
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);

    // 定义 FeedbackMessage 类型并创建一个共享指针
    // Define FeedbackMessage type and create a shared pointer
    using FeedbackMessage = typename ActionT::Impl::FeedbackMessage;
    typename FeedbackMessage::SharedPtr feedback_message =
        std::static_pointer_cast<FeedbackMessage>(message);

    // 获取目标 ID
    // Get the goal ID
    const GoalUUID& goal_id = feedback_message->goal_id.uuid;

    // 如果 goal_handles_ 中没有该目标 ID，则忽略此反馈消息
    // If the goal ID is not in goal_handles_, ignore this feedback message
    if (goal_handles_.count(goal_id) == 0) {
      RCLCPP_DEBUG(this->get_logger(), "Received feedback for unknown goal. Ignoring...");
      return;
    }

    // 获取对应目标 ID 的弱引用目标句柄
    // Get the weak reference goal handle for the corresponding goal ID
    typename GoalHandle::SharedPtr goal_handle = goal_handles_[goal_id].lock();

    // 如果没有更多的用户引用，忘记关于该目标的信息
    // Forget about the goal if there are no more user references
    if (!goal_handle) {
      RCLCPP_DEBUG(
          this->get_logger(), "Dropping weak reference to goal handle during feedback callback");
      goal_handles_.erase(goal_id);
      return;
    }

    // 创建一个反馈对象并将其从反馈消息中复制
    // Create a feedback object and copy it from the feedback message
    auto feedback = std::make_shared<Feedback>();
    *feedback = feedback_message->feedback;

    // 调用目标句柄的反馈回调函数
    // Call the feedback callback function of the goal handle
    goal_handle->call_feedback_callback(goal_handle, feedback);
  }

  /// \internal
  /// \brief 创建状态消息对象
  /// Create a status message object
  std::shared_ptr<void> create_status_message() const override {
    // 使用 ActionT::Impl::GoalStatusMessage 类型定义 GoalStatusMessage
    // Define GoalStatusMessage as the type of ActionT::Impl::GoalStatusMessage
    using GoalStatusMessage = typename ActionT::Impl::GoalStatusMessage;

    // 创建一个新的 GoalStatusMessage 对象，并返回其共享指针
    // Create a new GoalStatusMessage object and return its shared pointer
    return std::shared_ptr<void>(new GoalStatusMessage());
  }

  /// \internal
  /// \brief 处理状态消息
  /// \param[in] message 状态消息的共享指针
  /// Handle status messages
  /// \param[in] message Shared pointer to the status message
  void handle_status_message(std::shared_ptr<void> message) override {
    // 使用互斥锁保护 goal_handles_ 数据结构
    // Use a lock guard to protect the goal_handles_ data structure
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);

    // 使用 ActionT::Impl::GoalStatusMessage 类型定义 GoalStatusMessage
    // Define GoalStatusMessage as the type of ActionT::Impl::GoalStatusMessage
    using GoalStatusMessage = typename ActionT::Impl::GoalStatusMessage;

    // 将 message 转换为 GoalStatusMessage 类型的共享指针
    // Cast the message to a shared pointer of type GoalStatusMessage
    auto status_message = std::static_pointer_cast<GoalStatusMessage>(message);

    // 遍历 status_message 中的所有状态
    // Iterate through all statuses in the status_message
    for (const GoalStatus& status : status_message->status_list) {
      // 获取目标 ID
      // Get the goal ID
      const GoalUUID& goal_id = status.goal_info.goal_id.uuid;

      // 如果 goal_handles_ 中不存在该目标 ID，则忽略此状态
      // If the goal ID is not in goal_handles_, ignore this status
      if (goal_handles_.count(goal_id) == 0) {
        RCLCPP_DEBUG(this->get_logger(), "Received status for unknown goal. Ignoring...");
        continue;
      }

      // 获取与目标 ID 对应的弱引用共享指针
      // Get the weak reference shared pointer associated with the goal ID
      typename GoalHandle::SharedPtr goal_handle = goal_handles_[goal_id].lock();

      // 如果没有更多用户引用，则忘记该目标
      // Forget about the goal if there are no more user references
      if (!goal_handle) {
        RCLCPP_DEBUG(
            this->get_logger(), "Dropping weak reference to goal handle during status callback");
        goal_handles_.erase(goal_id);
        continue;
      }

      // 设置目标句柄的状态
      // Set the status of the goal handle
      goal_handle->set_status(status.status);
    }
  }

  /// \internal
  /// \brief 使目标处理程序具有获取结果的能力。
  /// Make the goal handle result-aware.
  ///
  /// \param[in] goal_handle 共享指针，指向一个 GoalHandle 对象。
  /// A shared pointer to a GoalHandle object.
  template <typename ActionT>
  void make_result_aware(typename GoalHandle::SharedPtr goal_handle) {
    // 避免发出多个请求
    // Avoid making more than one request
    if (goal_handle->set_result_awareness(true)) {
      return;
    }
    // 使用 GetResultService::Request 类型别名定义 GoalResultRequest
    // Define GoalResultRequest using the GetResultService::Request type alias
    using GoalResultRequest = typename ActionT::Impl::GetResultService::Request;
    // 创建一个 GoalResultRequest 的共享指针
    // Create a shared pointer to a GoalResultRequest
    auto goal_result_request = std::make_shared<GoalResultRequest>();
    // 设置 goal_result_request 的 goal_id 属性
    // Set the goal_id property of goal_result_request
    goal_result_request->goal_id.uuid = goal_handle->get_goal_id();
    try {
      // 发送结果请求，并提供一个回调函数来处理响应
      // Send the result request and provide a callback to handle the response
      this->send_result_request(
          std::static_pointer_cast<void>(goal_result_request),
          [goal_handle, this](std::shared_ptr<void> response) mutable {
            // 将响应包装在一个包含用户关心字段的结构中
            // Wrap the response in a struct with the fields a user cares about
            WrappedResult wrapped_result;
            // 使用 GetResultService::Response 类型别名定义 GoalResultResponse
            // Define GoalResultResponse using the GetResultService::Response type alias
            using GoalResultResponse = typename ActionT::Impl::GetResultService::Response;
            // 将响应转换为 GoalResultResponse 的共享指针
            // Cast the response to a shared pointer of GoalResultResponse
            auto result_response = std::static_pointer_cast<GoalResultResponse>(response);
            // 为 wrapped_result 的 result 属性创建一个新的 Result 对象
            // Create a new Result object for the result property of wrapped_result
            wrapped_result.result = std::make_shared<typename ActionT::Result>();
            // 将结果从 result_response 复制到 wrapped_result
            // Copy the result from result_response to wrapped_result
            *wrapped_result.result = result_response->result;
            // 设置 wrapped_result 的 goal_id 属性
            // Set the goal_id property of wrapped_result
            wrapped_result.goal_id = goal_handle->get_goal_id();
            // 设置 wrapped_result 的 code 属性
            // Set the code property of wrapped_result
            wrapped_result.code = static_cast<ResultCode>(result_response->status);
            // 将 wrapped_result 设置为 goal_handle 的结果
            // Set wrapped_result as the result of goal_handle
            goal_handle->set_result(wrapped_result);
            // 使用 lock_guard 对 goal_handles_mutex_ 加锁
            // Lock goal_handles_mutex_ using lock_guard
            std::lock_guard<std::mutex> lock(goal_handles_mutex_);
            // 从 goal_handles_ 中删除 goal_handle
            // Remove goal_handle from goal_handles_
            goal_handles_.erase(goal_handle->get_goal_id());
          });
    } catch (rclcpp::exceptions::RCLError& ex) {
      // 当用户尝试访问结果时，这将引发异常
      // This will cause an exception when the user tries to access the result
      goal_handle->invalidate(exceptions::UnawareGoalHandleError(ex.message));
    }
  }

  /// \internal
  /// \brief 异步取消目标请求 (Asynchronously cancel a goal request)
  /// \param cancel_request 取消目标请求的共享指针 (Shared pointer to the cancel request)
  /// \param cancel_callback 取消回调函数，默认为空 (Cancel callback function, default is nullptr)
  /// \return 返回一个共享的期望对象，包含取消响应的共享指针 (Returns a shared_future object
  /// containing a shared pointer to the cancel response)
  std::shared_future<typename CancelResponse::SharedPtr> async_cancel(
      typename CancelRequest::SharedPtr cancel_request, CancelCallback cancel_callback = nullptr) {
    // 将 promise 放入堆中以便移动它 (Put promise in the heap to move it around)
    // 创建一个 std::promise 对象，用于存储异步操作的结果 (Create a std::promise object to store the
    // result of the asynchronous operation)
    auto promise = std::make_shared<std::promise<typename CancelResponse::SharedPtr>>();
    // 从 promise 对象获取共享的 future 对象 (Get the shared_future object from the promise object)
    std::shared_future<typename CancelResponse::SharedPtr> future(promise->get_future());
    // 发送取消目标请求 (Send the cancel goal request)
    this->send_cancel_request(
        std::static_pointer_cast<void>(cancel_request),
        // 定义一个 lambda 函数处理收到的响应 (Define a lambda function to handle the received
        // response)
        [cancel_callback, promise](std::shared_ptr<void> response) mutable {
          // 将响应转换为 CancelResponse 类型的共享指针 (Cast the response to a shared pointer of
          // type CancelResponse)
          auto cancel_response = std::static_pointer_cast<CancelResponse>(response);
          // 将取消响应设置为 promise 的值 (Set the cancel response as the value of the promise)
          promise->set_value(cancel_response);
          // 如果提供了取消回调函数，执行它 (If a cancel callback function is provided, execute it)
          if (cancel_callback) {
            cancel_callback(cancel_response);
          }
        });
    // 返回共享的 future 对象 (Return the shared_future object)
    return future;
  }

  // 定义一个映射，将 GoalUUID 映射到弱指针类型的 GoalHandle (Define a map that maps GoalUUID to
  // weak pointers of type GoalHandle)
  std::map<GoalUUID, typename GoalHandle::WeakPtr> goal_handles_;
  // 定义一个互斥锁，用于保护对 goal_handles_ 的访问 (Define a mutex to protect access to
  // goal_handles_)
  std::mutex goal_handles_mutex_;
};
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CLIENT_HPP_

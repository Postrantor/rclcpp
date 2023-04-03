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

#ifndef RCLCPP_ACTION__SERVER_GOAL_HANDLE_HPP_
#define RCLCPP_ACTION__SERVER_GOAL_HANDLE_HPP_

#include <functional>
#include <memory>
#include <mutex>

#include "action_msgs/msg/goal_status.hpp"
#include "rcl_action/goal_handle.h"
#include "rcl_action/types.h"
#include "rclcpp_action/types.hpp"
#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action {

/// 基类，用于与服务器上的目标进行交互。
/// Base class to interact with goals on a server.
/// \internal
/**
 *
 * 此类不应由编写操作服务器的用户直接使用。
 * This class should not be used directly by users writing an action server.
 * 相反，用户将获得一个 `rclcpp_action::ServerGoalHandle<>` 实例。
 * Instead, users will be given an instance of `rclcpp_action::ServerGoalHandle<>`.
 *
 * 在内部，此类负责与 `rcl_action` API 进行接口。
 * Internally, this class is responsible for interfacing with the `rcl_action` API.
 */
class ServerGoalHandleBase {
public:
  /// 指示客户端是否已请求取消此目标。
  /// Indicate if client has requested this goal be cancelled.
  /// \return 如果已接受此目标的取消请求，则返回 true。
  /// \return true if a cancelation request has been accepted for this goal.
  RCLCPP_ACTION_PUBLIC
  bool is_canceling() const;

  /// \brief 表示目标是否处于挂起或执行状态。Indicate if goal is pending or executing.
  /// \return 如果目标已达到终止状态，则返回false。false if goal has reached a terminal state.
  RCLCPP_ACTION_PUBLIC
  bool is_active() const;

  /// \brief 表示目标是否正在执行。Indicate if goal is executing.
  /// \return 只有当目标处于执行状态时才返回true。true only if the goal is in an executing state.
  RCLCPP_ACTION_PUBLIC
  bool is_executing() const;

  /// \brief 析构函数，释放ServerGoalHandleBase对象。Destructor, release ServerGoalHandleBase
  /// object.
  RCLCPP_ACTION_PUBLIC
  virtual ~ServerGoalHandleBase();

protected:
  // ----------------------------------------------------------------------------
  // API 用于 ServerGoalHandleBase 和 ServerGoalHandle<> 之间的通信
  // API for communication between ServerGoalHandleBase and ServerGoalHandle<>

  /// \internal
  /// \brief 构造函数，接受一个 rcl_action_goal_handle_t 的共享指针作为参数
  /// \brief Constructor that accepts a shared pointer to an rcl_action_goal_handle_t as the
  /// argument
  RCLCPP_ACTION_PUBLIC
  ServerGoalHandleBase(
      std::shared_ptr<rcl_action_goal_handle_t> rcl_handle  // rcl_action_goal_handle_t 的共享指针
      // Shared pointer to rcl_action_goal_handle_t
      )
      : rcl_handle_(rcl_handle)  // 初始化成员变量 rcl_handle_
                                 // Initialize member variable rcl_handle_
  {}

  /// \internal
  /// \brief 中止目标处理
  /// \brief Abort the goal handling
  RCLCPP_ACTION_PUBLIC
  void _abort();

  /// \internal
  /// \brief 标记目标处理成功
  /// \brief Mark the goal handling as succeeded
  RCLCPP_ACTION_PUBLIC
  void _succeed();

  /// \internal
  /// \brief 取消目标
  /// \brief Cancel the goal
  RCLCPP_ACTION_PUBLIC
  void _cancel_goal();

  /// \internal
  /// \brief 标记目标已取消
  /// \brief Mark the goal as canceled
  RCLCPP_ACTION_PUBLIC
  void _canceled();

  /// \internal
  /// \brief 执行目标处理
  /// \brief Execute the goal handling
  RCLCPP_ACTION_PUBLIC
  void _execute();

  /// \internal
  /// \brief 如果目标从未达到终止状态，则将其转换为取消状态
  /// \brief Transition the goal to canceled state if it never reached a terminal state
  RCLCPP_ACTION_PUBLIC
  bool try_canceling() noexcept;

  // 结束 ServerGoalHandleBase 和 ServerGoalHandle<> 之间通信的 API
  // End API for communication between ServerGoalHandleBase and ServerGoalHandle<>
  // -----------------------------------------------------------------------------

private:
  /**
   * @brief rcl_handle_ 是一个 rcl_action_goal_handle_t 的共享指针，用于操作和管理 action goal。
   * @details 使用 std::shared_ptr 管理资源，以便在不再需要时自动释放内存。
   * @brief rcl_handle_ is a shared pointer to an rcl_action_goal_handle_t, used for manipulating
   * and managing action goals.
   * @details Uses std::shared_ptr for resource management, so the memory is automatically released
   * when no longer needed.
   */
  std::shared_ptr<rcl_action_goal_handle_t> rcl_handle_;

  /**
   * @brief rcl_handle_mutex_ 是一个可变互斥锁，用于保护对 rcl_handle_ 的访问。
   * @details 使用 mutable 关键字允许在 const 成员函数中修改它。
   * @brief rcl_handle_mutex_ is a mutable mutex used to protect access to rcl_handle_.
   * @details The mutable keyword allows it to be modified within const member functions.
   */
  mutable std::mutex rcl_handle_mutex_;
};

// Forward declare server
template <typename ActionT>
class Server;

/// 用于与服务器上的目标进行交互的类。
/// Class to interact with goals on a server.
/**
 * 使用此类来检查目标的状态以及设置结果。
 * Use this class to check the status of a goal as well as set the result.
 *
 * 此类不是由用户创建的，而是在接受目标时创建的。
 * This class is not meant to be created by a user, instead it is created when a goal has been
 * accepted. `Server`将创建一个实例，并在用户的`handle_accepted`回调中提供给用户。 A `Server` will
 * create an instance and give it to the user in their `handle_accepted` callback.
 *
 * 在内部，此类负责在C++操作类型和`rclcpp_action::ServerGoalHandleBase`的通用类型之间进行转换。
 * Internally, this class is responsible for converting between the C++ action type and generic
 * types for `rclcpp_action::ServerGoalHandleBase`.
 */
template <typename ActionT>
class ServerGoalHandle : public ServerGoalHandleBase {
public:
  /// 发送关于目标进度的更新。
  /// Send an update about the progress of a goal.
  /**
   * 只有在目标处于执行状态时才能调用此方法。
   * This must only be called when the goal is executing.
   * 如果目标的执行被推迟，则必须先调用`ServerGoalHandle::set_executing()`。
   * If execution of a goal is deferred then `ServerGoalHandle::set_executing()` must be called
   * first.
   *
   * \throws std::runtime_error 如果目标处于执行之外的任何状态。
   * \throws std::runtime_error If the goal is in any state besides executing.
   *
   * \param[in] feedback_msg 发布给客户端的消息。
   * \param[in] feedback_msg the message to publish to clients.
   */
  void publish_feedback(std::shared_ptr<typename ActionT::Feedback> feedback_msg) {
    // 创建一个新的反馈消息对象
    // Create a new feedback message object
    auto feedback_message = std::make_shared<typename ActionT::Impl::FeedbackMessage>();

    // 设置反馈消息的目标ID
    // Set the goal ID for the feedback message
    feedback_message->goal_id.uuid = uuid_;

    // 将传入的反馈消息内容赋值给新创建的反馈消息对象
    // Assign the incoming feedback message content to the newly created feedback message object
    feedback_message->feedback = *feedback_msg;

    // 调用基类方法，发布反馈消息
    // Call the base class method to publish the feedback message
    publish_feedback_(feedback_message);
  }

  /// 表示目标无法达到且已中止。 (Indicate that a goal could not be reached and has been aborted.)
  /**
   * 只有在目标执行但无法完成时才调用此方法。 (Only call this if the goal was executing but cannot
   * be completed.) 这是一个终止状态，在调用此方法后，不应再对目标句柄调用任何方法。 (This is a
   * terminal state, no more methods should be called on a goal handle after this is called.)
   *
   * \throws rclcpp::exceptions::RCLError 如果目标处于执行之外的任何状态。 (If the goal is in any
   * state besides executing.)
   *
   * \param[in] result_msg 发送给客户端的最终结果。 (the final result to send to clients.)
   */
  void abort(typename ActionT::Result::SharedPtr result_msg) {
    _abort();  // 调用内部的_abort方法。 (Call the internal _abort method.)

    // 创建一个GetResultService::Response类型的共享指针。 (Create a shared pointer of type
    // GetResultService::Response.)
    auto response = std::make_shared<typename ActionT::Impl::GetResultService::Response>();

    // 设置响应的状态为STATUS_ABORTED。 (Set the status of the response to STATUS_ABORTED.)
    response->status = action_msgs::msg::GoalStatus::STATUS_ABORTED;

    // 将结果消息设置为传入的result_msg。 (Set the result message to the passed-in result_msg.)
    response->result = *result_msg;

    // 调用on_terminal_state_方法处理终止状态。 (Call the on_terminal_state_ method to handle the
    // terminal state.)
    on_terminal_state_(uuid_, response);
  }

  /// 表示目标已成功。 (Indicate that a goal has succeeded.)
  /**
   * 只有在目标正在执行并达到期望的最终状态时才调用此方法。 (Only call this if the goal is executing
   * and has reached the desired final state.)
   * 这是一个终止状态，在调用此方法后，不应再对目标句柄调用任何方法。 (This is a terminal state, no
   * more methods should be called on a goal handle after this is called.)
   *
   * \throws rclcpp::exceptions::RCLError 如果目标处于执行之外的任何状态。 (If the goal is in any
   * state besides executing.)
   *
   * \param[in] result_msg 发送给客户端的最终结果。 (the final result to send to clients.)
   */
  void succeed(typename ActionT::Result::SharedPtr result_msg) {
    _succeed();  // 调用内部的_succeed方法。 (Call the internal _succeed method.)

    // 创建一个GetResultService::Response类型的共享指针。 (Create a shared pointer of type
    // GetResultService::Response.)
    auto response = std::make_shared<typename ActionT::Impl::GetResultService::Response>();

    // 设置响应的状态为STATUS_SUCCEEDED。 (Set the status of the response to STATUS_SUCCEEDED.)
    response->status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;

    // 将结果消息设置为传入的result_msg。 (Set the result message to the passed-in result_msg.)
    response->result = *result_msg;

    // 调用on_terminal_state_方法处理终止状态。 (Call the on_terminal_state_ method to handle the
    // terminal state.)
    on_terminal_state_(uuid_, response);
  }

  /// 表示一个目标已被取消。
  /// Indicate that a goal has been canceled.
  /**
   * 只有在目标处于执行或挂起状态但已被取消时才调用此方法。
   * Only call this if the goal is executing or pending, but has been canceled.
   * 这是一个终止状态，在调用此方法后，不应再对目标句柄调用任何方法。
   * This is a terminal state, no more methods should be called on a goal handle after this is
   * called.
   *
   * \throws rclcpp::exceptions::RCLError 如果目标处于执行之外的任何状态。
   * \throws rclcpp::exceptions::RCLError If the goal is in any state besides executing.
   *
   * \param[in] result_msg 发送给客户端的最终结果。
   * \param[in] result_msg the final result to send to clients.
   */
  void canceled(typename ActionT::Result::SharedPtr result_msg) {
    _canceled();
    auto response = std::make_shared<typename ActionT::Impl::GetResultService::Response>();
    response->status = action_msgs::msg::GoalStatus::STATUS_CANCELED;
    response->result = *result_msg;
    on_terminal_state_(uuid_, response);
  }

  /// 表示服务器开始执行一个目标。
  /// Indicate that the server is starting to execute a goal.
  /**
   * 只有在目标处于挂起状态时才调用此方法。
   * Only call this if the goal is pending.
   *
   * \throws rclcpp::exceptions::RCLError 如果目标处于执行之外的任何状态。
   * \throws rclcpp::exceptions::RCLError If the goal is in any state besides executing.
   */
  void execute() {
    _execute();
    on_executing_(uuid_);
  }

  /// 获取用户提供的描述目标的消息。
  /// Get the user provided message describing the goal.
  const std::shared_ptr<const typename ActionT::Goal> get_goal() const { return goal_; }

  /// 获取目标的唯一标识符
  /// Get the unique identifier of the goal
  const GoalUUID &get_goal_id() const { return uuid_; }

  virtual ~ServerGoalHandle() {
    // 如果在未达到终止状态的情况下允许句柄销毁，则取消目标
    // Cancel goal if handle was allowed to destruct without reaching a terminal state
    if (try_canceling()) {
      auto null_result = std::make_shared<typename ActionT::Impl::GetResultService::Response>();
      null_result->status = action_msgs::msg::GoalStatus::STATUS_CANCELED;
      on_terminal_state_(uuid_, null_result);
    }
  }

protected:
  /// \internal
  /// \brief ServerGoalHandle 构造函数
  /// \param rcl_handle 一个指向 rcl_action_goal_handle_t 的共享指针
  /// \param uuid Goal 请求的唯一标识符
  /// \param goal 用户提供的描述目标的消息
  /// \param on_terminal_state 当目标状态变为终止状态时调用的回调函数
  /// \param on_executing 当目标状态变为执行中时调用的回调函数
  /// \param publish_feedback 发布反馈消息的回调函数
  ///
  /// ServerGoalHandle constructor
  /// \param rcl_handle A shared pointer to rcl_action_goal_handle_t
  /// \param uuid A unique identifier for the goal request
  /// \param goal The user provided message describing the goal
  /// \param on_terminal_state Callback function called when the goal state becomes terminal
  /// \param on_executing Callback function called when the goal state becomes executing
  /// \param publish_feedback Callback function for publishing feedback messages
  ServerGoalHandle(
      std::shared_ptr<rcl_action_goal_handle_t> rcl_handle,
      GoalUUID uuid,
      std::shared_ptr<const typename ActionT::Goal> goal,
      std::function<void(const GoalUUID &, std::shared_ptr<void>)> on_terminal_state,
      std::function<void(const GoalUUID &)> on_executing,
      std::function<void(std::shared_ptr<typename ActionT::Impl::FeedbackMessage>)>
          publish_feedback)
      : ServerGoalHandleBase(rcl_handle),
        goal_(goal),
        uuid_(uuid),
        on_terminal_state_(on_terminal_state),
        on_executing_(on_executing),
        publish_feedback_(publish_feedback) {}

  /// 用户提供的描述目标的消息
  /// The user provided message describing the goal
  const std::shared_ptr<const typename ActionT::Goal> goal_;

  /// 目标请求的唯一标识符
  /// A unique identifier for the goal request
  const GoalUUID uuid_;

  // 声明 Server<ActionT> 类为友元类
  // Declare the Server<ActionT> class as a friend class
  friend class Server<ActionT>;

  /// 当目标状态变为终止状态时调用的回调函数
  /// Callback function called when the goal state becomes terminal
  std::function<void(const GoalUUID &, std::shared_ptr<void>)> on_terminal_state_;

  /// 当目标状态变为执行中时调用的回调函数
  /// Callback function called when the goal state becomes executing
  std::function<void(const GoalUUID &)> on_executing_;

  /// 发布反馈消息的回调函数
  /// Callback function for publishing feedback messages
  std::function<void(std::shared_ptr<typename ActionT::Impl::FeedbackMessage>)> publish_feedback_;
};
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__SERVER_GOAL_HANDLE_HPP_

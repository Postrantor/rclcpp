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

#ifndef RCLCPP_ACTION__CLIENT_GOAL_HANDLE_HPP_
#define RCLCPP_ACTION__CLIENT_GOAL_HANDLE_HPP_

#include <functional>
#include <future>
#include <memory>
#include <mutex>

#include "action_msgs/msg/goal_status.hpp"
#include "rcl_action/action_client.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/exceptions.hpp"
#include "rclcpp_action/types.hpp"
#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action {
/// \brief 可能的动作目标完成状态（The possible statuses that an action goal can finish with.）
enum class ResultCode : int8_t {
  /// \brief 未知状态（Unknown status）
  UNKNOWN = action_msgs::msg::GoalStatus::STATUS_UNKNOWN,
  /// \brief 成功状态（Succeeded status）
  SUCCEEDED = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
  /// \brief 取消状态（Canceled status）
  CANCELED = action_msgs::msg::GoalStatus::STATUS_CANCELED,
  /// \brief 中止状态（Aborted status）
  ABORTED = action_msgs::msg::GoalStatus::STATUS_ABORTED
};

// Forward declarations
template <typename ActionT>
class Client;

/// 用于与来自动作客户端的目标进行交互的类。 (Class for interacting with goals sent from action
/// clients.)
/**
 * 使用此类检查目标的状态以及获取结果。 (Use this class to check the status of a goal as well as get
 * the result.)
 *
 * 此类不是由用户创建的，而是在目标被接受时创建的。 (This class is not meant to be created by a
 * user, instead it is created when a goal has been accepted.) 在调用 `Client::async_send_goal`
 * 后，`Client` 将创建一个实例并将其返回给用户(通过一个 future)。 (A `Client` will create an
 * instance and return it to the user (via a future) after calling `Client::async_send_goal`.)
 */
template <typename ActionT>
class ClientGoalHandle {
public:
  // 定义智能指针类型，禁止拷贝 (Define smart pointer types, disallow copying)
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ClientGoalHandle)

  /**
   * @brief 包装器，定义动作的结果 (A wrapper that defines the result of an action)
   *
   * @tparam ActionT 动作类型 (Action type)
   */
  template <typename ActionT>
  struct WrappedResult {
    /**
     * @brief 目标的唯一标识符 (The unique identifier of the goal)
     */
    GoalUUID goal_id;

    /**
     * @brief 用于表示目标是被取消、中止还是成功的状态 (A status to indicate if the goal was
     * canceled, aborted, or succeeded)
     */
    ResultCode code;

    /**
     * @brief 随动作一起发送回来的用户定义字段 (User defined fields sent back with an action)
     */
    typename ActionT::Result::SharedPtr result;
  };

  // 使用 Feedback 类型定义 ActionT 的 Feedback 类型
  // Define the Feedback type for ActionT
  using Feedback = typename ActionT::Feedback;

  // 使用 Result 类型定义 ActionT 的 Result 类型
  // Define the Result type for ActionT
  using Result = typename ActionT::Result;

  // 定义 FeedbackCallback 类型，该类型为一个函数，接收 ClientGoalHandle<ActionT>::SharedPtr 和
  // const Feedback 的共享指针 Define the FeedbackCallback type, which is a function that takes a
  // ClientGoalHandle<ActionT>::SharedPtr and a shared pointer to const Feedback
  using FeedbackCallback = std::function<void(
      typename ClientGoalHandle<ActionT>::SharedPtr, const std::shared_ptr<const Feedback>)>;

  // 定义 ResultCallback 类型，该类型为一个函数，接收 WrappedResult 的引用
  // Define the ResultCallback type, which is a function that takes a reference to WrappedResult
  using ResultCallback = std::function<void(const WrappedResult& result)>;

  // 虚析构函数
  // Virtual destructor
  virtual ~ClientGoalHandle();

  /// 获取目标的唯一ID
  /// Get the unique ID for the goal.
  const GoalUUID& get_goal_id() const;

  /// 获取目标接受的时间
  /// Get the time when the goal was accepted.
  rclcpp::Time get_goal_stamp() const;

  /// 获取目标状态码
  /// Get the goal status code.
  int8_t get_status();

  /// 检查操作客户端是否已订阅目标的反馈
  /// Check if an action client has subscribed to feedback for the goal.
  bool is_feedback_aware();

  /// 检查操作客户端是否已请求目标的结果
  /// Check if an action client has requested the result for the goal.
  bool is_result_aware();

private:
  // The templated Client creates goal handles
  // 模板化的客户端创建目标句柄
  friend class Client<ActionT>;

  // 构造函数，用于初始化 ClientGoalHandle 对象
  // Constructor for initializing a ClientGoalHandle object
  ClientGoalHandle(
      const GoalInfo& info,                // 目标信息对象
      FeedbackCallback feedback_callback,  // 反馈回调函数
      ResultCallback result_callback);     // 结果回调函数

  // 设置反馈回调函数
  // Set the feedback callback function
  void set_feedback_callback(FeedbackCallback callback);

  // 设置结果回调函数
  // Set the result callback function
  void set_result_callback(ResultCallback callback);

  // 调用反馈回调函数
  // Call the feedback callback function
  void call_feedback_callback(
      typename ClientGoalHandle<ActionT>::SharedPtr shared_this,   // 共享指针
      typename std::shared_ptr<const Feedback> feedback_message);  // 反馈消息共享指针

  /// 获取一个指向目标结果的 future
  /// Get a future to the goal result.
  /**
   * 如果在发送原始目标请求时设置了 `ignore_result` 标志（参见
   * Client::async_send_goal），则不应调用此方法。 This method should not be called if the
   * `ignore_result` flag was set when sending the original goal request (see
   * Client::async_send_goal).
   *
   * `is_result_aware()` 可用于检查是否可以安全地调用此方法。
   * `is_result_aware()` can be used to check if it is safe to call this method.
   *
   * \throws exceptions::UnawareGoalHandleError If the the goal handle is unaware of the result.
   * \return A future to the result.
   */
  std::shared_future<WrappedResult> async_get_result();

  /// 返回 awareness 的先前值
  /// Returns the previous value of awareness
  bool set_result_awareness(bool awareness);

  // 设置状态
  // Set the status
  void set_status(int8_t status);

  // 设置结果
  // Set the result
  void set_result(const WrappedResult& wrapped_result);

  // 使目标句柄无效
  // Invalidate the goal handle
  void invalidate(const exceptions::UnawareGoalHandleError& ex);

  // 检查是否已失效
  // Check if it's invalidated
  bool is_invalidated() const;

  /**
   * @brief Goal信息结构体 (Goal information structure)
   */
  GoalInfo info_;

  /**
   * @brief 用于捕获异常的指针，初始为nullptr (Pointer for capturing exceptions, initially nullptr)
   */
  std::exception_ptr invalidate_exception_{nullptr};

  /**
   * @brief 表示是否知道结果的布尔值，默认为false (Boolean indicating if the result is known,
   * default to false)
   */
  bool is_result_aware_{false};

  /**
   * @brief 结果承诺对象，用于将来获取WrappedResult (Result promise object, used for obtaining
   * WrappedResult in the future)
   */
  std::promise<WrappedResult> result_promise_;

  /**
   * @brief 共享的future对象，用于获取WrappedResult (Shared future object, used for obtaining
   * WrappedResult)
   */
  std::shared_future<WrappedResult> result_future_;

  /**
   * @brief 反馈回调函数，初始为nullptr (Feedback callback function, initially nullptr)
   */
  FeedbackCallback feedback_callback_{nullptr};

  /**
   * @brief 结果回调函数，初始为nullptr (Result callback function, initially nullptr)
   */
  ResultCallback result_callback_{nullptr};

  /**
   * @brief 目标状态，默认为GoalStatus::STATUS_ACCEPTED (Goal status, default to
   * GoalStatus::STATUS_ACCEPTED)
   */
  int8_t status_{GoalStatus::STATUS_ACCEPTED};

  /**
   * @brief 用于保护handle的互斥锁 (Mutex for protecting the handle)
   */
  std::mutex handle_mutex_;
};
}  // namespace rclcpp_action

#include <rclcpp_action/client_goal_handle_impl.hpp>  // NOLINT(build/include_order)
#endif                                                // RCLCPP_ACTION__CLIENT_GOAL_HANDLE_HPP_

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

#ifndef RCLCPP_ACTION__CLIENT_GOAL_HANDLE_IMPL_HPP_
#define RCLCPP_ACTION__CLIENT_GOAL_HANDLE_IMPL_HPP_

#include <rcl_action/types.h>

#include <memory>

#include "rclcpp/logging.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/exceptions.hpp"

namespace rclcpp_action {

/**
 * @brief ClientGoalHandle 类模板定义，用于处理 ROS2 Action 客户端的目标。
 *        ClientGoalHandle class template definition for handling goals in a ROS2 Action client.
 *
 * @tparam ActionT 表示要处理的 Action 类型。表示要处理的 Action 类型。
 *                 Represents the type of Action to be handled.
 */
template <typename ActionT>
class ClientGoalHandle {
public:
  /**
   * @brief 构造函数。Constructor.
   *
   * @param info GoalInfo 对象，包含有关目标的信息。A GoalInfo object containing information about
   * the goal.
   * @param feedback_callback 当收到目标反馈时调用的回调函数。
   *                          Callback function to be called when feedback is received for the goal.
   * @param result_callback 当收到目标结果时调用的回调函数。
   *                        Callback function to be called when the result is received for the goal.
   */
  ClientGoalHandle(
      const GoalInfo& info, FeedbackCallback feedback_callback, ResultCallback result_callback);

  /// @brief 析构函数。Destructor.
  ~ClientGoalHandle();

  /**
   * @brief 获取目标 ID。Get the goal ID.
   *
   * @return 目标 ID 的引用。A reference to the goal ID.
   */
  const GoalUUID& get_goal_id() const;

  /**
   * @brief 获取目标时间戳。Get the goal timestamp.
   *
   * @return 目标时间戳。The goal timestamp.
   */
  rclcpp::Time get_goal_stamp() const;

  /**
   * @brief 异步获取目标结果。Asynchronously get the goal result.
   *
   * @return std::shared_future，用于访问目标结果。
   *         A std::shared_future to access the goal result.
   */
  std::shared_future<typename ClientGoalHandle<ActionT>::WrappedResult> async_get_result();
};

template <typename ActionT>
ClientGoalHandle<ActionT>::ClientGoalHandle(
    const GoalInfo& info, FeedbackCallback feedback_callback, ResultCallback result_callback)
    : info_(info),
      result_future_(result_promise_.get_future()),
      feedback_callback_(feedback_callback),
      result_callback_(result_callback) {
  // 构造函数初始化成员变量。Constructor initializes member variables.
}

template <typename ActionT>
ClientGoalHandle<ActionT>::~ClientGoalHandle() {
  // 析构函数。Destructor.
}

template <typename ActionT>
const GoalUUID& ClientGoalHandle<ActionT>::get_goal_id() const {
  // 返回目标 ID。Return the goal ID.
  return info_.goal_id.uuid;
}

template <typename ActionT>
rclcpp::Time ClientGoalHandle<ActionT>::get_goal_stamp() const {
  // 返回目标时间戳。Return the goal timestamp.
  return info_.stamp;
}

template <typename ActionT>
std::shared_future<typename ClientGoalHandle<ActionT>::WrappedResult>
ClientGoalHandle<ActionT>::async_get_result() {
  // 锁定互斥体。Lock the mutex.
  std::lock_guard<std::mutex> guard(handle_mutex_);

  // 如果不知道结果，抛出异常。Throw an exception if not aware of the result.
  if (!is_result_aware_) {
    throw exceptions::UnawareGoalHandleError();
  }

  // 返回结果的 future。Return the future of the result.
  return result_future_;
}

/**
 * @brief 设置客户端目标处理的结果 (Set the result of the client goal handle)
 *
 * @tparam ActionT 动作类型 (Action type)
 * @param wrapped_result 包装后的结果 (Wrapped result)
 */
template <typename ActionT>
void ClientGoalHandle<ActionT>::set_result(const WrappedResult& wrapped_result) {
  // 使用互斥锁保护状态和结果 (Use a mutex to protect the status and result)
  std::lock_guard<std::mutex> guard(handle_mutex_);

  // 设置状态为包装结果的代码 (Set the status to the code of the wrapped result)
  status_ = static_cast<int8_t>(wrapped_result.code);

  // 将包装结果设置为结果承诺的值 (Set the wrapped result as the value of the result promise)
  result_promise_.set_value(wrapped_result);

  // 如果有结果回调，调用它 (If there is a result callback, call it)
  if (result_callback_) {
    result_callback_(wrapped_result);
  }
}

/**
 * @brief 设置客户端目标处理的反馈回调 (Set the feedback callback of the client goal handle)
 *
 * @tparam ActionT 动作类型 (Action type)
 * @param callback 反馈回调 (Feedback callback)
 */
template <typename ActionT>
void ClientGoalHandle<ActionT>::set_feedback_callback(FeedbackCallback callback) {
  // 使用互斥锁保护反馈回调 (Use a mutex to protect the feedback callback)
  std::lock_guard<std::mutex> guard(handle_mutex_);

  // 设置反馈回调 (Set the feedback callback)
  feedback_callback_ = callback;
}

/**
 * @brief 设置客户端目标处理的结果回调 (Set the result callback of the client goal handle)
 *
 * @tparam ActionT 动作类型 (Action type)
 * @param callback 结果回调 (Result callback)
 */
template <typename ActionT>
void ClientGoalHandle<ActionT>::set_result_callback(ResultCallback callback) {
  // 使用互斥锁保护结果回调 (Use a mutex to protect the result callback)
  std::lock_guard<std::mutex> guard(handle_mutex_);

  // 设置结果回调 (Set the result callback)
  result_callback_ = callback;
}

/**
 * @brief 获取客户端目标处理的状态 (Get the status of the client goal handle)
 *
 * @tparam ActionT 动作类型 (Action type)
 * @return int8_t 状态 (Status)
 */
template <typename ActionT>
int8_t ClientGoalHandle<ActionT>::get_status() {
  // 使用互斥锁保护状态 (Use a mutex to protect the status)
  std::lock_guard<std::mutex> guard(handle_mutex_);

  // 返回状态 (Return the status)
  return status_;
}

/**
 * @brief 设置客户端目标处理的状态 (Set the status of the client goal handle)
 *
 * @tparam ActionT 动作类型 (Action type)
 * @param status 状态 (Status)
 */
template <typename ActionT>
void ClientGoalHandle<ActionT>::set_status(int8_t status) {
  // 使用互斥锁保护状态 (Use a mutex to protect the status)
  std::lock_guard<std::mutex> guard(handle_mutex_);

  // 设置状态 (Set the status)
  status_ = status;
}

/**
 * @brief 检查客户端目标处理是否关注反馈 (Check if the client goal handle is feedback aware)
 *
 * @tparam ActionT 动作类型 (Action type)
 * @return bool 是否关注反馈 (Whether it is feedback aware)
 */
template <typename ActionT>
bool ClientGoalHandle<ActionT>::is_feedback_aware() {
  // 使用互斥锁保护反馈回调 (Use a mutex to protect the feedback callback)
  std::lock_guard<std::mutex> guard(handle_mutex_);

  // 返回反馈回调是否存在 (Return whether the feedback callback exists)
  return feedback_callback_ != nullptr;
}

/**
 * @brief 检查客户端目标处理是否关注结果 (Check if the client goal handle is result aware)
 *
 * @tparam ActionT 动作类型 (Action type)
 * @return bool 是否关注结果 (Whether it is result aware)
 */
template <typename ActionT>
bool ClientGoalHandle<ActionT>::is_result_aware() {
  // 使用互斥锁保护结果回调 (Use a mutex to protect the result callback)
  std::lock_guard<std::mutex> guard(handle_mutex_);

  // 返回是否关注结果 (Return whether it is result aware)
  return is_result_aware_;
}
/**
 * @brief 设置是否关注结果的状态
 *
 * Set the awareness state for the result.
 *
 * @param[in] awareness 是否关注结果的状态 (The awareness state for the result)
 * @return bool 返回之前的状态 (Return the previous state)
 */
template <typename ActionT>
bool ClientGoalHandle<ActionT>::set_result_awareness(bool awareness) {
  std::lock_guard<std::mutex> guard(
      handle_mutex_);  // 锁定互斥量以确保线程安全 (Lock the mutex to ensure thread safety)
  bool previous = is_result_aware_;  // 获取之前的状态 (Get the previous state)
  is_result_aware_ = awareness;      // 更新状态 (Update the state)
  return previous;                   // 返回之前的状态 (Return the previous state)
}

/**
 * @brief 使目标句柄无效
 *
 * Invalidate the goal handle.
 *
 * @param[in] ex 异常类型 (Exception type)
 */
template <typename ActionT>
void ClientGoalHandle<ActionT>::invalidate(const exceptions::UnawareGoalHandleError& ex) {
  std::lock_guard<std::mutex> guard(
      handle_mutex_);  // 锁定互斥量以确保线程安全 (Lock the mutex to ensure thread safety)
  // 防止多次调用 (Guard against multiple calls)
  if (is_invalidated()) {
    return;
  }
  is_result_aware_ = false;                             // 更新状态 (Update the state)
  invalidate_exception_ = std::make_exception_ptr(ex);  // 设置异常指针 (Set the exception pointer)
  status_ = GoalStatus::STATUS_UNKNOWN;  // 更新目标状态 (Update the goal status)
  result_promise_.set_exception(
      invalidate_exception_);  // 设置结果承诺的异常 (Set the exception for the result promise)
}

/**
 * @brief 检查目标句柄是否已无效
 *
 * Check if the goal handle is invalidated.
 *
 * @return bool 返回目标句柄是否已无效 (Return whether the goal handle is invalidated)
 */
template <typename ActionT>
bool ClientGoalHandle<ActionT>::is_invalidated() const {
  return invalidate_exception_ !=
         nullptr;  // 检查异常指针是否为空 (Check if the exception pointer is null)
}

/**
 * @brief 调用反馈回调函数
 *
 * Call the feedback callback function.
 *
 * @param[in] shared_this 当前类的共享指针 (Shared pointer of the current class)
 * @param[in] feedback_message 反馈消息 (Feedback message)
 */
template <typename ActionT>
void ClientGoalHandle<ActionT>::call_feedback_callback(
    typename ClientGoalHandle<ActionT>::SharedPtr shared_this,
    typename std::shared_ptr<const Feedback> feedback_message) {
  if (shared_this.get() != this) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp_action"), "Sent feedback to wrong goal handle.");
    return;
  }
  std::lock_guard<std::mutex> guard(
      handle_mutex_);  // 锁定互斥量以确保线程安全 (Lock the mutex to ensure thread safety)
  if (nullptr == feedback_callback_) {
    // 正常情况下，可能在目标结果之后收到一些反馈消息 (Normally, some feedback messages may arrive
    // after the goal result)
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp_action"), "Received feedback but goal ignores it.");
    return;
  }
  feedback_callback_(
      shared_this, feedback_message);  // 调用反馈回调函数 (Call the feedback callback function)
}

}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CLIENT_GOAL_HANDLE_IMPL_HPP_

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

#include "rclcpp_action/server_goal_handle.hpp"

#include <memory>

#include "rcl_action/action_server.h"
#include "rcl_action/goal_handle.h"
#include "rclcpp/exceptions.hpp"

namespace rclcpp_action {
/**
 * @brief 析构函数，释放资源 (Destructor, releases resources)
 */
ServerGoalHandleBase::~ServerGoalHandleBase() {}

/**
 * @brief 判断目标状态是否为取消中 (Determine if the goal state is canceling)
 *
 * @return 如果目标状态为取消中，返回true；否则返回false (Returns true if the goal state is
 * canceling; otherwise, returns false)
 */
bool ServerGoalHandleBase::is_canceling() const {
  // 对互斥锁加锁，确保线程安全 (Lock the mutex to ensure thread safety)
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);

  // 初始化目标状态为未知 (Initialize the goal state as unknown)
  rcl_action_goal_state_t state = GOAL_STATE_UNKNOWN;

  // 获取目标句柄的状态 (Get the status of the goal handle)
  rcl_ret_t ret = rcl_action_goal_handle_get_status(rcl_handle_.get(), &state);

  // 检查获取状态是否成功 (Check if getting the status was successful)
  if (RCL_RET_OK != ret) {
    // 抛出异常 (Throw an exception)
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to get goal handle state");
  }

  // 返回目标状态是否为取消中 (Return whether the goal state is canceling)
  return GOAL_STATE_CANCELING == state;
}

/**
 * @brief 判断目标是否处于活动状态 (Determine if the goal is active)
 *
 * @return 如果目标处于活动状态，返回true；否则返回false (Returns true if the goal is active;
 * otherwise, returns false)
 */
bool ServerGoalHandleBase::is_active() const {
  // 对互斥锁加锁，确保线程安全 (Lock the mutex to ensure thread safety)
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);

  // 返回目标句柄是否处于活动状态 (Return whether the goal handle is active)
  return rcl_action_goal_handle_is_active(rcl_handle_.get());
}

/**
 * @brief 判断目标是否正在执行 (Determine if the goal is executing)
 *
 * @return 如果目标正在执行，返回true；否则返回false (Returns true if the goal is executing;
 * otherwise, returns false)
 */
bool ServerGoalHandleBase::is_executing() const {
  // 对互斥锁加锁，确保线程安全 (Lock the mutex to ensure thread safety)
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);

  // 初始化目标状态为未知 (Initialize the goal state as unknown)
  rcl_action_goal_state_t state = GOAL_STATE_UNKNOWN;

  // 获取目标句柄的状态 (Get the status of the goal handle)
  rcl_ret_t ret = rcl_action_goal_handle_get_status(rcl_handle_.get(), &state);

  // 检查获取状态是否成功 (Check if getting the status was successful)
  if (RCL_RET_OK != ret) {
    // 抛出异常 (Throw an exception)
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to get goal handle state");
  }

  // 返回目标状态是否为执行中 (Return whether the goal state is executing)
  return GOAL_STATE_EXECUTING == state;
}

/**
 * @brief 中止目标处理 (Abort the goal handling)
 */
void ServerGoalHandleBase::_abort() {
  // 上锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);

  // 更新目标状态为中止 (Update the goal state to abort)
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_ABORT);

  // 检查操作是否成功 (Check if the operation was successful)
  if (RCL_RET_OK != ret) {
    // 抛出异常 (Throw an exception)
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

/**
 * @brief 标记目标处理成功 (Mark the goal handling as succeeded)
 */
void ServerGoalHandleBase::_succeed() {
  // 上锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);

  // 更新目标状态为成功 (Update the goal state to succeed)
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_SUCCEED);

  // 检查操作是否成功 (Check if the operation was successful)
  if (RCL_RET_OK != ret) {
    // 抛出异常 (Throw an exception)
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

/**
 * @brief 取消目标处理 (Cancel the goal handling)
 */
void ServerGoalHandleBase::_cancel_goal() {
  // 上锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);

  // 更新目标状态为取消 (Update the goal state to cancel)
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_CANCEL_GOAL);

  // 检查操作是否成功 (Check if the operation was successful)
  if (RCL_RET_OK != ret) {
    // 抛出异常 (Throw an exception)
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

/**
 * @brief 标记目标处理已被取消 (Mark the goal handling as canceled)
 */
void ServerGoalHandleBase::_canceled() {
  // 上锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);

  // 更新目标状态为已取消 (Update the goal state to canceled)
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_CANCELED);

  // 检查操作是否成功 (Check if the operation was successful)
  if (RCL_RET_OK != ret) {
    // 抛出异常 (Throw an exception)
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

/**
 * @brief 执行目标处理 (Execute the goal handling)
 */
void ServerGoalHandleBase::_execute() {
  // 上锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);

  // 更新目标状态为执行 (Update the goal state to execute)
  rcl_ret_t ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_EXECUTE);

  // 检查操作是否成功 (Check if the operation was successful)
  if (RCL_RET_OK != ret) {
    // 抛出异常 (Throw an exception)
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

/**
 * @brief 尝试取消一个目标 (Try to cancel a goal)
 *
 * @return bool 如果成功取消，则返回 true；否则返回 false (Returns true if the goal is successfully
 * canceled; otherwise, returns false)
 */
bool ServerGoalHandleBase::try_canceling() noexcept {
  // 使用 lock_guard 对 rcl_handle_mutex_ 进行上锁，以确保线程安全 (Lock the rcl_handle_mutex_ using
  // lock_guard to ensure thread safety)
  std::lock_guard<std::mutex> lock(rcl_handle_mutex_);
  rcl_ret_t ret;

  // 检查目标是否可以取消 (Check if the goal is cancelable)
  const bool is_cancelable = rcl_action_goal_handle_is_cancelable(rcl_handle_.get());
  if (is_cancelable) {
    // 将状态转换为 CANCELING (Transition the state to CANCELING)
    ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_CANCEL_GOAL);
    if (RCL_RET_OK != ret) {
      return false;
    }
  }

  rcl_action_goal_state_t state = GOAL_STATE_UNKNOWN;
  // 获取当前状态 (Get the current state)
  ret = rcl_action_goal_handle_get_status(rcl_handle_.get(), &state);
  if (RCL_RET_OK != ret) {
    return false;
  }

  // 如果当前状态为 CANCELING，则执行取消操作 (If the current state is CANCELING, perform the cancel
  // operation)
  if (GOAL_STATE_CANCELING == state) {
    ret = rcl_action_update_goal_state(rcl_handle_.get(), GOAL_EVENT_CANCELED);
    return RCL_RET_OK == ret;
  }

  return false;
}
}  // namespace rclcpp_action

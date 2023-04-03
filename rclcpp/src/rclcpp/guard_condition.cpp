// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/guard_condition.hpp"

#include <functional>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

namespace rclcpp {

/**
 * @brief 构造一个GuardCondition对象 (Constructs a GuardCondition object)
 *
 * @param context 共享指针，指向rclcpp::Context对象 (A shared pointer to an rclcpp::Context object)
 * @param guard_condition_options 用于初始化guard condition的选项 (Options for initializing the
 * guard condition)
 */
GuardCondition::GuardCondition(
    rclcpp::Context::SharedPtr context, rcl_guard_condition_options_t guard_condition_options)
    : context_(context), rcl_guard_condition_{rcl_get_zero_initialized_guard_condition()} {
  // 检查context是否为空 (Check if context is nullptr)
  if (!context_) {
    throw std::invalid_argument("context argument unexpectedly nullptr");
  }

  // 初始化rcl_guard_condition_ (Initialize rcl_guard_condition_)
  rcl_ret_t ret = rcl_guard_condition_init(
      &this->rcl_guard_condition_, context_->get_rcl_context().get(), guard_condition_options);

  // 如果初始化失败，抛出异常 (If initialization fails, throw an exception)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to create guard condition");
  }
}

/**
 * @brief 析构GuardCondition对象 (Destructs the GuardCondition object)
 */
GuardCondition::~GuardCondition() {
  // 销毁rcl_guard_condition_ (Finalize rcl_guard_condition_)
  rcl_ret_t ret = rcl_guard_condition_fini(&this->rcl_guard_condition_);

  // 如果销毁失败，捕获并记录异常 (If finalization fails, catch and log the exception)
  if (RCL_RET_OK != ret) {
    try {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    } catch (const std::exception& exception) {
      RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"), "failed to finalize guard condition: %s", exception.what());
    }
  }
}

/**
 * @brief 获取GuardCondition的context (Get the context of the GuardCondition)
 *
 * @return 返回一个指向rclcpp::Context对象的共享指针 (Returns a shared pointer to an rclcpp::Context
 * object)
 */
rclcpp::Context::SharedPtr GuardCondition::get_context() const { return context_; }

/**
 * @brief 获取可修改的rcl_guard_condition_t引用 (Get a modifiable reference to
 * rcl_guard_condition_t)
 *
 * @return 返回rcl_guard_condition_的引用 (Returns a reference to rcl_guard_condition_)
 */
rcl_guard_condition_t& GuardCondition::get_rcl_guard_condition() { return rcl_guard_condition_; }

/**
 * @brief 获取只读的rcl_guard_condition_t引用 (Get a read-only reference to rcl_guard_condition_t)
 *
 * @return 返回rcl_guard_condition_的常量引用 (Returns a constant reference to rcl_guard_condition_)
 */
const rcl_guard_condition_t& GuardCondition::get_rcl_guard_condition() const {
  return rcl_guard_condition_;
}

/**
 * @brief 触发保护条件 (Trigger the guard condition)
 */
void GuardCondition::trigger() {
  // 调用 rcl_trigger_guard_condition 函数触发保护条件，并检查返回值
  // Call rcl_trigger_guard_condition function to trigger the guard condition and check the return
  // value
  rcl_ret_t ret = rcl_trigger_guard_condition(&rcl_guard_condition_);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  {
    // 对递归互斥锁进行加锁
    // Lock the recursive mutex
    std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

    // 如果存在触发回调函数，则执行回调函数，否则增加未读计数
    // If there is a trigger callback function, execute the callback function, otherwise increase
    // the unread count
    if (on_trigger_callback_) {
      on_trigger_callback_(1);
    } else {
      unread_count_++;
    }
  }
}

/**
 * @brief 交换等待集合中的使用状态 (Exchange the in-use state in the wait set)
 *
 * @param[in] in_use_state 新的使用状态 (The new in-use state)
 * @return bool 返回旧的使用状态 (Return the old in-use state)
 */
bool GuardCondition::exchange_in_use_by_wait_set_state(bool in_use_state) {
  return in_use_by_wait_set_.exchange(in_use_state);
}

/**
 * @brief 将保护条件添加到等待集合中 (Add the guard condition to the wait set)
 *
 * @param[in] wait_set 等待集合指针 (Pointer to the wait set)
 */
void GuardCondition::add_to_wait_set(rcl_wait_set_t* wait_set) {
  // 对递归互斥锁进行加锁
  // Lock the recursive mutex
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  if (exchange_in_use_by_wait_set_state(true)) {
    if (wait_set != wait_set_) {
      throw std::runtime_error("guard condition has already been added to a wait set.");
    }
  } else {
    wait_set_ = wait_set;
  }

  // 将保护条件添加到等待集合中，并检查返回值
  // Add the guard condition to the wait set and check the return value
  rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, &this->rcl_guard_condition_, NULL);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to add guard condition to wait set");
  }
}

/**
 * @brief 设置触发回调函数 (Set the trigger callback function)
 *
 * @param[in] callback 触发回调函数 (The trigger callback function)
 */
void GuardCondition::set_on_trigger_callback(std::function<void(size_t)> callback) {
  // 对递归互斥锁进行加锁
  // Lock the recursive mutex
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  // 如果存在回调函数，则设置回调函数并执行，否则将回调函数置为空
  // If there is a callback function, set the callback function and execute it, otherwise set the
  // callback function to nullptr
  if (callback) {
    on_trigger_callback_ = callback;

    if (unread_count_) {
      callback(unread_count_);
      unread_count_ = 0;
    }
  } else {
    on_trigger_callback_ = nullptr;
  }
}

}  // namespace rclcpp

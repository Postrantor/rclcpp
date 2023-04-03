// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/timer.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/exceptions.hpp"
#include "rcutils/logging_macros.h"

/// 使用 rclcpp::TimerBase;
using rclcpp::TimerBase;

/**
 * @brief 构造函数 (Constructor)
 *
 * @param clock 时钟共享指针 (Shared pointer to the clock)
 * @param period 定时器周期 (Timer period)
 * @param context 上下文共享指针 (Shared pointer to the context)
 */
TimerBase::TimerBase(
    rclcpp::Clock::SharedPtr clock,
    std::chrono::nanoseconds period,
    rclcpp::Context::SharedPtr context)
    : clock_(clock), timer_handle_(nullptr) {
  // 如果上下文为空，则获取全局默认上下文 (If the context is null, get the global default context)
  if (nullptr == context) {
    context = rclcpp::contexts::get_global_default_context();
  }

  auto rcl_context = context->get_rcl_context();

  // 初始化定时器句柄并设置清理回调 (Initialize the timer handle and set the cleanup callback)
  timer_handle_ = std::shared_ptr<rcl_timer_t>(new rcl_timer_t, [=](rcl_timer_t* timer) mutable {
    {
      std::lock_guard<std::mutex> clock_guard(clock->get_clock_mutex());
      if (rcl_timer_fini(timer) != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
            "rclcpp", "Failed to clean up rcl timer handle: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
    }
    delete timer;
    // 捕获的共享指针通过复制捕获，重置以确保在时钟之前完成定时器 (Captured shared pointers by copy,
    // reset to make sure timer is finalized before clock)
    clock.reset();
    rcl_context.reset();
  });

  *timer_handle_.get() = rcl_get_zero_initialized_timer();

  rcl_clock_t* clock_handle = clock_->get_clock_handle();
  {
    std::lock_guard<std::mutex> clock_guard(clock_->get_clock_mutex());
    rcl_ret_t ret = rcl_timer_init(
        timer_handle_.get(), clock_handle, rcl_context.get(), period.count(), nullptr,
        rcl_get_default_allocator());
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't initialize rcl timer handle");
    }
  }
}

/// 析构函数 (Destructor)
TimerBase::~TimerBase() {}

/**
 * @brief 取消定时器 (Cancel the timer)
 */
void TimerBase::cancel() {
  rcl_ret_t ret = rcl_timer_cancel(timer_handle_.get());
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't cancel timer");
  }
}

/**
 * @brief 检查定时器是否已取消 (Check if the timer is canceled)
 *
 * @return 是否已取消 (Whether it is canceled or not)
 */
bool TimerBase::is_canceled() {
  bool is_canceled = false;
  rcl_ret_t ret = rcl_timer_is_canceled(timer_handle_.get(), &is_canceled);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't get timer cancelled state");
  }
  return is_canceled;
}

/**
 * @brief 重置定时器 (Reset the timer)
 */
void TimerBase::reset() {
  rcl_ret_t ret = rcl_timer_reset(timer_handle_.get());
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't reset timer");
  }
}

/**
 * @brief 检查定时器是否准备好 (Check if the timer is ready)
 *
 * @return 是否准备好 (Whether it is ready or not)
 */
bool TimerBase::is_ready() {
  bool ready = false;
  rcl_ret_t ret = rcl_timer_is_ready(timer_handle_.get(), &ready);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to check timer");
  }
  return ready;
}

/**
 * @brief 获取触发定时器的时间 (Get the time until the timer triggers)
 *
 * @return 触发定时器的时间 (Time until the timer triggers)
 */
std::chrono::nanoseconds TimerBase::time_until_trigger() {
  int64_t time_until_next_call = 0;
  rcl_ret_t ret = rcl_timer_get_time_until_next_call(timer_handle_.get(), &time_until_next_call);
  if (ret == RCL_RET_TIMER_CANCELED) {
    return std::chrono::nanoseconds::max();
  } else if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Timer could not get time until next call");
  }
  return std::chrono::nanoseconds(time_until_next_call);
}

/// 获取定时器句柄 (Get the timer handle)
std::shared_ptr<const rcl_timer_t> TimerBase::get_timer_handle() { return timer_handle_; }

/**
 * @brief 交换等待集合中使用的状态 (Exchange the state used in the wait set)
 *
 * @param in_use_state 使用中的状态 (State in use)
 * @return 交换后的状态 (Exchanged state)
 */
bool TimerBase::exchange_in_use_by_wait_set_state(bool in_use_state) {
  return in_use_by_wait_set_.exchange(in_use_state);
}

// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__TIMER_HPP_
#define RCLCPP__TIMER_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <thread>
#include <type_traits>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/timer.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "tracetools/tracetools.h"
#include "tracetools/utils.hpp"

namespace rclcpp {

class TimerBase {
public:
  // 不可复制智能指针定义
  // Smart pointer definitions not copyable
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TimerBase)

  /// TimerBase 构造函数
  /// TimerBase constructor
  /**
   * \param clock 用于时间和休眠的时钟
   * \param period 计时器触发的间隔
   * \param context 节点上下文
   *
   * \param clock A clock to use for time and sleeping
   * \param period The interval at which the timer fires
   * \param context node context
   */
  RCLCPP_PUBLIC
  explicit TimerBase(
      Clock::SharedPtr clock, std::chrono::nanoseconds period, rclcpp::Context::SharedPtr context);

  /// TimerBase 析构函数
  /// TimerBase destructor
  RCLCPP_PUBLIC
  virtual ~TimerBase();

  /// 取消计时器。
  /// Cancel the timer.
  /**
   * \throws std::runtime_error 如果 rcl_timer_cancel 返回失败
   * \throws std::runtime_error if the rcl_timer_cancel returns a failure
   */
  RCLCPP_PUBLIC
  void cancel();

  /// 返回计时器取消状态。
  /// Return the timer cancellation state.
  /**
   * \return 如果计时器已取消，则为 true；否则为 false
   * \throws std::runtime_error 如果 rcl_get_error_state 返回 0
   * \throws rclcpp::exceptions::RCLError 根据 ret 返回某些子类异常
   *
   * \return true if the timer has been cancelled, false otherwise
   * \throws std::runtime_error if the rcl_get_error_state returns 0
   * \throws rclcpp::exceptions::RCLError some child class exception based on ret
   */
  RCLCPP_PUBLIC
  bool is_canceled();

  /// 重置计时器。
  /// Reset the timer.
  /**
   * \throws std::runtime_error 如果 rcl_timer_reset 返回失败
   * \throws std::runtime_error if the rcl_timer_reset returns a failure
   */
  RCLCPP_PUBLIC
  void reset();

  /// 表示我们即将执行回调。
  /// Indicate that we're about to execute the callback.
  /**
   * 多线程执行器利用这一点来避免多次安排回调。
   *
   * The multithreaded executor takes advantage of this to avoid scheduling
   * the callback multiple times.
   *
   * \return 如果应执行回调，则返回 `true`；如果计时器被取消，则返回 `false`。
   * \return `true` if the callback should be executed, `false` if the timer was canceled.
   */
  RCLCPP_PUBLIC
  virtual bool call() = 0;

  /// 当计时器信号发出时，调用回调函数。
  /// Call the callback function when the timer signal is emitted.
  RCLCPP_PUBLIC
  virtual void execute_callback() = 0;

  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_timer_t> get_timer_handle();

  /// 检查计时器距离下一个预定回调还有多长时间。
  /// Check how long the timer has until its next scheduled callback.
  /**
   * \return 一个 std::chrono::duration，表示到下一个回调的相对时间
   * 或者 std::chrono::nanoseconds::max()，如果计时器被取消。
   * \throws std::runtime_error 如果 rcl_timer_get_time_until_next_call 返回失败
   *
   * \return A std::chrono::duration representing the relative time until the next callback
   * or std::chrono::nanoseconds::max() if the timer is canceled.
   * \throws std::runtime_error if the rcl_timer_get_time_until_next_call returns a failure
   */
  RCLCPP_PUBLIC
  std::chrono::nanoseconds time_until_trigger();

  /// 时钟是否稳定（即，滴答之间的时间是否恒定？）
  /// Is the clock steady (i.e. is the time between ticks constant?)
  /** \return 如果此计时器使用的时钟稳定，则返回 True。 */
  /** \return True if the clock used by this timer is steady. */
  virtual bool is_steady() = 0;

  /// 检查计时器是否准备好触发回调。
  /// Check if the timer is ready to trigger the callback.
  /**
   * 该函数要求其调用者在此函数之后立即触发回调，
   * 因为它维护了上次触发回调的时间。
   * \return 如果计时器需要触发，则返回 True。
   * \throws std::runtime_error 如果检查计时器失败
   *
   * This function expects its caller to immediately trigger the callback after this function,
   * since it maintains the last time the callback was triggered.
   * \return True if the timer needs to trigger.
   * \throws std::runtime_error if it failed to check timer
   */
  RCLCPP_PUBLIC
  bool is_ready();

  /// 交换此计时器的 "由等待集使用" 状态。
  /// Exchange the "in use by wait set" state for this timer.
  /**
   * 这用于确保此计时器不被多个
   * 同时使用等待集。
   *
   * This is used to ensure this timer is not used by multiple
   * wait sets at the same time.
   *
   * \param[in] in_use_state 要交换到状态的新状态，true
   *   表示它现在由等待集使用，false 表示它不再由等待集使用。
   * \returns 上一个状态。
   *
   * \param[in] in_use_state the new state to exchange into the state, true
   *   indicates it is now in use by a wait set, and false is that it is no
   *   longer in use by a wait set.
   * \returns the previous state.
   */
  RCLCPP_PUBLIC
  bool exchange_in_use_by_wait_set_state(bool in_use_state);

protected:
  Clock::SharedPtr clock_;
  std::shared_ptr<rcl_timer_t> timer_handle_;

  std::atomic<bool> in_use_by_wait_set_{false};
};

using VoidCallbackType = std::function<void()>;
using TimerCallbackType = std::function<void(TimerBase &)>;

/// 通用计时器。定期执行用户指定的回调。
/// Generic timer. Periodically executes a user-specified callback.
template <
    typename FunctorT,
    typename std::enable_if<
        rclcpp::function_traits::same_arguments<FunctorT, VoidCallbackType>::value ||
        rclcpp::function_traits::same_arguments<FunctorT, TimerCallbackType>::value>::type * =
        nullptr>
class GenericTimer : public TimerBase {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericTimer)

  /// 默认构造函数。
  /// Default constructor.
  /**
   * \param[in] clock 提供当前时间的时钟。
   * \param[in] period 计时器触发的间隔。
   * \param[in] callback 用户指定的回调函数。
   * \param[in] context 要使用的自定义上下文。
   *
   * \param[in] clock The clock providing the current time.
   * \param[in] period The interval at which the timer fires.
   * \param[in] callback User-specified callback function.
   * \param[in] context custom context to be used.
   */
  explicit GenericTimer(
      Clock::SharedPtr clock,
      std::chrono::nanoseconds period,
      FunctorT &&callback,
      rclcpp::Context::SharedPtr context)
      : TimerBase(clock, period, context), callback_(std::forward<FunctorT>(callback)) {
    TRACEPOINT(
        rclcpp_timer_callback_added, static_cast<const void *>(get_timer_handle().get()),
        reinterpret_cast<const void *>(&callback_));
    TRACEPOINT(
        rclcpp_callback_register, reinterpret_cast<const void *>(&callback_),
        tracetools::get_symbol(callback_));
  }

  /// 默认析构函数。
  /// Default destructor.
  virtual ~GenericTimer() {
    // 停止计时器运行。
    // Stop the timer from running.
    cancel();
  }

  /**
   * \sa rclcpp::TimerBase::call
   * \throws std::runtime_error 如果通知计时器回调发生失败
   * \throws std::runtime_error if it failed to notify timer that callback will occur
   */
  bool call() override {
    rcl_ret_t ret = rcl_timer_call(timer_handle_.get());
    if (ret == RCL_RET_TIMER_CANCELED) {
      return false;
    }
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("Failed to notify timer that callback occurred");
    }
    return true;
  }

  /**
   * \sa rclcpp::TimerBase::execute_callback
   */
  void execute_callback() override {
    TRACEPOINT(callback_start, reinterpret_cast<const void *>(&callback_), false);
    execute_callback_delegate<>();
    TRACEPOINT(callback_end, reinterpret_cast<const void *>(&callback_));
  }

  // void specialization
  template <
      typename CallbackT = FunctorT,
      typename std::enable_if<
          rclcpp::function_traits::same_arguments<CallbackT, VoidCallbackType>::value>::type * =
          nullptr>
  void execute_callback_delegate() {
    callback_();
  }

  template <
      typename CallbackT = FunctorT,
      typename std::enable_if<
          rclcpp::function_traits::same_arguments<CallbackT, TimerCallbackType>::value>::type * =
          nullptr>
  void execute_callback_delegate() {
    callback_(*this);
  }

  /// 时钟是否稳定（即，滴答之间的时间是否恒定？）
  /// Is the clock steady (i.e. is the time between ticks constant?)
  /** \return 如果此计时器使用的时钟稳定，则返回 True。 */
  /** \return True if the clock used by this timer is steady. */
  bool is_steady() override { return clock_->get_clock_type() == RCL_STEADY_TIME; }

protected:
  RCLCPP_DISABLE_COPY(GenericTimer)

  FunctorT callback_;
};

template <
    typename FunctorT,
    typename std::enable_if<
        rclcpp::function_traits::same_arguments<FunctorT, VoidCallbackType>::value ||
        rclcpp::function_traits::same_arguments<FunctorT, TimerCallbackType>::value>::type * =
        nullptr>
class WallTimer : public GenericTimer<FunctorT> {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(WallTimer)

  /// 墙上计时器构造函数
  /// Wall timer constructor
  /**
   * \param period 计时器触发的间隔
   * \param callback 每个间隔执行的回调函数
   * \param context 节点上下文
   *
   * \param period The interval at which the timer fires
   * \param callback The callback function to execute every interval
   * \param context node context
   */
  WallTimer(
      std::chrono::nanoseconds period, FunctorT &&callback, rclcpp::Context::SharedPtr context)
      : GenericTimer<FunctorT>(
            std::make_shared<Clock>(RCL_STEADY_TIME), period, std::move(callback), context) {}

protected:
  RCLCPP_DISABLE_COPY(WallTimer)
};

}  // namespace rclcpp

#endif  // RCLCPP__TIMER_HPP_

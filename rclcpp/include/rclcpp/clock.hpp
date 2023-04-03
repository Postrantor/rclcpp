// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CLOCK_HPP_
#define RCLCPP__CLOCK_HPP_

#include <functional>
#include <memory>
#include <mutex>

#include "rcl/time.h"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcutils/time.h"
#include "rcutils/types/rcutils_ret.h"

namespace rclcpp {

class TimeSource;

/**
 * @class JumpHandler
 * @brief 跳跃处理器类，用于处理时间跳跃事件 (Jump handler class for handling time jump events)
 */
class JumpHandler {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(JumpHandler)

  /// 预处理回调函数类型定义 (Pre-processing callback function type definition)
  using pre_callback_t = std::function<void()>;
  /// 后处理回调函数类型定义 (Post-processing callback function type definition)
  using post_callback_t = std::function<void(const rcl_time_jump_t &)>;

  /**
   * @brief 构造函数 (Constructor)
   * @param pre_callback 预处理回调函数 (Pre-processing callback function)
   * @param post_callback 后处理回调函数 (Post-processing callback function)
   * @param threshold 跳跃阈值 (Jump threshold)
   */
  JumpHandler(
      pre_callback_t pre_callback,
      post_callback_t post_callback,
      const rcl_jump_threshold_t &threshold);

  /// 预处理回调函数成员变量 (Pre-processing callback function member variable)
  pre_callback_t pre_callback;
  /// 后处理回调函数成员变量 (Post-processing callback function member variable)
  post_callback_t post_callback;
  /// 通知阈值成员变量 (Notice threshold member variable)
  rcl_jump_threshold_t notice_threshold;
};

class Clock {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Clock)

  /// 默认构造函数 (Default constructor)
  /**
   * 使用给定的 clock_type 初始化 clock 实例。
   * (Initializes the clock instance with the given clock_type.)
   *
   * \param clock_type 时钟类型。(type of the clock.)
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  RCLCPP_PUBLIC
  explicit Clock(rcl_clock_type_t clock_type = RCL_SYSTEM_TIME);

  /// 析构函数 (Destructor)
  RCLCPP_PUBLIC
  ~Clock();

  /// 获取当前时间 (Get current time)
  /**
   * 返回由 clock_type 指定的时间源的当前时间。
   * (Returns current time from the time source specified by clock_type.)
   *
   * \return 当前时间。(current time.)
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  RCLCPP_PUBLIC
  Time now() const;

  /// 根据时钟类型睡眠到指定时间 (Sleep until a specified Time, according to clock type)
  /**
   * 对于 RCL_ROS_TIME 时钟类型的注意事项：
   * (Notes for RCL_ROS_TIME clock type:)
   *   - 如果 ros 时间处于活动状态并且接收到的时钟永远不会达到 `until`，则可以永久睡眠。
   *     (Can sleep forever if ros time is active and received clock never reaches `until`)
   *   - 如果 ROS 时间启用状态在睡眠期间发生更改，此方法将立即返回 false。
   *     (If ROS time enabled state changes during the sleep, this method will immediately return
   * false.) 当时间源发生变化时，没有一个一致的选择来睡眠时间，因此这取决于调用者在需要时再次调用。
   *     (There is not a consistent choice of sleeping time when the time source changes,
   *     so this is up to the caller to call again if needed.)
   *
   * \warning 使用 gcc < 10 或使用 gcc >= 10 且 pthreads 缺少函数 `pthread_cond_clockwait` 时，
   *          稳定时钟可能会使用系统时钟进行睡眠。如果是这样，稳定时钟的睡眠时间可能会受到系统时钟时间跳变的影响。
   *          根据稳定时钟的纪元和分辨率与系统时钟的比较，将稳定时钟持续时间转换为系统时钟时间可能会导致未定义的行为。
   *          有关更多信息，请参阅以下问题：
   *          https://gcc.gnu.org/bugzilla/show_bug.cgi?id=41861
   *          https://gcc.gnu.org/bugzilla/show_bug.cgi?id=58931
   *
   * \param until 根据当前时钟类型计算的绝对时间。(absolute time according to current clock type to
   * sleep until.) \param context 时钟应该用于检查 ROS 是否仍然初始化的 rclcpp 上下文。(the rclcpp
   * context the clock should use to check that ROS is still initialized.) \return 如果 `until`
   * 已过去，则立即返回 true。(true immediately if `until` is in the past) \return 当达到时间
   * `until` 时返回 true。(true when the time `until` is reached) \return
   * 如果无法可靠地达到时间，则返回 false，例如由于关闭或更改时间源。(false if time cannot be
   * reached reliably, for example from shutdown or a change of time source.) \throws
   * std::runtime_error 如果上下文无效。(if the context is invalid) \throws std::runtime_error 如果
   * `until` 的时钟类型与此时钟不同。(if `until` has a different clock type from this clock)
   */
  RCLCPP_PUBLIC
  bool sleep_until(Time until, Context::SharedPtr context = contexts::get_global_default_context());

  /// 睡眠指定的持续时间 (Sleep for a specified Duration)
  /**
   * 等效于 (Equivalent to)
   *
   * ```cpp
   * clock->sleep_until(clock->now() + rel_time, context)
   * ```
   *
   * 如果 `rel_time` 为零或负数，函数将立即返回。
   * (The function will return immediately if `rel_time` is zero or negative.)
   *
   * \param rel_time 要睡眠的时间长度。(the length of time to sleep for.)
   * \param context 时钟应该用于检查 ROS 是否仍然初始化的 rclcpp 上下文。(the rclcpp context the
   * clock should use to check that ROS is still initialized.) \return 当达到结束时间时返回
   * true。(true when the end time is reached) \return 如果无法可靠地达到时间，则返回
   * false，例如由于关闭或更改时间源。(false if time cannot be reached reliably, for example from
   * shutdown or a change of time source.) \throws std::runtime_error 如果上下文无效。(if the
   * context is invalid)
   */
  RCLCPP_PUBLIC
  bool sleep_for(
      Duration rel_time, Context::SharedPtr context = contexts::get_global_default_context());

  /// 检查时钟是否已启动 (Check if the clock is started)
  /**
   * 已启动的时钟是指反映非零时间的时钟。
   * (A started clock is a clock that reflects non-zero time.)
   * 通常，如果时钟使用 RCL_ROS_TIME 和 ROS 时间且时钟主题尚未发布任何内容，则时钟将处于未启动状态。
   * (Typically a clock will be unstarted if it is using RCL_ROS_TIME with ROS time and
   * nothing has been published on the clock topic yet.)
   *
   * \return 如果时钟已启动，则返回 true。(true if clock is started)
   * \throws std::runtime_error 如果时钟不是 rcl_clock_valid。(if the clock is not rcl_clock_valid)
   */
  RCLCPP_PUBLIC
  bool started();

  /// 等待时钟开始 (Wait until clock to start)
  /**
   * \rclcpp::Clock::started
   * \param context 要等待的上下文。(the context to wait in)
   * \return 如果时钟已经启动或变为启动，则返回 true。(true if clock was already started or became
   * started) \throws std::runtime_error 如果上下文无效或时钟不是 rcl_clock_valid。(if the context
   * is invalid or clock is not rcl_clock_valid)
   */
  RCLCPP_PUBLIC
  bool wait_until_started(Context::SharedPtr context = contexts::get_global_default_context());

  /// 带超时的等待时钟开始 (Wait for clock to start, with timeout)
  /**
   * 超时将以稳定时间进行等待。
   * (The timeout is waited in steady time.)
   *
   * \rclcpp::Clock::started
   * \param timeout 等待的最长时间。(the maximum time to wait for.)
   * \param context 要等待的上下文。(the context to wait in.)
   * \param wait_tick_ns 等待循环的每次迭代之间的等待时间（以纳秒为单位）。
   *                     (the time to wait between each iteration of the wait loop (in
   * nanoseconds).) \return 如果时钟有效或变为有效，则返回 true。(true if clock was or became valid)
   * \throws std::runtime_error 如果上下文无效或时钟不是 rcl_clock_valid。(if the context is invalid
   * or clock is not rcl_clock_valid)
   */
  RCLCPP_PUBLIC
  bool wait_until_started(
      const rclcpp::Duration &timeout,
      Context::SharedPtr context = contexts::get_global_default_context(),
      const rclcpp::Duration &wait_tick_ns = rclcpp::Duration(0, static_cast<uint32_t>(1e7)));

  /// 返回 `RCL_ROS_TIME` 类型的时钟是否处于活动状态 (Returns if the clock of the type
  /// `RCL_ROS_TIME` is active)
  /**
   * \return 如果时钟处于活动状态，则返回 true。(true if the clock is active)
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw if
   * the current clock does not have the clock_type `RCL_ROS_TIME`.
   */
  RCLCPP_PUBLIC
  bool ros_time_is_active();

  /// 返回 rcl_clock_t 时钟句柄 (Return the rcl_clock_t clock handle)
  RCLCPP_PUBLIC
  rcl_clock_t *get_clock_handle() noexcept;

  /// 获取时钟类型 (Get clock type)
  RCLCPP_PUBLIC
  rcl_clock_type_t get_clock_type() const noexcept;

  /// 获取时钟的互斥锁 (Get the clock's mutex)
  RCLCPP_PUBLIC
  std::mutex &get_clock_mutex() noexcept;

  // 添加一个回调，如果超过跳跃阈值则调用它。 (Add a callback to invoke if the jump threshold is
  // exceeded)
  /**
   * 只要返回的共享指针有效，这些回调函数就必须保持有效。
   * (These callback functions must remain valid as long as the
   * returned shared pointer is valid.)
   *
   * 函数将向回调队列注册回调。在时间跳变时，所有阈值大于时间跳变的回调都将被执行；
   * 逻辑将首先调用选定的 pre_callbacks，然后调用所有选定的 post_callbacks。
   * (Function will register callbacks to the callback queue. On time jump all
   * callbacks will be executed whose threshold is greater than the time jump;
   * The logic will first call selected pre_callbacks and then all selected
   * post_callbacks.)
   *
   * 仅当 clock_type 为 `RCL_ROS_TIME` 时，函数才适用。
   * (Function is only applicable if the clock_type is `RCL_ROS_TIME`)
   *
   * \param pre_callback 必须是非抛出的。(Must be non-throwing)
   * \param post_callback 必须是非抛出的。(Must be non-throwing.)
   * \param threshold 如果时间跳变大于阈值，则触发回调。(Callbacks will be triggered if the time
   * jump is greater than the threshold.)
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   * \throws std::bad_alloc if the allocation of the JumpHandler fails.
   * \warning 只要创建的任何 JumpHandler 有效，时钟实例就必须保持有效。 (the instance of the clock
   * must remain valid as long as any created JumpHandler.)
   */
  RCLCPP_PUBLIC
  JumpHandler::SharedPtr create_jump_callback(
      JumpHandler::pre_callback_t pre_callback,
      JumpHandler::post_callback_t post_callback,
      const rcl_jump_threshold_t &threshold);

private:
  // 调用时间跳转回调 (Invoke time jump callback)
  RCLCPP_PUBLIC
  static void on_time_jump(const rcl_time_jump_t *time_jump, bool before_jump, void *user_data);

  /// 私有内部存储 (Private internal storage)
  class Impl;

  std::shared_ptr<Impl> impl_;
};

}  // namespace rclcpp

#endif  // RCLCPP__CLOCK_HPP_

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

#include "rclcpp/clock.hpp"

#include <condition_variable>
#include <memory>
#include <thread>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/utilities.hpp"
#include "rcpputils/scope_exit.hpp"
#include "rcutils/logging_macros.h"

namespace rclcpp {

/**
 * @class Clock::Impl
 * @brief 内部实现类，用于封装 rcl_clock_t 类型的对象，并提供对其操作的方法。 (Internal
 * implementation class for encapsulating rcl_clock_t objects and providing methods for their
 * operations.)
 */
class Clock::Impl {
public:
  /**
   * @brief 构造函数，根据传入的 clock_type 初始化 rcl_clock_ 对象。 (Constructor, initializes the
   * rcl_clock_ object based on the input clock_type.)
   * @param clock_type 时钟类型 (rcl_clock_type_t) (Clock type (rcl_clock_type_t))
   */
  explicit Impl(rcl_clock_type_t clock_type) : allocator_{rcl_get_default_allocator()} {
    // 使用 rcl_clock_init 函数初始化 rcl_clock_ 对象，并检查返回值。 (Initialize the rcl_clock_
    // object using the rcl_clock_init function and check the return value.)
    rcl_ret_t ret = rcl_clock_init(clock_type, &rcl_clock_, &allocator_);
    if (ret != RCL_RET_OK) {
      // 如果返回值不是 RCL_RET_OK，则抛出异常。 (If the return value is not RCL_RET_OK, throw an
      // exception.)
      exceptions::throw_from_rcl_error(ret, "failed to initialize rcl clock");
    }
  }

  /**
   * @brief 析构函数，释放 rcl_clock_ 对象。 (Destructor, releases the rcl_clock_ object.)
   */
  ~Impl() {
    // 使用 rcl_clock_fini 函数释放 rcl_clock_ 对象，并检查返回值。 (Release the rcl_clock_ object
    // using the rcl_clock_fini function and check the return value.)
    rcl_ret_t ret = rcl_clock_fini(&rcl_clock_);
    if (ret != RCL_RET_OK) {
      // 如果返回值不是 RCL_RET_OK，则记录错误日志。 (If the return value is not RCL_RET_OK, log an
      // error.)
      RCUTILS_LOG_ERROR("Failed to fini rcl clock.");
    }
  }

  rcl_clock_t rcl_clock_;  ///< rcl_clock_t 类型的对象，用于存储时钟信息。 (Object of type
                           ///< rcl_clock_t for storing clock information.)
  rcl_allocator_t allocator_;  ///< 分配器对象，用于管理内存分配。 (Allocator object for managing
                               ///< memory allocation.)
  std::mutex clock_mutex_;  ///< 互斥锁，用于保护对 rcl_clock_ 对象的操作。 (Mutex for protecting
                            ///< operations on the rcl_clock_ object.)
};

/**
 * @brief 构造函数，初始化 JumpHandler 对象。
 * @param pre_callback 预跳转回调函数。
 * @param post_callback 跳转后回调函数。
 * @param threshold 跳转阈值。
 *
 * @brief Constructor, initializes the JumpHandler object.
 * @param pre_callback Pre-jump callback function.
 * @param post_callback Post-jump callback function.
 * @param threshold Jump threshold.
 */
JumpHandler::JumpHandler(
    pre_callback_t pre_callback,
    post_callback_t post_callback,
    const rcl_jump_threshold_t &threshold)
    : pre_callback(pre_callback), post_callback(post_callback), notice_threshold(threshold) {}

/**
 * @brief Clock 类的构造函数，用于创建一个新的时钟对象。
 * @param clock_type 时钟类型。
 *
 * @brief Constructor for the Clock class, used to create a new clock object.
 * @param clock_type The type of the clock.
 */
Clock::Clock(rcl_clock_type_t clock_type) : impl_(new Clock::Impl(clock_type)) {}

/**
 * @brief Clock 类的析构函数。
 *
 * @brief Destructor for the Clock class.
 */
Clock::~Clock() {}

/**
 * @brief 获取当前时间（Time 类型）。
 * @return 当前时间。
 *
 * @brief Get the current time (of type Time).
 * @return The current time.
 */
Time Clock::now() const {
  Time now(0, 0, impl_->rcl_clock_.type);

  auto ret = rcl_clock_get_now(&impl_->rcl_clock_, &now.rcl_time_.nanoseconds);
  if (ret != RCL_RET_OK) {
    exceptions::throw_from_rcl_error(ret, "could not get current time stamp");
  }

  return now;
}

/**
 * @brief 等待直到指定的时间。
 * @param until 指定的等待截止时间。
 * @param context 共享智能指针，指向当前的上下文对象。
 * @return 是否成功等待到指定时间。如果返回 false，表示上下文无效或时间源已更改。
 *
 * @brief Sleeps until the specified time.
 * @param until The specified deadline for waiting.
 * @param context Shared smart pointer pointing to the current context object.
 * @return Whether it successfully waited until the specified time. If false, the context is invalid
 * or the time source has changed.
 */
bool Clock::sleep_until(Time until, Context::SharedPtr context) {
  // 检查上下文是否有效，若无效则抛出异常
  // Check if the context is valid, throw an exception if it's not
  if (!context || !context->is_valid()) {
    throw std::runtime_error("context cannot be slept with because it's invalid");
  }

  // 检查指定的时间类型是否与当前时钟类型匹配，若不匹配则抛出异常
  // Check if the specified time type matches the current clock type, throw an exception if it
  // doesn't
  const auto this_clock_type = get_clock_type();
  if (until.get_clock_type() != this_clock_type) {
    throw std::runtime_error("until's clock type does not match this clock's type");
  }

  // 标记时间源是否发生变化
  // Flag to mark if the time source has changed
  bool time_source_changed = false;

  // 条件变量，用于唤醒线程
  // Condition variable to wake up the thread
  std::condition_variable cv;

  // 当上下文关闭时唤醒此线程
  // Wake this thread if the context is shutdown
  rclcpp::OnShutdownCallbackHandle shutdown_cb_handle =
      context->add_on_shutdown_callback([&cv]() { cv.notify_one(); });

  // 当此函数退出时，不再需要关闭回调
  // No longer need the shutdown callback when this function exits
  auto callback_remover = rcpputils::scope_exit([context, &shutdown_cb_handle]() {
    context->remove_on_shutdown_callback(shutdown_cb_handle);
  });

  // 如果当前时钟类型为 RCL_STEADY_TIME，则进行以下操作
  // If the current clock type is RCL_STEADY_TIME, perform the following operations
  if (this_clock_type == RCL_STEADY_TIME) {
    // 同步 RCL 稳定时钟和 chrono::steady_clock 的时间原点，因为它们可能不同
    // Synchronize the epochs of RCL steady clock and chrono::steady_clock, as they might be
    // different
    const Time rcl_entry = now();
    const std::chrono::steady_clock::time_point chrono_entry = std::chrono::steady_clock::now();
    const Duration delta_t = until - rcl_entry;
    const std::chrono::steady_clock::time_point chrono_until =
        chrono_entry + std::chrono::nanoseconds(delta_t.nanoseconds());

    // 在虚假唤醒的情况下循环，但注意上下文的关闭
    // Loop over spurious wakeups but notice the shutdown of the context
    std::unique_lock lock(impl_->clock_mutex_);
    while (now() < until && context->is_valid()) {
      cv.wait_until(lock, chrono_until);
    }
  } else if (this_clock_type == RCL_SYSTEM_TIME) {
    // 将指定的时间转换为 std::chrono::system_clock 时间点
    // Convert the specified time to a std::chrono::system_clock time point
    auto system_time = std::chrono::system_clock::time_point(
        // Cast because system clock resolution is too big for nanoseconds on some systems
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::nanoseconds(until.nanoseconds())));

    // 在虚假唤醒的情况下循环，但注意上下文的关闭
    // Loop over spurious wakeups but notice the shutdown of the context
    std::unique_lock lock(impl_->clock_mutex_);
    while (now() < until && context->is_valid()) {
      cv.wait_until(lock, system_time);
    }
  } else if (this_clock_type == RCL_ROS_TIME) {
    // 为任意数量的时间变化安装跳转处理程序，用于以下两个目的：
    // - 如果 ROS 时间处于活动状态，则在每个新时钟样本上检查是否达到了时间
    // - 通过 on_clock_change 检测时间源是否发生变化，以使睡眠失效
    // Install jump handler for any amount of time change, for two purposes:
    // - if ROS time is active, check if time reached on each new clock sample
    // - Trigger via on_clock_change to detect if time source changes, to invalidate sleep
    rcl_jump_threshold_t threshold;
    threshold.on_clock_change = true;
    // 0 is disable, so -1 and 1 are smallest possible time changes
    threshold.min_backward.nanoseconds = -1;
    threshold.min_forward.nanoseconds = 1;
    auto clock_handler = create_jump_callback(
        nullptr,
        [&cv, &time_source_changed](const rcl_time_jump_t &jump) {
          if (jump.clock_change != RCL_ROS_TIME_NO_CHANGE) {
            time_source_changed = true;
          }
          cv.notify_one();
        },
        threshold);

    if (!ros_time_is_active()) {
      auto system_time = std::chrono::system_clock::time_point(
          // Cast because system clock resolution is too big for nanoseconds on some systems
          std::chrono::duration_cast<std::chrono::system_clock::duration>(
              std::chrono::nanoseconds(until.nanoseconds())));

      // 在虚假唤醒的情况下循环，但注意上下文的关闭或时间源的变化
      // Loop over spurious wakeups but notice the shutdown of the context or the change of the time
      // source
      std::unique_lock lock(impl_->clock_mutex_);
      while (now() < until && context->is_valid() && !time_source_changed) {
        cv.wait_until(lock, system_time);
      }
    } else {
      // RCL_ROS_TIME 处于活动状态。
      // 只需等待而无需 "直到"，因为已安装的跳转回调在每个新样本上唤醒 cv。
      // RCL_ROS_TIME with ros_time_is_active.
      // Just wait without "until" because installed
      // jump callbacks wake the cv on every new sample.
      std::unique_lock lock(impl_->clock_mutex_);
      while (now() < until && context->is_valid() && !time_source_changed) {
        cv.wait(lock);
      }
    }
  }

  // 如果上下文无效或时间源已更改，则返回 false
  // Return false if the context is invalid or the time source has changed
  if (!context->is_valid() || time_source_changed) {
    return false;
  }

  // 返回当前时间是否大于等于指定的时间
  // Return whether the current time is greater than or equal to the specified time
  return now() >= until;
}

/**
 * @brief 休眠一段时间 (Sleep for a duration)
 *
 * @param rel_time 相对时间 (Relative time)
 * @param context 上下文 (Context)
 * @return bool 是否成功休眠 (Whether the sleep was successful)
 */
bool Clock::sleep_for(Duration rel_time, Context::SharedPtr context) {
  // 计算绝对时间并调用 sleep_until 函数 (Calculate absolute time and call sleep_until function)
  return sleep_until(now() + rel_time, context);
}

/**
 * @brief 判断时钟是否已启动 (Check if the clock has started)
 *
 * @return bool 时钟是否已启动 (Whether the clock has started)
 */
bool Clock::started() {
  // 检查时钟句柄是否有效 (Check if the clock handle is valid)
  if (!rcl_clock_valid(get_clock_handle())) {
    throw std::runtime_error("clock is not rcl_clock_valid");
  }
  // 获取时钟启动状态 (Get clock started status)
  return rcl_clock_time_started(get_clock_handle());
}

/**
 * @brief 等待时钟启动 (Wait until the clock starts)
 *
 * @param context 上下文 (Context)
 * @return bool 时钟是否已启动 (Whether the clock has started)
 */
bool Clock::wait_until_started(Context::SharedPtr context) {
  // 检查上下文是否有效 (Check if the context is valid)
  if (!context || !context->is_valid()) {
    throw std::runtime_error("context cannot be slept with because it's invalid");
  }
  // 检查时钟句柄是否有效 (Check if the clock handle is valid)
  if (!rcl_clock_valid(get_clock_handle())) {
    throw std::runtime_error("clock cannot be waited on as it is not rcl_clock_valid");
  }

  // 如果时钟已启动，返回 true (If the clock has started, return true)
  if (started()) {
    return true;
  } else {
    // 等待第一个非零时间 (Wait until the first non-zero time)
    return sleep_until(rclcpp::Time(0, 1, get_clock_type()), context);
  }
}

/**
 * @brief 带超时的等待时钟启动 (Wait until the clock starts with a timeout)
 *
 * @param timeout 超时时间 (Timeout duration)
 * @param context 上下文 (Context)
 * @param wait_tick_ns 等待间隔 (Wait interval)
 * @return bool 时钟是否已启动 (Whether the clock has started)
 */
bool Clock::wait_until_started(
    const Duration &timeout, Context::SharedPtr context, const Duration &wait_tick_ns) {
  // 检查上下文是否有效 (Check if the context is valid)
  if (!context || !context->is_valid()) {
    throw std::runtime_error("context cannot be slept with because it's invalid");
  }
  // 检查时钟句柄是否有效 (Check if the clock handle is valid)
  if (!rcl_clock_valid(get_clock_handle())) {
    throw std::runtime_error("clock cannot be waited on as it is not rcl_clock_valid");
  }

  // 创建超时时钟 (Create a timeout clock)
  Clock timeout_clock = Clock(RCL_STEADY_TIME);
  Time start = timeout_clock.now();

  // 每隔 wait_tick_ns 纳秒检查一次时钟是否启动，同时检查上下文以检测 rclcpp::shutdown() (Check if
  // the clock has started every wait_tick_ns nanoseconds, also check the context for
  // rclcpp::shutdown())
  while (!started() && context->is_valid()) {
    // 根据剩余时间和等待间隔决定休眠时长 (Determine sleep duration based on remaining time and wait
    // interval)
    if (timeout < wait_tick_ns) {
      timeout_clock.sleep_for(timeout);
    } else {
      Duration time_left = start + timeout - timeout_clock.now();
      if (time_left > wait_tick_ns) {
        timeout_clock.sleep_for(Duration(wait_tick_ns));
      } else {
        timeout_clock.sleep_for(time_left);
      }
    }

    // 检查是否超时 (Check if timed out)
    if (timeout_clock.now() - start > timeout) {
      return started();
    }
  }
  return started();
}

/**
 * @brief 判断 ROS 时间是否处于活动状态 (Check if ROS time is active)
 *
 * @return bool ROS 时间是否处于活动状态 (Whether ROS time is active)
 */
bool Clock::ros_time_is_active() {
  // 检查时钟实现是否有效 (Check if the clock implementation is valid)
  if (!rcl_clock_valid(&impl_->rcl_clock_)) {
    RCUTILS_LOG_ERROR("ROS time not valid!");
    return false;
  }

  // 获取 ROS 时间激活状态 (Get ROS time activation status)
  bool is_enabled = false;
  auto ret = rcl_is_enabled_ros_time_override(&impl_->rcl_clock_, &is_enabled);
  if (ret != RCL_RET_OK) {
    exceptions::throw_from_rcl_error(ret, "Failed to check ros_time_override_status");
  }
  return is_enabled;
}

// 获取时钟句柄 (Get clock handle)
rcl_clock_t *Clock::get_clock_handle() noexcept { return &impl_->rcl_clock_; }

// 获取时钟类型 (Get clock type)
rcl_clock_type_t Clock::get_clock_type() const noexcept { return impl_->rcl_clock_.type; }

// 获取时钟互斥锁 (Get clock mutex)
std::mutex &Clock::get_clock_mutex() noexcept { return impl_->clock_mutex_; }

/**
 * @brief 时间跳变回调函数 (Time jump callback function)
 *
 * @param time_jump 时间跳变信息 (Time jump information)
 * @param before_jump 是否在跳变前触发 (Whether triggered before the jump)
 * @param user_data 用户数据，用于存储 JumpHandler 对象 (User data for storing JumpHandler object)
 */
void Clock::on_time_jump(const rcl_time_jump_t *time_jump, bool before_jump, void *user_data) {
  // 获取 JumpHandler 对象 (Get JumpHandler object)
  const auto *handler = static_cast<JumpHandler *>(user_data);
  if (nullptr == handler) {
    return;
  }
  // 根据时间跳变前后执行相应的回调函数 (Execute the corresponding callback function according to
  // the time jump)
  if (before_jump && handler->pre_callback) {
    handler->pre_callback();
  } else if (!before_jump && handler->post_callback) {
    handler->post_callback(*time_jump);
  }
}

/**
 * @brief 创建跳跃处理器回调(Create a jump handler callback)
 *
 * @param[in] pre_callback 跳跃前的回调函数(Pre-jump callback function)
 * @param[in] post_callback 跳跃后的回调函数(Post-jump callback function)
 * @param[in] threshold 跳跃阈值(Jump threshold)
 * @return JumpHandler::SharedPtr 返回创建的跳跃处理器智能指针(Return the created jump handler smart
 * pointer)
 */
JumpHandler::SharedPtr Clock::create_jump_callback(
    JumpHandler::pre_callback_t pre_callback,
    JumpHandler::post_callback_t post_callback,
    const rcl_jump_threshold_t &threshold) {
  // 分配一个新的跳跃处理器(Allocate a new jump handler)
  JumpHandler::UniquePtr handler(new JumpHandler(pre_callback, post_callback, threshold));
  if (nullptr == handler) {
    throw std::bad_alloc{};
  }

  {
    std::lock_guard<std::mutex> clock_guard(impl_->clock_mutex_);
    // 尝试将跳跃回调添加到时钟(Try to add the jump callback to the clock)
    rcl_ret_t ret = rcl_clock_add_jump_callback(
        &impl_->rcl_clock_, threshold, Clock::on_time_jump, handler.get());
    if (RCL_RET_OK != ret) {
      exceptions::throw_from_rcl_error(ret, "Failed to add time jump callback");
    }
  }

  std::weak_ptr<Clock::Impl> weak_impl = impl_;
  // *INDENT-OFF*
  // 创建 shared_ptr，在所有副本销毁时自动移除回调(Create a shared_ptr that automatically removes
  // the callback when all copies are destructed)
  return JumpHandler::SharedPtr(handler.release(), [weak_impl](JumpHandler *handler) noexcept {
    auto shared_impl = weak_impl.lock();
    if (shared_impl) {
      std::lock_guard<std::mutex> clock_guard(shared_impl->clock_mutex_);
      rcl_ret_t ret =
          rcl_clock_remove_jump_callback(&shared_impl->rcl_clock_, Clock::on_time_jump, handler);
      if (RCL_RET_OK != ret) {
        RCUTILS_LOG_ERROR("Failed to remove time jump callback");
      }
    }
    delete handler;
  });
  // *INDENT-ON*
}

}  // namespace rclcpp

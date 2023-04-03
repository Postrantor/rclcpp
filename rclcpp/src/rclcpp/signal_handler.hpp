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

#ifndef RCLCPP__SIGNAL_HANDLER_HPP_
#define RCLCPP__SIGNAL_HANDLER_HPP_

#include <atomic>
#include <csignal>
#include <mutex>
#include <thread>

#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"

// includes for semaphore notification code
#if defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <dispatch/dispatch.h>
#else // posix
#include <semaphore.h>
#endif

// Determine if sigaction is available
#if __APPLE__ || _POSIX_C_SOURCE >= 1 || _XOPEN_SOURCE || _POSIX_SOURCE
#define RCLCPP_HAS_SIGACTION
#endif

namespace rclcpp
{

/// 负责管理SIGINT/SIGTERM信号处理。 (Responsible for managing the SIGINT/SIGTERM signal handling.)
/**
 * 这个类负责： (This class is responsible for:)
 *
 * - 安装SIGINT/SIGTERM信号处理器 (installing the signal handler for SIGINT/SIGTERM)
 * - 卸载SIGINT/SIGTERM信号处理器 (uninstalling the signal handler for SIGINT/SIGTERM)
 * - 创建一个线程来在信号处理器之外执行“on signal”工作 (creating a thread to execute "on signal" work outside of the signal handler)
 * - 在接收到SIGINT/SIGTERM时安全地通知专用的信号处理线程 (safely notifying the dedicated signal handling thread when receiving SIGINT/SIGTERM)
 * - 实现所有信号处理工作，如关闭上下文 (implementation of all of the signal handling work, like shutting down contexts)
 *
 * \internal
 */
class SignalHandler final
{
public:
  /// 返回此类的全局单例。 (Return the global singleton of this class.)
  static SignalHandler & get_global_signal_handler();

  /// 返回一个全局单例记录器，以避免需要在每个地方创建它。 (Return a global singleton logger to avoid needing to create it everywhere.)
  static rclcpp::Logger & get_logger();

  /// 安装SIGINT/SIGTERM信号处理器并启动专用的信号处理线程。 (Install the signal handler for SIGINT/SIGTERM and start the dedicated signal handling thread.)
  /**
   * 同时存储当前的信号处理器，以便在信号上调用，并在卸载此信号处理器时恢复。 (Also stores the current signal handler to be called on signal and to restore when uninstalling this signal handler.)
   *
   * \param signal_handler_options 选项，用于指示应安装哪些信号处理器。 (option to indicate which signal handlers should be installed.)
   */
  bool install(SignalHandlerOptions signal_handler_options = SignalHandlerOptions::All);

  /// 卸载SIGINT/SIGTERM信号处理器并加入专用的信号处理线程。 (Uninstall the signal handler for SIGINT/SIGTERM and join the dedicated singal handling thread.)
  /**
   * 同时恢复之前的信号处理器。 (Also restores the previous signal handler.)
   */
  bool uninstall();

  /// 如果已安装，则返回true；否则返回false。 (Return true if installed, false otherwise.)
  bool is_installed();

  /// 获取当前的信号处理器选项。 (Get the current signal handler options.)
  /**
   * 如果没有安装信号处理器，则返回SignalHandlerOptions::None。 (If no signal handler is installed, SignalHandlerOptions::None is returned.)
   */
  rclcpp::SignalHandlerOptions get_current_signal_handler_options();

private:
  /// 信号处理器类型，取决于平台。 (Signal handler type, platform dependent.)
#if defined(RCLCPP_HAS_SIGACTION)
  using signal_handler_type = struct sigaction;
#else
  using signal_handler_type = void (*)(int);
#endif

  SignalHandler() = default;

  ~SignalHandler();

  SignalHandler(const SignalHandler &) = delete;
  SignalHandler(SignalHandler &&) = delete;
  SignalHandler & operator=(const SignalHandler &) = delete;
  SignalHandler && operator=(SignalHandler &&) = delete;

  /// sigaction和非sigaction版本之间的通用信号处理器代码。 (Common signal handler code between sigaction and non-sigaction versions.)
  void signal_handler_common();

#if defined(RCLCPP_HAS_SIGACTION)
  /// 信号处理器函数。 (Signal handler function.)
  static void signal_handler(int signal_value, siginfo_t * siginfo, void * context);
#else
  /// 信号处理器函数。 (Signal handler function.)
  static void signal_handler(int signal_value);
#endif

  /// 专用信号处理线程的目标。 (Target of the dedicated signal handling thread.)
  void deferred_signal_handler();

  /// 设置等待信号()或notify_signal_handler()所需的任何内容。 (Setup anything that is necessary for wait_for_signal() or notify_signal_handler().)
  /**
   * 在调用wait_for_signal()或notify_signal_handler()之前必须调用此方法。 (This must be called before wait_for_signal() or notify_signal_handler().)
   * 这不是线程安全的。 (This is not thread-safe.)
   */
  void setup_wait_for_signal();

  /// 撤销setup_wait_for_signal()中完成的所有设置。 (Undo all setup done in setup_wait_for_signal().)
  /**
   * 调用此方法后，不得调用wait_for_signal()或notify_signal_handler()。 (Must not call wait_for_signal() or notify_signal_handler() after calling this.)
   *
   * 这不是线程安全的。 (This is not thread-safe.)
   */
  void teardown_wait_for_signal() noexcept;

  /// 以信号安全的方式等待来自notify_signal_handler()的通知。 (Wait for a notification from notify_signal_handler() in a signal safe way.)
  /**
   * 如果发布信号量失败，此静态方法可能会抛出异常。 (This static method may throw if posting the semaphore fails.)
   *
   * 这不是线程安全的。 (This is not thread-safe.)
   */
  void wait_for_signal();

  /// 以信号安全的方式通知阻塞wait_for_signal()调用。 (Notify blocking wait_for_signal() calls in a signal safe way.)
  /**
   * 这用于从信号处理器通知deferred_signal_handler()线程开始工作。 (This is used to notify the deferred_signal_handler() thread to start work from the signal handler.)
   *
   * 这是线程安全的。 (This is thread-safe.)
   */
  void notify_signal_handler() noexcept;

  static signal_handler_type
  set_signal_handler(int signal_value, const signal_handler_type & signal_handler);

  signal_handler_type get_old_signal_handler(int signum);

  rclcpp::SignalHandlerOptions signal_handlers_options_ = rclcpp::SignalHandlerOptions::None;

  signal_handler_type old_sigint_handler_;
  signal_handler_type old_sigterm_handler_;

  // 记录器实例 (logger instance)
  rclcpp::Logger logger_ = rclcpp::get_logger("rclcpp");

  // 是否接收到信号。 (Whether or not a signal has been received.)
  std::atomic_bool signal_received_ = false;
  // 执行信号处理任务的线程。 (A thread to which singal handling tasks are deferred.)
  std::thread signal_handler_thread_;

  // 用于同步install()和uninstall()方法的互斥锁。 (A mutex used to synchronize the install() and uninstall() methods.)
  std::mutex install_mutex_;
  // 信号处理器是否已安装。 (Whether or not the signal handler has been installed.)
  std::atomic_bool installed_ = false;

  // 等待信号的信号量是否已设置。 (Whether or not the semaphore for wait_for_signal is setup.)
  std::atomic_bool wait_for_signal_is_setup_;
  // 存储wait_for_signal信号量的存储空间。 (Storage for the wait_for_signal semaphore.)
#if defined(_WIN32)
  HANDLE signal_handler_sem_;
#elif defined(__APPLE__)
  dispatch_semaphore_t signal_handler_sem_;
#else // posix
  sem_t signal_handler_sem_;
#endif
};

} // namespace rclcpp

#endif // RCLCPP__SIGNAL_HANDLER_HPP_

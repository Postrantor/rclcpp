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

#include "signal_handler.hpp"

#include <atomic>
#include <csignal>
#include <mutex>
#include <string>
#include <thread>

// includes for semaphore notification code
#if defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <dispatch/dispatch.h>
#else  // posix
#include <semaphore.h>
#endif

#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"
#include "rcutils/strerror.h"
#include "rmw/impl/cpp/demangle.hpp"

using rclcpp::SignalHandler;
using rclcpp::SignalHandlerOptions;

/**
 * @brief 设置信号处理器 (Set the signal handler)
 *
 * @param signal_value 信号值 (Signal value)
 * @param signal_handler 要设置的信号处理器函数 (The signal handler function to be set)
 * @return SignalHandler::signal_handler_type 旧的信号处理器 (Old signal handler)
 */
SignalHandler::signal_handler_type SignalHandler::set_signal_handler(
    int signal_value, const SignalHandler::signal_handler_type &signal_handler) {
  // 定义信号处理器安装是否失败的标志 (Define the flag for whether the signal handler installation
  // failed)
  bool signal_handler_install_failed;

  // 声明旧的信号处理器 (Declare the old signal handler)
  SignalHandler::signal_handler_type old_signal_handler;

  // 如果定义了 RCLCPP_HAS_SIGACTION (If RCLCPP_HAS_SIGACTION is defined)
#if defined(RCLCPP_HAS_SIGACTION)
  // 使用 sigaction 函数设置信号处理器，并获取旧的信号处理器 (Use the sigaction function to set the
  // signal handler and get the old signal handler)
  ssize_t ret = sigaction(signal_value, &signal_handler, &old_signal_handler);

  // 判断信号处理器安装是否失败 (Check if the signal handler installation failed)
  signal_handler_install_failed = (ret == -1);
#else
  // 使用 std::signal 函数设置信号处理器，并获取旧的信号处理器 (Use the std::signal function to set
  // the signal handler and get the old signal handler)
  old_signal_handler = std::signal(signal_value, signal_handler);

  // 判断信号处理器安装是否失败 (Check if the signal handler installation failed)
  signal_handler_install_failed = (old_signal_handler == SIG_ERR);
#endif

  // 如果信号处理器安装失败 (If the signal handler installation failed)
  if (signal_handler_install_failed) {
    // 定义错误字符串缓冲区 (Define the error string buffer)
    char error_string[1024];

    // 使用 rcutils_strerror 函数获取错误信息 (Use the rcutils_strerror function to get the error
    // message)
    rcutils_strerror(error_string, sizeof(error_string));

    // 构造异常消息 (Construct the exception message)
    auto msg = "Failed to set signal handler (" + std::to_string(errno) + "): " + error_string;

    // 抛出运行时异常 (Throw a runtime exception)
    throw std::runtime_error(msg);
  }

  // 返回旧的信号处理器 (Return the old signal handler)
  return old_signal_handler;
}

// 遗憾的是，这里需要宏（或重复的代码），
// 因为信号处理程序必须是一个函数指针。
// Unfortunately, macros (or duplicated code) are needed here,
// as the signal handler must be a function pointer.

#if defined(RCLCPP_HAS_SIGACTION)

// 定义 signal_handler 函数，接收三个参数：信号编号、信号信息和上下文
// Define the signal_handler function with three parameters: signal number, signal information, and
// context
void SignalHandler::signal_handler(int signum, siginfo_t *siginfo, void *context) {
  // 输出日志信息，显示接收到的信号编号
  // Output log information, showing the received signal number
  RCLCPP_INFO(SignalHandler::get_logger(), "signal_handler(signum=%d)", signum);

  // 获取全局信号处理器实例
  // Get the global signal handler instance
  auto &instance = SignalHandler::get_global_signal_handler();

  // 获取旧的信号处理程序
  // Get the old signal handler
  auto old_signal_handler = instance.get_old_signal_handler(signum);

  // 检查旧的信号处理程序是否有 SA_SIGINFO 标志
  // Check if the old signal handler has the SA_SIGINFO flag
  if (old_signal_handler.sa_flags & SA_SIGINFO) {
    // 如果旧的信号处理程序不为空，则调用它
    // If the old signal handler is not NULL, call it
    if (old_signal_handler.sa_sigaction != NULL) {
      old_signal_handler.sa_sigaction(signum, siginfo, context);
    }
  } else {
    // 检查旧的信号处理程序是否已设置、非默认值且未被忽略
    // Check if the old signal handler is set, not default, and not ignored
    if (old_signal_handler.sa_handler != NULL &&    /* Is set */
        old_signal_handler.sa_handler != SIG_DFL && /* Is not default*/
        old_signal_handler.sa_handler != SIG_IGN)   /* Is not ignored */
    {
      // 调用旧的信号处理程序
      // Call the old signal handler
      old_signal_handler.sa_handler(signum);
    }
  }

  // 调用通用信号处理程序
  // Call the common signal handler
  instance.signal_handler_common();
}
#else

// 定义 signal_handler 函数，接收一个参数：信号编号
// Define the signal_handler function with one parameter: signal number
void SignalHandler::signal_handler(int signum) {
  // 输出日志信息，显示接收到的信号编号
  // Output log information, showing the received signal number
  RCLCPP_INFO(SignalHandler::get_logger(), "signal_handler(signum=%d)", signum);

  // 获取全局信号处理器实例
  // Get the global signal handler instance
  auto &instance = SignalHandler::get_global_signal_handler();

  // 获取旧的信号处理程序
  // Get the old signal handler
  auto old_signal_handler = instance.get_old_signal_handler(signum);

  // 检查旧的信号处理程序是否不等于 SIG_ERR、SIG_IGN 和 SIG_DFL
  // Check if the old signal handler is not equal to SIG_ERR, SIG_IGN, and SIG_DFL
  if (SIG_ERR != old_signal_handler && SIG_IGN != old_signal_handler &&
      SIG_DFL != old_signal_handler) {
    // 调用旧的信号处理程序
    // Call the old signal handler
    old_signal_handler(signum);
  }

  // 调用通用信号处理程序
  // Call the common signal handler
  instance.signal_handler_common();
}
#endif

/**
 * @brief 获取 SignalHandler 类的 logger 对象 (Get the logger object of the SignalHandler class)
 *
 * @return 返回 logger 对象的引用 (Return a reference to the logger object)
 */
rclcpp::Logger &SignalHandler::get_logger() {
  // 获取全局 SignalHandler 对象并返回其 logger_ 成员变量的引用
  // Get the global SignalHandler object and return a reference to its logger_ member variable
  return SignalHandler::get_global_signal_handler().logger_;
}

/**
 * @brief 获取全局 SignalHandler 对象 (Get the global SignalHandler object)
 *
 * @return 返回全局 SignalHandler 对象的引用 (Return a reference to the global SignalHandler object)
 */
SignalHandler &SignalHandler::get_global_signal_handler() {
  // 定义一个静态的 SignalHandler 对象，实现单例模式
  // Define a static SignalHandler object, implementing the singleton pattern
  static SignalHandler signal_handler;

  // 返回静态的 SignalHandler 对象的引用
  // Return a reference to the static SignalHandler object
  return signal_handler;
}

/**
 * @brief 安装信号处理器 (Install the signal handler)
 *
 * @param[in] signal_handler_options 信号处理选项（Signal handling options）
 * @return bool 成功安装返回 true，否则返回 false（true if successfully installed, otherwise false）
 */
bool SignalHandler::install(SignalHandlerOptions signal_handler_options) {
  // 使用互斥锁保护安装过程（Protect the installation process with a mutex lock）
  std::lock_guard<std::mutex> lock(install_mutex_);

  // 检查是否已经安装了信号处理器（Check if the signal handler is already installed）
  bool already_installed = installed_.exchange(true);
  if (already_installed) {
    return false;
  }

  // 如果没有设置信号处理选项，则直接返回成功（If no signal handling options are set, return success
  // directly）
  if (signal_handler_options == SignalHandlerOptions::None) {
    return true;
  }

  // 存储信号处理选项（Store the signal handling options）
  signal_handlers_options_ = signal_handler_options;

  try {
    // 设置等待信号（Set up waiting for signals）
    setup_wait_for_signal();

    // 初始化信号接收状态为 false（Initialize the signal received status to false）
    signal_received_.store(false);

    // 定义信号处理器参数类型（Define the signal handler argument type）
    SignalHandler::signal_handler_type handler_argument;

#if defined(RCLCPP_HAS_SIGACTION)
    // 清空 handler_argument 结构体（Clear the handler_argument struct）
    memset(&handler_argument, 0, sizeof(handler_argument));

    // 初始化信号集（Initialize the signal set）
    sigemptyset(&handler_argument.sa_mask);

    // 设置信号处理函数（Set the signal handling function）
    handler_argument.sa_sigaction = &this->signal_handler;

    // 设置信号处理标志（Set the signal handling flags）
    handler_argument.sa_flags = SA_SIGINFO;
#else
    // 设置信号处理函数（Set the signal handling function）
    handler_argument = &this->signal_handler;
#endif

    // 如果设置了 SigInt 或 All 选项，安装 SIGINT 信号处理器（If SigInt or All option is set,
    // install the SIGINT signal handler）
    if (signal_handler_options == SignalHandlerOptions::SigInt ||
        signal_handler_options == SignalHandlerOptions::All) {
      old_sigint_handler_ = set_signal_handler(SIGINT, handler_argument);
    }

    // 如果设置了 SigTerm 或 All 选项，安装 SIGTERM 信号处理器（If SigTerm or All option is set,
    // install the SIGTERM signal handler）
    if (signal_handler_options == SignalHandlerOptions::SigTerm ||
        signal_handler_options == SignalHandlerOptions::All) {
      old_sigterm_handler_ = set_signal_handler(SIGTERM, handler_argument);
    }

    // 创建信号处理线程（Create the signal handling thread）
    signal_handler_thread_ = std::thread(&SignalHandler::deferred_signal_handler, this);
  } catch (...) {
    // 如果出现异常，将 installed 状态设为 false 并重新抛出异常（If an exception occurs, set the
    // installed status to false and rethrow the exception）
    installed_.store(false);
    throw;
  }

  // 输出调试信息：信号处理器已安装（Output debug information: Signal handler installed）
  RCLCPP_DEBUG(get_logger(), "signal handler installed");

  // 返回成功（Return success）
  return true;
}

/**
 * @brief 卸载信号处理器（Uninstall the signal handler）
 *
 * @return 是否成功卸载（Whether the uninstallation is successful）
 */
bool SignalHandler::uninstall() {
  // 使用互斥锁保护安装状态（Use a mutex lock to protect the installation status）
  std::lock_guard<std::mutex> lock(install_mutex_);

  // 获取并设置安装状态为 false（Get and set the installed state to false）
  bool installed = installed_.exchange(false);

  // 如果未安装，则直接返回 false（If not installed, return false directly）
  if (!installed) {
    return false;
  }

  try {
    // TODO(wjwwood): what happens if someone overrides our signal handler then calls uninstall?
    //   I think we need to assert that we're the current signal handler, and mitigate if not.

    // 如果需要处理 SIGINT 信号或者所有信号（If handling SIGINT or all signals）
    if (SignalHandlerOptions::SigInt == signal_handlers_options_ ||
        SignalHandlerOptions::All == signal_handlers_options_) {
      // 恢复旧的 SIGINT 信号处理器（Restore the old SIGINT signal handler）
      set_signal_handler(SIGINT, old_sigint_handler_);
    }

    // 如果需要处理 SIGTERM 信号或者所有信号（If handling SIGTERM or all signals）
    if (SignalHandlerOptions::SigTerm == signal_handlers_options_ ||
        SignalHandlerOptions::All == signal_handlers_options_) {
      // 恢复旧的 SIGTERM 信号处理器（Restore the old SIGTERM signal handler）
      set_signal_handler(SIGTERM, old_sigterm_handler_);
    }

    // 设置信号处理选项为 None（Set signal handler options to None）
    signal_handlers_options_ = SignalHandlerOptions::None;

    // 输出调试信息（Output debug information）
    RCLCPP_DEBUG(get_logger(), "SignalHandler::uninstall(): notifying deferred signal handler");

    // 通知信号处理器（Notify the signal handler）
    notify_signal_handler();

    // 如果信号处理线程可 join，则进行 join 操作（If the signal handler thread is joinable, perform
    // the join operation）
    if (signal_handler_thread_.joinable()) {
      signal_handler_thread_.join();
    }

    // 销毁等待信号的相关资源（Destroy resources related to waiting for signals）
    teardown_wait_for_signal();
  } catch (...) {
    // 发生异常时，将安装状态设置回 true，并重新抛出异常（In case of an exception, set the installed
    // state back to true and rethrow the exception）
    installed_.exchange(true);
    throw;
  }

  // 输出调试信息（Output debug information）
  RCLCPP_DEBUG(get_logger(), "signal handler uninstalled");

  // 返回成功卸载（Return successful uninstallation）
  return true;
}

/**
 * @brief 检查信号处理器是否已安装（Check if the signal handler is installed）
 *
 * @return 信号处理器是否已安装（Whether the signal handler is installed）
 */
bool SignalHandler::is_installed() { return installed_.load(); }

/**
 * @brief 析构函数，卸载信号处理器 (Destructor, uninstall the signal handlers)
 */
SignalHandler::~SignalHandler() {
  try {
    // 尝试卸载信号处理器 (Try to uninstall the signal handlers)
    uninstall();
  } catch (const std::exception &exc) {
    // 如果捕获到异常，输出错误信息 (If an exception is caught, output the error message)
    RCLCPP_ERROR(
        get_logger(),
        "caught %s exception when uninstalling signal handlers in rclcpp::~SignalHandler: %s",
        rmw::impl::cpp::demangle(exc).c_str(), exc.what());
  } catch (...) {
    // 如果捕获到未知异常，输出错误信息 (If an unknown exception is caught, output the error
    // message)
    RCLCPP_ERROR(
        get_logger(),
        "caught unknown exception when uninstalling signal handlers in rclcpp::~SignalHandler");
  }
}

/**
 * @brief 获取旧的信号处理器 (Get the old signal handler)
 *
 * @param signum 信号值 (Signal value)
 * @return 返回对应信号的旧信号处理器 (Return the old signal handler for the corresponding signal)
 */
SignalHandler::signal_handler_type SignalHandler::get_old_signal_handler(int signum) {
  // 如果信号值为 SIGINT (If the signal value is SIGINT)
  if (SIGINT == signum) {
    return old_sigint_handler_;
  } else if (SIGTERM == signum) {  // 如果信号值为 SIGTERM (If the signal value is SIGTERM)
    return old_sigterm_handler_;
  }
#if defined(RCLCPP_HAS_SIGACTION)
  // 定义默认信号处理器并初始化 (Define the default signal handler and initialize it)
  SignalHandler::signal_handler_type ret;
  memset(&ret, 0, sizeof(ret));
  sigemptyset(&ret.sa_mask);
  ret.sa_handler = SIG_DFL;
  return ret;
#else
  // 返回默认信号处理器 (Return the default signal handler)
  return SIG_DFL;
#endif
}

/**
 * @brief 通用信号处理函数 (Common signal handler function)
 */
void SignalHandler::signal_handler_common() {
  // 获取全局信号处理器实例 (Get the global signal handler instance)
  auto &instance = SignalHandler::get_global_signal_handler();

  // 设置信号接收标志为 true (Set the signal received flag to true)
  instance.signal_received_.store(true);

  // 输出调试信息 (Output debug information)
  RCLCPP_DEBUG(get_logger(), "signal_handler(): notifying deferred signal handler");

  // 通知延迟信号处理器 (Notify the deferred signal handler)
  instance.notify_signal_handler();
}

/**
 * @brief 延迟信号处理函数 (Deferred signal handler function)
 */
void SignalHandler::deferred_signal_handler() {
  // 循环处理信号 (Loop to handle signals)
  while (true) {
    // 如果接收到信号，将信号接收标志设置为 false (If a signal is received, set the signal received
    // flag to false)
    if (signal_received_.exchange(false)) {
      // 输出调试信息 (Output debug information)
      RCLCPP_DEBUG(get_logger(), "deferred_signal_handler(): shutting down");

      // 遍历 rclcpp 上下文 (Iterate through rclcpp contexts)
      for (auto context_ptr : rclcpp::get_contexts()) {
        // 如果上下文的初始化选项中设置了 shutdown_on_signal (If the context's init options have
        // shutdown_on_signal set)
        if (context_ptr->get_init_options().shutdown_on_signal) {
          // 输出调试信息 (Output debug information)
          RCLCPP_DEBUG(
              get_logger(),
              "deferred_signal_handler(): "
              "shutting down rclcpp::Context @ %p, because it had shutdown_on_signal == true",
              static_cast<void *>(context_ptr.get()));

          // 关闭上下文 (Shutdown the context)
          context_ptr->shutdown("signal handler");
        }
      }
    }

    // 如果信号处理器未安装，则退出循环 (If the signal handler is not installed, break the loop)
    if (!is_installed()) {
      RCLCPP_DEBUG(get_logger(), "deferred_signal_handler(): signal handling uninstalled");
      break;
    }

    // 输出调试信息，等待 SIGINT/SIGTERM 信号或卸载信号处理器 (Output debug information, waiting for
    // SIGINT/SIGTERM signal or uninstalling the signal handler)
    RCLCPP_DEBUG(
        get_logger(), "deferred_signal_handler(): waiting for SIGINT/SIGTERM or uninstall");

    // 等待信号 (Wait for signal)
    wait_for_signal();

    // 输出调试信息，表示由于收到 SIGINT/SIGTERM 信号或卸载信号处理器而唤醒 (Output debug
    // information, indicating waking up due to receiving SIGINT/SIGTERM signal or uninstalling the
    // signal handler)
    RCLCPP_DEBUG(
        get_logger(), "deferred_signal_handler(): woken up due to SIGINT/SIGTERM or uninstall");
  }
}

/**
 * @brief 设置等待信号的处理方法 (Setup the waiting method for signal handling)
 */
void SignalHandler::setup_wait_for_signal() {
#if defined(_WIN32)
  // 在 Windows 平台下创建一个信号量 (Create a semaphore on Windows platform)
  signal_handler_sem_ = CreateSemaphore(
      NULL,   // 默认的安全属性 (default security attributes)
      0,      // 初始信号量计数 (initial semaphore count)
      1,      // 最大信号量计数 (maximum semaphore count)
      NULL);  // 未命名的信号量 (unnamed semaphore)
  if (NULL == signal_handler_sem_) {
    throw std::runtime_error("CreateSemaphore() failed in setup_wait_for_signal()");
  }
#elif defined(__APPLE__)
  // 在 Apple 平台下创建一个信号量 (Create a semaphore on Apple platform)
  signal_handler_sem_ = dispatch_semaphore_create(0);
#else  // posix
  // 在 POSIX 平台下初始化一个信号量 (Initialize a semaphore on POSIX platform)
  if (-1 == sem_init(&signal_handler_sem_, 0, 0)) {
    throw std::runtime_error(std::string("sem_init() failed: ") + strerror(errno));
  }
#endif
  // 设置等待信号已准备就绪 (Set that waiting for signal is ready)
  wait_for_signal_is_setup_.store(true);
}

/**
 * @brief 拆除等待信号的处理方法 (Teardown the waiting method for signal handling)
 */
void SignalHandler::teardown_wait_for_signal() noexcept {
  if (!wait_for_signal_is_setup_.exchange(false)) {
    return;
  }
#if defined(_WIN32)
  // 在 Windows 平台下关闭信号量句柄 (Close the semaphore handle on Windows platform)
  CloseHandle(signal_handler_sem_);
#elif defined(__APPLE__)
  // 在 Apple 平台下释放信号量 (Release the semaphore on Apple platform)
  dispatch_release(signal_handler_sem_);
#else  // posix
  // 在 POSIX 平台下销毁信号量 (Destroy the semaphore on POSIX platform)
  if (-1 == sem_destroy(&signal_handler_sem_)) {
    RCLCPP_ERROR(get_logger(), "invalid semaphore in teardown_wait_for_signal()");
  }
#endif
}

/**
 * @brief 等待信号的处理方法 (Waiting method for signal handling)
 */
void SignalHandler::wait_for_signal() {
  if (!wait_for_signal_is_setup_.load()) {
    RCLCPP_ERROR(get_logger(), "called wait_for_signal() before setup_wait_for_signal()");
    return;
  }
#if defined(_WIN32)
  DWORD dw_wait_result = WaitForSingleObject(signal_handler_sem_, INFINITE);
  switch (dw_wait_result) {
    case WAIT_ABANDONED:
      RCLCPP_ERROR(
          get_logger(), "WaitForSingleObject() failed in wait_for_signal() with WAIT_ABANDONED: %s",
          GetLastError());
      break;
    case WAIT_OBJECT_0:
      // 成功 (successful)
      break;
    case WAIT_TIMEOUT:
      RCLCPP_ERROR(get_logger(), "WaitForSingleObject() timedout out in wait_for_signal()");
      break;
    case WAIT_FAILED:
      RCLCPP_ERROR(
          get_logger(), "WaitForSingleObject() failed in wait_for_signal(): %s", GetLastError());
      break;
    default:
      RCLCPP_ERROR(
          get_logger(), "WaitForSingleObject() gave unknown return in wait_for_signal(): %s",
          GetLastError());
  }
#elif defined(__APPLE__)
  // 在 Apple 平台下等待信号量 (Wait for the semaphore on Apple platform)
  dispatch_semaphore_wait(signal_handler_sem_, DISPATCH_TIME_FOREVER);
#else  // posix
  int s;
  // 在 POSIX 平台下等待信号量，处理中断情况 (Wait for the semaphore on POSIX platform, handle
  // interruptions)
  do {
    s = sem_wait(&signal_handler_sem_);
  } while (-1 == s && EINTR == errno);
#endif
}

/**
 * @brief 通知信号处理程序 (Notify the signal handler)
 *
 * @param[in] 无参数 (No parameters)
 * @return 无返回值 (No return value)
 */
void SignalHandler::notify_signal_handler() noexcept {
  // 检查是否已设置等待信号 (Check if the wait for signal is set up)
  if (!wait_for_signal_is_setup_.load()) {
    return;
  }

#if defined(_WIN32)  // Windows 平台 (Windows platform)
  // 释放信号量 (Release the semaphore)
  if (!ReleaseSemaphore(signal_handler_sem_, 1, NULL)) {
    // 输出错误信息 (Output error message)
    RCLCPP_ERROR(
        get_logger(), "ReleaseSemaphore() failed in notify_signal_handler(): %s", GetLastError());
  }
#elif defined(__APPLE__)  // Apple 平台 (Apple platform)
  // 发送信号量 (Signal the semaphore)
  dispatch_semaphore_signal(signal_handler_sem_);
#else                     // posix (POSIX 平台 (POSIX platform))
  // 发送信号量 (Post the semaphore)
  if (-1 == sem_post(&signal_handler_sem_)) {
    // 输出错误信息 (Output error message)
    RCLCPP_ERROR(get_logger(), "sem_post failed in notify_signal_handler()");
  }
#endif
}

/**
 * @brief 获取当前信号处理程序选项 (Get the current signal handler options)
 *
 * @param[in] 无参数 (No parameters)
 * @return 当前信号处理程序选项 (Current signal handler options)
 */
rclcpp::SignalHandlerOptions SignalHandler::get_current_signal_handler_options() {
  // 返回当前信号处理程序选项 (Return the current signal handler options)
  return signal_handlers_options_;
}

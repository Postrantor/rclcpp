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

#ifndef RCLCPP__UTILITIES_HPP_
#define RCLCPP__UTILITIES_HPP_

#include <chrono>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/context.hpp"
#include "rclcpp/init_options.hpp"
#include "rclcpp/visibility_control.hpp"

#ifdef ANDROID  // 如果当前编译环境是 Android (If the current compilation environment is Android)
#include <sstream>  // 引入 sstream 头文件，用于字符串流操作 (Include the sstream header file for string stream operations)

namespace std       // 使用标准命名空间 std (Using the standard namespace std)
{
// 定义一个模板函数 to_string，用于将泛型 T 类型的值转换为 std::string 类型
// (Define a template function to_string to convert values of generic type T to std::string type)
template <typename T>
std::string to_string(T value) {
  std::ostringstream
      os;  // 创建一个 std::ostringstream 对象 os (Create an std::ostringstream object os)
  os << value;  // 将传入的 value 值写入 os 字符串流中 (Write the passed value into the os string
                // stream)
  return os.str();  // 返回 os 字符串流中的 std::string 类型数据 (Return the std::string type data
                    // in the os string stream)
}
}  // namespace std
#endif

namespace rclcpp {

/// 用于指示 rclcpp 应安装哪些信号处理程序的选项。
/// Option to indicate which signal handlers rclcpp should install.
enum class SignalHandlerOptions {
  /// 安装 sigint 和 sigterm，这是默认行为。
  /// Install both sigint and sigterm, this is the default behavior.
  All,
  /// 仅安装 sigint 处理程序。
  /// Install only a sigint handler.
  SigInt,
  /// 仅安装 sigterm 处理程序。
  /// Install only a sigterm handler.
  SigTerm,
  /// 不安装任何信号处理程序。
  /// Do not install any signal handler.
  None,
};

/// 通过 rmw 实现初始化通信并设置全局信号处理程序。
/// Initialize communications via the rmw implementation and set up a global signal handler.
/**
 * 初始化可通过函数 rclcpp::contexts::get_global_default_context() 访问的全局上下文。
 * Initializes the global context which is accessible via the function
 * rclcpp::contexts::get_global_default_context().
 * 另外，使用函数 rclcpp::install_signal_handlers() 安装全局信号处理程序。
 * Also, installs the global signal handlers with the function
 * rclcpp::install_signal_handlers().
 *
 * 有关参数和可能的异常的更多详细信息，请参阅 rclcpp::Context::init()
 * \sa rclcpp::Context::init() for more details on arguments and possible exceptions
 *
 * \param[in] argc 要解析的命令行参数数量。
 * \param[in] argc number of command-line arguments to parse.
 * \param[in] argv 要解析的命令行参数数组。
 * \param[in] argv array of command-line arguments to parse.
 * \param[in] init_options 要应用的初始化选项。
 * \param[in] init_options initialization options to apply.
 * \param[in] signal_handler_options 选项，用于指示应安装哪些信号处理程序。
 * \param[in] signal_handler_options option to indicate which signal handlers should be installed.
 */
RCLCPP_PUBLIC
void init(
    int argc,
    char const *const *argv,
    const InitOptions &init_options = InitOptions(),
    SignalHandlerOptions signal_handler_options = SignalHandlerOptions::All);

/// 为 rclcpp 安装全局信号处理程序。
/// Install the global signal handler for rclcpp.
/**
 * 此函数每个进程只需运行一次。
 * This function should only need to be run one time per process.
 * 它由 rclcpp::init() 隐式运行，因此如果已经运行了 rclcpp::init()，则不需要手动运行此函数。
 * It is implicitly run by rclcpp::init(), and therefore this function does not
 * need to be run manually if rclcpp::init() has already been run.
 *
 * 信号处理程序将关闭所有已初始化的上下文。
 * The signal handler will shutdown all initialized context.
 * 它还会中断 ROS 中的任何阻塞功能，使它们能够对系统状态（如关闭）的任何更改作出反应。
 * It will also interrupt any blocking functions in ROS allowing them react to
 * any changes in the state of the system (like shutdown).
 *
 * 此功能是线程安全的。
 * This function is thread-safe.
 *
 * \param[in] signal_handler_options 选项，用于指示应安装哪些信号处理程序。
 * \param[in] signal_handler_options option to indicate which signal handlers should be installed.
 * \return 如果此函数安装了信号处理程序，则返回 true；如果已经安装了，则返回 false。
 * \return true if signal handler was installed by this function, false if already installed.
 */
RCLCPP_PUBLIC
bool install_signal_handlers(
    SignalHandlerOptions signal_handler_options = SignalHandlerOptions::All);

/// 如果已安装信号处理程序，则返回 true，否则返回 false。
/// Return true if the signal handlers are installed, otherwise false.
RCLCPP_PUBLIC
bool signal_handlers_installed();

/// 获取当前信号处理程序选项。
/// Get the current signal handler options.
/**
 * 如果没有安装信号处理程序，则返回 SignalHandlerOptions::None。
 * If no signal handler is installed, SignalHandlerOptions::None is returned.
 */
RCLCPP_PUBLIC
SignalHandlerOptions get_current_signal_handler_options();

/// 卸载 rclcpp 的全局信号处理程序。
/// Uninstall the global signal handler for rclcpp.
/**
 * 此函数不一定需要调用，但可以用于撤消 rclcpp::install_signal_handlers() 或 rclcpp::init()
 * 在信号处理方面所做的操作。 This function does not necessarily need to be called, but can be used
 * to undo what rclcpp::install_signal_handlers() or rclcpp::init() do with respect to signal
 * handling. 如果选择使用它，此函数只需运行一次。 If you choose to use it, this function only needs
 * to be run one time. 它由 rclcpp::shutdown() 隐式运行，因此如果已经运行了
 * rclcpp::shutdown()，则不需要手动运行此函数。 It is implicitly run by rclcpp::shutdown(), and
 * therefore this function does not need to be run manually if rclcpp::shutdown() has already been
 * run.
 *
 * 此功能是线程安全的。
 * This function is thread-safe.
 *
 * \return 如果此函数卸载了信号处理程序，则返回 true；如果尚未安装，则返回 false。
 * \return true if signal handler was uninstalled by this function, false if was not installed.
 */
RCLCPP_PUBLIC
bool uninstall_signal_handlers();

/// 通过 rmw 实现初始化通信并设置全局信号处理程序。
/// Initialize communications via the rmw implementation and set up a global signal handler.
/**
 * 另外从参数向量中删除 ROS 特定的参数。
 * Additionally removes ROS-specific arguments from the argument vector.
 *
 * 有关参数和可能的异常的更多详细信息，请参阅 rclcpp::Context::init()
 * \sa rclcpp::Context::init() for more details on arguments and possible exceptions
 * \returns 参数向量中不是 ROS 参数的成员。
 * \returns Members of the argument vector that are not ROS arguments.
 * \throws remove_ros_arguments 可能抛出的任何内容
 * \throws anything remove_ros_arguments can throw
 */
RCLCPP_PUBLIC
std::vector<std::string> init_and_remove_ros_arguments(
    int argc, char const *const *argv, const InitOptions &init_options = InitOptions());

/// 从参数向量中删除 ROS 特定参数。
/// Remove ROS-specific arguments from argument vector.
/**
 * 某些参数可能不是作为 ROS 参数。
 * Some arguments may not have been intended as ROS arguments.
 * 该函数将参数填充到向量中。
 * This function populates the arguments in a vector.
 * 由于第一个参数始终被视为进程名称，因此向量将始终包含进程名称。
 * Since the first argument is always assumed to be a process name, the vector
 * will always contain the process name.
 *
 * \param[in] argc 参数数量。
 * \param[in] argc Number of arguments.
 * \param[in] argv 参数向量。
 * \param[in] argv Argument vector.
 * \returns 参数向量中不是 ROS 参数的成员。
 * \returns Members of the argument vector that are not ROS arguments.
 * \throws throw_from_rcl_error 可能抛出的任何内容
 * \throws anything throw_from_rcl_error can throw
 * \throws rclcpp::exceptions::RCLError 如果解析失败
 * \throws rclcpp::exceptions::RCLError if the parsing fails
 */
RCLCPP_PUBLIC
std::vector<std::string> remove_ros_arguments(int argc, char const *const *argv);

/// 检查 rclcpp 的状态。
/// Check rclcpp's status.
/**
 * 对于已关闭的上下文，或者由于接收到 rclcpp 信号处理程序的 SIGINT 而关闭的上下文，此方法可能返回
 * false。 This may return false for a context which has been shutdown, or for a context that was
 * shutdown due to SIGINT being received by the rclcpp signal handler.
 *
 * 如果给定上下文为 nullptr，则使用全局上下文，即 rclcpp::init() 初始化的上下文。
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * \param[in] context 可选项，检查此 Context 是否已关闭。
 * \param[in] context Optional check for shutdown of this Context.
 * \return 如果已调用关闭，则返回 false，否则返回 true
 * \return false if shutdown has been called, true otherwise
 */
RCLCPP_PUBLIC
bool ok(rclcpp::Context::SharedPtr context = nullptr);

/// 关闭 rclcpp 上下文，使其对派生实体无效。
/// Shutdown rclcpp context, invalidating it for derived entities.
/**
 * 如果给定上下文为 nullptr，则使用全局上下文，即 rclcpp::init() 初始化的上下文。
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * 如果使用全局上下文，则还将卸载信号处理程序。
 * If the global context is used, then the signal handlers are also uninstalled.
 *
 * 这还会导致调用 "on_shutdown" 回调。
 * This will also cause the "on_shutdown" callbacks to be called.
 *
 * 请参阅 rclcpp::Context::shutdown()
 * \sa rclcpp::Context::shutdown()
 * \param[in] context 要关闭的可选上下文
 * \param[in] context Optional to be shutdown
 * \param[in] reason 传递给上下文关闭方法的可选字符串
 * \param[in] reason Optional string passed to the context shutdown method
 * \return 如果关闭成功，则返回 true；如果上下文已经关闭，则返回 false
 * \return true if shutdown was successful, false if context was already shutdown
 */
RCLCPP_PUBLIC
bool shutdown(
    rclcpp::Context::SharedPtr context = nullptr,
    const std::string &reason = "user called rclcpp::shutdown()");

/// 注册一个函数，当在上下文中调用 shutdown 时调用该函数。
/// Register a function to be called when shutdown is called on the context.
/**
 * 如果给定上下文为 nullptr，则使用全局上下文，即 rclcpp::init() 初始化的上下文。
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * 当通过 Context::shutdown() 方法关闭关联的 Context 时，将调用这些回调。
 * These callbacks are called when the associated Context is shutdown with the
 * Context::shutdown() method.
 * 当由 SIGINT 处理程序关闭时，shutdown，因此这些回调是从专用信号处理线程异步调用的，在收到 SIGINT
 * 信号后的某个时间点。 When shutdown by the SIGINT handler, shutdown, and therefore these
 * callbacks, is called asynchronously from the dedicated signal handling thread, at some point
 * after the SIGINT signal is received.
 *
 * 请参阅 rclcpp::Context::on_shutdown()
 * \sa rclcpp::Context::on_shutdown()
 * \param[in] callback 在给定上下文关闭时要调用的回调
 * \param[in] callback to be called when the given context is shutdown
 * \param[in] context 要关联的上下文
 * \param[in] context with which to associate the context
 */
RCLCPP_PUBLIC
void on_shutdown(std::function<void()> callback, rclcpp::Context::SharedPtr context = nullptr);

/// 使用全局条件变量阻塞指定的时间量。
/// Use the global condition variable to block for the specified amount of time.
/**
 * 如果由于 shutdown() 或信号处理程序导致关联的上下文无效，此函数可能会提前中断。
 * This function can be interrupted early if the associated context becomes
 * invalid due to shutdown() or the signal handler.
 * 请参阅 rclcpp::Context::sleep_for
 * \sa rclcpp::Context::sleep_for
 *
 * 如果给定上下文为 nullptr，则使用全局上下文，即 rclcpp::init() 初始化的上下文。
 * If nullptr is given for the context, then the global context is used, i.e.
 * the context initialized by rclcpp::init().
 *
 * \param[in] nanoseconds A std::chrono::duration representing how long to sleep for.
 * \param[in] context Optional which may interrupt this sleep
 * \return true if the condition variable did not timeout.
 */
RCLCPP_PUBLIC
bool sleep_for(
    const std::chrono::nanoseconds &nanoseconds, rclcpp::Context::SharedPtr context = nullptr);

/// 安全地检查加法是否会溢出。 (Safely check if addition will overflow.)
/**
 * 操作数的类型 T 应该已经定义了
 * std::numeric_limits<T>::max(), `>`, `<` 和 `-` 运算符。
 * (The type of the operands, T, should have defined
 * std::numeric_limits<T>::max(), `>`, `<` and `-` operators.)
 *
 * \param[in] x 是第一个加数。 (x is the first addend.)
 * \param[in] y 是第二个加数。 (y is the second addend.)
 * \tparam T 是操作数的类型。 (T is type of the operands.)
 * \return 如果 x + y 的和大于 T::max 值，则返回 True。 (True if the x + y sum is greater than
 * T::max value.)
 */
template <typename T>
bool add_will_overflow(const T x, const T y) {
  // 当 y > 0 且 x > (std::numeric_limits<T>::max() - y) 时，表示加法将导致溢出。
  // (When y > 0 and x > (std::numeric_limits<T>::max() - y), it means the addition will cause an
  // overflow.)
  return (y > 0) && (x > (std::numeric_limits<T>::max() - y));
}

/// 安全地检查加法是否会下溢。 (Safely check if addition will underflow.)
/**
 * 操作数的类型 T 应该已经定义了
 * std::numeric_limits<T>::min(), `>`, `<` 和 `-` 运算符。
 * (The type of the operands, T, should have defined
 * std::numeric_limits<T>::min(), `>`, `<` and `-` operators.)
 *
 * \param[in] x 是第一个加数。 (x is the first addend.)
 * \param[in] y 是第二个加数。 (y is the second addend.)
 * \tparam T 是操作数的类型。 (T is type of the operands.)
 * \return 如果 x + y 的和小于 T::min 值，则返回 True。 (True if the x + y sum is less than T::min
 * value.)
 */
template <typename T>
bool add_will_underflow(const T x, const T y) {
  // 当 y < 0 且 x < (std::numeric_limits<T>::min() - y) 时，表示加法将导致下溢。
  // (When y < 0 and x < (std::numeric_limits<T>::min() - y), it means the addition will cause an
  // underflow.)
  return (y < 0) && (x < (std::numeric_limits<T>::min() - y));
}

/// 安全地检查减法是否会溢出。 (Safely check if subtraction will overflow.)
/**
 * 操作数的类型 T 应该已经定义了
 * std::numeric_limits<T>::max(), `>`, `<` 和 `+` 运算符。
 * (The type of the operands, T, should have defined
 * std::numeric_limits<T>::max(), `>`, `<` and `+` operators.)
 *
 * \param[in] x 是被减数。 (x is the minuend.)
 * \param[in] y 是减数。 (y is the subtrahend.)
 * \tparam T 是操作数的类型。 (T is type of the operands.)
 * \return 如果差值 `x - y` 的和大于 T::max 值，则返回 True。 (True if the difference `x - y` sum is
 * grater than T::max value.)
 */
template <typename T>
bool sub_will_overflow(const T x, const T y) {
  // 当 y < 0 且 x > (std::numeric_limits<T>::max() + y) 时，表示减法将导致溢出。
  // (When y < 0 and x > (std::numeric_limits<T>::max() + y), it means the subtraction will cause an
  // overflow.)
  return (y < 0) && (x > (std::numeric_limits<T>::max() + y));
}

/// 安全地检查减法是否会下溢。 (Safely check if subtraction will underflow.)
/**
 * 操作数的类型 T 应该已经定义了
 * std::numeric_limits<T>::min(), `>`, `<` 和 `+` 运算符。
 * (The type of the operands, T, should have defined
 * std::numeric_limits<T>::min(), `>`, `<` and `+` operators.)
 *
 * \param[in] x 是被减数。 (x is the minuend.)
 * \param[in] y 是减数。 (y is the subtrahend.)
 * \tparam T 是操作数的类型。 (T is type of the operands.)
 * \return 如果差值 `x - y` 的和小于 T::min 值，则返回 True。 (True if the difference `x - y` sum is
 * less than T::min value.)
 */
template <typename T>
bool sub_will_underflow(const T x, const T y) {
  // 当 y > 0 且 x < (std::numeric_limits<T>::min() + y) 时，表示减法将导致下溢。
  // (When y > 0 and x < (std::numeric_limits<T>::min() + y), it means the subtraction will cause an
  // underflow.)
  return (y > 0) && (x < (std::numeric_limits<T>::min() + y));
}

/// 返回给定的字符串。 (Return the given string.)
/**
 * 此函数重载用于将任何字符串转换为 C 风格字符串。
 * (This function is overloaded to transform any string to C-style string.)
 *
 * \param[in] string_in 是要返回的字符串。 (string_in is the string to be returned)
 * \return 给定的字符串。 (the given string)
 */
RCLCPP_PUBLIC
const char *get_c_string(const char *string_in);

/// 从给定的 std::string 中返回 C 字符串。 (Return the C string from the given std::string.)
/**
 * \param[in] string_in 是一个 std::string。 (string_in is a std::string)
 * \return 来自 std::string 的 C 字符串。 (the C string from the std::string)
 */
RCLCPP_PUBLIC
const char *get_c_string(const std::string &string_in);

/// 从给定的 std::vector<std::string> 中返回 std::vector 的 C 字符串。 (Return the std::vector of C
/// string from the given std::vector<std::string>.)
/**
 * \param[in] strings_in 是一个 std::vector 的 std::string。 (strings_in is a std::vector of
 * std::string) \return 从 std::vector<std::string> 中获取的 std::vector 的 C 字符串。 (the
 * std::vector of C string from the std::vector<std::string>)
 */
RCLCPP_PUBLIC
std::vector<const char *> get_c_vector_string(const std::vector<std::string> &strings_in);

}  // namespace rclcpp

#endif  // RCLCPP__UTILITIES_HPP_

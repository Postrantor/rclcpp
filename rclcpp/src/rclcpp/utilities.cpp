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

#include "rclcpp/utilities.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "./signal_handler.hpp"
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/detail/utilities.hpp"
#include "rclcpp/exceptions.hpp"

namespace rclcpp {

/*!
 * \brief 初始化 ROS2 节点 (Initialize the ROS2 node)
 *
 * \param[in] argc 命令行参数的数量 (Number of command line arguments)
 * \param[in] argv 命令行参数的数组 (Array of command line arguments)
 * \param[in] init_options 初始化选项对象 (Initialization options object)
 * \param[in] signal_handler_options 信号处理器选项对象 (Signal handler options object)
 */
void init(
    int argc,
    char const *const *argv,
    const InitOptions &init_options,
    SignalHandlerOptions signal_handler_options) {
  // 获取全局默认上下文 (Get the global default context)
  using rclcpp::contexts::get_global_default_context;
  get_global_default_context()->init(argc, argv, init_options);

  // 安装信号处理器 (Install the signal handlers)
  install_signal_handlers(signal_handler_options);
}

/*!
 * \brief 安装信号处理器 (Install signal handlers)
 *
 * \param[in] signal_handler_options 信号处理器选项对象 (Signal handler options object)
 * \return 是否成功安装信号处理器 (Whether the signal handlers were successfully installed)
 */
bool install_signal_handlers(SignalHandlerOptions signal_handler_options) {
  return SignalHandler::get_global_signal_handler().install(signal_handler_options);
}

/*!
 * \brief 检查信号处理器是否已安装 (Check if signal handlers are installed)
 *
 * \return 信号处理器是否已安装 (Whether the signal handlers are installed)
 */
bool signal_handlers_installed() {
  return SignalHandler::get_global_signal_handler().is_installed();
}

/*!
 * \brief 获取当前信号处理器选项 (Get current signal handler options)
 *
 * \return 当前信号处理器选项对象 (Current signal handler options object)
 */
SignalHandlerOptions get_current_signal_handler_options() {
  return SignalHandler::get_global_signal_handler().get_current_signal_handler_options();
}

/*!
 * \brief 卸载信号处理器 (Uninstall signal handlers)
 *
 * \return 是否成功卸载信号处理器 (Whether the signal handlers were successfully uninstalled)
 */
bool uninstall_signal_handlers() { return SignalHandler::get_global_signal_handler().uninstall(); }

/*!
 * \brief 移除 ROS 参数 (Remove ROS arguments)
 *
 * \param[in] argv 命令行参数的数组 (Array of command line arguments)
 * \param[in] args rcl_arguments_t 结构体指针 (Pointer to rcl_arguments_t structure)
 * \param[in] alloc 分配器对象 (Allocator object)
 * \return 返回移除 ROS 参数后的命令行参数向量 (Return a vector of command line arguments after
 * removing ROS arguments)
 */
static std::vector<std::string> _remove_ros_arguments(
    char const *const *argv, const rcl_arguments_t *args, rcl_allocator_t alloc) {
  rcl_ret_t ret;
  int nonros_argc = 0;
  const char **nonros_argv = NULL;

  // 移除 ROS 参数 (Remove ROS arguments)
  ret = rcl_remove_ros_arguments(argv, args, alloc, &nonros_argc, &nonros_argv);

  if (RCL_RET_OK != ret || nonros_argc < 0) {
    // 处理异常 (Handle exception)
    exceptions::RCLError exc(ret, rcl_get_error_state(), "");
    rcl_reset_error();
    if (NULL != nonros_argv) {
      alloc.deallocate(nonros_argv, alloc.state);
    }
    throw exc;
  }

  // 将移除 ROS 参数后的命令行参数存储到向量中 (Store the command line arguments after removing ROS
  // arguments in a vector)
  std::vector<std::string> return_arguments(static_cast<size_t>(nonros_argc));

  for (size_t ii = 0; ii < static_cast<size_t>(nonros_argc); ++ii) {
    return_arguments[ii] = std::string(nonros_argv[ii]);
  }

  if (NULL != nonros_argv) {
    alloc.deallocate(nonros_argv, alloc.state);
  }

  return return_arguments;
}

/*!
 * \brief 初始化并移除 ROS 参数 (Initialize and remove ROS arguments)
 *
 * \param[in] argc 命令行参数的数量 (Number of command line arguments)
 * \param[in] argv 命令行参数的数组 (Array of command line arguments)
 * \param[in] init_options 初始化选项对象 (Initialization options object)
 * \return 返回移除 ROS 参数后的命令行参数向量 (Return a vector of command line arguments after
 * removing ROS arguments)
 */
std::vector<std::string> init_and_remove_ros_arguments(
    int argc, char const *const *argv, const InitOptions &init_options) {
  init(argc, argv, init_options);

  using rclcpp::contexts::get_global_default_context;
  auto rcl_context = get_global_default_context()->get_rcl_context();
  return _remove_ros_arguments(argv, &(rcl_context->global_arguments), rcl_get_default_allocator());
}

/*!
 * \brief 移除 ROS 参数 (Remove ROS arguments)
 *
 * \param[in] argc 命令行参数的数量 (Number of command line arguments)
 * \param[in] argv 命令行参数的数组 (Array of command line arguments)
 * \return 返回移除 ROS 参数后的命令行参数向量 (Return a vector of command line arguments after
 * removing ROS arguments)
 */
std::vector<std::string> remove_ros_arguments(int argc, char const *const *argv) {
  rcl_allocator_t alloc = rcl_get_default_allocator();
  rcl_arguments_t parsed_args = rcl_get_zero_initialized_arguments();

  rcl_ret_t ret;

  // 解析命令行参数 (Parse command line arguments)
  ret = rcl_parse_arguments(argc, argv, alloc, &parsed_args);
  if (RCL_RET_OK != ret) {
    exceptions::throw_from_rcl_error(ret, "failed to parse arguments");
  }

  std::vector<std::string> return_arguments;
  try {
    return_arguments = _remove_ros_arguments(argv, &parsed_args, alloc);
  } catch (exceptions::RCLError &exc) {
    if (RCL_RET_OK != rcl_arguments_fini(&parsed_args)) {
      exc.formatted_message +=
          std::string(", failed also to cleanup parsed arguments, leaking memory: ") +
          rcl_get_error_string().str;
      rcl_reset_error();
    }
    throw exc;
  }

  // 清理解析后的参数 (Clean up parsed arguments)
  ret = rcl_arguments_fini(&parsed_args);
  if (RCL_RET_OK != ret) {
    exceptions::throw_from_rcl_error(ret, "failed to cleanup parsed arguments, leaking memory");
  }

  return return_arguments;
}

/*!
 * \brief 检查 ROS2 节点是否正常运行 (Check if the ROS2 node is running normally)
 *
 * \param[in] context 上下文共享指针 (Shared pointer to the context)
 * \return ROS2 节点是否正常运行 (Whether the ROS2 node is running normally)
 */
bool ok(Context::SharedPtr context) {
  using rclcpp::contexts::get_global_default_context;
  if (nullptr == context) {
    context = get_global_default_context();
  }
  return context->is_valid();
}

/*!
 * \brief 关闭 ROS2 节点 (Shutdown the ROS2 node)
 *
 * \param[in] context 上下文共享指针 (Shared pointer to the context)
 * \param[in] reason 关闭原因 (Reason for shutdown)
 * \return 是否成功关闭 ROS2 节点 (Whether the ROS2 node was successfully shut down)
 */
bool shutdown(Context::SharedPtr context, const std::string &reason) {
  using rclcpp::contexts::get_global_default_context;
  auto default_context = get_global_default_context();
  if (nullptr == context) {
    context = default_context;
  }
  bool ret = context->shutdown(reason);
  if (context == default_context) {
    uninstall_signal_handlers();
  }
  return ret;
}

/*!
 * \brief 注册关闭回调函数 (Register shutdown callback function)
 *
 * \param[in] callback 回调函数 (Callback function)
 * \param[in] context 上下文共享指针 (Shared pointer to the context)
 */
void on_shutdown(std::function<void()> callback, Context::SharedPtr context) {
  using rclcpp::contexts::get_global_default_context;
  if (nullptr == context) {
    context = get_global_default_context();
  }
  context->on_shutdown(callback);
}

/*!
 * \brief 睡眠一段时间 (Sleep for a period of time)
 *
 * \param[in] nanoseconds 睡眠的纳秒数 (Number of nanoseconds to sleep)
 * \param[in] context 上下文共享指针 (Shared pointer to the context)
 * \return 是否成功睡眠 (Whether the sleep was successful)
 */
bool sleep_for(const std::chrono::nanoseconds &nanoseconds, Context::SharedPtr context) {
  using rclcpp::contexts::get_global_default_context;
  if (nullptr == context) {
    context = get_global_default_context();
  }
  return context->sleep_for(nanoseconds);
}

// 获取 C 风格字符串 (Get C-style string)
const char *get_c_string(const char *string_in) { return string_in; }
const char *get_c_string(const std::string &string_in) { return string_in.c_str(); }

/*!
 * \brief 将 std::vector<std::string> 转换为 std::vector<const char *> (Convert
 * std::vector<std::string> to std::vector<const char *>)
 *
 * \param[in] strings_in 输入的字符串向量 (Input vector of strings)
 * \return 返回转换后的 const char * 向量 (Return the converted vector of const char *)
 */
std::vector<const char *> get_c_vector_string(const std::vector<std::string> &strings_in) {
  std::vector<const char *> cstrings;
  cstrings.reserve(strings_in.size());

  for (size_t i = 0; i < strings_in.size(); ++i) {
    cstrings.push_back(strings_in[i].c_str());
  }

  return cstrings;
}

}  // namespace rclcpp

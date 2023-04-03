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

#include "rclcpp/logger.hpp"

#include <string>

#include "rcl_logging_interface/rcl_logging_interface.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

namespace rclcpp {

/**
 * @brief 获取具有指定名称的日志记录器 (Get a logger with the specified name)
 * @param[in] name 日志记录器的名称 (The name of the logger)
 * @return 返回具有指定名称的日志记录器 (Return a logger with the specified name)
 */
Logger get_logger(const std::string &name) {
#if RCLCPP_LOGGING_ENABLED
  // 如果启用了日志记录功能，返回具有指定名称的日志记录器
  // (If logging is enabled, return a logger with the specified name)
  return rclcpp::Logger(name);
#else
  // 否则，忽略名称并返回默认的日志记录器
  // (Otherwise, ignore the name and return the default logger)
  (void)name;
  return rclcpp::Logger();
#endif
}

/**
 * @brief 获取与给定节点关联的日志记录器 (Get the logger associated with the given node)
 * @param[in] node 节点指针 (A pointer to the node)
 * @return 返回与给定节点关联的日志记录器 (Return the logger associated with the given node)
 */
Logger get_node_logger(const rcl_node_t *node) {
  // 获取节点的日志记录器名称 (Get the logger name of the node)
  const char *logger_name = rcl_node_get_logger_name(node);

  // 如果获取到的日志记录器名称为空，则使用默认的 "rclcpp" 日志记录器，并报告错误
  // (If the obtained logger name is empty, use the default "rclcpp" logger and report an error)
  if (nullptr == logger_name) {
    auto logger = rclcpp::get_logger("rclcpp");
    RCLCPP_ERROR(
        logger, "failed to get logger name from node at address %p",
        static_cast<void *>(const_cast<rcl_node_t *>(node)));
    return logger;
  }

  // 返回具有获取到的名称的日志记录器 (Return a logger with the obtained name)
  return rclcpp::get_logger(logger_name);
}

/**
 * @brief 获取日志记录目录 (Get the logging directory)
 * @return 返回日志记录目录的路径 (Return the path of the logging directory)
 */
rcpputils::fs::path get_logging_directory() {
  char *log_dir = NULL;
  auto allocator = rcutils_get_default_allocator();

  // 获取日志记录目录 (Get the logging directory)
  rcl_logging_ret_t ret = rcl_logging_get_logging_directory(allocator, &log_dir);

  // 如果获取失败，抛出异常 (If the acquisition fails, throw an exception)
  if (RCL_LOGGING_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // 构造并返回日志记录目录的路径 (Construct and return the path of the logging directory)
  std::string path{log_dir};
  allocator.deallocate(log_dir, allocator.state);
  return path;
}

/**
 * @brief 设置日志记录器的级别 (Set the level of the logger)
 * @param[in] level 日志记录器的级别 (The level of the logger)
 */
void Logger::set_level(Level level) {
  // 设置日志记录器的级别 (Set the level of the logger)
  rcutils_ret_t rcutils_ret =
      rcutils_logging_set_logger_level(get_name(), static_cast<RCUTILS_LOG_SEVERITY>(level));

  // 如果设置失败，根据错误类型抛出异常 (If the setting fails, throw an exception according to the
  // error type)
  if (rcutils_ret != RCUTILS_RET_OK) {
    if (rcutils_ret == RCUTILS_RET_INVALID_ARGUMENT) {
      exceptions::throw_from_rcl_error(
          RCL_RET_INVALID_ARGUMENT, "Invalid parameter", rcutils_get_error_state(),
          rcutils_reset_error);
    }
    exceptions::throw_from_rcl_error(
        RCL_RET_ERROR, "Couldn't set logger level", rcutils_get_error_state(), rcutils_reset_error);
  }
}

}  // namespace rclcpp

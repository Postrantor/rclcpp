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

#ifndef RCLCPP__LOGGER_HPP_
#define RCLCPP__LOGGER_HPP_

#include <memory>
#include <string>

#include "rcl/node.h"
#include "rclcpp/visibility_control.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/logging.h"

/**
 * \def RCLCPP_LOGGING_ENABLED
 * 当此定义为 true（默认）时，日志记录器工厂函数将正常工作。
 * (When this define evaluates to true (default), logger factory functions will
 * behave normally.)
 * 当为 false 时，日志记录器工厂函数将创建虚拟日志记录器以避免操作对象的计算开销。
 * (When false, logger factory functions will create dummy loggers to avoid
 * computational expense in manipulating objects.)
 * 这应与 `RCLCPP_LOG_MIN_SEVERITY` 结合使用以编译日志宏。
 * (This should be used in combination with `RCLCPP_LOG_MIN_SEVERITY` to compile
 * out logging macros.)
 */
// TODO(dhood): determine this automatically from `RCLCPP_LOG_MIN_SEVERITY`
#ifndef RCLCPP_LOGGING_ENABLED
#define RCLCPP_LOGGING_ENABLED 1
#endif

namespace rclcpp {

// 前向声明用于 friend 语句。
// (Forward declaration is used for friend statement.)
namespace node_interfaces {
class NodeLogging;
}

class Logger;

/// 返回一个命名的日志记录器。
/// (Return a named logger.)
/**
 * 返回的日志记录器名称将包括任何命名惯例，例如名称前缀。
 * (The returned logger's name will include any naming conventions, such as a
 * name prefix.)
 * 目前没有这样的命名惯例，但将来可能会引入。
 * (Currently there are no such naming conventions but they may be introduced in
 * the future.)
 *
 * \param[in] name 日志记录器的名称
 * \return 包括命名惯例的完全限定名称的日志记录器，或
 * \return 如果禁用日志记录，则返回虚拟日志记录器。
 * (\param[in] name the name of the logger
 * \return a logger with the fully-qualified name including naming conventions, or
 * \return a dummy logger if logging is disabled.)
 */
RCLCPP_PUBLIC
Logger get_logger(const std::string &name);

/// 使用 rcl_node_t 返回一个命名的日志记录器。
/// (Return a named logger using an rcl_node_t.)
/**
 * 这是一个方便的函数，它进行错误检查并返回节点日志记录器名称，如果无法获取节点名称，则返回
 * "rclcpp"。 (This is a convenience function that does error checking and returns the node logger
 * name, or "rclcpp" if it is unable to get the node name.)
 *
 * \param[in] node 从中获取日志记录器名称的 rcl 节点
 * \return 基于节点名称的日志记录器，或者如果有错误，则返回 "rclcpp"
 * (\param[in] node the rcl node from which to get the logger name
 * \return a logger based on the node name, or "rclcpp" if there's an error)
 */
RCLCPP_PUBLIC
Logger get_node_logger(const rcl_node_t *node);

/// 获取当前日志目录。
/// (Get the current logging directory.)
/**
 * 有关如何确定日志目录的详细信息，请参见 rcl_logging_get_logging_directory()。
 * (For more details of how the logging directory is determined,
 * see rcl_logging_get_logging_directory().)
 *
 * \returns 正在使用的日志目录。
 * \throws rclcpp::exceptions::RCLError 如果发生意外错误。
 * (\returns the logging directory being used.
 * \throws rclcpp::exceptions::RCLError if an unexpected error occurs.)
 */
RCLCPP_PUBLIC
rcpputils::fs::path get_logging_directory();

class Logger {
public:
  /// 日志级别类型的枚举。
  /// (An enum for the type of logger level.)
  enum class Level {
    Unset = RCUTILS_LOG_SEVERITY_UNSET,  ///< 未设置的日志级别（The unset log level）
    Debug = RCUTILS_LOG_SEVERITY_DEBUG,  ///< 调试日志级别（The debug log level）
    Info = RCUTILS_LOG_SEVERITY_INFO,    ///< 信息日志级别（The info log level）
    Warn = RCUTILS_LOG_SEVERITY_WARN,    ///< 警告日志级别（The warn log level）
    Error = RCUTILS_LOG_SEVERITY_ERROR,  ///< 错误日志级别（The error log level）
    Fatal = RCUTILS_LOG_SEVERITY_FATAL,  ///< 致命日志级别（The fatal log level）
  };

private:
  friend Logger rclcpp::get_logger(const std::string &name);
  friend ::rclcpp::node_interfaces::NodeLogging;

  /// 虚拟日志记录器的构造函数。
  /// (Constructor of a dummy logger.)
  /**
   * 当禁用日志记录时使用此功能：请参见 `RCLCPP_LOGGING_ENABLED`。
   * (This is used when logging is disabled: see `RCLCPP_LOGGING_ENABLED`.)
   * 不能直接调用此函数，请参见 `rclcpp::get_logger`。
   * (This cannot be called directly, see `rclcpp::get_logger` instead.)
   */
  Logger() : name_(nullptr) {}

  /// 命名日志记录器的构造函数。
  /// (Constructor of a named logger.)
  /**
   * 不能直接调用此函数，请参见 `rclcpp::get_logger`。
   * (This cannot be called directly, see `rclcpp::get_logger` instead.)
   */
  explicit Logger(const std::string &name) : name_(new std::string(name)) {}

  std::shared_ptr<const std::string> name_;

public:
  RCLCPP_PUBLIC
  Logger(const Logger &) = default;

  /// 获取此日志记录器的名称。
  /// (Get the name of this logger.)
  /**
   * \return 包括任何前缀的日志记录器的完整名称，或
   * \return 如果此日志记录器无效（例如，因为禁用了日志记录），则返回 `nullptr`。
   * (\return the full name of the logger including any prefixes, or
   * \return `nullptr` if this logger is invalid (e.g. because logging is
   *   disabled).)
   */
  RCLCPP_PUBLIC
  const char *get_name() const {
    if (!name_) {
      return nullptr;
    }
    return name_->c_str();
  }

  /// 返回此日志记录器的后代日志记录器。
  /// (Return a logger that is a descendant of this logger.)
  /**
   * 子日志记录器的全名将包括表示其是此日志记录器后代的任何层次结构约定。
   * (The child logger's full name will include any hierarchy conventions that
   * indicate it is a descendant of this logger.)
   * 例如，```get_logger('abc').get_child('def')``` 将返回名称为 `abc.def` 的日志记录器。
   * (For example, ```get_logger('abc').get_child('def')``` will return a logger
   * with name `abc.def`.)
   *
   * \param[in] suffix 子日志记录器的后缀
   * \return 包括后缀的完全限定名称的日志记录器，或
   * \return 如果此日志记录器无效（例如，因为禁用了日志记录），则返回虚拟日志记录器。
   * (\param[in] suffix the child logger's suffix
   * \return a logger with the fully-qualified name including the suffix, or
   * \return a dummy logger if this logger is invalid (e.g. because logging is
   *   disabled).)
   */
  RCLCPP_PUBLIC
  Logger get_child(const std::string &suffix) {
    if (!name_) {
      return Logger();
    }
    return Logger(*name_ + "." + suffix);
  }

  /// 设置当前日志记录器的级别。
  /// (Set level for current logger.)
  /**
   * \param[in] level 日志记录器的级别
   * \throws rclcpp::exceptions::RCLInvalidArgument 如果级别无效。
   * \throws rclcpp::exceptions::RCLError 如果发生其他错误。
   * (\param[in] level the logger's level
   * \throws rclcpp::exceptions::RCLInvalidArgument if level is invalid.
   * \throws rclcpp::exceptions::RCLError if other error happens.)
   */
  RCLCPP_PUBLIC
  void set_level(Level level);
};

}  // namespace rclcpp

#endif  // RCLCPP__LOGGER_HPP_

// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXCEPTIONS__EXCEPTIONS_HPP_
#define RCLCPP__EXCEPTIONS__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <vector>

#include "rcl/error_handling.h"
#include "rcl/types.h"
#include "rclcpp/visibility_control.hpp"
#include "rcpputils/join.hpp"

namespace rclcpp {
namespace exceptions {

/// 当试图使用无效节点的方法时抛出的异常 (Thrown when a method is trying to use a node, but it is
/// invalid)
class InvalidNodeError : public std::runtime_error {
public:
  // 构造函数，设置错误信息为 "node is invalid" (Constructor, set error message to "node is
  // invalid")
  InvalidNodeError() : std::runtime_error("node is invalid") {}
};

/// 当名称（节点、命名空间、主题等）无效时抛出的异常 (Thrown when any kind of name (node, namespace,
/// topic, etc.) is invalid)
class NameValidationError : public std::invalid_argument {
public:
  // 构造函数，设置错误信息及相关属性 (Constructor, set error message and related attributes)
  NameValidationError(
      const char* name_type_, const char* name_, const char* error_msg_, size_t invalid_index_)
      : std::invalid_argument(format_error(name_type_, name_, error_msg_, invalid_index_)),
        name_type(name_type_),
        name(name_),
        error_msg(error_msg_),
        invalid_index(invalid_index_) {}

  // 格式化错误信息的静态方法 (Static method for formatting error messages)
  static std::string format_error(
      const char* name_type, const char* name, const char* error_msg, size_t invalid_index);

  // 名称类型 (Name type)
  const std::string name_type;
  // 名称 (Name)
  const std::string name;
  // 错误信息 (Error message)
  const std::string error_msg;
  // 无效索引 (Invalid index)
  const size_t invalid_index;
};

/// 当节点名称无效时抛出的异常 (Thrown when a node name is invalid)
class InvalidNodeNameError : public NameValidationError {
public:
  // 构造函数，设置节点名称、错误信息及无效索引 (Constructor, set node name, error message, and
  // invalid index)
  InvalidNodeNameError(const char* node_name, const char* error_msg, size_t invalid_index)
      : NameValidationError("node name", node_name, error_msg, invalid_index) {}
};

/// 当节点命名空间无效时抛出的异常 (Thrown when a node namespace is invalid)
class InvalidNamespaceError : public NameValidationError {
public:
  // 构造函数，设置命名空间、错误信息及无效索引 (Constructor, set namespace, error message, and
  // invalid index)
  InvalidNamespaceError(const char* namespace_, const char* error_msg, size_t invalid_index)
      : NameValidationError("namespace", namespace_, error_msg, invalid_index) {}
};

/// 抛出一个基于 rcl 错误创建的 C++ std::exception。
/**
 * 传递 nullptr 作为 reset_error 是安全的，将避免调用任何函数
 * 重置错误。
 *
 * \param ret 当前错误状态的返回代码
 * \param prefix 如果适用的话，添加到错误前面的字符串（并非所有错误都有自定义消息）
 * \param error_state 要从中创建异常的错误状态，如果为 nullptr，则使用 rcl_get_error_state
 * \param reset_error 在抛出之前要调用的函数，它将清除错误状态
 * \throws std::invalid_argument 如果 ret 是 RCL_RET_OK
 * \throws std::runtime_error 如果 rcl_get_error_state 返回 0
 * \throws RCLErrorBase 基于 ret 的某个子类异常
 */
/* *INDENT-OFF* */  // Uncrustify 还不能正确理解 [[noreturn]]
RCLCPP_PUBLIC
void throw_from_rcl_error [[noreturn]] (
    rcl_ret_t ret,
    const std::string& prefix = "",
    const rcl_error_state_t* error_state = nullptr,
    reset_error_function_t reset_error = rcl_reset_error);
/* *INDENT-ON* */

class RCLErrorBase {
public:
  RCLCPP_PUBLIC
  RCLErrorBase(rcl_ret_t ret, const rcl_error_state_t* error_state);
  virtual ~RCLErrorBase() {}

  rcl_ret_t ret;                  ///< rcl 返回代码
  std::string message;            ///< 错误消息
  std::string file;               ///< 发生错误的文件名
  size_t line;                    ///< 发生错误的行号
  std::string formatted_message;  ///< 格式化后的错误消息
};

/**
 * @class RCLError
 * @brief 创建时，返回代码与其他专用异常不匹配。
 *        Created when the return code does not match one of the other specialized exceptions.
 */
class RCLError : public RCLErrorBase, public std::runtime_error {
public:
  // 公共函数声明 Public function declarations
  RCLCPP_PUBLIC

  /**
   * @brief 构造函数，接受返回值、错误状态和前缀。
   *        Constructor that accepts a return value, error state, and prefix.
   * @param ret 返回值 Return value
   * @param error_state 错误状态 Error state
   * @param prefix 前缀 Prefix
   */
  RCLError(rcl_ret_t ret, const rcl_error_state_t* error_state, const std::string& prefix);

  /**
   * @brief 构造函数，接受基础异常和前缀。
   *        Constructor that accepts a base exception and prefix.
   * @param base_exc 基础异常 Base exception
   * @param prefix 前缀 Prefix
   */
  RCLError(const RCLErrorBase& base_exc, const std::string& prefix);
};

/**
 * @class RCLBadAlloc
 * @brief 创建时，返回值为 RCL_RET_BAD_ALLOC。
 *        Created when the ret is RCL_RET_BAD_ALLOC.
 */
class RCLBadAlloc : public RCLErrorBase, public std::bad_alloc {
public:
  // 公共函数声明 Public function declarations
  RCLCPP_PUBLIC

  /**
   * @brief 构造函数，接受返回值和错误状态。
   *        Constructor that accepts a return value and error state.
   * @param ret 返回值 Return value
   * @param error_state 错误状态 Error state
   */
  RCLBadAlloc(rcl_ret_t ret, const rcl_error_state_t* error_state);

  /**
   * @brief 构造函数，接受基础异常。
   *        Constructor that accepts a base exception.
   * @param base_exc 基础异常 Base exception
   */
  explicit RCLBadAlloc(const RCLErrorBase& base_exc);
};

/**
 * @class RCLInvalidArgument
 * @brief 创建时，返回值为 RCL_RET_INVALID_ARGUMENT。
 *        Created when the ret is RCL_RET_INVALID_ARGUMENT.
 */
class RCLInvalidArgument : public RCLErrorBase, public std::invalid_argument {
public:
  // 公共函数声明 Public function declarations
  RCLCPP_PUBLIC

  /**
   * @brief 构造函数，接受返回值、错误状态和前缀。
   *        Constructor that accepts a return value, error state, and prefix.
   * @param ret 返回值 Return value
   * @param error_state 错误状态 Error state
   * @param prefix 前缀 Prefix
   */
  RCLInvalidArgument(
      rcl_ret_t ret, const rcl_error_state_t* error_state, const std::string& prefix);

  /**
   * @brief 构造函数，接受基础异常和前缀。
   *        Constructor that accepts a base exception and prefix.
   * @param base_exc 基础异常 Base exception
   * @param prefix 前缀 Prefix
   */
  RCLInvalidArgument(const RCLErrorBase& base_exc, const std::string& prefix);
};

/**
 * @class RCLInvalidROSArgsError
 * @brief 创建时，返回值为 RCL_RET_INVALID_ROS_ARGS。
 *        Created when the ret is RCL_RET_INVALID_ROS_ARGS.
 */
class RCLInvalidROSArgsError : public RCLErrorBase, public std::runtime_error {
public:
  // 公共函数声明 Public function declarations
  RCLCPP_PUBLIC

  /**
   * @brief 构造函数，接受返回值、错误状态和前缀。
   *        Constructor that accepts a return value, error state, and prefix.
   * @param ret 返回值 Return value
   * @param error_state 错误状态 Error state
   * @param prefix 前缀 Prefix
   */
  RCLInvalidROSArgsError(
      rcl_ret_t ret, const rcl_error_state_t* error_state, const std::string& prefix);

  /**
   * @brief 构造函数，接受基础异常和前缀。
   *        Constructor that accepts a base exception and prefix.
   * @param base_exc 基础异常 Base exception
   * @param prefix 前缀 Prefix
   */
  RCLInvalidROSArgsError(const RCLErrorBase& base_exc, const std::string& prefix);
};

/// \brief 当发现未解析的 ROS 特定参数时抛出。Thrown when unparsed ROS specific arguments are found.
class UnknownROSArgsError : public std::runtime_error {
public:
  /// \brief 构造函数。Constructor.
  /// \param[in] unknown_ros_args_in 未知的 ROS 参数。Unknown ROS arguments.
  explicit UnknownROSArgsError(std::vector<std::string>&& unknown_ros_args_in)
      : std::runtime_error(
            "found unknown ROS arguments: '" + rcpputils::join(unknown_ros_args_in, "', '") + "'"),
        unknown_ros_args(unknown_ros_args_in) {}

  const std::vector<std::string> unknown_ros_args;
};

/// \brief 当遇到无效的 rclcpp::Event 对象或 SharedPtr 时抛出。Thrown when an invalid rclcpp::Event
/// object or SharedPtr is encountered.
class InvalidEventError : public std::runtime_error {
public:
  InvalidEventError() : std::runtime_error("event is invalid") {}
};

/// \brief 当在预期找到已注册的 rclcpp::Event 的地方遇到未注册的 rclcpp::Event 时抛出。Thrown when
/// an unregistered rclcpp::Event is encountered where a registered one was expected.
class EventNotRegisteredError : public std::runtime_error {
public:
  EventNotRegisteredError() : std::runtime_error("event already registered") {}
};

/// \brief 如果传递的参数不一致或无效，则抛出。Thrown if passed parameters are inconsistent or
/// invalid.
class InvalidParametersException : public std::runtime_error {
public:
  // Inherit constructors from runtime_error.
  using std::runtime_error::runtime_error;
};

/// \brief 如果传递的参数值无效，则抛出。Thrown if passed parameter value is invalid.
class InvalidParameterValueException : public std::runtime_error {
  // Inherit constructors from runtime_error.
  using std::runtime_error::runtime_error;
};

/// \brief 如果请求的参数类型无效，则抛出。Thrown if requested parameter type is invalid.
/**
 * 本质上与 rclcpp::ParameterTypeException 相同，但在错误消息中包含参数名称。
 * Essentially the same as rclcpp::ParameterTypeException, but with parameter
 * name in the error message.
 */
class InvalidParameterTypeException : public std::runtime_error {
public:
  /// \brief 构造一个实例。Construct an instance.
  /// \param[in] name 参数名称。The name of the parameter.
  /// \param[in] message 自定义异常消息。Custom exception message.
  RCLCPP_PUBLIC
  InvalidParameterTypeException(const std::string& name, const std::string message)
      : std::runtime_error("parameter '" + name + "' has invalid type: " + message) {}
};

/// \brief 如果用户尝试创建未初始化的静态类型参数，则抛出。Thrown if user attempts to create an
/// uninitialized statically typed parameter.
/**
 * (参见 https://github.com/ros2/rclcpp/issues/1691)
 * (see https://github.com/ros2/rclcpp/issues/1691)
 */
class UninitializedStaticallyTypedParameterException : public std::runtime_error {
public:
  /// \brief 构造一个实例。Construct an instance.
  /// \param[in] name 参数名称。The name of the parameter.
  RCLCPP_PUBLIC
  explicit UninitializedStaticallyTypedParameterException(const std::string& name)
      : std::runtime_error("Statically typed parameter '" + name + "' must be initialized.") {}
};

/// \brief 如果参数已经声明，则抛出。Thrown if parameter is already declared.
class ParameterAlreadyDeclaredException : public std::runtime_error {
  // Inherit constructors from runtime_error.
  using std::runtime_error::runtime_error;
};

/// Thrown if parameter is not declared, e.g. either set or get was called without first declaring.
// 如果参数未声明，例如在首次声明之前调用了 set 或 get，则抛出此异常。
class ParameterNotDeclaredException : public std::runtime_error {
  // Inherit constructors from runtime_error.
  // 从 runtime_error 继承构造函数。
  using std::runtime_error::runtime_error;
};

/// Thrown if parameter is immutable and therefore cannot be undeclared.
// 如果参数是不可变的，因此无法取消声明，则抛出此异常。
class ParameterImmutableException : public std::runtime_error {
  // Inherit constructors from runtime_error.
  // 从 runtime_error 继承构造函数。
  using std::runtime_error::runtime_error;
};

/// Thrown if parameter is modified while in a set callback.
// 如果在 set 回调中修改了参数，则抛出此异常。
class ParameterModifiedInCallbackException : public std::runtime_error {
  // Inherit constructors from runtime_error.
  // 从 runtime_error 继承构造函数。
  using std::runtime_error::runtime_error;
};

/// Thrown when an uninitialized parameter is accessed.
// 当访问未初始化的参数时，抛出此异常。
class ParameterUninitializedException : public std::runtime_error {
public:
  /// Construct an instance.
  // 构造一个实例。
  /**
   * \param[in] name the name of the parameter.
   * \param[in] name 参数的名称。
   */
  explicit ParameterUninitializedException(const std::string& name)
      : std::runtime_error("parameter '" + name + "' is not initialized") {}
};

/// Thrown if the QoS overrides provided aren't valid.
// 如果提供的 QoS 覆盖无效，则抛出此异常。
class InvalidQosOverridesException : public std::runtime_error {
  // Inherit constructors from runtime_error.
  // 从 runtime_error 继承构造函数。
  using std::runtime_error::runtime_error;
};

/// Thrown if a QoS compatibility check fails.
// 如果 QoS 兼容性检查失败，则抛出此异常。
class QoSCheckCompatibleException : public std::runtime_error {
  // Inherit constructors from runtime_error.
  // 从 runtime_error 继承构造函数。
  using std::runtime_error::runtime_error;
};

}  // namespace exceptions
}  // namespace rclcpp

#endif  // RCLCPP__EXCEPTIONS__EXCEPTIONS_HPP_

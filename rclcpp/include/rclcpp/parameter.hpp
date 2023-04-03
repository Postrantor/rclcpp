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

#ifndef RCLCPP__PARAMETER_HPP_
#define RCLCPP__PARAMETER_HPP_

#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/parameter.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

class Parameter;

namespace node_interfaces {
struct ParameterInfo;
}  // namespace node_interfaces

namespace detail {

/**
 * @brief 这个辅助函数是必需的，因为您不能对类方法进行特化，
 *        所以我们专门针对这个模板函数进行特化，并从未特化的但依赖的类方法中调用它。
 *        This helper function is required because you cannot do specialization on a
 *        class method, so instead we specialize this template function and call it
 *        from the unspecialized, but dependent, class method.
 *
 * @tparam T 参数值类型 (Parameter value type)
 * @param parameter 指向 rclcpp::Parameter 对象的指针 (Pointer to an rclcpp::Parameter object)
 * @return 返回参数值 (Returns the parameter value)
 */
template <typename T>
auto get_value_helper(const rclcpp::Parameter* parameter);

}  // namespace detail

/// 存储任意参数的结构，带有模板化的 get/set 方法。
/// Structure to store an arbitrary parameter with templated get/set methods.
class Parameter {
public:
  /// 使用空名称和 rclcpp::PARAMETER_NOT_SET 类型的参数值构造。
  /// Construct with an empty name and a parameter value of type rclcpp::PARAMETER_NOT_SET.
  RCLCPP_PUBLIC
  Parameter();

  /// 使用给定的名称和 rclcpp::PARAMETER_NOT_SET 类型的参数值构造。
  /// Construct with given name and a parameter value of type rclcpp::PARAMETER_NOT_SET.
  RCLCPP_PUBLIC
  explicit Parameter(const std::string& name);

  /// 使用给定的名称和给定的参数值构造。
  /// Construct with given name and given parameter value.
  RCLCPP_PUBLIC
  Parameter(const std::string& name, const ParameterValue& value);

  /// 使用给定的名称和给定的参数值构造。
  /// Construct with given name and given parameter value.
  template <typename ValueTypeT>
  Parameter(const std::string& name, ValueTypeT value) : Parameter(name, ParameterValue(value)) {}

  RCLCPP_PUBLIC
  explicit Parameter(const rclcpp::node_interfaces::ParameterInfo& parameter_info);

  /// 相等运算符。
  /// Equal operator.
  RCLCPP_PUBLIC
  bool operator==(const Parameter& rhs) const;

  /// 不等运算符。
  /// Not equal operator.
  RCLCPP_PUBLIC
  bool operator!=(const Parameter& rhs) const;

  /// 获取参数的类型。
  /// Get the type of the parameter.
  RCLCPP_PUBLIC
  ParameterType get_type() const;

  /// 获取参数的类型名称。
  /// Get the type name of the parameter.
  RCLCPP_PUBLIC
  std::string get_type_name() const;

  /// 获取参数的名称。
  /// Get the name of the parameter.
  RCLCPP_PUBLIC
  const std::string& get_name() const;

  /// 以参数消息的形式获取参数值。
  /// Get value of parameter as a parameter message.
  RCLCPP_PUBLIC
  rcl_interfaces::msg::ParameterValue get_value_message() const;

  /// 获取参数值的内部存储。
  /// Get the internal storage for the parameter value.
  RCLCPP_PUBLIC
  const rclcpp::ParameterValue& get_parameter_value() const;

  /// 使用 rclcpp::ParameterType 作为模板参数获取参数值。
  /// Get value of parameter using rclcpp::ParameterType as template argument.
  /**
   * \throws rclcpp::exceptions::InvalidParameterTypeException if the type doesn't match
   */
  template <ParameterType ParamT>
  decltype(auto) get_value() const {
    return value_.get<ParamT>();
  }

  /// 使用 C++ 类型作为模板参数获取参数值。
  /// Get value of parameter using c++ types as template argument.
  template <typename T>
  decltype(auto) get_value() const;

  /// 以布尔值形式获取参数值。
  /// Get value of parameter as boolean.
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  bool as_bool() const;

  /// 以整数形式获取参数值。
  /// Get value of parameter as integer.
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  int64_t as_int() const;

  /// 以双精度浮点数形式获取参数值。
  /// Get value of parameter as double.
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  double as_double() const;

  /// 以字符串形式获取参数值。
  /// Get value of parameter as string.
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  const std::string& as_string() const;

  /// 以字节数组（vector<uint8_t>）形式获取参数值。
  /// Get value of parameter as byte array (vector<uint8_t>).
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  const std::vector<uint8_t>& as_byte_array() const;

  /// 以布尔数组（vector<bool>）形式获取参数值。
  /// Get value of parameter as bool array (vector<bool>).
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  const std::vector<bool>& as_bool_array() const;

  /// 以整数数组（vector<int64_t>）形式获取参数值。
  /// Get value of parameter as integer array (vector<int64_t>).
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  const std::vector<int64_t>& as_integer_array() const;

  /// 以双精度浮点数数组（vector<double>）形式获取参数值。
  /// Get value of parameter as double array (vector<double>).
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  const std::vector<double>& as_double_array() const;

  /// 以字符串数组（vector<std::string>）形式获取参数值。
  /// Get value of parameter as string array (vector<std::string>).
  /**
   * \throws rclcpp::ParameterTypeException if the type doesn't match
   */
  RCLCPP_PUBLIC
  const std::vector<std::string>& as_string_array() const;

  /// 将参数消息转换为 Parameter 类对象。
  /// Convert a parameter message in a Parameter class object.
  RCLCPP_PUBLIC
  static Parameter from_parameter_msg(const rcl_interfaces::msg::Parameter& parameter);

  /// 将类转换为参数消息。
  /// Convert the class in a parameter message.
  RCLCPP_PUBLIC
  rcl_interfaces::msg::Parameter to_parameter_msg() const;

  /// 以字符串形式获取参数值。
  /// Get value of parameter as a string.
  RCLCPP_PUBLIC
  std::string value_to_string() const;

private:
  std::string name_;
  ParameterValue value_;
};

/// 返回一个用于字典的参数的 JSON 编码版本。
/// Return a json encoded version of the parameter intended for a dict.
RCLCPP_PUBLIC
std::string _to_json_dict_entry(const Parameter& param);

/// 输出运算符重载，将 rclcpp::Parameter 对象输出到输出流中。
/// Overload the output operator to output an rclcpp::Parameter object to the output stream.
RCLCPP_PUBLIC
std::ostream& operator<<(std::ostream& os, const rclcpp::Parameter& pv);

/// 输出运算符重载，将一组 rclcpp::Parameter 对象输出到输出流中。
/// Overload the output operator to output a group of rclcpp::Parameter objects to the output
/// stream.
RCLCPP_PUBLIC
std::ostream& operator<<(std::ostream& os, const std::vector<Parameter>& parameters);

namespace detail {

// 获取参数值的辅助函数，用于从 rclcpp::Parameter 对象中获取指定类型的值。
// Helper function to get the parameter value, used to get the specified type of value from the
// rclcpp::Parameter object.
template <typename T>
auto get_value_helper(const rclcpp::Parameter* parameter) {
  return parameter->get_parameter_value().get<T>();
}

// 专门化，允许 Parameter::get() 返回参数值对象的 const 引用。
// Specialization allowing Parameter::get() to return a const ref to the parameter value object.
template <>
inline auto get_value_helper<rclcpp::ParameterValue>(const rclcpp::Parameter* parameter) {
  return parameter->get_parameter_value();
}

// 专门化，允许 Parameter::get() 返回参数本身的 const 引用。
// Specialization allowing Parameter::get() to return a const ref to the parameter itself.
template <>
inline auto get_value_helper<rclcpp::Parameter>(const rclcpp::Parameter* parameter) {
  // 使用此 lambda 确保返回的是 const 引用（而不是副本）。
  // Use this lambda to ensure it's a const reference being returned (and not a copy).
  auto type_enforcing_lambda = [&parameter]() -> const rclcpp::Parameter& { return *parameter; };
  return type_enforcing_lambda();
}

}  // namespace detail

/// \cond
// 获取参数值的函数模板，根据指定的类型获取参数值。
// Function template to get the parameter value, getting the parameter value according to the
// specified type.
template <typename T>
decltype(auto) Parameter::get_value() const {
  try {
    // 使用辅助函数专门处理 ParameterValue 和 Parameter 的情况。
    // Use the helper to specialize for the ParameterValue and Parameter cases.
    return detail::get_value_helper<T>(this);
  } catch (const ParameterTypeException& ex) {
    throw exceptions::InvalidParameterTypeException(this->name_, ex.what());
  }
}
/// \endcond

}  // namespace rclcpp

namespace std {

/// 返回一个用于列表的参数的 JSON 编码版本。
/// Return a json encoded version of the parameter intended for a list.
RCLCPP_PUBLIC
std::string to_string(const rclcpp::Parameter& param);

/// 返回一组参数的 JSON 编码版本，作为字符串。
/// Return a json encoded version of a vector of parameters, as a string.
RCLCPP_PUBLIC
std::string to_string(const std::vector<rclcpp::Parameter>& parameters);

}  // namespace std

#endif  // RCLCPP__PARAMETER_HPP_

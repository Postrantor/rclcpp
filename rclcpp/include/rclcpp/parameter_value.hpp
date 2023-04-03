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

#ifndef RCLCPP__PARAMETER_VALUE_HPP_
#define RCLCPP__PARAMETER_VALUE_HPP_

#include <exception>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/**
 * @brief 枚举 ParameterType 定义了不同的参数类型 (Enumeration for different parameter types)
 */
enum ParameterType : uint8_t {
  // 参数未设置时的默认值 (Default value when the parameter is not set)
  PARAMETER_NOT_SET =
      rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET,  //!< 未设置参数 (Not set parameter)
  // 布尔型参数 (Boolean parameter)
  PARAMETER_BOOL =
      rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,  //!< 布尔型参数 (Boolean parameter)
  // 整数型参数 (Integer parameter)
  PARAMETER_INTEGER =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,  //!< 整数型参数 (Integer parameter)
  // 双精度浮点型参数 (Double precision floating point parameter)
  PARAMETER_DOUBLE =
      rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,  //!< 双精度浮点型参数 (Double precision
                                                             //!< floating point parameter)
  // 字符串型参数 (String parameter)
  PARAMETER_STRING =
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING,  //!< 字符串型参数 (String parameter)
  // 字节数组型参数 (Byte array parameter)
  PARAMETER_BYTE_ARRAY =
      rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY,  //!< 字节数组型参数 (Byte array
                                                                 //!< parameter)
  // 布尔数组型参数 (Boolean array parameter)
  PARAMETER_BOOL_ARRAY =
      rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY,  //!< 布尔数组型参数 (Boolean array
                                                                 //!< parameter)
  // 整数数组型参数 (Integer array parameter)
  PARAMETER_INTEGER_ARRAY =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY,  //!< 整数数组型参数 (Integer
                                                                    //!< array parameter)
  // 双精度浮点数组型参数 (Double precision floating point array parameter)
  PARAMETER_DOUBLE_ARRAY =
      rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY,  //!< 双精度浮点数组型参数 (Double
                                                                   //!< precision floating point
                                                                   //!< array parameter)
  // 字符串数组型参数 (String array parameter)
  PARAMETER_STRING_ARRAY =
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,  //!< 字符串数组型参数 (String
                                                                   //!< array parameter)
};

/// 返回参数类型的名称 (Return the name of a parameter type)
RCLCPP_PUBLIC
std::string to_string(ParameterType type);

/// 重载 << 运算符，用于输出参数类型 (Overload the << operator for outputting parameter type)
RCLCPP_PUBLIC
std::ostream &operator<<(std::ostream &os, ParameterType type);

/// 表示参数类型与预期类型不匹配的异常类 (Exception class indicating the parameter type does not
/// match the expected type)
class ParameterTypeException : public std::runtime_error {
public:
  /// 构造函数 (Constructor)
  /**
   * \param[in] expected 预期的参数类型 (The expected parameter type)
   * \param[in] actual 实际的参数类型 (The actual parameter type)
   */
  RCLCPP_PUBLIC
  ParameterTypeException(ParameterType expected, ParameterType actual)
      // 使用预期和实际的参数类型构造异常信息 (Construct the exception message using the expected
      // and actual parameter types)
      : std::runtime_error(
            "expected [" + to_string(expected) + "] got [" + to_string(actual) + "]") {}
};

/// \brief 存储参数的类型和值 (Store the type and value of a parameter.)
class ParameterValue {
public:
  /// \brief 使用 PARAMETER_NOT_SET 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_NOT_SET.)
  RCLCPP_PUBLIC
  ParameterValue();

  /// \brief 使用消息构造一个参数值 (Construct a parameter value from a message.)
  /// \param[in] value 参数值的消息形式 (The message form of the parameter value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const rcl_interfaces::msg::ParameterValue &value);

  /// \brief 使用 PARAMETER_BOOL 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_BOOL.) \param[in] bool_value 布尔值 (The boolean value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const bool bool_value);

  /// \brief 使用 PARAMETER_INTEGER 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_INTEGER.) \param[in] int_value 整数值 (The integer value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const int int_value);

  /// \brief 使用 PARAMETER_INTEGER 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_INTEGER.) \param[in] int_value 64位整数值 (The 64-bit integer value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const int64_t int_value);

  /// \brief 使用 PARAMETER_DOUBLE 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_DOUBLE.) \param[in] double_value 单精度浮点值 (The single-precision floating-point
  /// value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const float double_value);

  /// \brief 使用 PARAMETER_DOUBLE 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_DOUBLE.) \param[in] double_value 双精度浮点值 (The double-precision floating-point
  /// value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const double double_value);

  /// \brief 使用 PARAMETER_STRING 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_STRING.) \param[in] string_value 字符串值 (The string value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::string &string_value);

  /// \brief 使用 PARAMETER_STRING 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_STRING.) \param[in] string_value C字符串值 (The C-style string value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const char *string_value);

  /// \brief 使用 PARAMETER_BYTE_ARRAY 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_BYTE_ARRAY.) \param[in] byte_array_value 字节数组值 (The byte array value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<uint8_t> &byte_array_value);

  /// \brief 使用 PARAMETER_BOOL_ARRAY 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_BOOL_ARRAY.) \param[in] bool_array_value 布尔数组值 (The boolean array value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<bool> &bool_array_value);

  /// \brief 使用 PARAMETER_INTEGER_ARRAY 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_INTEGER_ARRAY.) \param[in] int_array_value 整数数组值 (The integer array value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<int> &int_array_value);

  /// \brief 使用 PARAMETER_INTEGER_ARRAY 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_INTEGER_ARRAY.) \param[in] int_array_value 64位整数数组值 (The 64-bit integer array
  /// value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<int64_t> &int_array_value);

  /// \brief 使用 PARAMETER_DOUBLE_ARRAY 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_DOUBLE_ARRAY.) \param[in] double_array_value 单精度浮点数组值 (The single-precision
  /// floating-point array value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<float> &double_array_value);

  /// \brief 使用 PARAMETER_DOUBLE_ARRAY 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_DOUBLE_ARRAY.) \param[in] double_array_value 双精度浮点数组值 (The double-precision
  /// floating-point array value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<double> &double_array_value);

  /// \brief 使用 PARAMETER_STRING_ARRAY 类型构造一个参数值 (Construct a parameter value with type
  /// PARAMETER_STRING_ARRAY.) \param[in] string_array_value 字符串数组值 (The string array value.)
  RCLCPP_PUBLIC
  explicit ParameterValue(const std::vector<std::string> &string_array_value);

  /// \brief 返回一个枚举值，表示设置的值的类型。(Return an enum indicating the type of the set
  /// value.) \return ParameterType 枚举值。(An enumeration value of type ParameterType.)
  RCLCPP_PUBLIC
  ParameterType get_type() const;

  /// \brief 将参数值填充到消息中并返回。(Return a message populated with the parameter value.)
  /// \return rcl_interfaces::msg::ParameterValue 消息结构体，包含参数值。(A message structure of
  /// type rcl_interfaces::msg::ParameterValue containing the parameter value.)
  RCLCPP_PUBLIC
  rcl_interfaces::msg::ParameterValue to_value_msg() const;

  /// \brief 等于操作符。(Equal operator.)
  /// \param[in] rhs 另一个 ParameterValue 对象，用于比较。(The other ParameterValue object to
  /// compare against.) \return bool 如果两个 ParameterValue 对象相等，则返回 true，否则返回
  /// false。(Returns true if both ParameterValue objects are equal, otherwise returns false.)
  RCLCPP_PUBLIC
  bool operator==(const ParameterValue &rhs) const;

  /// \brief 不等于操作符。(Not equal operator.)
  /// \param[in] rhs 另一个 ParameterValue 对象，用于比较。(The other ParameterValue object to
  /// compare against.) \return bool 如果两个 ParameterValue 对象不相等，则返回 true，否则返回
  /// false。(Returns true if both ParameterValue objects are not equal, otherwise returns false.)
  RCLCPP_PUBLIC
  bool operator!=(const ParameterValue &rhs) const;

  /**
   * @brief 获取参数值的模板函数，根据参数类型返回对应的参数值引用 (Template function to get the
   * parameter value, returns a reference to the corresponding parameter value based on the
   * parameter type)
   *
   * @tparam type 参数类型 (Parameter type)
   * @return 对应参数类型的引用 (Reference of the corresponding parameter type)
   * @throws ParameterTypeException 当请求的参数类型与实际参数类型不匹配时抛出异常 (Throws an
   * exception when the requested parameter type does not match the actual parameter type)
   */
  template <ParameterType type>
  constexpr typename std::enable_if<type == ParameterType::PARAMETER_BOOL, const bool &>::type get()
      const {
    // 检查参数类型是否为布尔类型 (Check if the parameter type is boolean)
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      throw ParameterTypeException(ParameterType::PARAMETER_BOOL, get_type());
    }
    // 返回布尔类型参数值的引用 (Return the reference to the boolean parameter value)
    return value_.bool_value;
  }

  template <ParameterType type>
  constexpr typename std::enable_if<type == ParameterType::PARAMETER_INTEGER, const int64_t &>::type
  get() const {
    // 检查参数类型是否为整数类型 (Check if the parameter type is integer)
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      throw ParameterTypeException(ParameterType::PARAMETER_INTEGER, get_type());
    }
    // 返回整数类型参数值的引用 (Return the reference to the integer parameter value)
    return value_.integer_value;
  }

  template <ParameterType type>
  constexpr typename std::enable_if<type == ParameterType::PARAMETER_DOUBLE, const double &>::type
  get() const {
    // 检查参数类型是否为双精度浮点类型 (Check if the parameter type is double)
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      throw ParameterTypeException(ParameterType::PARAMETER_DOUBLE, get_type());
    }
    // 返回双精度浮点类型参数值的引用 (Return the reference to the double parameter value)
    return value_.double_value;
  }

  template <ParameterType type>
  constexpr
      typename std::enable_if<type == ParameterType::PARAMETER_STRING, const std::string &>::type
      get() const {
    // 检查参数类型是否为字符串类型 (Check if the parameter type is string)
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      throw ParameterTypeException(ParameterType::PARAMETER_STRING, get_type());
    }
    // 返回字符串类型参数值的引用 (Return the reference to the string parameter value)
    return value_.string_value;
  }

  template <ParameterType type>
  constexpr typename std::
      enable_if<type == ParameterType::PARAMETER_BYTE_ARRAY, const std::vector<uint8_t> &>::type
      get() const {
    // 检查参数类型是否为字节数组类型 (Check if the parameter type is byte array)
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY) {
      throw ParameterTypeException(ParameterType::PARAMETER_BYTE_ARRAY, get_type());
    }
    // 返回字节数组类型参数值的引用 (Return the reference to the byte array parameter value)
    return value_.byte_array_value;
  }

  template <ParameterType type>
  constexpr typename std::
      enable_if<type == ParameterType::PARAMETER_BOOL_ARRAY, const std::vector<bool> &>::type
      get() const {
    // 检查参数类型是否为布尔数组类型 (Check if the parameter type is boolean array)
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY) {
      throw ParameterTypeException(ParameterType::PARAMETER_BOOL_ARRAY, get_type());
    }
    // 返回布尔数组类型参数值的引用 (Return the reference to the boolean array parameter value)
    return value_.bool_array_value;
  }

  template <ParameterType type>
  constexpr typename std::
      enable_if<type == ParameterType::PARAMETER_INTEGER_ARRAY, const std::vector<int64_t> &>::type
      get() const {
    // 检查参数类型是否为整数数组类型 (Check if the parameter type is integer array)
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY) {
      throw ParameterTypeException(ParameterType::PARAMETER_INTEGER_ARRAY, get_type());
    }
    // 返回整数数组类型参数值的引用 (Return the reference to the integer array parameter value)
    return value_.integer_array_value;
  }

  template <ParameterType type>
  constexpr typename std::
      enable_if<type == ParameterType::PARAMETER_DOUBLE_ARRAY, const std::vector<double> &>::type
      get() const {
    // 检查参数类型是否为双精度浮点数组类型 (Check if the parameter type is double array)
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      throw ParameterTypeException(ParameterType::PARAMETER_DOUBLE_ARRAY, get_type());
    }
    // 返回双精度浮点数组类型参数值的引用 (Return the reference to the double array parameter value)
    return value_.double_array_value;
  }

  template <ParameterType type>
  constexpr typename std::enable_if<
      type == ParameterType::PARAMETER_STRING_ARRAY,
      const std::vector<std::string> &>::type
  get() const {
    // 检查参数类型是否为字符串数组类型 (Check if the parameter type is string array)
    if (value_.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY) {
      throw ParameterTypeException(ParameterType::PARAMETER_STRING_ARRAY, get_type());
    }
    // 返回字符串数组类型参数值的引用 (Return the reference to the string array parameter value)
    return value_.string_array_value;
  }

  /**
   * @brief 获取指定类型的参数值 (Get the parameter value of the specified type)
   *
   * @tparam type 指定的参数类型 (The specified parameter type)
   * @return 返回指定类型的参数值引用 (Return a reference to the parameter value of the specified
   * type)
   */
  // 以下 get() 变体允许使用基本类型 (The following get() variants allow the use of primitive types)

  template <typename type>
  constexpr typename std::enable_if<std::is_same<type, bool>::value, const bool &>::type get()
      const {
    // 如果类型为 bool，则返回 PARAMETER_BOOL 类型的参数值 (If the type is bool, return the
    // parameter value of PARAMETER_BOOL type)
    return get<ParameterType::PARAMETER_BOOL>();
  }

  template <typename type>
  constexpr typename std::enable_if<
      std::is_integral<type>::value && !std::is_same<type, bool>::value,
      const int64_t &>::type
  get() const {
    // 如果类型为整数且不是 bool，则返回 PARAMETER_INTEGER 类型的参数值 (If the type is an integer
    // and not bool, return the parameter value of PARAMETER_INTEGER type)
    return get<ParameterType::PARAMETER_INTEGER>();
  }

  template <typename type>
  constexpr typename std::enable_if<std::is_floating_point<type>::value, const double &>::type get()
      const {
    // 如果类型为浮点数，则返回 PARAMETER_DOUBLE 类型的参数值 (If the type is floating point, return
    // the parameter value of PARAMETER_DOUBLE type)
    return get<ParameterType::PARAMETER_DOUBLE>();
  }

  template <typename type>
  constexpr typename std::
      enable_if<std::is_convertible<type, std::string>::value, const std::string &>::type
      get() const {
    // 如果类型可转换为 std::string，则返回 PARAMETER_STRING 类型的参数值 (If the type is
    // convertible to std::string, return the parameter value of PARAMETER_STRING type)
    return get<ParameterType::PARAMETER_STRING>();
  }

  template <typename type>
  constexpr typename std::enable_if<
      std::is_convertible<type, const std::vector<uint8_t> &>::value,
      const std::vector<uint8_t> &>::type
  get() const {
    // 如果类型可转换为 std::vector<uint8_t>，则返回 PARAMETER_BYTE_ARRAY 类型的参数值 (If the type
    // is convertible to std::vector<uint8_t>, return the parameter value of PARAMETER_BYTE_ARRAY
    // type)
    return get<ParameterType::PARAMETER_BYTE_ARRAY>();
  }

  template <typename type>
  constexpr typename std::enable_if<
      std::is_convertible<type, const std::vector<bool> &>::value,
      const std::vector<bool> &>::type
  get() const {
    // 如果类型可转换为 std::vector<bool>，则返回 PARAMETER_BOOL_ARRAY 类型的参数值 (If the type is
    // convertible to std::vector<bool>, return the parameter value of PARAMETER_BOOL_ARRAY type)
    return get<ParameterType::PARAMETER_BOOL_ARRAY>();
  }

  template <typename type>
  constexpr typename std::enable_if<
      std::is_convertible<type, const std::vector<int> &>::value,
      const std::vector<int64_t> &>::type
  get() const {
    // 如果类型可转换为 std::vector<int>，则返回 PARAMETER_INTEGER_ARRAY 类型的参数值 (If the type
    // is convertible to std::vector<int>, return the parameter value of PARAMETER_INTEGER_ARRAY
    // type)
    return get<ParameterType::PARAMETER_INTEGER_ARRAY>();
  }

  template <typename type>
  constexpr typename std::enable_if<
      std::is_convertible<type, const std::vector<int64_t> &>::value,
      const std::vector<int64_t> &>::type
  get() const {
    // 如果类型可转换为 std::vector<int64_t>，则返回 PARAMETER_INTEGER_ARRAY 类型的参数值 (If the
    // type is convertible to std::vector<int64_t>, return the parameter value of
    // PARAMETER_INTEGER_ARRAY type)
    return get<ParameterType::PARAMETER_INTEGER_ARRAY>();
  }

  template <typename type>
  constexpr typename std::enable_if<
      std::is_convertible<type, const std::vector<float> &>::value,
      const std::vector<double> &>::type
  get() const {
    // 如果类型可转换为 std::vector<float>，则返回 PARAMETER_DOUBLE_ARRAY 类型的参数值 (If the type
    // is convertible to std::vector<float>, return the parameter value of PARAMETER_DOUBLE_ARRAY
    // type)
    return get<ParameterType::PARAMETER_DOUBLE_ARRAY>();
  }

  template <typename type>
  constexpr typename std::enable_if<
      std::is_convertible<type, const std::vector<double> &>::value,
      const std::vector<double> &>::type
  get() const {
    // 如果类型可转换为 std::vector<double>，则返回 PARAMETER_DOUBLE_ARRAY 类型的参数值 (If the type
    // is convertible to std::vector<double>, return the parameter value of PARAMETER_DOUBLE_ARRAY
    // type)
    return get<ParameterType::PARAMETER_DOUBLE_ARRAY>();
  }

  template <typename type>
  constexpr typename std::enable_if<
      std::is_convertible<type, const std::vector<std::string> &>::value,
      const std::vector<std::string> &>::type
  get() const {
    // 如果类型可转换为 std::vector<std::string>，则返回 PARAMETER_STRING_ARRAY 类型的参数值 (If the
    // type is convertible to std::vector<std::string>, return the parameter value of
    // PARAMETER_STRING_ARRAY type)
    return get<ParameterType::PARAMETER_STRING_ARRAY>();
  }

private:
  rcl_interfaces::msg::ParameterValue value_;
};

/// Return the value of a parameter as a string
RCLCPP_PUBLIC
std::string to_string(const ParameterValue &type);

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_VALUE_HPP_

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

#include "rclcpp/parameter_value.hpp"

#include <string>
#include <vector>

using rclcpp::ParameterType;
using rclcpp::ParameterValue;

/**
 * @brief 将 ParameterType 转换为字符串表示形式 (Converts a ParameterType to its string
 * representation)
 *
 * @param type ParameterType 枚举值 (The ParameterType enumeration value)
 * @return std::string 参数类型的字符串表示 (String representation of the parameter type)
 */
std::string rclcpp::to_string(const ParameterType type) {
  // 使用 switch 语句处理不同的参数类型 (Using a switch statement to handle different parameter
  // types)
  switch (type) {
    case ParameterType::PARAMETER_NOT_SET:
      // 参数未设置 (Parameter not set)
      return "not set";
    case ParameterType::PARAMETER_BOOL:
      // 布尔类型 (Boolean type)
      return "bool";
    case ParameterType::PARAMETER_INTEGER:
      // 整数类型 (Integer type)
      return "integer";
    case ParameterType::PARAMETER_DOUBLE:
      // 双精度浮点类型 (Double-precision floating-point type)
      return "double";
    case ParameterType::PARAMETER_STRING:
      // 字符串类型 (String type)
      return "string";
    case ParameterType::PARAMETER_BYTE_ARRAY:
      // 字节数组类型 (Byte array type)
      return "byte_array";
    case ParameterType::PARAMETER_BOOL_ARRAY:
      // 布尔数组类型 (Boolean array type)
      return "bool_array";
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      // 整数数组类型 (Integer array type)
      return "integer_array";
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      // 双精度浮点数组类型 (Double-precision floating-point array type)
      return "double_array";
    case ParameterType::PARAMETER_STRING_ARRAY:
      // 字符串数组类型 (String array type)
      return "string_array";
    default:
      // 未知类型 (Unknown type)
      return "unknown type";
  }
}

/**
 * @brief 重载 << 运算符，用于将 ParameterType 输出到 std::ostream (Overloads the << operator to
 * output a ParameterType to an std::ostream)
 *
 * @param os 输出流 (The output stream)
 * @param type 参数类型 (The parameter type)
 * @return std::ostream& 更新后的输出流 (The updated output stream)
 */
std::ostream& rclcpp::operator<<(std::ostream& os, const ParameterType type) {
  // 使用 to_string 函数将参数类型转换为字符串表示形式，并输出到流中 (Using the to_string function
  // to convert the parameter type to its string representation and output it to the stream)
  os << rclcpp::to_string(type);
  return os;
}

/**
 * @brief 将给定数组转换为字符串表示形式 (Converts a given array to its string representation)
 *
 * @tparam ValType 数组值类型 (Array value type)
 * @tparam PrintType 打印类型 (Print type)
 * @param array 要转换为字符串的数组 (The array to be converted to a string)
 * @param format_flags 格式标志 (Format flags)
 * @return std::string 数组的字符串表示形式 (String representation of the array)
 */
template <typename ValType, typename PrintType = ValType>
std::string array_to_string(
    const std::vector<ValType>& array, const std::ios::fmtflags format_flags = std::ios::dec) {
  // 创建一个 std::stringstream 对象来存储数组的字符串表示形式 (Create a std::stringstream object to
  // store the string representation of the array)
  std::stringstream type_array;
  bool first_item = true;
  // 添加数组开始的 "[" 符号 (Add the "[" symbol at the beginning of the array)
  type_array << "[";

  // 设置格式标志 (Set format flags)
  type_array.setf(format_flags, std::ios_base::basefield | std::ios::boolalpha);
  type_array << std::showbase;

  // 遍历数组中的每个值 (Iterate through each value in the array)
  for (const ValType& value : array) {
    // 如果不是第一个元素，添加逗号和空格作为分隔符 (If not the first element, add a comma and space
    // as separators)
    if (!first_item) {
      type_array << ", ";
    } else {
      first_item = false;
    }
    // 将值添加到 stringstream 中 (Add the value to the stringstream)
    type_array << static_cast<PrintType>(value);
  }

  // 添加数组结束的 "]" 符号 (Add the "]" symbol at the end of the array)
  type_array << "]";
  return type_array.str();
}

/**
 * @brief 将 ParameterValue 对象转换为字符串表示形式 (Converts a ParameterValue object to its string
 * representation)
 *
 * @param value 要转换的 ParameterValue 对象 (The ParameterValue object to be converted)
 * @return std::string 参数值的字符串表示形式 (String representation of the parameter value)
 */
std::string rclcpp::to_string(const ParameterValue& value) {
  // 根据参数值的类型进行处理 (Process according to the type of parameter value)
  switch (value.get_type()) {
    // 参数未设置 (Parameter not set)
    case ParameterType::PARAMETER_NOT_SET:
      return "not set";
    // 布尔型参数 (Boolean parameter)
    case ParameterType::PARAMETER_BOOL:
      return value.get<bool>() ? "true" : "false";
    // 整数型参数 (Integer parameter)
    case ParameterType::PARAMETER_INTEGER:
      return std::to_string(value.get<int>());
    // 双精度浮点型参数 (Double-precision floating-point parameter)
    case ParameterType::PARAMETER_DOUBLE:
      return std::to_string(value.get<double>());
    // 字符串型参数 (String parameter)
    case ParameterType::PARAMETER_STRING:
      return value.get<std::string>();
    // 字节型数组参数 (Byte array parameter)
    case ParameterType::PARAMETER_BYTE_ARRAY:
      return array_to_string<uint8_t, int>(value.get<std::vector<uint8_t>>(), std::ios::hex);
    // 布尔型数组参数 (Boolean array parameter)
    case ParameterType::PARAMETER_BOOL_ARRAY:
      return array_to_string(value.get<std::vector<bool>>(), std::ios::boolalpha);
    // 整数型数组参数 (Integer array parameter)
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      return array_to_string(value.get<std::vector<int64_t>>());
    // 双精度浮点型数组参数 (Double-precision floating-point array parameter)
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      return array_to_string(value.get<std::vector<double>>());
    // 字符串型数组参数 (String array parameter)
    case ParameterType::PARAMETER_STRING_ARRAY:
      return array_to_string(value.get<std::vector<std::string>>());
    // 未知类型参数 (Unknown type parameter)
    default:
      return "unknown type";
  }
}

/**
 * @brief 默认构造函数，将类型设置为 PARAMETER_NOT_SET (Default constructor, sets the type to
 * PARAMETER_NOT_SET)
 */
ParameterValue::ParameterValue() {
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
}

/**
 * @brief 使用给定的 rcl_interfaces::msg::ParameterValue 对象初始化 ParameterValue 类的实例
 * (Initializes an instance of the ParameterValue class with the given
 * rcl_interfaces::msg::ParameterValue object)
 *
 * @param value 要使用的 rcl_interfaces::msg::ParameterValue 对象 (The
 * rcl_interfaces::msg::ParameterValue object to be used)
 */
ParameterValue::ParameterValue(const rcl_interfaces::msg::ParameterValue& value) {
  value_ = value;
  // 根据参数值的类型进行处理 (Process according to the type of parameter value)
  switch (value.type) {
    case PARAMETER_BOOL:
    case PARAMETER_INTEGER:
    case PARAMETER_DOUBLE:
    case PARAMETER_STRING:
    case PARAMETER_BYTE_ARRAY:
    case PARAMETER_BOOL_ARRAY:
    case PARAMETER_INTEGER_ARRAY:
    case PARAMETER_DOUBLE_ARRAY:
    case PARAMETER_STRING_ARRAY:
    case PARAMETER_NOT_SET:
      break;
    // 未知类型参数 (Unknown type parameter)
    default:
      // TODO(wjwwood): 使用自定义异常 (Use custom exception)
      throw std::runtime_error("Unknown type: " + std::to_string(value.type));
  }
}

/**
 * @brief 构造函数，用于创建一个布尔类型的参数值对象 (Constructor for creating a boolean parameter
 * value object)
 * @param bool_value 布尔值 (Boolean value)
 */
ParameterValue::ParameterValue(const bool bool_value) {
  // 将布尔值存储在 value_ 结构体中 (Store the boolean value in the value_ struct)
  value_.bool_value = bool_value;
  // 设置参数类型为布尔类型 (Set the parameter type to boolean)
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
}

/**
 * @brief 构造函数，用于创建一个整数类型的参数值对象 (Constructor for creating an integer parameter
 * value object)
 * @param int_value 整数值 (Integer value)
 */
ParameterValue::ParameterValue(const int int_value) {
  // 将整数值存储在 value_ 结构体中 (Store the integer value in the value_ struct)
  value_.integer_value = int_value;
  // 设置参数类型为整数类型 (Set the parameter type to integer)
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
}

/**
 * @brief 构造函数，用于创建一个 64 位整数类型的参数值对象 (Constructor for creating a 64-bit
 * integer parameter value object)
 * @param int_value 64 位整数值 (64-bit integer value)
 */
ParameterValue::ParameterValue(const int64_t int_value) {
  // 将 64 位整数值存储在 value_ 结构体中 (Store the 64-bit integer value in the value_ struct)
  value_.integer_value = int_value;
  // 设置参数类型为整数类型 (Set the parameter type to integer)
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
}

/**
 * @brief 构造函数，用于创建一个浮点数类型的参数值对象 (Constructor for creating a floating-point
 * parameter value object)
 * @param double_value 浮点数值 (Floating-point value)
 */
ParameterValue::ParameterValue(const float double_value) {
  // 将浮点数值存储在 value_ 结构体中，并将其转换为双精度类型 (Store the floating-point value in the
  // value_ struct, and convert it to double type)
  value_.double_value = static_cast<double>(double_value);
  // 设置参数类型为双精度浮点数类型 (Set the parameter type to double floating-point)
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
}

/**
 * @brief 构造函数，用于创建一个双精度浮点数类型的参数值对象 (Constructor for creating a double
 * floating-point parameter value object)
 * @param double_value 双精度浮点数值 (Double floating-point value)
 */
ParameterValue::ParameterValue(const double double_value) {
  // 将双精度浮点数值存储在 value_ 结构体中 (Store the double floating-point value in the value_
  // struct)
  value_.double_value = double_value;
  // 设置参数类型为双精度浮点数类型 (Set the parameter type to double floating-point)
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
}

/**
 * @brief 构造函数，用于创建一个字符串类型的参数值对象 (Constructor for creating a string parameter
 * value object)
 * @param string_value 字符串值 (String value)
 */
ParameterValue::ParameterValue(const std::string& string_value) {
  value_.string_value = string_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
}

/// \brief 构造函数，用于将字符串转换为ParameterValue对象
/// \param string_value 输入的字符串值
/// Constructor that converts a string to a ParameterValue object
/// \param string_value The input string value
ParameterValue::ParameterValue(const char* string_value)
    : ParameterValue(std::string(string_value)) {}

/// \brief 构造函数，用于将uint8_t向量转换为ParameterValue对象
/// \param byte_array_value 输入的uint8_t向量值
/// Constructor that converts a vector of uint8_t to a ParameterValue object
/// \param byte_array_value The input vector of uint8_t values
ParameterValue::ParameterValue(const std::vector<uint8_t>& byte_array_value) {
  value_.byte_array_value = byte_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY;
}

/// \brief 构造函数，用于将布尔向量转换为ParameterValue对象
/// \param bool_array_value 输入的布尔向量值
/// Constructor that converts a vector of bools to a ParameterValue object
/// \param bool_array_value The input vector of bool values
ParameterValue::ParameterValue(const std::vector<bool>& bool_array_value) {
  value_.bool_array_value = bool_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
}

/// \brief 构造函数，用于将int向量转换为ParameterValue对象
/// \param int_array_value 输入的int向量值
/// Constructor that converts a vector of ints to a ParameterValue object
/// \param int_array_value The input vector of int values
ParameterValue::ParameterValue(const std::vector<int>& int_array_value) {
  value_.integer_array_value.assign(int_array_value.cbegin(), int_array_value.cend());
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
}

/// \brief 构造函数，用于将int64_t向量转换为ParameterValue对象
/// \param int_array_value 输入的int64_t向量值
/// Constructor that converts a vector of int64_t to a ParameterValue object
/// \param int_array_value The input vector of int64_t values
ParameterValue::ParameterValue(const std::vector<int64_t>& int_array_value) {
  value_.integer_array_value = int_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
}

/// \brief 构造函数，用于将float向量转换为ParameterValue对象
/// \param float_array_value 输入的float向量值
/// Constructor that converts a vector of floats to a ParameterValue object
/// \param float_array_value The input vector of float values
ParameterValue::ParameterValue(const std::vector<float>& float_array_value) {
  value_.double_array_value.assign(float_array_value.cbegin(), float_array_value.cend());
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
}

/// \brief 构造函数，用于将double向量转换为ParameterValue对象
/// \param double_array_value 输入的double向量值
/// Constructor that converts a vector of doubles to a ParameterValue object
/// \param double_array_value The input vector of double values
ParameterValue::ParameterValue(const std::vector<double>& double_array_value) {
  value_.double_array_value = double_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
}

/// \brief 构造函数，用于将字符串向量转换为ParameterValue对象
/// \param string_array_value 输入的字符串向量值
/// Constructor that converts a vector of strings to a ParameterValue object
/// \param string_array_value The input vector of string values
ParameterValue::ParameterValue(const std::vector<std::string>& string_array_value) {
  value_.string_array_value = string_array_value;
  value_.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
}

/// \brief 获取参数类型
/// \return 参数类型
/// Get the parameter type
/// \return The parameter type
ParameterType ParameterValue::get_type() const { return static_cast<ParameterType>(value_.type); }

/// \brief 将ParameterValue对象转换为rcl_interfaces::msg::ParameterValue消息
/// \return 转换后的消息
/// Converts the ParameterValue object to an rcl_interfaces::msg::ParameterValue message
/// \return The converted message
rcl_interfaces::msg::ParameterValue ParameterValue::to_value_msg() const { return value_; }

/// \brief 比较两个ParameterValue对象是否相等
/// \param rhs 另一个要比较的ParameterValue对象
/// \return 如果相等则返回true，否则返回false
/// Compares two ParameterValue objects for equality
/// \param rhs Another ParameterValue object to compare with
/// \return true if equal, false otherwise
bool ParameterValue::operator==(const ParameterValue& rhs) const {
  return this->value_ == rhs.value_;
}

/// \brief 比较两个ParameterValue对象是否不相等
/// \param rhs 另一个要比较的ParameterValue对象
/// \return 如果不相等则返回true，否则返回false
/// Compares two ParameterValue objects for inequality
/// \param rhs Another ParameterValue object to compare with
/// \return true if not equal, false otherwise
bool ParameterValue::operator!=(const ParameterValue& rhs) const {
  return this->value_ != rhs.value_;
}

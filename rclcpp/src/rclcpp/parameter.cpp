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

#include "rclcpp/parameter.hpp"

#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/node_interfaces/node_parameters.hpp"
#include "rclcpp/utilities.hpp"

using rclcpp::Parameter;
using rclcpp::ParameterType;

/**
 * @brief 默认构造函数，用于创建一个空的参数对象 (Default constructor, used to create an empty
 * parameter object)
 */
Parameter::Parameter() : name_("") {}

/**
 * @brief 构造函数，用于通过名称创建一个参数对象 (Constructor, used to create a parameter object
 * with a given name)
 * @param name 参数的名称 (Name of the parameter)
 */
Parameter::Parameter(const std::string& name) : name_(name), value_() {}

/**
 * @brief 构造函数，用于通过名称和值创建一个参数对象 (Constructor, used to create a parameter object
 * with a given name and value)
 * @param name 参数的名称 (Name of the parameter)
 * @param value 参数的值 (Value of the parameter)
 */
Parameter::Parameter(const std::string& name, const rclcpp::ParameterValue& value)
    : name_(name), value_(value) {}

/**
 * @brief 构造函数，用于通过 ParameterInfo 对象创建一个参数对象 (Constructor, used to create a
 * parameter object with a given ParameterInfo object)
 * @param parameter_info 包含参数描述和值的 ParameterInfo 对象 (ParameterInfo object containing the
 * parameter descriptor and value)
 */
Parameter::Parameter(const rclcpp::node_interfaces::ParameterInfo& parameter_info)
    : Parameter(parameter_info.descriptor.name, parameter_info.value) {}

/**
 * @brief 判断两个参数对象是否相等 (Determine if two parameter objects are equal)
 * @param rhs 另一个参数对象 (Another parameter object)
 * @return 如果两个参数对象相等，则返回 true，否则返回 false (Returns true if the two parameter
 * objects are equal, false otherwise)
 */
bool Parameter::operator==(const Parameter& rhs) const {
  return this->name_ == rhs.name_ && this->value_ == rhs.value_;
}

/**
 * @brief 判断两个参数对象是否不相等 (Determine if two parameter objects are not equal)
 * @param rhs 另一个参数对象 (Another parameter object)
 * @return 如果两个参数对象不相等，则返回 true，否则返回 false (Returns true if the two parameter
 * objects are not equal, false otherwise)
 */
bool Parameter::operator!=(const Parameter& rhs) const { return !(*this == rhs); }

/**
 * @brief 获取参数的类型 (Get the type of the parameter)
 * @return 参数的类型 (Type of the parameter)
 */
ParameterType Parameter::get_type() const { return value_.get_type(); }

/**
 * @brief 获取参数类型的名称 (Get the name of the parameter type)
 * @return 参数类型的名称 (Name of the parameter type)
 */
std::string Parameter::get_type_name() const { return rclcpp::to_string(get_type()); }

/**
 * @brief 获取参数的名称 (Get the name of the parameter)
 * @return 参数的名称 (Name of the parameter)
 */
const std::string& Parameter::get_name() const { return name_; }

/**
 * @brief 获取参数值的消息表示形式 (Get the message representation of the parameter value)
 * @return 参数值的消息表示形式 (Message representation of the parameter value)
 */
rcl_interfaces::msg::ParameterValue Parameter::get_value_message() const {
  return value_.to_value_msg();
}

/**
 * @brief 获取参数的值 (Get the value of the parameter)
 * @return 参数的值 (Value of the parameter)
 */
const rclcpp::ParameterValue& Parameter::get_parameter_value() const { return value_; }

/**
 * @brief 将参数值转换为布尔值 (Convert the parameter value to a boolean)
 * @return 参数值的布尔表示形式 (Boolean representation of the parameter value)
 */
bool Parameter::as_bool() const { return get_value<ParameterType::PARAMETER_BOOL>(); }

/**
 * @brief 将参数值转换为整数 (Convert the parameter value to an integer)
 * @return 参数值的整数表示形式 (Integer representation of the parameter value)
 */
int64_t Parameter::as_int() const { return get_value<ParameterType::PARAMETER_INTEGER>(); }

/**
 * @brief 将参数值转换为双精度浮点数 (Convert the parameter value to a double-precision floating
 * point number)
 * @return 参数值的双精度浮点数表示形式 (Double-precision floating point representation of the
 * parameter value)
 */
double Parameter::as_double() const { return get_value<ParameterType::PARAMETER_DOUBLE>(); }

/**
 * @brief 将参数值转换为字符串 (Convert the parameter value to a string)
 * @return 参数值的字符串表示形式 (String representation of the parameter value)
 */
const std::string& Parameter::as_string() const {
  return get_value<ParameterType::PARAMETER_STRING>();
}

/**
 * @brief 将参数值转换为字节数组 (Convert the parameter value to a byte array)
 * @return 参数值的字节数组表示形式 (Byte array representation of the parameter value)
 */
const std::vector<uint8_t>& Parameter::as_byte_array() const {
  return get_value<ParameterType::PARAMETER_BYTE_ARRAY>();
}

/**
 * @brief 返回布尔数组类型的参数值 (Return the parameter value as a boolean array)
 *
 * @return std::vector<bool> 布尔数组 (Boolean array)
 */
const std::vector<bool>& Parameter::as_bool_array() const {
  // 获取布尔数组类型的参数值 (Get the parameter value as a boolean array)
  return get_value<ParameterType::PARAMETER_BOOL_ARRAY>();
}

/**
 * @brief 返回整数数组类型的参数值 (Return the parameter value as an integer array)
 *
 * @return std::vector<int64_t> 整数数组 (Integer array)
 */
const std::vector<int64_t>& Parameter::as_integer_array() const {
  // 获取整数数组类型的参数值 (Get the parameter value as an integer array)
  return get_value<ParameterType::PARAMETER_INTEGER_ARRAY>();
}

/**
 * @brief 返回双精度浮点数数组类型的参数值 (Return the parameter value as a double array)
 *
 * @return std::vector<double> 双精度浮点数数组 (Double array)
 */
const std::vector<double>& Parameter::as_double_array() const {
  // 获取双精度浮点数数组类型的参数值 (Get the parameter value as a double array)
  return get_value<ParameterType::PARAMETER_DOUBLE_ARRAY>();
}

/**
 * @brief 返回字符串数组类型的参数值 (Return the parameter value as a string array)
 *
 * @return std::vector<std::string> 字符串数组 (String array)
 */
const std::vector<std::string>& Parameter::as_string_array() const {
  // 获取字符串数组类型的参数值 (Get the parameter value as a string array)
  return get_value<ParameterType::PARAMETER_STRING_ARRAY>();
}

/**
 * @brief 从参数消息中创建 Parameter 对象 (Create a Parameter object from a parameter message)
 *
 * @param parameter rcl_interfaces::msg::Parameter 参数消息对象 (Parameter message object)
 * @return Parameter 创建的 Parameter 对象 (Created Parameter object)
 */
Parameter Parameter::from_parameter_msg(const rcl_interfaces::msg::Parameter& parameter) {
  // 使用参数消息的名称和值创建 Parameter 对象 (Create a Parameter object using the name and value
  // from the parameter message)
  return Parameter(parameter.name, parameter.value);
}

/**
 * @brief 将 Parameter 对象转换为参数消息 (Convert a Parameter object to a parameter message)
 *
 * @return rcl_interfaces::msg::Parameter 转换后的参数消息 (Converted parameter message)
 */
rcl_interfaces::msg::Parameter Parameter::to_parameter_msg() const {
  rcl_interfaces::msg::Parameter parameter;
  // 设置参数消息的名称 (Set the name of the parameter message)
  parameter.name = name_;
  // 设置参数消息的值 (Set the value of the parameter message)
  parameter.value = value_.to_value_msg();
  // 返回参数消息 (Return the parameter message)
  return parameter;
}

/**
 * @brief 将参数值转换为字符串 (Convert the parameter value to a string)
 *
 * @return std::string 参数值的字符串表示形式 (String representation of the parameter value)
 */
std::string Parameter::value_to_string() const { return rclcpp::to_string(value_); }

/**
 * @brief 将参数转换为 JSON 字典项 (Convert a parameter to a JSON dictionary entry)
 *
 * @param param Parameter 参数对象 (Parameter object)
 * @return std::string JSON 字典项字符串 (JSON dictionary entry string)
 */
std::string rclcpp::_to_json_dict_entry(const Parameter& param) {
  std::stringstream ss;
  // 添加参数名称 (Add the parameter name)
  ss << "\"" << param.get_name() << "\": ";
  // 添加参数类型和值 (Add the parameter type and value)
  ss << "{\"type\": \"" << param.get_type_name() << "\", ";
  ss << "\"value\": \"" << param.value_to_string() << "\"}";
  return ss.str();
}

/**
 * @brief 输出运算符重载，将 Parameter 对象输出到输出流中 (Overloaded output operator, outputting a
 * Parameter object to an ostream)
 *
 * @param os std::ostream 输出流 (Output stream)
 * @param pv rclcpp::Parameter Parameter 对象 (Parameter object)
 * @return std::ostream& 输出流 (Output stream)
 */
std::ostream& rclcpp::operator<<(std::ostream& os, const rclcpp::Parameter& pv) {
  // 输出 Parameter 对象的字符串表示形式 (Output the string representation of the Parameter object)
  os << std::to_string(pv);
  return os;
}

/**
 * @brief 输出运算符重载，将参数向量输出到输出流中 (Overloaded output operator, outputting a vector
 * of parameters to an ostream)
 *
 * @param os std::ostream 输出流 (Output stream)
 * @param parameters std::vector<Parameter> 参数向量 (Vector of parameters)
 * @return std::ostream& 输出流 (Output stream)
 */
std::ostream& rclcpp::operator<<(std::ostream& os, const std::vector<Parameter>& parameters) {
  // 输出参数向量的字符串表示形式 (Output the string representation of the vector of parameters)
  os << std::to_string(parameters);
  return os;
}

/**
 * @brief 将 Parameter 对象转换为字符串 (Convert a Parameter object to a string)
 *
 * @param param rclcpp::Parameter 参数对象 (Parameter object)
 * @return std::string 参数对象的字符串表示形式 (String representation of the parameter object)
 */
std::string std::to_string(const rclcpp::Parameter& param) {
  std::stringstream ss;
  // 添加参数名称、类型和值 (Add the parameter name, type and value)
  ss << "{\"name\": \"" << param.get_name() << "\", ";
  ss << "\"type\": \"" << param.get_type_name() << "\", ";
  ss << "\"value\": \"" << param.value_to_string() << "\"}";
  return ss.str();
}

/**
 * @brief 将参数向量转换为字符串 (Convert a vector of parameters to a string)
 *
 * @param parameters std::vector<rclcpp::Parameter> 参数向量 (Vector of parameters)
 * @return std::string 参数向量的字符串表示形式 (String representation of the vector of parameters)
 */
std::string std::to_string(const std::vector<rclcpp::Parameter>& parameters) {
  std::stringstream ss;
  ss << "{";
  bool first = true;
  // 遍历参数向量并将每个参数添加到字符串中 (Iterate through the vector of parameters and add each
  // one to the string)
  for (const auto& pv : parameters) {
    if (first == false) {
      ss << ", ";
    } else {
      first = false;
    }
    ss << rclcpp::_to_json_dict_entry(pv);
  }
  ss << "}";
  return ss.str();
}

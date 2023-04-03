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

#include "rclcpp/future_return_code.hpp"

#include <iostream>
#include <string>
#include <type_traits>

namespace rclcpp {

/**
 * @brief 重载 << 运算符，用于输出 rclcpp::FutureReturnCode 的字符串表示形式
 *        (Overload the << operator for outputting the string representation of
 * rclcpp::FutureReturnCode)
 * @param os 输出流对象 (Output stream object)
 * @param future_return_code rclcpp::FutureReturnCode 枚举值 (Enumeration value of
 * rclcpp::FutureReturnCode)
 * @return std::ostream& 返回输出流对象 (Return the output stream object)
 */
std::ostream& operator<<(std::ostream& os, const rclcpp::FutureReturnCode& future_return_code) {
  // 调用 to_string 函数将 future_return_code 转换为字符串并输出 (Call the to_string function to
  // convert future_return_code to a string and output it)
  return os << to_string(future_return_code);
}

/**
 * @brief 将 rclcpp::FutureReturnCode 转换为字符串表示形式 (Convert rclcpp::FutureReturnCode to its
 * string representation)
 * @param future_return_code rclcpp::FutureReturnCode 枚举值 (Enumeration value of
 * rclcpp::FutureReturnCode)
 * @return std::string 返回 future_return_code 的字符串表示形式 (Return the string representation of
 * future_return_code)
 */
std::string to_string(const rclcpp::FutureReturnCode& future_return_code) {
  // 使用底层枚举类型 (Use the underlying enumeration type)
  using enum_type = std::underlying_type<FutureReturnCode>::type;

  // 初始化前缀字符串为 "Unknown enum value (" (Initialize the prefix string to "Unknown enum value
  // (")
  std::string prefix = "Unknown enum value (";

  // 将 future_return_code 转换为其底层枚举类型的字符串表示形式 (Convert future_return_code to its
  // underlying enumeration type's string representation)
  std::string ret_as_string = std::to_string(static_cast<enum_type>(future_return_code));

  // 使用 switch 语句处理不同的 future_return_code 枚举值 (Use a switch statement to handle
  // different future_return_code enumeration values)
  switch (future_return_code) {
    case FutureReturnCode::SUCCESS:
      // 如果 future_return_code 为 SUCCESS，则将前缀字符串设置为 "SUCCESS (" (If future_return_code
      // is SUCCESS, set the prefix string to "SUCCESS (")
      prefix = "SUCCESS (";
      break;
    case FutureReturnCode::INTERRUPTED:
      // 如果 future_return_code 为 INTERRUPTED，则将前缀字符串设置为 "INTERRUPTED (" (If
      // future_return_code is INTERRUPTED, set the prefix string to "INTERRUPTED (")
      prefix = "INTERRUPTED (";
      break;
    case FutureReturnCode::TIMEOUT:
      // 如果 future_return_code 为 TIMEOUT，则将前缀字符串设置为 "TIMEOUT (" (If future_return_code
      // is TIMEOUT, set the prefix string to "TIMEOUT (")
      prefix = "TIMEOUT (";
      break;
  }

  // 返回拼接后的完整字符串表示形式 (Return the concatenated complete string representation)
  return prefix + ret_as_string + ")";
}

}  // namespace rclcpp

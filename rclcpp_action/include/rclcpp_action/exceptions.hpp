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

#ifndef RCLCPP_ACTION__EXCEPTIONS_HPP_
#define RCLCPP_ACTION__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace rclcpp_action {
namespace exceptions {
/**
 * @class UnknownGoalHandleError
 * @brief 自定义异常类，表示未知的目标句柄错误 (Custom exception class representing unknown goal
 * handle errors)
 * @details 继承自 std::invalid_argument (Derived from std::invalid_argument)
 */
class UnknownGoalHandleError : public std::invalid_argument {
public:
  /**
   * @brief 构造函数，初始化异常消息 (Constructor, initializes the exception message)
   */
  UnknownGoalHandleError() : std::invalid_argument("Goal handle is not known to this client.") {}
};

/**
 * @class UnawareGoalHandleError
 * @brief 自定义异常类，表示不了解目标句柄错误 (Custom exception class representing unaware goal
 * handle errors)
 * @details 继承自 std::runtime_error (Derived from std::runtime_error)
 */
class UnawareGoalHandleError : public std::runtime_error {
public:
  /**
   * @brief 带有默认消息的构造函数 (Constructor with default message)
   * @param message 异常消息，默认为 "Goal handle is not tracking the goal result." (Exception
   * message, default is "Goal handle is not tracking the goal result.")
   */
  UnawareGoalHandleError(
      const std::string& message = "Goal handle is not tracking the goal result.")
      : std::runtime_error(message) {}
};

}  // namespace exceptions

}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__EXCEPTIONS_HPP_

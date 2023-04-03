// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__MESSAGE_INFO_HPP_
#define RCLCPP__MESSAGE_INFO_HPP_

#include "rclcpp/visibility_control.hpp"
#include "rmw/types.h"

namespace rclcpp {

/// \class MessageInfo
/// \brief 附加的订阅消息的元数据信息。 (Additional meta data about messages taken from
/// subscriptions.)
class RCLCPP_PUBLIC MessageInfo {
public:
  /// 默认空构造函数。 (Default empty constructor.)
  MessageInfo() = default;

  /// 转换构造函数，故意不标记为 explicit。 (Conversion constructor, which is intentionally not
  /// marked as explicit.)
  /**
   * \param[in] rmw_message_info 用于初始化类的消息信息 (message info to initialize the class)
   */
  // cppcheck-suppress noExplicitConstructor
  MessageInfo(const rmw_message_info_t& rmw_message_info);  // NOLINT(runtime/explicit)

  /// 虚析构函数。 (Virtual destructor.)
  virtual ~MessageInfo();

  /// 以底层 rmw 消息信息类型返回消息信息。 (Return the message info as the underlying rmw message
  /// info type.)
  const rmw_message_info_t& get_rmw_message_info() const;

  /// 以底层 rmw 消息信息类型返回消息信息。 (Return the message info as the underlying rmw message
  /// info type.)
  rmw_message_info_t& get_rmw_message_info();

private:
  /// rmw 消息信息变量。 (rmw message info variable.)
  rmw_message_info_t rmw_message_info_;
};

}  // namespace rclcpp

#endif  // RCLCPP__MESSAGE_INFO_HPP_

// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_ACTION__QOS_HPP_
#define RCLCPP_ACTION__QOS_HPP_

#include <rclcpp/qos.hpp>

#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action {

/**
 * @class DefaultActionStatusQoS
 * @brief 默认动作状态服务质量类 (Default action status Quality of Service class)
 *
 * 这个类继承了 rclcpp::QoS，用于设置默认的动作状态服务质量。
 * (This class inherits from rclcpp::QoS and is used to set the default action status Quality of
 * Service.)
 */
class DefaultActionStatusQoS : public rclcpp::QoS {
public:
  /**
   * @brief 导出符号 (Export symbol)
   *
   * RCLCPP_ACTION_PUBLIC 是一个宏，用于在 Windows 系统上导出符号以供其他模块使用。
   * (RCLCPP_ACTION_PUBLIC is a macro for exporting symbols on Windows systems for use in other
   * modules.)
   */
  RCLCPP_ACTION_PUBLIC

  /**
   * @brief 构造函数 (Constructor)
   *
   * DefaultActionStatusQoS 的构造函数，用于创建 DefaultActionStatusQoS 对象。
   * (The constructor for DefaultActionStatusQoS, used to create a DefaultActionStatusQoS object.)
   */
  DefaultActionStatusQoS();
};

}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__QOS_HPP_

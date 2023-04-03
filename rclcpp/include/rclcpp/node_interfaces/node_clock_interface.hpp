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

#ifndef RCLCPP__NODE_INTERFACES__NODE_CLOCK_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_CLOCK_INTERFACE_HPP_

#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/detail/node_interfaces_helpers.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/// 纯虚接口类，用于实现节点 API 中的 NodeClock 部分。
/// Pure virtual interface class for the NodeClock part of the Node API.
class NodeClockInterface {
public:
  // 只使用智能指针别名，不允许拷贝和赋值。
  // Use smart pointer aliases only, disallow copy and assignment.
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeClockInterface)

  // 公共虚析构函数，使用默认实现。
  // Public virtual destructor with default implementation.
  RCLCPP_PUBLIC
  virtual ~NodeClockInterface() = default;

  /// 获取一个 ROS 时钟，该时钟将由节点保持更新。
  /// Get a ROS clock which will be kept up to date by the node.
  /**
   * \return 返回一个共享指针，指向 ROS 时钟对象。
   * \return A shared pointer to a ROS clock object.
   */
  RCLCPP_PUBLIC
  virtual rclcpp::Clock::SharedPtr get_clock() = 0;

  /// 获取一个 const ROS 时钟，该时钟将由节点保持更新。
  /// Get a const ROS clock which will be kept up to date by the node.
  /**
   * \return 返回一个常量共享指针，指向 ROS 时钟对象。
   * \return A constant shared pointer to a ROS clock object.
   */
  RCLCPP_PUBLIC
  virtual rclcpp::Clock::ConstSharedPtr get_clock() const = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeClockInterface, clock)

#endif  // RCLCPP__NODE_INTERFACES__NODE_CLOCK_INTERFACE_HPP_

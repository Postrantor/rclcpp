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

#ifndef RCLCPP__NODE_INTERFACES__NODE_TIME_SOURCE_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_TIME_SOURCE_INTERFACE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/detail/node_interfaces_helpers.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/// @brief 纯虚接口类，用于实现节点时间源部分的节点API。
/// @brief Pure virtual interface class for the NodeTimeSource part of the Node API.
class NodeTimeSourceInterface {
public:
  // 定义智能指针别名，仅使用智能指针
  // Define smart pointer aliases, using only smart pointers
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTimeSourceInterface)

  // 设置公共访问权限
  // Set public access level
  RCLCPP_PUBLIC
  // 虚析构函数，默认实现，用于在派生类中正确释放资源
  // Virtual destructor with default implementation, used for correctly releasing resources in
  // derived classes
  virtual ~NodeTimeSourceInterface() = default;
};

}  // namespace node_interfaces
}  // namespace rclcpp

RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeTimeSourceInterface, time_source)

#endif  // RCLCPP__NODE_INTERFACES__NODE_TIME_SOURCE_INTERFACE_HPP_

// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_TIMERS_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_TIMERS_INTERFACE_HPP_

#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/detail/node_interfaces_helpers.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/// 纯虚拟接口类，用于 Node API 的 NodeTimers 部分。
/// Pure virtual interface class for the NodeTimers part of the Node API.
class NodeTimersInterface {
public:
  // 只使用智能指针别名
  // Use smart pointer aliases only
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTimersInterface)

  // 公共虚析构函数
  // Public virtual destructor
  RCLCPP_PUBLIC
  virtual ~NodeTimersInterface() = default;

  /// 向节点添加定时器。
  /// Add a timer to the node.
  /**
   * \param[in] timer 定时器的共享指针。
   * \param[in] callback_group 回调组的共享指针。
   *
   * \param[in] timer Shared pointer of the timer.
   * \param[in] callback_group Shared pointer of the callback group.
   */
  RCLCPP_PUBLIC
  virtual void add_timer(
      rclcpp::TimerBase::SharedPtr timer, rclcpp::CallbackGroup::SharedPtr callback_group) = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeTimersInterface, timers)

#endif  // RCLCPP__NODE_INTERFACES__NODE_TIMERS_INTERFACE_HPP_

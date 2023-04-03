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

#ifndef RCLCPP__NODE_INTERFACES__NODE_TIMERS_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_TIMERS_HPP_

#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/// \file node_timers.hpp
/// \brief 实现 NodeTimers 类，作为 Node API 的一部分 (Implementation of the NodeTimers class, part
/// of the Node API)

/// NodeTimers 类实现 (NodeTimers class implementation)
class NodeTimers : public NodeTimersInterface {
public:
  // 只允许智能指针别名 (Allow smart pointer aliases only)
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTimers)

  /// 构造函数 (Constructor)
  /**
   * \param[in] node_base 节点基类指针 (Pointer to the node base class)
   */
  RCLCPP_PUBLIC
  explicit NodeTimers(rclcpp::node_interfaces::NodeBaseInterface* node_base);

  /// 析构函数 (Destructor)
  RCLCPP_PUBLIC
  virtual ~NodeTimers();

  /// 向节点添加定时器 (Add a timer to the node)
  /**
   * \param[in] timer 定时器的共享指针 (Shared pointer to the timer)
   * \param[in] callback_group 回调组的共享指针 (Shared pointer to the callback group)
   */
  RCLCPP_PUBLIC
  void add_timer(
      rclcpp::TimerBase::SharedPtr timer, rclcpp::CallbackGroup::SharedPtr callback_group) override;

private:
  // 禁用拷贝构造函数和赋值操作符 (Disable copy constructor and assignment operator)
  RCLCPP_DISABLE_COPY(NodeTimers)

  // 节点基类指针 (Pointer to the node base class)
  rclcpp::node_interfaces::NodeBaseInterface* node_base_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_TIMERS_HPP_

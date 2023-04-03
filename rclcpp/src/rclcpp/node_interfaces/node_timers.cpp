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

#include "rclcpp/node_interfaces/node_timers.hpp"

#include <string>

#include "tracetools/tracetools.h"

using rclcpp::node_interfaces::NodeTimers;

/**
 * @brief 构造函数，初始化节点定时器 (Constructor, initializes the node timer)
 *
 * @param node_base 指向节点基础接口的指针 (Pointer to the NodeBaseInterface)
 */
NodeTimers::NodeTimers(rclcpp::node_interfaces::NodeBaseInterface *node_base)
    : node_base_(node_base)  // 初始化节点基础接口 (Initialize the node base interface)
{}

/**
 * @brief 析构函数 (Destructor)
 */
NodeTimers::~NodeTimers() {}

/**
 * @brief 添加一个定时器到回调组中 (Add a timer to the callback group)
 *
 * @param timer 定时器共享指针 (Shared pointer to the timer)
 * @param callback_group 回调组共享指针 (Shared pointer to the callback group)
 */
void NodeTimers::add_timer(
    rclcpp::TimerBase::SharedPtr timer, rclcpp::CallbackGroup::SharedPtr callback_group) {
  // 判断回调组是否存在 (Check if the callback group exists)
  if (callback_group) {
    // 判断回调组是否在节点中 (Check if the callback group is in the node)
    if (!node_base_->callback_group_in_node(callback_group)) {
      // 抛出异常 (Throw an exception)
      throw std::runtime_error("Cannot create timer, group not in node.");
    }
  } else {
    // 获取默认回调组 (Get the default callback group)
    callback_group = node_base_->get_default_callback_group();
  }

  // 向回调组中添加定时器 (Add the timer to the callback group)
  callback_group->add_timer(timer);

  // 获取通知保护条件 (Get the notify guard condition)
  auto &node_gc = node_base_->get_notify_guard_condition();

  // 触发通知保护条件 (Trigger the notify guard condition)
  try {
    node_gc.trigger();
    callback_group->trigger_notify_guard_condition();
  } catch (const rclcpp::exceptions::RCLError &ex) {
    // 抛出异常 (Throw an exception)
    throw std::runtime_error(
        std::string("failed to notify wait set on timer creation: ") + ex.what());
  }

  // 添加跟踪点 (Add a tracepoint)
  TRACEPOINT(
      rclcpp_timer_link_node, static_cast<const void *>(timer->get_timer_handle().get()),
      static_cast<const void *>(node_base_->get_rcl_node_handle()));
}

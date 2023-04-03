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

#include "rclcpp/node_interfaces/node_waitables.hpp"

#include <string>

using rclcpp::node_interfaces::NodeWaitables;

/**
 * @brief 构造函数，初始化 NodeWaitables 对象
 * @param node_base 一个指向 NodeBaseInterface 类型的指针
 *
 * @brief Constructor, initializes the NodeWaitables object
 * @param node_base A pointer to a NodeBaseInterface type
 */
NodeWaitables::NodeWaitables(rclcpp::node_interfaces::NodeBaseInterface* node_base)
    : node_base_(node_base)  // 初始化成员变量 node_base_
{}

/**
 * @brief 析构函数
 *
 * @brief Destructor
 */
NodeWaitables::~NodeWaitables() {}

/**
 * @brief 添加一个 waitable 到指定的回调组中
 * @param waitable_ptr 指向 Waitable 类型的共享指针
 * @param group 指向 CallbackGroup 类型的共享指针
 *
 * @brief Add a waitable to the specified callback group
 * @param waitable_ptr Shared pointer to a Waitable type
 * @param group Shared pointer to a CallbackGroup type
 */
void NodeWaitables::add_waitable(
    rclcpp::Waitable::SharedPtr waitable_ptr, rclcpp::CallbackGroup::SharedPtr group) {
  if (group) {                                         // 如果 group 已经存在
    if (!node_base_->callback_group_in_node(group)) {  // 检查 group 是否在当前节点中
      // TODO(jacobperron): 使用自定义异常
      throw std::runtime_error("Cannot create waitable, group not in node.");  // 抛出异常
    }
  } else {                                             // 如果 group 不存在
    group = node_base_->get_default_callback_group();  // 获取默认的回调组
  }

  group->add_waitable(waitable_ptr);  // 将 waitable 添加到回调组中

  // 通知执行器，使用父节点创建了一个新的 waitable
  auto& node_gc = node_base_->get_notify_guard_condition();
  try {
    node_gc.trigger();                        // 触发通知
    group->trigger_notify_guard_condition();  // 触发回调组的通知
  } catch (const rclcpp::exceptions::RCLError& ex) {
    throw std::runtime_error(
        std::string("failed to notify wait set on waitable creation: ") + ex.what());  // 抛出异常
  }
}

/**
 * @brief 从指定的回调组中移除一个 waitable
 * @param waitable_ptr 指向 Waitable 类型的共享指针
 * @param group 指向 CallbackGroup 类型的共享指针
 *
 * @brief Remove a waitable from the specified callback group
 * @param waitable_ptr Shared pointer to a Waitable type
 * @param group Shared pointer to a CallbackGroup type
 */
void NodeWaitables::remove_waitable(
    rclcpp::Waitable::SharedPtr waitable_ptr, rclcpp::CallbackGroup::SharedPtr group) noexcept {
  if (group) {                                         // 如果 group 存在
    if (!node_base_->callback_group_in_node(group)) {  // 检查 group 是否在当前节点中
      return;                                          // 如果不在，则直接返回
    }
    group->remove_waitable(waitable_ptr);              // 从回调组中移除 waitable
  } else {                                             // 如果 group 不存在
    node_base_->get_default_callback_group()->remove_waitable(
        waitable_ptr);                                 // 从默认的回调组中移除 waitable
  }
}

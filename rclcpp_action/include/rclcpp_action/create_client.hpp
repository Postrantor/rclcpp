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

#ifndef RCLCPP_ACTION__CREATE_CLIENT_HPP_
#define RCLCPP_ACTION__CREATE_CLIENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action {
创建一个动作客户端。(Create an action client.)
/**
 * 这个函数等同于 \sa create_client()，但是使用单独的节点接口来创建客户端。
 * (This function is equivalent to \sa create_client()` however is using the individual
 * node interfaces to create the client.)
 *
 * \param[in] node_base_interface 对应节点的节点基本接口。 (The node base interface of the
 * corresponding node.) \param[in] node_graph_interface 对应节点的节点图形接口。 (The node graph
 * interface of the corresponding node.) \param[in] node_logging_interface 对应节点的节点日志接口。
 * (The node logging interface of the corresponding node.) \param[in] node_waitables_interface
 * 对应节点的节点可等待接口。 (The node waitables interface of the corresponding node.) \param[in]
 * name 动作名称。 (The action name.) \param[in] group 动作客户端将添加到此回调组。如果为
 * `nullptr`，则将动作客户端添加到默认回调组。 (The action client will be added to this callback
 * group. If `nullptr`, then the action client is added to the default callback group.) \param[in]
 * options 传递给底层 `rcl_action_client_t` 的选项。 (Options to pass to the underlying
 * `rcl_action_client_t`.)
 */
template <typename ActionT>
typename Client<ActionT>::SharedPtr create_client(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    const std::string &name,
    rclcpp::CallbackGroup::SharedPtr group = nullptr,
    const rcl_action_client_options_t &options = rcl_action_client_get_default_options()) {
  // 创建一个弱指针，指向节点的可等待接口 (Create a weak pointer pointing to the node's waitables
  // interface)
  std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
      node_waitables_interface;
  // 创建一个弱指针，指向回调组 (Create a weak pointer pointing to the callback group)
  std::weak_ptr<rclcpp::CallbackGroup> weak_group = group;
  // 检查回调组是否为空 (Check if the callback group is null)
  bool group_is_null = (nullptr == group.get());

  // 定义删除器，在动作客户端不再需要时，释放资源 (Define a deleter to release resources when the
  // action client is no longer needed)
  auto deleter = [weak_node, weak_group, group_is_null](Client<ActionT> *ptr) {
    // 如果指针为空，直接返回 (If the pointer is null, return directly)
    if (nullptr == ptr) {
      return;
    }
    // 尝试锁定共享节点 (Try to lock the shared node)
    auto shared_node = weak_node.lock();
    if (shared_node) {
      // API期望一个共享指针，给它一个什么都不做的删除器 (The API expects a shared pointer, give it
      // one with a deleter that does nothing.)
      std::shared_ptr<Client<ActionT>> fake_shared_ptr(ptr, [](Client<ActionT> *) {});

      if (group_is_null) {
        // 已添加到默认组 (Was added to default group)
        shared_node->remove_waitable(fake_shared_ptr, nullptr);
      } else {
        // 已添加到特定组 (Was added to a specific group)
        auto shared_group = weak_group.lock();
        if (shared_group) {
          shared_node->remove_waitable(fake_shared_ptr, shared_group);
        }
      }
    }
    delete ptr;
  };

  // 创建一个动作客户端的共享指针 (Create a shared pointer of the action client)
  std::shared_ptr<Client<ActionT>> action_client(
      new Client<ActionT>(
          node_base_interface, node_graph_interface, node_logging_interface, name, options),
      deleter);

  // 将动作客户端添加到可等待接口 (Add the action client to the waitables interface)
  node_waitables_interface->add_waitable(action_client, group);
  return action_client;
}

/**
 * 创建一个动作客户端。(Create an action client.)
 * \param[in] node 动作客户端将添加到此节点。 (The action client will be added to this node.)
 * \param[in] name 动作名称。 (The action name.)
 * \param[in] group 动作客户端将添加到此回调组。如果为 `nullptr`，则将动作客户端添加到默认回调组。
 *   (The action client will be added to this callback group.
 *   If `nullptr`, then the action client is added to the default callback group.)
 * \param[in] options 传递给底层 `rcl_action_client_t` 的选项。
 *   (Options to pass to the underlying `rcl_action_client_t`.)
 */
template <typename ActionT, typename NodeT>
typename Client<ActionT>::SharedPtr create_client(
    NodeT node,
    const std::string &name,
    rclcpp::CallbackGroup::SharedPtr group = nullptr,
    const rcl_action_client_options_t &options = rcl_action_client_get_default_options()) {
  // 调用 rclcpp_action::create_client 函数创建一个动作客户端，并返回其共享指针
  // 使用节点的各个接口、动作名称、回调组和选项作为参数
  // (Call the rclcpp_action::create_client function to create an action client and return its
  // shared pointer, using the various interfaces of the node, action name, callback group, and
  // options as arguments)
  return rclcpp_action::create_client<ActionT>(
      node->get_node_base_interface(), node->get_node_graph_interface(),
      node->get_node_logging_interface(), node->get_node_waitables_interface(), name, group,
      options);
}
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CREATE_CLIENT_HPP_

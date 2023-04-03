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

#ifndef RCLCPP_ACTION__CREATE_SERVER_HPP_
#define RCLCPP_ACTION__CREATE_SERVER_HPP_

#include <memory>
#include <string>

#include "rcl_action/action_server.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action {
/// 创建一个动作服务器 (Create an action server).
/**
 * 所有提供的回调函数必须是非阻塞的 (All provided callback functions must be non-blocking).
 * 此函数等同于 \sa create_server()`，但是使用单独的节点接口来创建服务器 (This function is
 * equivalent to \sa create_server()` however is using the individual node interfaces to create the
 * server).
 *
 * 有关更多信息，请参阅 Server::Server() (\sa Server::Server() for more information).
 *
 * \param[in] node_base_interface 对应节点的节点基本接口 (The node base interface of the
 * corresponding node). \param[in] node_clock_interface 对应节点的节点时钟接口 (The node clock
 * interface of the corresponding node). \param[in] node_logging_interface 对应节点的节点日志接口
 * (The node logging interface of the corresponding node). \param[in] node_waitables_interface
 * 对应节点的节点可等待接口 (The node waitables interface of the corresponding node). \param[in]
 * name 动作名称 (The action name). \param[in] handle_goal 决定目标是否应被接受或拒绝的回调 (A
 * callback that decides if a goal should be accepted or rejected). \param[in] handle_cancel
 * 决定是否尝试取消目标的回调 (A callback that decides if a goal should be attempted to be
 * canceled). 返回此回调仅表示服务器将尝试取消目标 (The return from this callback only indicates if
 * the server will try to cancel a goal). 并不表示目标实际上已经取消 (It does not indicate if the
 * goal was actually canceled). \param[in] handle_accepted 调用此回调以向用户提供目标句柄 (A
 * callback that is called to give the user a handle to the goal). \param[in] options 传递给底层
 * `rcl_action_server_t` 的选项 (Options to pass to the underlying `rcl_action_server_t`).
 * \param[in] group 动作服务器将添加到此回调组 (The action server will be added to this callback
 * group). 如果为 `nullptr`，则将动作服务器添加到默认回调组 (If `nullptr`, then the action server is
 * added to the default callback group).
 */
template <typename ActionT>
typename Server<ActionT>::SharedPtr create_server(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    const std::string &name,
    typename Server<ActionT>::GoalCallback handle_goal,
    typename Server<ActionT>::CancelCallback handle_cancel,
    typename Server<ActionT>::AcceptedCallback handle_accepted,
    const rcl_action_server_options_t &options = rcl_action_server_get_default_options(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr) {
  // 创建一个指向节点可等待接口的弱引用 (Create a weak reference to the node waitables interface)
  std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
      node_waitables_interface;
  // 创建一个指向回调组的弱引用 (Create a weak reference to the callback group)
  std::weak_ptr<rclcpp::CallbackGroup> weak_group = group;
  // 检查回调组是否为空 (Check if the callback group is null)
  bool group_is_null = (nullptr == group.get());

  // 定义删除器，用于在动作服务器对象不再需要时清理资源 (Define a deleter for cleaning up resources
  // when the action server object is no longer needed)
  auto deleter = [weak_node, weak_group, group_is_null](Server<ActionT> *ptr) {
    // 如果指针为空，则不执行任何操作 (If the pointer is nullptr, do nothing)
    if (nullptr == ptr) {
      return;
    }
    // 获取节点可等待接口的共享引用 (Get a shared reference to the node waitables interface)
    auto shared_node = weak_node.lock();
    if (shared_node) {
      // API期望一个共享指针，给它一个不执行任何操作的删除器 (API expects a shared pointer, give it
      // one with a deleter that does nothing)
      std::shared_ptr<Server<ActionT>> fake_shared_ptr(ptr, [](Server<ActionT> *) {});

      if (group_is_null) {
        // 添加到默认组 (Was added to default group)
        shared_node->remove_waitable(fake_shared_ptr, nullptr);
      } else {
        // 添加到特定组 (Was added to a specific group)
        auto shared_group = weak_group.lock();
        if (shared_group) {
          shared_node->remove_waitable(fake_shared_ptr, shared_group);
        }
      }
    }
    // 删除指针 (Delete the pointer)
    delete ptr;
  };

  // 创建一个新的动作服务器实例并传递相关参数 (Create a new action server instance and pass the
  // relevant parameters)
  std::shared_ptr<Server<ActionT>> action_server(
      new Server<ActionT>(
          node_base_interface, node_clock_interface, node_logging_interface, name, options,
          handle_goal, handle_cancel, handle_accepted),
      deleter);

  // 将动作服务器添加到节点的可等待接口 (Add the action server to the node's waitables interface)
  node_waitables_interface->add_waitable(action_server, group);
  // 返回动作服务器实例 (Return the action server instance)
  return action_server;
}

/// 创建一个动作服务器。 (Create an action server)
/**
 * 提供的所有回调函数都必须是非阻塞的。 (All provided callback functions must be non-blocking)
 *
 * 有关更多信息，请参阅 Server::Server()。 (See Server::Server() for more information)
 *
 * \param[in] node 动作服务器将添加到此节点。 (The action server will be added to this node)
 * \param[in] name 动作名称。 (The action name)
 * \param[in] handle_goal 决定目标是否应被接受或拒绝的回调。 (A callback that decides if a goal
 * should be accepted or rejected) \param[in] handle_cancel 决定目标是否应尝试取消的回调。 (A
 * callback that decides if a goal should be attempted to be canceled)
 *  此回调的返回值仅表示服务器是否会尝试取消目标。 (The return from this callback only indicates if
 * the server will try to cancel a goal) 它并不表示目标是否实际取消。 (It does not indicate if the
 * goal was actually canceled) \param[in] handle_accepted 调用此回调以向用户提供目标句柄。 (A
 * callback that is called to give the user a handle to the goal) \param[in] options 传递给底层
 * `rcl_action_server_t` 的选项。 (Options to pass to the underlying `rcl_action_server_t`)
 * \param[in] group 动作服务器将添加到此回调组。 (The action server will be added to this callback
 * group) 如果为 `nullptr`，则动作服务器将添加到默认回调组。 (If `nullptr`, then the action server
 * is added to the default callback group)
 */
template <typename ActionT, typename NodeT>
typename Server<ActionT>::SharedPtr create_server(
    NodeT node,
    const std::string &name,
    typename Server<ActionT>::GoalCallback handle_goal,
    typename Server<ActionT>::CancelCallback handle_cancel,
    typename Server<ActionT>::AcceptedCallback handle_accepted,
    const rcl_action_server_options_t &options = rcl_action_server_get_default_options(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr) {
  // 使用给定的节点接口、名称和回调创建动作服务器。 (Create the action server with the given node
  // interfaces, name, and callbacks)
  return create_server<ActionT>(
      node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), node->get_node_waitables_interface(), name, handle_goal,
      handle_cancel, handle_accepted, options, group);
}
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__CREATE_SERVER_HPP_

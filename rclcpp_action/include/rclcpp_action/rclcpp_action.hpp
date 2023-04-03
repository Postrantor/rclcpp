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

/** \mainpage rclcpp_action: ROS Action Client Library for C++
 *
 * `rclcpp_action` 提供了用于与 ROS Actions 进行交互的规范 C++ API。
 * (`rclcpp_action` provides the canonical C++ API for interacting with ROS Actions.)
 *
 * 它由以下主要组件组成：
 * (It consists of these main components:)
 *
 * - 动作客户端（Action Client）
 *   - rclcpp_action/client.hpp
 *   - rclcpp_action/create_client.hpp
 *   - rclcpp_action/client_goal_handle.hpp
 * - 动作服务器（Action Server）
 *   - rclcpp_action/server.hpp
 *   - rclcpp_action/create_server.hpp
 *   - rclcpp_action/server_goal_handle.hpp
 */

#ifndef RCLCPP_ACTION__RCLCPP_ACTION_HPP_
#define RCLCPP_ACTION__RCLCPP_ACTION_HPP_

#include <csignal>
#include <memory>

#include "rclcpp_action/client.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_action/visibility_control.hpp"

#endif  // RCLCPP_ACTION__RCLCPP_ACTION_HPP_

        // from chat gpt 4
/*
// 包含必要的头文件
// (Include necessary header files)
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"

// 主函数
// (Main function)
int main(int argc, char **argv)
{
  // 初始化 ROS2 节点
  // (Initialize ROS2 node)
  rclcpp::init(argc, argv);

  // 创建一个共享指针，指向 rclcpp::Node 类型的实例
  // (Create a shared pointer pointing to an instance of rclcpp::Node)
  auto node = std::make_shared<rclcpp::Node>("action_example");

  // 创建动作客户端
  // (Create action client)
  auto action_client = rclcpp_action::create_client<ActionType>(node, "action_name");

  // 创建动作服务器
  // (Create action server)
  auto action_server = rclcpp_action::create_server<ActionType>(
    node,
    "action_name",
    [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle) {
      // 处理目标请求的回调函数
      // (Callback function for handling goal requests)
    },
    [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle) {
      // 处理目标取消的回调函数
      // (Callback function for handling goal cancellations)
    },
    [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle) {
      // 处理目标结果的回调函数
      // (Callback function for handling goal results)
    });

  // 运行 ROS2 节点
  // (Run ROS2 node)
  rclcpp::spin(node);

  // 清理并关闭节点
  // (Clean up and shutdown node)
  rclcpp::shutdown();

  return 0;
}
*/
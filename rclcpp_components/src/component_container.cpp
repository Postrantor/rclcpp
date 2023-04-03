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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

/**
 * @file main.cpp
 *
 * @brief 主函数，用于初始化 ROS2 节点并运行组件容器 (Main function to initialize the ROS2 node and
 * run the component container)
 */
int main(int argc, char* argv[]) {
  // 初始化 ROS2 (Initialize ROS2)
  rclcpp::init(argc, argv);

  // 创建单线程执行器 (Create a single-threaded executor)
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // 创建组件管理器节点，将执行器作为参数传递 (Create a ComponentManager node, passing the executor
  // as an argument)
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);

  // 将组件管理器节点添加到执行器中 (Add the ComponentManager node to the executor)
  exec->add_node(node);

  // 执行器开始执行 (Start spinning the executor)
  exec->spin();

  return 0;
}

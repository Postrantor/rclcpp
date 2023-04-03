// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/executors.hpp"

/**
 * @brief 对节点执行部分处理 (Spins a node partially)
 *
 * @param[in] node_ptr 指向节点基础接口的共享指针 (Shared pointer to the node base interface)
 */
void rclcpp::spin_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr) {
  // 创建一个单线程执行器 (Create a single-threaded executor)
  rclcpp::executors::SingleThreadedExecutor exec;

  // 对给定节点执行部分处理 (Spin the given node partially)
  exec.spin_node_some(node_ptr);
}

/**
 * @brief 对节点执行部分处理 (Spins a node partially)
 *
 * @param[in] node_ptr 指向节点的共享指针 (Shared pointer to the node)
 */
void rclcpp::spin_some(rclcpp::Node::SharedPtr node_ptr) {
  // 调用另一个 spin_some 函数，传入节点基础接口 (Call the other spin_some function, passing the
  // node base interface)
  rclcpp::spin_some(node_ptr->get_node_base_interface());
}

/**
 * @brief 对节点进行连续处理 (Spins a node continuously)
 *
 * @param[in] node_ptr 指向节点基础接口的共享指针 (Shared pointer to the node base interface)
 */
void rclcpp::spin(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr) {
  // 创建一个单线程执行器 (Create a single-threaded executor)
  rclcpp::executors::SingleThreadedExecutor exec;

  // 将节点添加到执行器中 (Add the node to the executor)
  exec.add_node(node_ptr);

  // 对节点进行连续处理 (Spin the node continuously)
  exec.spin();

  // 从执行器中移除节点 (Remove the node from the executor)
  exec.remove_node(node_ptr);
}

/**
 * @brief 对节点进行连续处理 (Spins a node continuously)
 *
 * @param[in] node_ptr 指向节点的共享指针 (Shared pointer to the node)
 */
void rclcpp::spin(rclcpp::Node::SharedPtr node_ptr) {
  // 调用另一个 spin 函数，传入节点基础接口 (Call the other spin function, passing the node base
  // interface)
  rclcpp::spin(node_ptr->get_node_base_interface());
}

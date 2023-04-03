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
 * @brief 主函数
 * @param argc 命令行参数的数量
 * @param argv 命令行参数的数组
 * @return 返回程序执行状态，0 表示成功，其他表示失败
 *
 * @brief Main function
 * @param argc The number of command line arguments
 * @param argv The array of command line arguments
 * @return Returns the program execution status, 0 for success, other values for failure
 */
int main(int argc, char* argv[]) {
  /// 初始化 ROS2 节点 (Initialize the ROS2 node)
  rclcpp::init(argc, argv);

  // 创建一个多线程执行器 (Create a multi-threaded executor)
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // 创建一个组件管理器节点 (Create a component manager node)
  auto node = std::make_shared<rclcpp_components::ComponentManager>();

  // 检查是否有 "thread_num" 参数 (Check if there is a "thread_num" parameter)
  if (node->has_parameter("thread_num")) {
    // 获取 "thread_num" 参数值 (Get the value of the "thread_num" parameter)
    const auto thread_num = node->get_parameter("thread_num").as_int();

    // 根据指定的线程数创建一个多线程执行器 (Create a multi-threaded executor with the specified
    // number of threads)
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions{}, thread_num);

    // 将执行器设置为组件管理器节点 (Set the executor for the component manager node)
    node->set_executor(exec);
  } else {
    // 如果没有 "thread_num" 参数，将默认执行器设置为组件管理器节点 (If there is no "thread_num"
    // parameter, set the default executor for the component manager node)
    node->set_executor(exec);
  }

  // 将组件管理器节点添加到执行器中 (Add the component manager node to the executor)
  exec->add_node(node);

  // 开始执行器循环 (Start the executor loop)
  exec->spin();
}

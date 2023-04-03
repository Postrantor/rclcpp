// Copyright 2021 Open Source Robotics Foundation, Inc.
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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_components/component_manager_isolated.hpp"

/**
 * @brief 主函数，用于初始化 ROS2 节点和执行器，并根据参数选择是否使用多线程执行器。
 *        Main function, used to initialize ROS2 node and executor, and select whether to use
 * multi-threaded executor based on arguments.
 *
 * @param argc 命令行参数个数。Number of command line arguments.
 * @param argv 命令行参数。Command line arguments.
 * @return int 返回值。Return value.
 */
int main(int argc, char* argv[]) {
  // 初始化 ROS2。Initialize ROS2.
  rclcpp::init(argc, argv);

  // 解析命令行参数。Parse command line arguments.
  bool use_multi_threaded_executor{false};
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  // 遍历参数，查找是否指定了使用多线程执行器。Iterate through arguments to find if the
  // multi-threaded executor is specified.
  for (auto& arg : args) {
    if (arg == std::string("--use_multi_threaded_executor")) {
      use_multi_threaded_executor = true;
    }
  }

  // 创建执行器和组件管理器。Create executor and component manager.
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::Node::SharedPtr node;

  // 根据参数选择是否使用多线程执行器。Select whether to use multi-threaded executor based on
  // arguments.
  if (use_multi_threaded_executor) {
    using ComponentManagerIsolated =
        rclcpp_components::ComponentManagerIsolated<rclcpp::executors::MultiThreadedExecutor>;
    node = std::make_shared<ComponentManagerIsolated>(exec);
  } else {
    using ComponentManagerIsolated =
        rclcpp_components::ComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor>;
    node = std::make_shared<ComponentManagerIsolated>(exec);
  }

  // 将节点添加到执行器。Add the node to the executor.
  exec->add_node(node);

  // 执行器开始执行。Start spinning the executor.
  exec->spin();
}

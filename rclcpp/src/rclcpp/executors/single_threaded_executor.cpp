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

#include "rclcpp/executors/single_threaded_executor.hpp"

#include "rclcpp/any_executable.hpp"
#include "rcpputils/scope_exit.hpp"

using rclcpp::executors::SingleThreadedExecutor;

/**
 * @brief 构造函数，用于创建 SingleThreadedExecutor 对象 (Constructor for creating a
 * SingleThreadedExecutor object)
 *
 * @param options ExecutorOptions 对象，包含执行器的配置选项 (An ExecutorOptions object containing
 * the configuration options for the executor)
 */
SingleThreadedExecutor::SingleThreadedExecutor(const rclcpp::ExecutorOptions& options)
    : rclcpp::Executor(options) {}

/**
 * @brief 析构函数，用于销毁 SingleThreadedExecutor 对象 (Destructor for destroying a
 * SingleThreadedExecutor object)
 */
SingleThreadedExecutor::~SingleThreadedExecutor() {}

/**
 * @brief spin 函数，用于执行可执行对象 (The spin function, used to execute executables)
 */
void SingleThreadedExecutor::spin() {
  // 判断是否已经在 spinning，如果是，则抛出异常 (Check if it is already spinning, if so, throw an
  // exception)
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }

  // 在退出作用域时，将 spinning 设置为 false (Set spinning to false when exiting the scope)
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););

  // 当 ROS 正常运行且 spinning 为 true 时，继续执行循环 (Continue the loop when ROS is running
  // normally and spinning is true)
  while (rclcpp::ok(this->context_) && spinning.load()) {
    // 创建一个 AnyExecutable 对象 (Create an AnyExecutable object)
    rclcpp::AnyExecutable any_executable;

    // 获取下一个可执行对象，如果存在则执行 (Get the next executable, if it exists, execute it)
    if (get_next_executable(any_executable)) {
      execute_any_executable(any_executable);
    }
  }
}

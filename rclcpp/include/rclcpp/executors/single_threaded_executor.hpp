// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__SINGLE_THREADED_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__SINGLE_THREADED_EXECUTOR_HPP_

#include <rmw/rmw.h>

#include <cassert>
#include <cstdlib>
#include <memory>
#include <vector>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace executors {

/// 单线程执行器实现 (Single-threaded executor implementation)
/**
 * 这是由 rclcpp::spin 创建的默认执行器 (This is the default executor created by rclcpp::spin)
 */
class SingleThreadedExecutor : public rclcpp::Executor {
public:
  // 定义智能指针类型 (Define smart pointer types)
  RCLCPP_SMART_PTR_DEFINITIONS(SingleThreadedExecutor)

  /// 默认构造函数。参见 Executor 的默认构造函数 (Default constructor. See the default constructor
  /// for Executor)
  /**
   * \param[in] options 执行器选项，默认为 rclcpp::ExecutorOptions() (The executor options, default
   * to rclcpp::ExecutorOptions())
   */
  RCLCPP_PUBLIC
  explicit SingleThreadedExecutor(
      const rclcpp::ExecutorOptions& options = rclcpp::ExecutorOptions());

  /// 默认析构函数 (Default destructor)
  RCLCPP_PUBLIC
  virtual ~SingleThreadedExecutor();

  /// spin 的单线程实现 (Single-threaded implementation of spin)
  /**
   * 此函数将阻塞，直到有工作进入，执行它，然后重复该过程，直到取消 (This function will block until
   * work comes in, execute it, and then repeat the process until canceled) 可以通过调用
   * rclcpp::Executor::cancel() 或者在相关上下文配置为 SIGINT 时关闭来中断它 (It may be interrupt by
   * a call to rclcpp::Executor::cancel() or by ctrl-c if the associated context is configured to
   * shutdown on SIGINT) \throws std::runtime_error 当 spin() 在已经旋转时被调用时抛出 (Throw
   * std::runtime_error when spin() called while already spinning)
   */
  RCLCPP_PUBLIC
  void spin() override;

private:
  // 禁用拷贝构造函数和赋值运算符 (Disable copy constructor and assignment operator)
  RCLCPP_DISABLE_COPY(SingleThreadedExecutor)
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__SINGLE_THREADED_EXECUTOR_HPP_

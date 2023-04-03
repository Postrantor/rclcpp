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

#ifndef RCLCPP__EXECUTORS__MULTI_THREADED_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__MULTI_THREADED_EXECUTOR_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace executors {

/**
 * @class MultiThreadedExecutor
 * @brief 多线程执行器，继承自 rclcpp::Executor 类。
 * @details 该类用于实现 ROS2 节点的多线程执行。
 */
class MultiThreadedExecutor : public rclcpp::Executor {
public:
  // 定义智能指针类型
  RCLCPP_SMART_PTR_DEFINITIONS(MultiThreadedExecutor)

  /**
   * @brief 构造函数
   * @param options 执行器通用选项
   * @param number_of_threads 线程池中的线程数，默认值为 0，表示使用检测到的 CPU 核心数（至少为 2）
   * @param yield_before_execute 如果为 true，在获取工作并释放锁之后，但在执行工作之前调用
   * std::this_thread::yield()
   * @param timeout 最长等待时间
   *
   * Constructor for MultiThreadedExecutor.
   * For the yield_before_execute option, when true std::this_thread::yield()
   * will be called after acquiring work (as an AnyExecutable) and
   * releasing the spinning lock, but before executing the work.
   * This is useful for reproducing some bugs related to taking work more than
   * once.
   *
   * \param options common options for all executors
   * \param number_of_threads number of threads to have in the thread pool,
   *   the default 0 will use the number of cpu cores found (minimum of 2)
   * \param yield_before_execute if true std::this_thread::yield() is called
   * \param timeout maximum time to wait
   */
  RCLCPP_PUBLIC
  explicit MultiThreadedExecutor(
      const rclcpp::ExecutorOptions& options = rclcpp::ExecutorOptions(),
      size_t number_of_threads = 0,
      bool yield_before_execute = false,
      std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  // 析构函数
  RCLCPP_PUBLIC
  virtual ~MultiThreadedExecutor();

  /**
   * @brief 启动执行器
   * @throws std::runtime_error 当 spin() 在已经开始 spinning 时被调用
   *
   * \sa rclcpp::Executor:spin() for more details
   * \throws std::runtime_error when spin() called while already spinning
   */
  RCLCPP_PUBLIC
  void spin() override;

  // 获取线程池中线程的数量
  RCLCPP_PUBLIC
  size_t get_number_of_threads();

protected:
  // 线程运行函数
  RCLCPP_PUBLIC
  void run(size_t this_thread_number);

private:
  // 禁用拷贝构造和赋值操作符
  RCLCPP_DISABLE_COPY(MultiThreadedExecutor)

  // 等待互斥锁
  std::mutex wait_mutex_;
  // 线程数
  size_t number_of_threads_;
  // 是否在执行前调用 yield
  bool yield_before_execute_;
  // 下一个执行的超时时间
  std::chrono::nanoseconds next_exec_timeout_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__MULTI_THREADED_EXECUTOR_HPP_

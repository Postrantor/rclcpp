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

#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"
#include "rcpputils/scope_exit.hpp"

using rclcpp::executors::MultiThreadedExecutor;

/**
 * @brief 构造一个多线程执行器 (Construct a multi-threaded executor)
 *
 * @param options 执行器选项 (Executor options)
 * @param number_of_threads 线程数量 (Number of threads)
 * @param yield_before_execute 在执行前是否让出线程 (Whether to yield the thread before execution)
 * @param next_exec_timeout 下一个可执行任务的超时时间 (Timeout for the next executable task)
 */
MultiThreadedExecutor::MultiThreadedExecutor(
    const rclcpp::ExecutorOptions& options,
    size_t number_of_threads,
    bool yield_before_execute,
    std::chrono::nanoseconds next_exec_timeout)
    : rclcpp::Executor(options),
      yield_before_execute_(yield_before_execute),
      next_exec_timeout_(next_exec_timeout) {
  // 设置线程数量 (Set the number of threads)
  number_of_threads_ =
      number_of_threads > 0 ? number_of_threads : std::max(std::thread::hardware_concurrency(), 2U);

  // 如果线程数量为1，给出警告并建议使用 SingleThreadedExecutor (If the number of threads is 1, give
  // a warning and suggest using SingleThreadedExecutor)
  if (number_of_threads_ == 1) {
    RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "MultiThreadedExecutor is used with a single thread.\n"
        "Use the SingleThreadedExecutor instead.");
  }
}

// 析构函数 (Destructor)
MultiThreadedExecutor::~MultiThreadedExecutor() {}

/**
 * @brief 开始执行 (Start spinning)
 */
void MultiThreadedExecutor::spin() {
  // 如果已经在 spinning，则抛出异常 (If already spinning, throw an exception)
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  {
    std::lock_guard wait_lock{wait_mutex_};
    // 创建线程并绑定 run 函数 (Create threads and bind the run function)
    for (; thread_id < number_of_threads_ - 1; ++thread_id) {
      auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
      threads.emplace_back(func);
    }
  }

  // 在当前线程运行 run 函数 (Run the run function on the current thread)
  run(thread_id);
  // 等待所有线程结束 (Wait for all threads to finish)
  for (auto& thread : threads) {
    thread.join();
  }
}

/**
 * @brief 获取线程数量 (Get the number of threads)
 *
 * @return 线程数量 (Number of threads)
 */
size_t MultiThreadedExecutor::get_number_of_threads() { return number_of_threads_; }

/**
 * @brief 线程运行函数 (Thread running function)
 *
 * @param this_thread_number 当前线程号 (Current thread number)
 */
void MultiThreadedExecutor::run(size_t this_thread_number) {
  (void)this_thread_number;
  // 当 ROS 正常运行且 spinning 为 true 时，执行循环 (Execute the loop when ROS is running normally
  // and spinning is true)
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_exec;
    {
      std::lock_guard wait_lock{wait_mutex_};
      // 如果 ROS 不正常运行或 spinning 为 false，返回 (If ROS is not running normally or spinning
      // is false, return)
      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }
      // 获取下一个可执行任务 (Get the next executable task)
      if (!get_next_executable(any_exec, next_exec_timeout_)) {
        continue;
      }
    }
    // 如果设置了 yield_before_execute，让出线程 (If yield_before_execute is set, yield the thread)
    if (yield_before_execute_) {
      std::this_thread::yield();
    }

    // 执行任意可执行任务 (Execute any executable task)
    execute_any_executable(any_exec);

    // 清除回调组，以防止 AnyExecutable 析构函数重置回调组的 `can_be_taken_from` (Clear the
    // callback_group to prevent the AnyExecutable destructor from resetting the callback group
    // `can_be_taken_from`)
    any_exec.callback_group.reset();
  }
}

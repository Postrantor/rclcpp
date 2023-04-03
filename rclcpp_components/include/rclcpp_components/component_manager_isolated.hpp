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

#ifndef RCLCPP_COMPONENTS__COMPONENT_MANAGER_ISOLATED_HPP__
#define RCLCPP_COMPONENTS__COMPONENT_MANAGER_ISOLATED_HPP__

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp_components/component_manager.hpp"

namespace rclcpp_components {
/**
 * @brief ComponentManagerIsolated 使用专用单线程执行器为每个组件提供服务。
 *        ComponentManagerIsolated uses dedicated single-threaded executors for each components.
 *
 * @tparam ExecutorT 执行器类型，默认为 rclcpp::executors::SingleThreadedExecutor
 *        Executor type, default is rclcpp::executors::SingleThreadedExecutor
 */
template <typename ExecutorT = rclcpp::executors::SingleThreadedExecutor>
class ComponentManagerIsolated : public rclcpp_components::ComponentManager {
  // 使用 rclcpp_components::ComponentManager 的构造函数
  // Use the constructor of rclcpp_components::ComponentManager
  using rclcpp_components::ComponentManager::ComponentManager;

  /**
   * @struct DedicatedExecutorWrapper
   * @brief 用于封装执行器和线程的结构体，以便在独立组件管理器中使用。
   *        A structure to wrap the executor and thread for use in the component manager isolated.
   */
  struct DedicatedExecutorWrapper {
    std::shared_ptr<rclcpp::Executor>
        executor;        ///< 执行器的共享指针。A shared pointer to the executor.
    std::thread thread;  ///< 包含在结构体中的线程。The thread contained within the structure.
    std::atomic_bool thread_initialized;  ///< 原子布尔值，表示线程是否已初始化。An atomic boolean
                                          ///< indicating if the thread is initialized.

    /**
     * @brief 构造函数。由于原子变量没有实现复制/移动操作符，因此默认情况下该结构不可复制/移动。
     *        Constructor for the wrapper. This is necessary as atomic variables don't have
     *        copy/move operators implemented, so this structure is not copyable/movable by default.
     * @param exec 执行器的共享指针。A shared pointer to the executor.
     */
    explicit DedicatedExecutorWrapper(std::shared_ptr<rclcpp::Executor> exec)
        : executor(exec), thread_initialized(false) {}
  };

public:
  ~ComponentManagerIsolated() {
    // 如果 node_wrappers_ 非空，则执行以下操作。
    // If node_wrappers_ is not empty, perform the following actions.
    if (node_wrappers_.size()) {
      // 遍历 dedicated_executor_wrappers_ 中的所有执行器包装器。
      // Iterate through all executor wrappers in dedicated_executor_wrappers_.
      for (auto& executor_wrapper : dedicated_executor_wrappers_) {
        // 取消执行器。
        // Cancel the executor.
        cancel_executor(executor_wrapper.second);
      }
      // 清空 node_wrappers_。
      // Clear node_wrappers_.
      node_wrappers_.clear();
    }
  }

protected:
  /// 添加组件节点到执行器模型，它在 on_load_node() 中被调用
  /// Add component node to executor model, it's invoked in on_load_node()
  /**
   * \param node_id  载入的组件节点在 node_wrappers_ 中的 node_id
   * \param node_id  node_id of loaded component node in node_wrappers_
   */
  void add_node_to_executor(uint64_t node_id) override {
    // 创建一个 ExecutorT 类型的共享指针
    // Create a shared_ptr of ExecutorT type
    auto exec = std::make_shared<ExecutorT>();

    // 将 node_wrappers_ 中对应 node_id 的节点基础接口添加到执行器中
    // Add the node base interface of the corresponding node_id in node_wrappers_ to the executor
    exec->add_node(node_wrappers_[node_id].get_node_base_interface());

    // 使用 emplace 而不是 std::move，因为原子操作不允许移动语义
    // Use emplace rather than std::move since move operations are deleted for atomics
    auto result = dedicated_executor_wrappers_.emplace(std::make_pair(node_id, exec));

    // 获取 DedicatedExecutorWrapper 的引用
    // Get reference to DedicatedExecutorWrapper
    DedicatedExecutorWrapper& wrapper = result.first->second;

    // 设置 DedicatedExecutorWrapper 的 executor 属性
    // Set the executor property of DedicatedExecutorWrapper
    wrapper.executor = exec;

    // 获取 thread_initialized 的引用
    // Get reference to thread_initialized
    auto& thread_initialized = wrapper.thread_initialized;

    // 创建一个新线程，将执行器传递给它并在该线程上执行 spin()
    // Create a new thread, pass the executor to it and execute spin() on that thread
    wrapper.thread = std::thread([exec, &thread_initialized]() {
      // 设置线程已初始化的标志
      // Set the flag for thread initialized
      thread_initialized = true;

      // 在新线程上执行执行器的 spin() 函数
      // Execute the spin() function of the executor on the new thread
      exec->spin();
    });
  }

  /// 从执行器模型中移除组件节点，它在 on_unload_node() 中被调用
  /// Remove component node from executor model, it's invoked in on_unload_node()
  /**
   * \param node_id  载入的组件节点在 node_wrappers_ 中的 node_id
   * \param node_id  node_id of loaded component node in node_wrappers_
   */
  void remove_node_from_executor(uint64_t node_id) override {
    // 查找指定 node_id 的执行器包装器
    // Find the executor wrapper for the specified node_id
    auto executor_wrapper = dedicated_executor_wrappers_.find(node_id);

    // 如果找到了指定的执行器包装器
    // If the specified executor wrapper is found
    if (executor_wrapper != dedicated_executor_wrappers_.end()) {
      // 取消执行器
      // Cancel the executor
      cancel_executor(executor_wrapper->second);

      // 从 dedicated_executor_wrappers_ 中删除找到的执行器包装器
      // Erase the found executor wrapper from dedicated_executor_wrappers_
      dedicated_executor_wrappers_.erase(executor_wrapper);
    }
  }

private:
  /// 停止一个旋转执行器，避免竞争条件。
  /// Stops a spinning executor avoiding race conditions.
  /**
   * @param executor_wrapper 要停止的执行器及其关联的线程
   * @param executor_wrapper executor to stop and its associated thread
   */
  void cancel_executor(DedicatedExecutorWrapper& executor_wrapper) {
    // 验证执行器线程是否已开始旋转。
    // 如果没有，则等待线程开始以确保
    // cancel() 将完全停止执行
    //
    // 这可以防止在创建执行器旋转线程和取消执行器之间发生的先前竞争条件
    //
    // Verify that the executor thread has begun spinning.
    // If it has not, then wait until the thread starts to ensure
    // that cancel() will fully stop the execution
    //
    // This prevents a previous race condition that occurs between the
    // creation of the executor spin thread and cancelling an executor

    if (!executor_wrapper.thread_initialized) {
      // 获取节点基本接口的上下文
      // Get the context from the node base interface
      auto context = this->get_node_base_interface()->get_context();

      // 确保执行器正在旋转或我们正在关闭。
      // Guarantee that either the executor is spinning or we are shutting down.
      while (!executor_wrapper.executor->is_spinning() && rclcpp::ok(context)) {
        // 这是一个任意的小延迟，以避免忙循环
        // This is an arbitrarily small delay to avoid busy looping
        rclcpp::sleep_for(std::chrono::milliseconds(1));
      }
    }

    // 在 while 循环之后，我们确定执行器现在正在旋转，所以
    // 对 cancel() 的调用将起作用。
    // After the while loop we are sure that the executor is now spinning, so
    // the call to cancel() will work.
    executor_wrapper.executor->cancel();
    // 等待线程任务返回
    // Wait for the thread task to return
    executor_wrapper.thread.join();
  }

  // 定义一个无序映射，存储 DedicatedExecutorWrapper 类型的对象
  // Define an unordered_map to store DedicatedExecutorWrapper type objects
  std::unordered_map<uint64_t, DedicatedExecutorWrapper> dedicated_executor_wrappers_;
};

}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__COMPONENT_MANAGER_ISOLATED_HPP__

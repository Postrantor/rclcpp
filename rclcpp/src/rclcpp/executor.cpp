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

#include "rclcpp/executor.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "rcl/allocator.h"
#include "rcl/error_handling.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rcpputils/scope_exit.hpp"
#include "rcutils/logging_macros.h"
#include "tracetools/tracetools.h"

using namespace std::chrono_literals;

using rclcpp::AnyExecutable;
using rclcpp::Executor;
using rclcpp::ExecutorOptions;
using rclcpp::FutureReturnCode;
using rclcpp::exceptions::throw_from_rcl_error;

/**
 * @brief 构造函数，用于初始化 Executor 类的对象 (Constructor for initializing an object of the
 * Executor class)
 *
 * @param options 包含 rclcpp::ExecutorOptions 的对象，用于配置 Executor (An object containing
 * rclcpp::ExecutorOptions, used to configure the Executor)
 */
Executor::Executor(const rclcpp::ExecutorOptions &options)
    : spinning(false),
      interrupt_guard_condition_(options.context),
      shutdown_guard_condition_(std::make_shared<rclcpp::GuardCondition>(options.context)),
      memory_strategy_(options.memory_strategy) {
  // 存储稍后使用的上下文 (Store the context for later use)
  context_ = options.context;

  // 添加一个关机回调 (Add a shutdown callback)
  shutdown_callback_handle_ = context_->add_on_shutdown_callback(
      [weak_gc = std::weak_ptr<rclcpp::GuardCondition>{shutdown_guard_condition_}]() {
        // 检查弱指针是否可用 (Check if the weak pointer is valid)
        auto strong_gc = weak_gc.lock();
        if (strong_gc) {
          // 触发强指针对应的 GuardCondition (Trigger the GuardCondition corresponding to the strong
          // pointer)
          strong_gc->trigger();
        }
      });

  // 至少有2个 GuardConditions：1个用于 ctrl-c guard cond，另1个用于执行器的 guard cond
  // (interrupt_guard_condition_) (The number of guard conditions is always at least 2: 1 for the
  // ctrl-c guard cond, and one for the executor's guard cond (interrupt_guard_condition_))
  memory_strategy_->add_guard_condition(*shutdown_guard_condition_.get());

  // 添加执行器的 guard condition (Put the executor's guard condition in)
  memory_strategy_->add_guard_condition(interrupt_guard_condition_);
  rcl_allocator_t allocator = memory_strategy_->get_allocator();

  // 初始化 wait_set (Initialize the wait_set)
  rcl_ret_t ret =
      rcl_wait_set_init(&wait_set_, 0, 2, 0, 0, 0, 0, context_->get_rcl_context().get(), allocator);
  if (RCL_RET_OK != ret) {
    // 输出错误信息 (Output error information)
    RCUTILS_LOG_ERROR_NAMED("rclcpp", "failed to create wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
    // 抛出异常 (Throw an exception)
    throw_from_rcl_error(ret, "Failed to create wait set in Executor constructor");
  }
}

/**
 * @brief Executor 析构函数，用于在 Executor 销毁时解除与回调组、节点的关联，并清理资源。
 * @details Executor destructor, used to disassociate callback groups and nodes when the executor is
 * destroyed, and clean up resources.
 */
Executor::~Executor() {
  // 遍历 weak_groups_to_nodes_，解除回调组与执行器的关联。
  // Traverse weak_groups_to_nodes_, disassociating callback groups from the executor.
  for (auto &pair : weak_groups_to_nodes_) {
    auto group = pair.first.lock();
    if (group) {
      std::atomic_bool &has_executor = group->get_associated_with_executor_atomic();
      has_executor.store(false);
    }
  }

  // 遍历 weak_nodes_，解除节点与执行器的关联。
  // Traverse weak_nodes_, disassociating nodes from the executor.
  std::for_each(
      weak_nodes_.begin(), weak_nodes_.end(),
      [](rclcpp::node_interfaces::NodeBaseInterface::WeakPtr weak_node_ptr) {
        auto shared_node_ptr = weak_node_ptr.lock();
        if (shared_node_ptr) {
          std::atomic_bool &has_executor = shared_node_ptr->get_associated_with_executor_atomic();
          has_executor.store(false);
        }
      });

  // 清空存储的弱指针容器。
  // Clear the stored weak pointer containers.
  weak_nodes_.clear();
  weak_groups_associated_with_executor_to_nodes_.clear();
  weak_groups_to_nodes_associated_with_executor_.clear();
  weak_groups_to_nodes_.clear();

  // 移除并释放与回调组相关的 guard_conditions。
  // Remove and release guard_conditions related to callback groups.
  for (const auto &pair : weak_groups_to_guard_conditions_) {
    auto guard_condition = pair.second;
    memory_strategy_->remove_guard_condition(guard_condition);
  }
  weak_groups_to_guard_conditions_.clear();

  // 结束 wait_set。
  // Finalize the wait_set.
  if (rcl_wait_set_fini(&wait_set_) != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED("rclcpp", "failed to destroy wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

  // 移除并释放 sigint guard condition
  // Remove and release the sigint guard condition
  memory_strategy_->remove_guard_condition(shutdown_guard_condition_.get());
  memory_strategy_->remove_guard_condition(&interrupt_guard_condition_);

  // 从 Context 中移除已注册的 shutdown 回调句柄
  // Remove the registered shutdown callback handle from the Context
  if (!context_->remove_on_shutdown_callback(shutdown_callback_handle_)) {
    RCUTILS_LOG_ERROR_NAMED("rclcpp", "failed to remove registered on_shutdown callback");
    rcl_reset_error();
  }
}

/**
 * @brief 获取所有回调组 (Get all callback groups)
 *
 * @return std::vector<rclcpp::CallbackGroup::WeakPtr> 回调组列表 (List of callback groups)
 */
std::vector<rclcpp::CallbackGroup::WeakPtr> Executor::get_all_callback_groups() {
  // 创建一个回调组的向量 (Create a vector for callback groups)
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;

  // 对互斥锁进行加锁，保护共享资源 (Lock the mutex to protect shared resources)
  std::lock_guard<std::mutex> guard{mutex_};

  // 遍历与执行器关联的弱回调组映射 (Iterate through the weak callback group map associated with the
  // executor)
  for (const auto &group_node_ptr : weak_groups_associated_with_executor_to_nodes_) {
    // 将回调组添加到向量中 (Add the callback group to the vector)
    groups.push_back(group_node_ptr.first);
  }

  // 遍历与执行器关联的节点的弱回调组映射 (Iterate through the weak callback group map of nodes
  // associated with the executor)
  for (auto const &group_node_ptr : weak_groups_to_nodes_associated_with_executor_) {
    // 将回调组添加到向量中 (Add the callback group to the vector)
    groups.push_back(group_node_ptr.first);
  }

  // 返回回调组的向量 (Return the vector of callback groups)
  return groups;
}

/**
 * @brief 获取手动添加的回调组 (Get manually added callback groups)
 *
 * @return std::vector<rclcpp::CallbackGroup::WeakPtr> 回调组列表 (List of callback groups)
 */
std::vector<rclcpp::CallbackGroup::WeakPtr> Executor::get_manually_added_callback_groups() {
  // 创建一个回调组的向量 (Create a vector for callback groups)
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;

  // 对互斥锁进行加锁，保护共享资源 (Lock the mutex to protect shared resources)
  std::lock_guard<std::mutex> guard{mutex_};

  // 遍历与执行器关联的弱回调组映射 (Iterate through the weak callback group map associated with the
  // executor)
  for (auto const &group_node_ptr : weak_groups_associated_with_executor_to_nodes_) {
    // 将回调组添加到向量中 (Add the callback group to the vector)
    groups.push_back(group_node_ptr.first);
  }

  // 返回回调组的向量 (Return the vector of callback groups)
  return groups;
}

/**
 * @brief 获取从节点自动添加的回调组 (Get automatically added callback groups from nodes)
 *
 * @return std::vector<rclcpp::CallbackGroup::WeakPtr> 回调组列表 (List of callback groups)
 */
std::vector<rclcpp::CallbackGroup::WeakPtr>
Executor::get_automatically_added_callback_groups_from_nodes() {
  // 创建一个回调组的向量 (Create a vector for callback groups)
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;

  // 对互斥锁进行加锁，保护共享资源 (Lock the mutex to protect shared resources)
  std::lock_guard<std::mutex> guard{mutex_};

  // 遍历与执行器关联的节点的弱回调组映射 (Iterate through the weak callback group map of nodes
  // associated with the executor)
  for (auto const &group_node_ptr : weak_groups_to_nodes_associated_with_executor_) {
    // 将回调组添加到向量中 (Add the callback group to the vector)
    groups.push_back(group_node_ptr.first);
  }

  // 返回回调组的向量 (Return the vector of callback groups)
  return groups;
}

/**
 * @brief 添加与执行器关联的节点的回调组 (Add callback groups from nodes associated with the
 * executor)
 */
void Executor::add_callback_groups_from_nodes_associated_to_executor() {
  // 遍历弱节点列表 (Iterate through the list of weak nodes)
  for (auto &weak_node : weak_nodes_) {
    // 尝试锁定弱节点 (Try to lock the weak node)
    auto node = weak_node.lock();

    // 如果节点存在 (If the node exists)
    if (node) {
      // 对每个回调组执行以下操作 (Do the following for each callback group)
      node->for_each_callback_group(
          [this, node](rclcpp::CallbackGroup::SharedPtr shared_group_ptr) {
            // 如果回调组应自动添加到执行器，并且尚未关联 (If the callback group should be
            // automatically added to the executor and is not yet associated)
            if (shared_group_ptr->automatically_add_to_executor_with_node() &&
                !shared_group_ptr->get_associated_with_executor_atomic().load()) {
              // 将回调组添加到映射中 (Add the callback group to the map)
              this->add_callback_group_to_map(
                  shared_group_ptr, node, weak_groups_to_nodes_associated_with_executor_, true);
            }
          });
    }
  }
}

/**
 * @brief 添加回调组到映射表中 (Add callback group to the map)
 *
 * @param group_ptr 回调组共享指针 (Shared pointer of the callback group)
 * @param node_ptr 节点基本接口的共享指针 (Shared pointer of the node base interface)
 * @param weak_groups_to_nodes 弱回调组到节点映射表的引用 (Reference to the weak callback groups to
 * nodes map)
 * @param notify 是否通知中断 (Whether to notify interrupt)
 */
void Executor::add_callback_group_to_map(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap &weak_groups_to_nodes,
    bool notify) {
  // 如果回调组已经有一个执行器 (If the callback group already has an executor)
  std::atomic_bool &has_executor = group_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Callback group has already been added to an executor.");
  }

  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;
  auto insert_info = weak_groups_to_nodes.insert(std::make_pair(weak_group_ptr, node_ptr));
  bool was_inserted = insert_info.second;
  if (!was_inserted) {
    throw std::runtime_error("Callback group was already added to executor.");
  }
  // 同时添加到包含所有回调组的映射表中 (Also add to the map that contains all callback groups)
  weak_groups_to_nodes_.insert(std::make_pair(weak_group_ptr, node_ptr));

  if (node_ptr->get_context()->is_valid()) {
    auto callback_group_guard_condition =
        group_ptr->get_notify_guard_condition(node_ptr->get_context());
    weak_groups_to_guard_conditions_[weak_group_ptr] = callback_group_guard_condition.get();
    // 将回调组的通知条件添加到守卫条件句柄中 (Add the callback group's notify condition to the
    // guard condition handles)
    memory_strategy_->add_guard_condition(*callback_group_guard_condition);
  }

  if (notify) {
    // 中断等待以处理新节点 (Interrupt waiting to handle new node)
    try {
      interrupt_guard_condition_.trigger();
    } catch (const rclcpp::exceptions::RCLError &ex) {
      throw std::runtime_error(
          std::string("Failed to trigger guard condition on callback group add: ") + ex.what());
    }
  }
}

/**
 * @brief 添加回调组 (Add callback group)
 *
 * @param group_ptr 回调组共享指针 (Shared pointer of the callback group)
 * @param node_ptr 节点基本接口的共享指针 (Shared pointer of the node base interface)
 * @param notify 是否通知中断 (Whether to notify interrupt)
 */
void Executor::add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    bool notify) {
  // 锁定互斥体以保护数据 (Lock the mutex to protect data)
  std::lock_guard<std::mutex> guard{mutex_};
  this->add_callback_group_to_map(
      group_ptr, node_ptr, weak_groups_associated_with_executor_to_nodes_, notify);
}

/**
 * @brief 添加节点到执行器 (Add a node to the executor)
 *
 * @param node_ptr 要添加的节点共享指针 (The shared pointer of the node to add)
 * @param notify 是否触发中断保护条件 (Whether to trigger the interrupt guard condition)
 */
void Executor::add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify) {
  // 检查节点是否已经有执行器 (Check if the node already has an executor)
  std::atomic_bool &has_executor = node_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error(
        std::string("Node '") + node_ptr->get_fully_qualified_name() +
        "' has already been added to an executor.");
  }
  // 对互斥量进行加锁，防止多线程问题 (Lock the mutex to prevent multithreading issues)
  std::lock_guard<std::mutex> guard{mutex_};
  // 遍历节点的回调组 (Iterate through the node's callback groups)
  node_ptr->for_each_callback_group(
      [this, node_ptr, notify](rclcpp::CallbackGroup::SharedPtr group_ptr) {
        // 如果回调组未与执行器关联且自动添加到执行器 (If the callback group is not associated with
        // an executor and is set to be automatically added)
        if (!group_ptr->get_associated_with_executor_atomic().load() &&
            group_ptr->automatically_add_to_executor_with_node()) {
          this->add_callback_group_to_map(
              group_ptr, node_ptr, weak_groups_to_nodes_associated_with_executor_, notify);
        }
      });

  // 将节点添加到 weak_nodes_ 中 (Add the node to weak_nodes_)
  weak_nodes_.push_back(node_ptr);
}

/**
 * @brief 从映射中移除回调组 (Remove a callback group from the map)
 *
 * @param group_ptr 要移除的回调组共享指针 (The shared pointer of the callback group to remove)
 * @param weak_groups_to_nodes 弱回调组到节点的映射 (The weak callback groups to nodes map)
 * @param notify 是否触发中断保护条件 (Whether to trigger the interrupt guard condition)
 */
void Executor::remove_callback_group_from_map(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap &weak_groups_to_nodes,
    bool notify) {
  // 定义节点共享指针 (Define the node shared pointer)
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr;
  // 定义弱回调组指针 (Define the weak callback group pointer)
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;
  // 查找弱回调组在映射中的位置 (Find the position of the weak callback group in the map)
  auto iter = weak_groups_to_nodes.find(weak_group_ptr);
  if (iter != weak_groups_to_nodes.end()) {
    // 获取对应的节点 (Get the corresponding node)
    node_ptr = iter->second.lock();
    if (node_ptr == nullptr) {
      throw std::runtime_error("Node must not be deleted before its callback group(s).");
    }
    // 从映射中删除回调组 (Remove the callback group from the map)
    weak_groups_to_nodes.erase(iter);
    weak_groups_to_nodes_.erase(group_ptr);
    // 更新回调组的关联执行器标志 (Update the associated executor flag of the callback group)
    std::atomic_bool &has_executor = group_ptr->get_associated_with_executor_atomic();
    has_executor.store(false);
  } else {
    throw std::runtime_error("Callback group needs to be associated with executor.");
  }
  // 如果节点已被匹配并移除，中断等待 (If the node was matched and removed, interrupt waiting)
  if (!has_node(node_ptr, weak_groups_to_nodes_associated_with_executor_) &&
      !has_node(node_ptr, weak_groups_associated_with_executor_to_nodes_)) {
    auto iter = weak_groups_to_guard_conditions_.find(weak_group_ptr);
    if (iter != weak_groups_to_guard_conditions_.end()) {
      memory_strategy_->remove_guard_condition(iter->second);
    }
    weak_groups_to_guard_conditions_.erase(weak_group_ptr);

    // 如果需要通知，则触发中断保护条件 (If notification is required, trigger the interrupt guard
    // condition)
    if (notify) {
      try {
        interrupt_guard_condition_.trigger();
      } catch (const rclcpp::exceptions::RCLError &ex) {
        throw std::runtime_error(
            std::string("Failed to trigger guard condition on callback group remove: ") +
            ex.what());
      }
    }
  }
}

/**
 * @brief 移除回调组 (Remove a callback group)
 *
 * @param group_ptr 要移除的回调组指针 (Pointer to the callback group to remove)
 * @param notify 是否通知 (Whether to notify or not)
 */
void Executor::remove_callback_group(rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify) {
  // 加锁保护 (Lock guard for protection)
  std::lock_guard<std::mutex> guard{mutex_};

  // 从映射中移除回调组 (Remove the callback group from the map)
  this->remove_callback_group_from_map(
      group_ptr, weak_groups_associated_with_executor_to_nodes_, notify);
}

/**
 * @brief 添加节点 (Add a node)
 *
 * @param node_ptr 要添加的节点指针 (Pointer to the node to add)
 * @param notify 是否通知 (Whether to notify or not)
 */
void Executor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify) {
  // 添加节点到基本接口 (Add the node to the base interface)
  this->add_node(node_ptr->get_node_base_interface(), notify);
}

/**
 * @brief 移除节点 (Remove a node)
 *
 * @param node_ptr 要移除的节点指针 (Pointer to the node to remove)
 * @param notify 是否通知 (Whether to notify or not)
 */
void Executor::remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify) {
  // 检查节点是否与执行器关联 (Check if the node is associated with an executor)
  if (!node_ptr->get_associated_with_executor_atomic().load()) {
    throw std::runtime_error("Node needs to be associated with an executor.");
  }

  // 加锁保护 (Lock guard for protection)
  std::lock_guard<std::mutex> guard{mutex_};

  // 初始化节点查找标记 (Initialize node found flag)
  bool found_node = false;

  // 遍历 weak_nodes_ 寻找要移除的节点 (Iterate through weak_nodes_ to find the node to remove)
  auto node_it = weak_nodes_.begin();
  while (node_it != weak_nodes_.end()) {
    bool matched = (node_it->lock() == node_ptr);
    if (matched) {
      found_node = true;
      node_it = weak_nodes_.erase(node_it);
    } else {
      ++node_it;
    }
  }

  // 如果未找到节点，则抛出异常 (If the node is not found, throw an exception)
  if (!found_node) {
    throw std::runtime_error("Node needs to be associated with this executor.");
  }

  // 遍历 weak_groups_to_nodes_associated_with_executor_ 移除回调组 (Iterate through
  // weak_groups_to_nodes_associated_with_executor_ to remove callback groups)
  for (auto it = weak_groups_to_nodes_associated_with_executor_.begin();
       it != weak_groups_to_nodes_associated_with_executor_.end();) {
    auto weak_node_ptr = it->second;
    auto shared_node_ptr = weak_node_ptr.lock();
    auto group_ptr = it->first.lock();

    // 在移除之前增加迭代器，以防止失效 (Increment iterator before removing in case it's
    // invalidated)
    it++;
    if (shared_node_ptr == node_ptr) {
      remove_callback_group_from_map(
          group_ptr, weak_groups_to_nodes_associated_with_executor_, notify);
    }
  }

  // 设置节点与执行器关联标记为 false (Set the node associated with executor flag to false)
  std::atomic_bool &has_executor = node_ptr->get_associated_with_executor_atomic();
  has_executor.store(false);
}

/**
 * @brief 删除节点
 * @param node_ptr 要删除的节点指针
 * @param notify 是否通知其他对象
 *
 * @brief Remove a node
 * @param node_ptr Pointer to the node to be removed
 * @param notify Whether to notify other objects
 */
void Executor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify) {
  // 调用 remove_node 函数，传入基本接口和是否通知标志
  // Call the remove_node function with the base interface and the notify flag
  this->remove_node(node_ptr->get_node_base_interface(), notify);
}

/**
 * @brief 在指定超时时间内旋转一次节点
 * @param node 要旋转的节点
 * @param timeout 超时时间（纳秒）
 *
 * @brief Spin a node once within the specified timeout
 * @param node Node to spin
 * @param timeout Timeout duration (nanoseconds)
 */
void Executor::spin_node_once_nanoseconds(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node, std::chrono::nanoseconds timeout) {
  // 添加节点，不通知其他对象
  // Add the node without notifying other objects
  this->add_node(node, false);

  // 设置非阻塞标志为 true 并执行 spin_once
  // Set non-blocking flag to true and perform spin_once
  spin_once(timeout);

  // 移除节点，不通知其他对象
  // Remove the node without notifying other objects
  this->remove_node(node, false);
}

/**
 * @brief 旋转节点若干次
 * @param node 要旋转的节点
 *
 * @brief Spin a node some times
 * @param node Node to spin
 */
void Executor::spin_node_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node) {
  // 添加节点，不通知其他对象
  // Add the node without notifying other objects
  this->add_node(node, false);

  // 执行 spin_some
  // Execute spin_some
  spin_some();

  // 移除节点，不通知其他对象
  // Remove the node without notifying other objects
  this->remove_node(node, false);
}

/**
 * @brief 旋转节点若干次
 * @param node 要旋转的节点
 *
 * @brief Spin a node some times
 * @param node Node to spin
 */
void Executor::spin_node_some(std::shared_ptr<rclcpp::Node> node) {
  // 使用基本接口执行 spin_node_some
  // Perform spin_node_some using the base interface
  this->spin_node_some(node->get_node_base_interface());
}

/**
 * @brief 在最大持续时间内旋转若干次
 * @param max_duration 最大持续时间（纳秒）
 *
 * @brief Spin some times within the maximum duration
 * @param max_duration Maximum duration (nanoseconds)
 */
void Executor::spin_some(std::chrono::nanoseconds max_duration) {
  // 调用 spin_some_impl 函数
  // Call the spin_some_impl function
  return this->spin_some_impl(max_duration, false);
}

/**
 * @brief 在最大持续时间内旋转所有次数
 * @param max_duration 最大持续时间（纳秒）
 *
 * @brief Spin all times within the maximum duration
 * @param max_duration Maximum duration (nanoseconds)
 */
void Executor::spin_all(std::chrono::nanoseconds max_duration) {
  // 检查 max_duration 是否小于 0
  // Check if max_duration is less than 0
  if (max_duration < 0ns) {
    throw std::invalid_argument("max_duration must be greater than or equal to 0");
  }

  // 调用 spin_some_impl 函数
  // Call the spin_some_impl function
  return this->spin_some_impl(max_duration, true);
}

/**
 * @brief 实现旋转若干次或所有次数的逻辑
 * @param max_duration 最大持续时间（纳秒）
 * @param exhaustive 是否旋转所有次数
 *
 * @brief Implement the logic for spinning some times or all times
 * @param max_duration Maximum duration (nanoseconds)
 * @param exhaustive Whether to spin all times
 */
void Executor::spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive) {
  // 省略具体实现，详见代码注释
  // Omitted specific implementation, see code comments for details
}

/**
 * @brief 在指定超时时间内旋转一次的实现
 * @param timeout 超时时间（纳秒）
 *
 * @brief Implementation of spinning once within the specified timeout
 * @param timeout Timeout duration (nanoseconds)
 */
void Executor::spin_once_impl(std::chrono::nanoseconds timeout) {
  // 创建 AnyExecutable 对象
  // Create an AnyExecutable object
  AnyExecutable any_exec;

  // 获取下一个可执行对象
  // Get the next executable object
  if (get_next_executable(any_exec, timeout)) {
    // 执行任意可执行对象
    // Execute any executable object
    execute_any_executable(any_exec);
  }
}

/**
 * @brief 执行器执行一次 (Execute the executor once)
 *
 * @param timeout 超时时间 (Timeout duration)
 *
 * @throws std::runtime_error 如果在已经旋转的情况下调用 spin_once() (If spin_once() is called while
 * already spinning)
 */
void Executor::spin_once(std::chrono::nanoseconds timeout) {
  // 检查是否已经在旋转，如果是，则抛出异常 (Check if already spinning, if so, throw an exception)
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_once() called while already spinning");
  }
  // 确保在作用域结束时停止旋转 (Ensure spinning stops when scope ends)
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););
  // 实际执行 spin_once 的实现 (Actually execute the spin_once implementation)
  spin_once_impl(timeout);
}

/**
 * @brief 取消执行器的旋转 (Cancel the executor's spinning)
 *
 * @throws std::runtime_error 如果触发 guard condition 失败 (If triggering the guard condition
 * fails)
 */
void Executor::cancel() {
  // 停止旋转 (Stop spinning)
  spinning.store(false);
  try {
    // 触发中断 guard condition (Trigger the interrupt guard condition)
    interrupt_guard_condition_.trigger();
  } catch (const rclcpp::exceptions::RCLError &ex) {
    throw std::runtime_error(
        std::string("Failed to trigger guard condition in cancel: ") + ex.what());
  }
}

/**
 * @brief 设置内存策略 (Set the memory strategy)
 *
 * @param memory_strategy 内存策略共享指针 (Shared pointer to the memory strategy)
 *
 * @throws std::runtime_error 如果接收到空的内存策略 (If a NULL memory strategy is received)
 */
void Executor::set_memory_strategy(
    rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy) {
  // 检查内存策略是否为空，如果是，则抛出异常 (Check if the memory strategy is null, if so, throw an
  // exception)
  if (memory_strategy == nullptr) {
    throw std::runtime_error("Received NULL memory strategy in executor.");
  }
  // 使用互斥锁保护内存策略的设置 (Use a mutex lock to protect the setting of the memory strategy)
  std::lock_guard<std::mutex> guard{mutex_};
  memory_strategy_ = memory_strategy;
}

/**
 * @brief 执行任意可执行对象（Execute any executable object）
 *
 * @param any_exec 任意可执行对象的引用（Reference to any executable object）
 */
void Executor::execute_any_executable(AnyExecutable &any_exec) {
  // 如果当前没有在 spinning，则直接返回（If not currently spinning, return directly）
  if (!spinning.load()) {
    return;
  }
  // 如果有定时器对象，执行定时器（If there is a timer object, execute the timer）
  if (any_exec.timer) {
    TRACEPOINT(
        rclcpp_executor_execute,
        static_cast<const void *>(any_exec.timer->get_timer_handle().get()));
    execute_timer(any_exec.timer);
  }
  // 如果有订阅对象，执行订阅（If there is a subscription object, execute the subscription）
  if (any_exec.subscription) {
    TRACEPOINT(
        rclcpp_executor_execute,
        static_cast<const void *>(any_exec.subscription->get_subscription_handle().get()));
    execute_subscription(any_exec.subscription);
  }
  // 如果有服务对象，执行服务（If there is a service object, execute the service）
  if (any_exec.service) {
    execute_service(any_exec.service);
  }
  // 如果有客户端对象，执行客户端（If there is a client object, execute the client）
  if (any_exec.client) {
    execute_client(any_exec.client);
  }
  // 如果有等待对象，执行等待对象（If there is a waitable object, execute the waitable object）
  if (any_exec.waitable) {
    any_exec.waitable->execute(any_exec.data);
  }
  // 重置回调组，无论类型如何（Reset the callback_group, regardless of type）
  any_exec.callback_group->can_be_taken_from().store(true);
  // 唤醒等待，因为它可能需要重新计算或者之前被阻塞的工作现在可用（Wake the wait, because it may
  // need to be recalculated or work that was previously blocked is now available）
  try {
    interrupt_guard_condition_.trigger();
  } catch (const rclcpp::exceptions::RCLError &ex) {
    throw std::runtime_error(
        std::string("Failed to trigger guard condition from execute_any_executable: ") + ex.what());
  }
}

/**
 * @brief 执行并处理错误（Take and do error handling）
 *
 * @param action_description 动作描述（Action description）
 * @param topic_or_service_name 主题或服务名称（Topic or service name）
 * @param take_action 执行动作的函数对象（Function object for taking action）
 * @param handle_action 处理动作的函数对象（Function object for handling action）
 */
static void take_and_do_error_handling(
    const char *action_description,
    const char *topic_or_service_name,
    std::function<bool()> take_action,
    std::function<void()> handle_action) {
  bool taken = false;
  try {
    taken = take_action();
  } catch (const rclcpp::exceptions::RCLError &rcl_error) {
    RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "executor %s '%s' unexpectedly failed: %s",
        action_description, topic_or_service_name, rcl_error.what());
  }
  if (taken) {
    handle_action();
  } else {
    // 消息或服务未被获取（Message or Service was not taken for some reason）
    // 注意，这可能是正常的，如果底层中间件需要中断等待，它是允许的（Note that this can be normal,
    // if the underlying middleware needs to interrupt wait spuriously it is allowed）
    // 所以在这种情况下，执行器不能在唤醒和实体实际拥有数据之前尝试获取数据（So in that case the
    // executor cannot tell the difference in a spurious wake up and an entity actually having data
    // until trying to take the data）
    RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "executor %s '%s' failed to take anything",
        action_description, topic_or_service_name);
  }
}

/**
 * @brief 执行订阅操作，处理订阅到的消息 (Execute the subscription operation and handle the
 * subscribed messages)
 *
 * @param[in] subscription 订阅对象的智能指针 (A shared pointer to the subscription object)
 */
void Executor::execute_subscription(rclcpp::SubscriptionBase::SharedPtr subscription) {
  // 创建一个消息信息对象 (Create a message info object)
  rclcpp::MessageInfo message_info;
  // 设置消息来源为非内部进程 (Set the message source to be not from an intra-process)
  message_info.get_rmw_message_info().from_intra_process = false;

  // 如果订阅的消息是序列化的 (If the subscribed message is serialized)
  if (subscription->is_serialized()) {
    // 从中间件通过进程间通信获取序列化消息的副本 (Get a copy of the serialized message from the
    // middleware via inter-process communication)
    std::shared_ptr<SerializedMessage> serialized_msg = subscription->create_serialized_message();
    take_and_do_error_handling(
        "taking a serialized message from topic", subscription->get_topic_name(),
        [&]() { return subscription->take_serialized(*serialized_msg.get(), message_info); },
        [&]() { subscription->handle_serialized_message(serialized_msg, message_info); });
    // 返回序列化消息 (Return the serialized message)
    subscription->return_serialized_message(serialized_msg);
  } else if (subscription->can_loan_messages()) {
    // 从中间件通过进程间通信获取借用的消息，传递给用户回调，然后返回 (Get a loaned message from the
    // middleware via inter-process communication, pass it to the user callback, and then return it)
    void *loaned_msg = nullptr;
    // TODO(wjwwood): refactor this into methods on subscription when LoanedMessage
    //   is extened to support subscriptions as well.
    take_and_do_error_handling(
        "taking a loaned message from topic", subscription->get_topic_name(),
        [&]() {
          rcl_ret_t ret = rcl_take_loaned_message(
              subscription->get_subscription_handle().get(), &loaned_msg,
              &message_info.get_rmw_message_info(), nullptr);
          if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
            return false;
          } else if (RCL_RET_OK != ret) {
            rclcpp::exceptions::throw_from_rcl_error(ret);
          }
          return true;
        },
        [&]() { subscription->handle_loaned_message(loaned_msg, message_info); });
    // 如果借用的消息不为空，则返回借用的消息 (If the loaned message is not null, return the loaned
    // message)
    if (nullptr != loaned_msg) {
      rcl_ret_t ret = rcl_return_loaned_message_from_subscription(
          subscription->get_subscription_handle().get(), loaned_msg);
      if (RCL_RET_OK != ret) {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "rcl_return_loaned_message_from_subscription() failed for subscription on topic '%s': "
            "%s",
            subscription->get_topic_name(), rcl_get_error_string().str);
      }
      loaned_msg = nullptr;
    }
  } else {
    // 从中间件通过进程间通信获取消息数据的副本 (Get a copy of the message data from the middleware
    // via inter-process communication)
    std::shared_ptr<void> message = subscription->create_message();
    take_and_do_error_handling(
        "taking a message from topic", subscription->get_topic_name(),
        [&]() { return subscription->take_type_erased(message.get(), message_info); },
        [&]() { subscription->handle_message(message, message_info); });
    // 返回消息 (Return the message)
    subscription->return_message(message);
  }
}

/**
 * @brief 执行定时器回调 (Execute timer callback)
 *
 * @param timer 定时器智能指针 (Timer shared pointer)
 */
void Executor::execute_timer(rclcpp::TimerBase::SharedPtr timer) {
  // 执行定时器回调 (Execute the timer callback)
  timer->execute_callback();
}

/**
 * @brief 执行服务端请求处理 (Execute service server request handling)
 *
 * @param service 服务端智能指针 (Service server shared pointer)
 */
void Executor::execute_service(rclcpp::ServiceBase::SharedPtr service) {
  // 创建请求头 (Create request header)
  auto request_header = service->create_request_header();

  // 创建请求 (Create request)
  std::shared_ptr<void> request = service->create_request();

  // 获取并处理错误 (Take and handle errors)
  take_and_do_error_handling(
      "taking a service server request from service", service->get_service_name(),
      // 获取类型擦除的请求 (Take type erased request)
      [&]() { return service->take_type_erased_request(request.get(), *request_header); },
      // 处理请求 (Handle request)
      [&]() { service->handle_request(request_header, request); });
}

/**
 * @brief 执行客户端响应处理 (Execute client response handling)
 *
 * @param client 客户端智能指针 (Client shared pointer)
 */
void Executor::execute_client(rclcpp::ClientBase::SharedPtr client) {
  // 创建请求头 (Create request header)
  auto request_header = client->create_request_header();

  // 创建响应 (Create response)
  std::shared_ptr<void> response = client->create_response();

  // 获取并处理错误 (Take and handle errors)
  take_and_do_error_handling(
      "taking a service client response from service", client->get_service_name(),
      // 获取类型擦除的响应 (Take type erased response)
      [&]() { return client->take_type_erased_response(response.get(), *request_header); },
      // 处理响应 (Handle response)
      [&]() { client->handle_response(request_header, response); });
}

/**
 * @brief 等待工作的执行器函数
 *
 * Executor function that waits for work to be available.
 *
 * @param timeout 等待超时时间 (Timeout duration to wait for work)
 */
void Executor::wait_for_work(std::chrono::nanoseconds timeout) {
  // 记录跟踪点 (Record tracepoint)
  TRACEPOINT(rclcpp_executor_wait_for_work, timeout.count());
  {
    std::lock_guard<std::mutex> guard(mutex_);

    // 检查 weak_nodes_
    // 中是否有未被执行器拥有的回调组，将其添加到收集实体的回调组列表中，并将其设置为不允许添加到其他执行器
    // (Check weak_nodes_ to find any callback group that is not owned by an executor and add it to
    // the list of callbackgroups for collect entities. Also exchange to false so it is not allowed
    // to add to another executor)
    add_callback_groups_from_nodes_associated_to_executor();

    // 收集要等待的订阅和计时器 (Collect the subscriptions and timers to be waited on)
    memory_strategy_->clear_handles();
    bool has_invalid_weak_groups_or_nodes =
        memory_strategy_->collect_entities(weak_groups_to_nodes_);

    if (has_invalid_weak_groups_or_nodes) {
      std::vector<rclcpp::CallbackGroup::WeakPtr> invalid_group_ptrs;
      for (auto pair : weak_groups_to_nodes_) {
        auto weak_group_ptr = pair.first;
        auto weak_node_ptr = pair.second;
        if (weak_group_ptr.expired() || weak_node_ptr.expired()) {
          invalid_group_ptrs.push_back(weak_group_ptr);
        }
      }
      std::for_each(
          invalid_group_ptrs.begin(), invalid_group_ptrs.end(),
          [this](rclcpp::CallbackGroup::WeakPtr group_ptr) {
            if (weak_groups_to_nodes_associated_with_executor_.find(group_ptr) !=
                weak_groups_to_nodes_associated_with_executor_.end()) {
              weak_groups_to_nodes_associated_with_executor_.erase(group_ptr);
            }
            if (weak_groups_associated_with_executor_to_nodes_.find(group_ptr) !=
                weak_groups_associated_with_executor_to_nodes_.end()) {
              weak_groups_associated_with_executor_to_nodes_.erase(group_ptr);
            }
            auto callback_guard_pair = weak_groups_to_guard_conditions_.find(group_ptr);
            if (callback_guard_pair != weak_groups_to_guard_conditions_.end()) {
              auto guard_condition = callback_guard_pair->second;
              weak_groups_to_guard_conditions_.erase(group_ptr);
              memory_strategy_->remove_guard_condition(guard_condition);
            }
            weak_groups_to_nodes_.erase(group_ptr);
          });
    }

    // 清除等待集 (Clear wait set)
    rcl_ret_t ret = rcl_wait_set_clear(&wait_set_);
    if (ret != RCL_RET_OK) {
      throw_from_rcl_error(ret, "Couldn't clear wait set");
    }

    // 等待的可变大小已包含在其他实体的大小中 (The size of waitables are accounted for in size of
    // the other entities)
    ret = rcl_wait_set_resize(
        &wait_set_, memory_strategy_->number_of_ready_subscriptions(),
        memory_strategy_->number_of_guard_conditions(), memory_strategy_->number_of_ready_timers(),
        memory_strategy_->number_of_ready_clients(), memory_strategy_->number_of_ready_services(),
        memory_strategy_->number_of_ready_events());
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "Couldn't resize the wait set");
    }

    // 将句柄添加到等待集中 (Add handles to the wait set)
    if (!memory_strategy_->add_handles_to_wait_set(&wait_set_)) {
      throw std::runtime_error("Couldn't fill wait set");
    }
  }

  // 等待并检查状态 (Wait and check status)
  rcl_ret_t status =
      rcl_wait(&wait_set_, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count());
  if (status == RCL_RET_WAIT_SET_EMPTY) {
    RCUTILS_LOG_WARN_NAMED(
        "rclcpp", "empty wait set received in rcl_wait(). This should never happen.");
  } else if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(status, "rcl_wait() failed");
  }

  // 检查等待集中的空句柄，并从内存策略中的回调实体句柄中移除它们 (Check the null handles in the
  // wait set and remove them from the handles in memory strategy for callback-based entities)
  std::lock_guard<std::mutex> guard(mutex_);
  memory_strategy_->remove_null_handles(&wait_set_);
}

/**
 * @brief 根据回调组获取节点 (Get node by callback group)
 *
 * @param weak_groups_to_nodes 回调组到节点的弱映射 (Weak mapping of callback groups to nodes)
 * @param group 要查找的回调组共享指针 (Shared pointer of the callback group to be searched)
 * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
 * 返回找到的节点共享指针，如果未找到则返回 nullptr (Return the shared pointer of the found node,
 * return nullptr if not found)
 */
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr Executor::get_node_by_group(
    const rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap
        &weak_groups_to_nodes,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 如果回调组为空，则返回 nullptr (If the callback group is empty, return nullptr)
  if (!group) {
    return nullptr;
  }
  // 创建回调组的弱指针 (Create a weak pointer for the callback group)
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr(group);
  // 在 weak_groups_to_nodes 中查找 weak_group_ptr (Find weak_group_ptr in weak_groups_to_nodes)
  const auto finder = weak_groups_to_nodes.find(weak_group_ptr);
  // 如果找到了 weak_group_ptr (If weak_group_ptr is found)
  if (finder != weak_groups_to_nodes.end()) {
    // 获取节点共享指针并返回 (Get the shared pointer of the node and return it)
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr = finder->second.lock();
    return node_ptr;
  }
  // 如果未找到，则返回 nullptr (If not found, return nullptr)
  return nullptr;
}

/**
 * @brief 根据定时器获取回调组 (Get callback group by timer)
 *
 * @param timer 要查找的定时器共享指针 (Shared pointer of the timer to be searched)
 * @return rclcpp::CallbackGroup::SharedPtr 返回找到的回调组共享指针，如果未找到则返回 nullptr
 * (Return the shared pointer of the found callback group, return nullptr if not found)
 */
rclcpp::CallbackGroup::SharedPtr Executor::get_group_by_timer(rclcpp::TimerBase::SharedPtr timer) {
  // 使用互斥锁保护线程安全 (Protect thread safety with a mutex lock)
  std::lock_guard<std::mutex> guard{mutex_};
  // 遍历 weak_groups_associated_with_executor_to_nodes_ 中的键值对 (Iterate through key-value pairs
  // in weak_groups_associated_with_executor_to_nodes_)
  for (const auto &pair : weak_groups_associated_with_executor_to_nodes_) {
    // 获取回调组 (Get the callback group)
    auto group = pair.first.lock();
    // 如果回调组为空，则继续下一次循环 (If the callback group is empty, continue to the next
    // iteration)
    if (!group) {
      continue;
    }
    // 查找与给定定时器匹配的定时器引用 (Find timer references that match the given timer)
    auto timer_ref =
        group->find_timer_ptrs_if([timer](const rclcpp::TimerBase::SharedPtr &timer_ptr) -> bool {
          return timer_ptr == timer;
        });
    // 如果找到了匹配的定时器引用，则返回该回调组 (If a matching timer reference is found, return
    // the callback group)
    if (timer_ref) {
      return group;
    }
  }

  // 遍历 weak_groups_to_nodes_associated_with_executor_ 中的键值对 (Iterate through key-value pairs
  // in weak_groups_to_nodes_associated_with_executor_)
  for (const auto &pair : weak_groups_to_nodes_associated_with_executor_) {
    // 获取回调组 (Get the callback group)
    auto group = pair.first.lock();
    // 如果回调组为空，则继续下一次循环 (If the callback group is empty, continue to the next
    // iteration)
    if (!group) {
      continue;
    }
    // 查找与给定定时器匹配的定时器引用 (Find timer references that match the given timer)
    auto timer_ref =
        group->find_timer_ptrs_if([timer](const rclcpp::TimerBase::SharedPtr &timer_ptr) -> bool {
          return timer_ptr == timer;
        });
    // 如果找到了匹配的定时器引用，则返回该回调组 (If a matching timer reference is found, return
    // the callback group)
    if (timer_ref) {
      return group;
    }
  }
  // 如果未找到匹配的回调组，则返回 nullptr (If no matching callback group is found, return nullptr)
  return nullptr;
}

/**
 * @brief 获取下一个准备好的可执行对象（Get the next ready executable）
 *
 * @param[out] any_executable 任意可执行对象（AnyExecutable object）
 * @return 是否成功获取到可执行对象（Whether a ready executable was found）
 */
bool Executor::get_next_ready_executable(AnyExecutable &any_executable) {
  // 从映射中获取下一个准备好的可执行对象（Get the next ready executable from the map）
  bool success = get_next_ready_executable_from_map(any_executable, weak_groups_to_nodes_);
  return success;
}

/**
 * @brief 从映射中获取下一个准备好的可执行对象（Get the next ready executable from the map）
 *
 * @param[out] any_executable 任意可执行对象（AnyExecutable object）
 * @param[in] weak_groups_to_nodes 弱回调组与节点映射（Weak callback groups to nodes map）
 * @return 是否成功获取到可执行对象（Whether a ready executable was found）
 */
bool Executor::get_next_ready_executable_from_map(
    AnyExecutable &any_executable,
    const rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap
        &weak_groups_to_nodes) {
  // 创建跟踪点（Create tracepoint）
  TRACEPOINT(rclcpp_executor_get_next_ready);
  bool success = false;
  // 锁定互斥体以保护共享资源（Lock the mutex to protect shared resources）
  std::lock_guard<std::mutex> guard{mutex_};
  // 检查计时器，看是否有已准备好的（Check the timers to see if there are any that are ready）
  memory_strategy_->get_next_timer(any_executable, weak_groups_to_nodes);
  if (any_executable.timer) {
    success = true;
  }
  if (!success) {
    // 检查订阅，看是否有已准备好的（Check the subscriptions to see if there are any that are
    // ready）
    memory_strategy_->get_next_subscription(any_executable, weak_groups_to_nodes);
    if (any_executable.subscription) {
      success = true;
    }
  }
  if (!success) {
    // 检查服务，看是否有已准备好的（Check the services to see if there are any that are ready）
    memory_strategy_->get_next_service(any_executable, weak_groups_to_nodes);
    if (any_executable.service) {
      success = true;
    }
  }
  if (!success) {
    // 检查客户端，看是否有已准备好的（Check the clients to see if there are any that are ready）
    memory_strategy_->get_next_client(any_executable, weak_groups_to_nodes);
    if (any_executable.client) {
      success = true;
    }
  }
  if (!success) {
    // 检查等待对象，看是否有已准备好的（Check the waitables to see if there are any that are
    // ready）
    memory_strategy_->get_next_waitable(any_executable, weak_groups_to_nodes);
    if (any_executable.waitable) {
      any_executable.data = any_executable.waitable->take_data();
      success = true;
    }
  }
  // 此时，any_executable 应该是一个有效的订阅或计时器，或者是一个空的共享指针
  // (At this point any_executable should be valid with either a valid subscription
  // or a valid timer, or it should be a null shared_ptr)
  if (success) {
    rclcpp::CallbackGroup::WeakPtr weak_group_ptr = any_executable.callback_group;
    auto iter = weak_groups_to_nodes.find(weak_group_ptr);
    if (iter == weak_groups_to_nodes.end()) {
      success = false;
    }
  }

  if (success) {
    // 如果有效，检查组是否互斥，然后相应地标记它。检查回调组是否属于此执行器
    // (If it is valid, check to see if the group is mutually exclusive or
    // not, then mark it accordingly. Check if the callback_group belongs to this executor)
    if (any_executable.callback_group &&
        any_executable.callback_group->type() == CallbackGroupType::MutuallyExclusive) {
      // 否则不应该被拿走（It should not have been taken otherwise）
      assert(any_executable.callback_group->can_be_taken_from().load());
      // 设置为 false，表示正在从该组运行某些内容
      // 当 any_executable 被执行或销毁时，这将重置为 true
      // (Set to false to indicate something is being run from this group
      // This is reset to true either when the any_executable is executed or when the
      // any_executable is destructued)
      any_executable.callback_group->can_be_taken_from().store(false);
    }
  }
  // 如果没有准备好的可执行对象，返回 false（If there is no ready executable, return false）
  return success;
}

/**
 * @brief 获取下一个可执行的实体（Get the next executable entity）
 *
 * @param any_executable 用于存储获取到的可执行实体的引用（A reference to store the obtained
 * executable entity）
 * @param timeout 超时时间（Timeout duration）
 * @return 如果成功获取到可执行实体，则返回true，否则返回false（Returns true if an executable entity
 * is successfully obtained, otherwise returns false）
 */
bool Executor::get_next_executable(
    AnyExecutable &any_executable, std::chrono::nanoseconds timeout) {
  bool success = false;
  // 检查是否有任何需要服务的订阅或定时器（Check if there are any subscriptions or timers that need
  // service）
  // TODO(wjwwood): 提高此函数的运行效率（Improve the run-to-run efficiency of this function）
  success = get_next_ready_executable(any_executable);
  // 如果没有找到可执行实体（If no executable entity is found）
  if (!success) {
    // 等待订阅或定时器可以工作（Wait for subscriptions or timers to be available for work）
    wait_for_work(timeout);
    if (!spinning.load()) {
      return false;
    }
    // 再次尝试获取可执行实体（Try getting the executable entity again）
    success = get_next_ready_executable(any_executable);
  }
  return success;
}

/**
 * @brief 检查给定的节点是否存在于 weak_groups_to_nodes 映射中（Check if the given node exists in
 * the weak_groups_to_nodes map）
 *
 * @param node_ptr 要检查的节点智能指针（The shared pointer of the node to check）
 * @param weak_groups_to_nodes 从弱回调组到节点的映射（Map from weak callback groups to nodes）
 * @return 如果节点存在于映射中，则返回true，否则返回false（Returns true if the node exists in the
 * map, otherwise returns false）
 */
bool Executor::has_node(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    const rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap
        &weak_groups_to_nodes) const {
  return std::find_if(
             weak_groups_to_nodes.begin(), weak_groups_to_nodes.end(),
             [&](const WeakCallbackGroupsToNodesMap::value_type &other) -> bool {
               auto other_ptr = other.second.lock();
               // 检查当前遍历的节点是否与给定节点相同（Check if the current traversed node is the
               // same as the given node）
               return other_ptr == node_ptr;
             }) != weak_groups_to_nodes.end();
}

/**
 * @brief 返回执行器是否正在运行（Return whether the executor is spinning）
 *
 * @return 如果执行器正在运行，则返回true，否则返回false（Returns true if the executor is spinning,
 * otherwise returns false）
 */
bool Executor::is_spinning() { return spinning; }

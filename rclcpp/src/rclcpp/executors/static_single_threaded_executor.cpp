// Copyright 2019 Nobleo Technology
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

#include "rclcpp/executors/static_single_threaded_executor.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

#include "rcpputils/scope_exit.hpp"

using rclcpp::executors::StaticSingleThreadedExecutor;
using rclcpp::experimental::ExecutableList;

/*!
 * \brief 构造函数，用于创建 StaticSingleThreadedExecutor 实例。
 * Constructor for creating a StaticSingleThreadedExecutor instance.
 * \param options 传入的 ExecutorOptions 对象。
 * The ExecutorOptions object passed in.
 */
StaticSingleThreadedExecutor::StaticSingleThreadedExecutor(const rclcpp::ExecutorOptions& options)
    : rclcpp::Executor(options) {
  // 创建一个 StaticExecutorEntitiesCollector 实例。
  // Create a StaticExecutorEntitiesCollector instance.
  entities_collector_ = std::make_shared<StaticExecutorEntitiesCollector>();
}

/*!
 * \brief 析构函数，用于销毁 StaticSingleThreadedExecutor 实例。
 * Destructor for destroying a StaticSingleThreadedExecutor instance.
 */
StaticSingleThreadedExecutor::~StaticSingleThreadedExecutor() {
  // 如果 entities_collector_ 已经初始化，调用它的 fini() 函数。
  // If entities_collector_ is initialized, call its fini() function.
  if (entities_collector_->is_init()) {
    entities_collector_->fini();
  }
}

/*!
 * \brief 无限循环执行可用的回调。
 * Continuously execute available callbacks.
 */
void StaticSingleThreadedExecutor::spin() {
  // 检查是否已经在 spinning，如果是，则抛出运行时错误。
  // Check if already spinning, if so, throw a runtime error.
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););

  // 根据 weak_nodes_ 设置 memory_strategy_ 和 exec_list_
  // Prepare wait_set_ based on memory_strategy_
  entities_collector_->init(&wait_set_, memory_strategy_);

  // 当 ROS 上下文有效且正在 spinning 时，刷新 wait set 并等待工作
  // While the ROS context is valid and spinning, refresh the wait set and wait for work
  while (rclcpp::ok(this->context_) && spinning.load()) {
    entities_collector_->refresh_wait_set();
    execute_ready_executables();
  }
}

/*!
 * \brief 在给定的最大持续时间内执行一些可用回调。
 * Execute some available callbacks within a given maximum duration.
 * \param max_duration 最大持续时间。
 * The maximum duration.
 */
void StaticSingleThreadedExecutor::spin_some(std::chrono::nanoseconds max_duration) {
  // 在此上下文中，0 输入 max_duration 表示无持续时间限制
  // In this context, a 0 input max_duration means no duration limit
  if (std::chrono::nanoseconds(0) == max_duration) {
    max_duration = std::chrono::nanoseconds::max();
  }

  return this->spin_some_impl(max_duration, false);
}

/*!
 * \brief 在给定的最大持续时间内执行所有可用回调。
 * Execute all available callbacks within a given maximum duration.
 * \param max_duration 最大持续时间。
 * The maximum duration.
 */
void StaticSingleThreadedExecutor::spin_all(std::chrono::nanoseconds max_duration) {
  if (max_duration < std::chrono::nanoseconds(0)) {
    throw std::invalid_argument("max_duration must be greater than or equal to 0");
  }
  return this->spin_some_impl(max_duration, true);
}

/*!
 * \brief 在给定的最大持续时间内实现 spin_some 或 spin_all。
 * Implement spin_some or spin_all within a given maximum duration.
 * \param max_duration 最大持续时间。
 * The maximum duration.
 * \param exhaustive 是否执行所有可用回调。
 * Whether to execute all available callbacks.
 */
void StaticSingleThreadedExecutor::spin_some_impl(
    std::chrono::nanoseconds max_duration, bool exhaustive) {
  // 确保 entities collector 已初始化
  // Make sure the entities collector has been initialized
  if (!entities_collector_->is_init()) {
    entities_collector_->init(&wait_set_, memory_strategy_);
  }

  auto start = std::chrono::steady_clock::now();
  auto max_duration_not_elapsed = [max_duration, start]() {
    if (std::chrono::nanoseconds(0) == max_duration) {
      // 如果需要，告知无限期旋转
      // told to spin forever if need be
      return true;
    } else if (std::chrono::steady_clock::now() - start < max_duration) {
      // 告知只旋转一定的最大时长
      // told to spin only for some maximum amount of time
      return true;
    }
    // 旋转时间过长
    // spun too long
    return false;
  };

  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_some() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););

  while (rclcpp::ok(context_) && spinning.load() && max_duration_not_elapsed()) {
    // 获取现在就绪的可执行文件
    // Get executables that are ready now
    entities_collector_->refresh_wait_set(std::chrono::milliseconds::zero());
    // 执行就绪的可执行文件
    // Execute ready executables
    bool work_available = execute_ready_executables();
    if (!work_available || !exhaustive) {
      break;
    }
  }
}

/*!
 * \brief 在给定的超时时间内实现 spin_once。
 * Implement spin_once within a given timeout.
 * \param timeout 超时时间。
 * The timeout.
 */
void StaticSingleThreadedExecutor::spin_once_impl(std::chrono::nanoseconds timeout) {
  // 确保 entities collector 已初始化
  // Make sure the entities collector has been initialized
  if (!entities_collector_->is_init()) {
    entities_collector_->init(&wait_set_, memory_strategy_);
  }

  if (rclcpp::ok(context_) && spinning.load()) {
    // 等待直到我们有一个就绪的实体或超时过期
    // Wait until we have a ready entity or timeout expired
    entities_collector_->refresh_wait_set(timeout);
    // 执行已准备好的可执行文件
    // Execute ready executables
    execute_ready_executables(true);
  }
}

/*!
 * \brief 添加回调组。
 * Add a callback group.
 * \param group_ptr 回调组的 shared_ptr。
 * The shared_ptr of the callback group.
 * \param node_ptr 节点基本接口的 shared_ptr。
 * The shared_ptr of the node base interface.
 * \param notify 是否在添加新节点时中断等待。
 * Whether to interrupt waiting when adding a new node.
 */
void StaticSingleThreadedExecutor::add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    bool notify) {
  bool is_new_node = entities_collector_->add_callback_group(group_ptr, node_ptr);
  if (is_new_node && notify) {
    // 中断等待以处理新节点
    // Interrupt waiting to handle new node
    interrupt_guard_condition_.trigger();
  }
}

/*!
 * \brief 添加一个节点到 StaticSingleThreadedExecutor
 * \param node_ptr 指向要添加的节点的智能指针
 * \param notify 是否通知执行器处理新节点
 *
 * \fn void StaticSingleThreadedExecutor::add_node(
 *  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
 */
// Add a node to the StaticSingleThreadedExecutor
void StaticSingleThreadedExecutor::add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify) {
  // 尝试将节点添加到实体收集器中，返回是否为新节点
  // Attempt to add the node to the entities collector, return whether it is a new node
  bool is_new_node = entities_collector_->add_node(node_ptr);
  if (is_new_node && notify) {
    // 如果是新节点且需要通知，则中断等待以处理新节点
    // If it's a new node and needs to be notified, interrupt waiting to handle the new node
    interrupt_guard_condition_.trigger();
  }
}

/*!
 * \brief 添加一个节点到 StaticSingleThreadedExecutor
 * \param node_ptr 指向要添加的节点的智能指针
 * \param notify 是否通知执行器处理新节点
 *
 * \fn void StaticSingleThreadedExecutor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool
 * notify)
 */
// Add a node to the StaticSingleThreadedExecutor
void StaticSingleThreadedExecutor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify) {
  this->add_node(node_ptr->get_node_base_interface(), notify);
}

/*!
 * \brief 从 StaticSingleThreadedExecutor 中移除回调组
 * \param group_ptr 指向要移除的回调组的智能指针
 * \param notify 是否通知执行器处理移除操作
 *
 * \fn void StaticSingleThreadedExecutor::remove_callback_group(
 *  rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify)
 */
// Remove a callback group from the StaticSingleThreadedExecutor
void StaticSingleThreadedExecutor::remove_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify) {
  // 从实体收集器中移除回调组，返回是否移除成功
  // Remove the callback group from the entities collector, return whether it was removed
  // successfully
  bool node_removed = entities_collector_->remove_callback_group(group_ptr);
  // 如果节点匹配并被移除，中断等待
  // If the node was matched and removed, interrupt waiting
  if (node_removed && notify) {
    interrupt_guard_condition_.trigger();
  }
}

/*!
 * \brief 从 StaticSingleThreadedExecutor 中移除节点
 * \param node_ptr 指向要移除的节点的智能指针
 * \param notify 是否通知执行器处理移除操作
 *
 * \fn void StaticSingleThreadedExecutor::remove_node(
 *  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
 */
// Remove a node from the StaticSingleThreadedExecutor
void StaticSingleThreadedExecutor::remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify) {
  // 从实体收集器中移除节点，返回是否移除成功
  // Remove the node from the entities collector, return whether it was removed successfully
  bool node_removed = entities_collector_->remove_node(node_ptr);
  if (!node_removed) {
    throw std::runtime_error("Node needs to be associated with this executor.");
  }
  // 如果节点匹配并被移除，中断等待
  // If the node was matched and removed, interrupt waiting
  if (notify) {
    interrupt_guard_condition_.trigger();
  }
}

/*!
 * \brief 获取所有回调组的弱指针列表
 *
 * \fn std::vector<rclcpp::CallbackGroup::WeakPtr>
 * StaticSingleThreadedExecutor::get_all_callback_groups()
 */
// Get a list of weak pointers to all callback groups
std::vector<rclcpp::CallbackGroup::WeakPtr>
StaticSingleThreadedExecutor::get_all_callback_groups() {
  return entities_collector_->get_all_callback_groups();
}

/*!
 * \brief 获取手动添加的回调组的弱指针列表
 *
 * \fn std::vector<rclcpp::CallbackGroup::WeakPtr>
 * StaticSingleThreadedExecutor::get_manually_added_callback_groups()
 */
// Get a list of weak pointers to manually added callback groups
std::vector<rclcpp::CallbackGroup::WeakPtr>
StaticSingleThreadedExecutor::get_manually_added_callback_groups() {
  return entities_collector_->get_manually_added_callback_groups();
}

/*!
 * \brief 获取从节点自动添加的回调组的弱指针列表
 *
 * \fn std::vector<rclcpp::CallbackGroup::WeakPtr>
 * StaticSingleThreadedExecutor::get_automatically_added_callback_groups_from_nodes()
 */
// Get a list of weak pointers to automatically added callback groups from nodes
std::vector<rclcpp::CallbackGroup::WeakPtr>
StaticSingleThreadedExecutor::get_automatically_added_callback_groups_from_nodes() {
  return entities_collector_->get_automatically_added_callback_groups_from_nodes();
}

/*!
 * \brief 从 StaticSingleThreadedExecutor 中移除节点
 * \param node_ptr 指向要移除的节点的智能指针
 * \param notify 是否通知执行器处理移除操作
 *
 * \fn void StaticSingleThreadedExecutor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool
 * notify)
 */
// Remove a node from the StaticSingleThreadedExecutor
void StaticSingleThreadedExecutor::remove_node(
    std::shared_ptr<rclcpp::Node> node_ptr, bool notify) {
  this->remove_node(node_ptr->get_node_base_interface(), notify);
}

/*!
 * \brief 执行准备好的可执行对象
 * \param spin_once 如果为 true，则仅执行一个可执行对象
 * \return 是否有任何可执行对象被执行
 *
 * \fn bool StaticSingleThreadedExecutor::execute_ready_executables(bool spin_once)
 */
// Execute ready executables
bool StaticSingleThreadedExecutor::execute_ready_executables(bool spin_once) {
  bool any_ready_executable = false;

  // 执行所有准备好的订阅
  // Execute all the ready subscriptions
  for (size_t i = 0; i < wait_set_.size_of_subscriptions; ++i) {
    if (i < entities_collector_->get_number_of_subscriptions()) {
      if (wait_set_.subscriptions[i]) {
        execute_subscription(entities_collector_->get_subscription(i));
        if (spin_once) {
          return true;
        }
        any_ready_executable = true;
      }
    }
  }
  // 执行所有准备好的定时器
  // Execute all the ready timers
  for (size_t i = 0; i < wait_set_.size_of_timers; ++i) {
    if (i < entities_collector_->get_number_of_timers()) {
      if (wait_set_.timers[i] && entities_collector_->get_timer(i)->is_ready()) {
        auto timer = entities_collector_->get_timer(i);
        timer->call();
        execute_timer(std::move(timer));
        if (spin_once) {
          return true;
        }
        any_ready_executable = true;
      }
    }
  }
  // 执行所有准备好的服务
  // Execute all the ready services
  for (size_t i = 0; i < wait_set_.size_of_services; ++i) {
    if (i < entities_collector_->get_number_of_services()) {
      if (wait_set_.services[i]) {
        execute_service(entities_collector_->get_service(i));
        if (spin_once) {
          return true;
        }
        any_ready_executable = true;
      }
    }
  }
  // 执行所有准备好的客户端
  // Execute all the ready clients
  for (size_t i = 0; i < wait_set_.size_of_clients; ++i) {
    if (i < entities_collector_->get_number_of_clients()) {
      if (wait_set_.clients[i]) {
        execute_client(entities_collector_->get_client(i));
        if (spin_once) {
          return true;
        }
        any_ready_executable = true;
      }
    }
  }
  // 执行所有准备好的可等待对象
  // Execute all the ready waitables
  for (size_t i = 0; i < entities_collector_->get_number_of_waitables(); ++i) {
    auto waitable = entities_collector_->get_waitable(i);
    if (waitable->is_ready(&wait_set_)) {
      auto data = waitable->take_data();
      waitable->execute(data);
      if (spin_once) {
        return true;
      }
      any_ready_executable = true;
    }
  }
  return any_ready_executable;
}

// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__STATIC_EXECUTOR_ENTITIES_COLLECTOR_HPP_
#define RCLCPP__EXECUTORS__STATIC_EXECUTOR_ENTITIES_COLLECTOR_HPP_

#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <vector>

#include "rcl/guard_condition.h"
#include "rcl/wait.h"
#include "rclcpp/experimental/executable_list.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp {
namespace executors {
/**
 * @brief 定义一个类型别名，用于存储回调组和节点之间的关系映射。
 * Define a type alias for storing the mapping between callback groups and nodes.
 *
 * @tparam rclcpp::CallbackGroup::WeakPtr 回调组弱指针类型。
 *        A weak pointer type to a callback group.
 * @tparam rclcpp::node_interfaces::NodeBaseInterface::WeakPtr 节点基础接口弱指针类型。
 *        A weak pointer type to a node base interface.
 * @tparam std::owner_less<rclcpp::CallbackGroup::WeakPtr> 用于比较两个弱指针的函数对象。
 *        A function object for comparing two weak pointers.
 */
typedef std::map<
    rclcpp::CallbackGroup::WeakPtr,
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
    std::owner_less<rclcpp::CallbackGroup::WeakPtr>>
    WeakCallbackGroupsToNodesMap;

/**
 * @class StaticExecutorEntitiesCollector
 * @brief 收集静态执行器中的实体（Collects entities in the static executor）
 *
 * 这个类继承了 rclcpp::Waitable 和 std::enable_shared_from_this，
 * 用于收集静态执行器中的实体，如订阅者、服务等。
 * (This class inherits from rclcpp::Waitable and std::enable_shared_from_this,
 * and is used to collect entities in the static executor, such as subscribers, services, etc.)
 */
class StaticExecutorEntitiesCollector final
    : public rclcpp::Waitable,
      public std::enable_shared_from_this<StaticExecutorEntitiesCollector> {
public:
  // 定义智能指针类型 (Define smart pointer types)
  RCLCPP_SMART_PTR_DEFINITIONS(StaticExecutorEntitiesCollector)

  // 构造函数 (Constructor)
  RCLCPP_PUBLIC
  StaticExecutorEntitiesCollector() = default;

  // 析构函数 (Destructor)
  RCLCPP_PUBLIC
  ~StaticExecutorEntitiesCollector();

  /// 初始化 StaticExecutorEntitiesCollector (Initialize StaticExecutorEntitiesCollector)
  /**
   * \param p_wait_set 用于执行器的等待集引用 (A reference to the wait set to be used in the
   * executor) \param memory_strategy 设置内存策略的共享指针 (Shared pointer to the memory strategy
   * to set) \throws std::runtime_error 如果内存策略为空 (if memory strategy is null)
   */
  RCLCPP_PUBLIC
  void init(
      rcl_wait_set_t *p_wait_set,
      rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy);

  /// 清除资源以完成 StaticExecutorEntitiesCollector (Finalize StaticExecutorEntitiesCollector to
  /// clear resources)
  RCLCPP_PUBLIC
  bool is_init() { return initialized_; }

  RCLCPP_PUBLIC
  void fini();

  /// 执行 Waitable (Execute the waitable)
  RCLCPP_PUBLIC
  void execute(std::shared_ptr<void> &data) override;

  /// 获取数据，以便可以使用 `execute` 消耗它 (Take the data so that it can be consumed with
  /// `execute`)
  /**
   * 对于 `StaticExecutorEntitiesCollector`，这总是返回 `nullptr`
   * (For `StaticExecutorEntitiesCollector`, this always return `nullptr`)
   * \sa rclcpp::Waitable::take_data()
   */
  RCLCPP_PUBLIC
  std::shared_ptr<void> take_data() override;

  /// 将句柄添加到 wait_set 并等待工作 (Function to add_handles_to_wait_set and wait for work)
  /**
   * 阻塞直到 wait_set 准备好或超时 (block until the wait set is ready or until the timeout has been
   * exceeded) \throws std::runtime_error 如果无法清除或填充 wait_set (if wait set couldn't be
   * cleared or filled) \throws rcl_wait 中的任何 rcl 错误，\see
   * rclcpp::exceptions::throw_from_rcl_error() (any rcl errors from rcl_wait, \see
   * rclcpp::exceptions::throw_from_rcl_error())
   */
  RCLCPP_PUBLIC
  void refresh_wait_set(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /**
   * \throws std::runtime_error 如果无法将 guard 条件添加到 wait_set (if it couldn't add guard
   * condition to wait set)
   */
  RCLCPP_PUBLIC
  void add_to_wait_set(rcl_wait_set_t *wait_set) override;

  RCLCPP_PUBLIC
  size_t get_number_of_ready_guard_conditions() override;

  /// 将回调组添加到执行器 (Add a callback group to an executor)
  /**
   * \see rclcpp::Executor::add_callback_group
   */
  RCLCPP_PUBLIC
  bool add_callback_group(
      rclcpp::CallbackGroup::SharedPtr group_ptr,
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

  /// 将回调组添加到执行器 (Add a callback group to an executor)
  /**
   * \see rclcpp::Executor::add_callback_group
   * \return 回调组中的节点是否为新节点的布尔值 (boolean whether the node from the callback group is
   * new)
   */
  RCLCPP_PUBLIC
  bool add_callback_group(
      rclcpp::CallbackGroup::SharedPtr group_ptr,
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
      WeakCallbackGroupsToNodesMap &weak_groups_to_nodes);

  /// 从执行器中移除回调组 (Remove a callback group from the executor)
  /**
   * \see rclcpp::Executor::remove_callback_group
   */
  RCLCPP_PUBLIC
  bool remove_callback_group(rclcpp::CallbackGroup::SharedPtr group_ptr);

  /// 从执行器中移除回调组。 (Remove a callback group from the executor.)
  /**
   * \see rclcpp::Executor::remove_callback_group_from_map
   */
  RCLCPP_PUBLIC
  bool remove_callback_group_from_map(
      // 回调组共享指针 (Callback group shared pointer)
      rclcpp::CallbackGroup::SharedPtr group_ptr,
      // 弱回调组到节点映射的引用 (Reference to weak callback groups to nodes map)
      WeakCallbackGroupsToNodesMap &weak_groups_to_nodes);

  /**
   * \see rclcpp::Executor::add_node()
   * \throw std::runtime_error if node was already added
   */
  RCLCPP_PUBLIC
  bool add_node(
      // 节点基础接口共享指针 (Node base interface shared pointer)
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

  /**
   * \see rclcpp::Executor::remove_node()
   * \throw std::runtime_error if no guard condition is associated with node.
   */
  RCLCPP_PUBLIC
  bool remove_node(
      // 节点基础接口共享指针 (Node base interface shared pointer)
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_all_callback_groups();

  /// 获取属于执行器的回调组。 (Get callback groups that belong to executor.)
  /**
   * \see rclcpp::Executor::get_manually_added_callback_groups()
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_manually_added_callback_groups();

  /// 获取属于执行器的回调组。 (Get callback groups that belong to executor.)
  /**
   * \see rclcpp::Executor::get_automatically_added_callback_groups_from_nodes()
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_automatically_added_callback_groups_from_nodes();

  /// 完成所有可用的排队工作而不阻塞。 (Complete all available queued work without blocking.)
  /**
   * 此函数检查在保护条件被触发后（或者发生虚假唤醒）我们是否真正准备好执行（例如重新收集实体）
   * (This function checks if after the guard condition was triggered
   * or a spurious wakeup happened) we are really ready to execute
   * i.e. re-collect entities
   */
  RCLCPP_PUBLIC
  bool is_ready(rcl_wait_set_t *wait_set) override;

  /// 返回计时器数量 (Return number of timers)
  /**
   * \return number of timers
   */
  RCLCPP_PUBLIC
  size_t get_number_of_timers() { return exec_list_.number_of_timers; }

  /// 返回订阅数量 (Return number of subscriptions)
  /**
   * \return number of subscriptions
   */
  RCLCPP_PUBLIC
  size_t get_number_of_subscriptions() { return exec_list_.number_of_subscriptions; }

  /// 返回服务数量 (Return number of services)
  /**
   * \return number of services
   */
  RCLCPP_PUBLIC
  size_t get_number_of_services() { return exec_list_.number_of_services; }

  /// 返回客户端数量 (Return number of clients)
  /**
   * \return number of clients
   */
  RCLCPP_PUBLIC
  size_t get_number_of_clients() { return exec_list_.number_of_clients; }

  /// 返回等待项数量 (Return number of waitables)
  /**
   * \return number of waitables
   */
  RCLCPP_PUBLIC
  size_t get_number_of_waitables() { return exec_list_.number_of_waitables; }

  /** 通过索引返回一个SubscriptionBase共享指针。 (Return a SubscriptionBase SharedPtr by index.)
   * \param[in] i 订阅基础的索引 (The index of the SubscriptionBase)
   * \return 一个订阅基础共享指针 (A SubscriptionBase shared pointer)
   * \throws std::out_of_range 如果参数大于结构的大小。 (if the argument is higher than the size of
   * the structure.)
   */
  RCLCPP_PUBLIC
  rclcpp::SubscriptionBase::SharedPtr get_subscription(size_t i) {
    return exec_list_.subscription[i];
  }

  /**
   * \brief 返回给定索引处的 TimerBase 共享指针 (Return a TimerBase Sharedptr by index)
   *
   * \param[in] i TimerBase 的索引 (The index of the TimerBase)
   * \return TimerBase 的共享指针 (a TimerBase shared pointer)
   * \throws std::out_of_range 如果参数大于大小 (if the argument is higher than the size)
   */
  RCLCPP_PUBLIC
  rclcpp::TimerBase::SharedPtr get_timer(size_t i) {
    // 返回 exec_list_ 中索引为 i 的 timer 共享指针 (Return the timer shared pointer at index i in
    // exec_list_)
    return exec_list_.timer[i];
  }

  /**
   * \brief 返回给定索引处的 ServiceBase 共享指针 (Return a ServiceBase Sharedptr by index)
   *
   * \param[in] i ServiceBase 的索引 (The index of the ServiceBase)
   * \return ServiceBase 的共享指针 (a ServiceBase shared pointer)
   * \throws std::out_of_range 如果参数大于大小 (if the argument is higher than the size)
   */
  RCLCPP_PUBLIC
  rclcpp::ServiceBase::SharedPtr get_service(size_t i) {
    // 返回 exec_list_ 中索引为 i 的 service 共享指针 (Return the service shared pointer at index i
    // in exec_list_)
    return exec_list_.service[i];
  }

  /**
   * \brief 返回给定索引处的 ClientBase 共享指针 (Return a ClientBase Sharedptr by index)
   *
   * \param[in] i ClientBase 的索引 (The index of the ClientBase)
   * \return ClientBase 的共享指针 (a ClientBase shared pointer)
   * \throws std::out_of_range 如果参数大于大小 (if the argument is higher than the size)
   */
  RCLCPP_PUBLIC
  rclcpp::ClientBase::SharedPtr get_client(size_t i) {
    // 返回 exec_list_ 中索引为 i 的 client 共享指针 (Return the client shared pointer at index i in
    // exec_list_)
    return exec_list_.client[i];
  }

  /**
   * \brief 返回给定索引处的 Waitable 共享指针 (Return a Waitable Sharedptr by index)
   *
   * \param[in] i Waitable 的索引 (The index of the Waitable)
   * \return Waitable 的共享指针 (a Waitable shared pointer)
   * \throws std::out_of_range 如果参数大于大小 (if the argument is higher than the size)
   */
  RCLCPP_PUBLIC
  rclcpp::Waitable::SharedPtr get_waitable(size_t i) {
    // 返回 exec_list_ 中索引为 i 的 waitable 共享指针 (Return the waitable shared pointer at index
    // i in exec_list_)
    return exec_list_.waitable[i];
  }

private:
  /// 函数用于重新分配 wait set 中实体的空间。
  /// Function to reallocate space for entities in the wait set.
  /**
   * \throws std::runtime_error 如果无法清除或调整 wait set 的大小，则抛出异常。
   * \throws std::runtime_error if wait set couldn't be cleared or resized.
   */
  void prepare_wait_set();

  /// 填充可执行列表。
  /// Fill the executable list.
  void fill_executable_list();

  /// 填充内存策略。
  /// Fill the memory strategy.
  void fill_memory_strategy();

  /// 判断节点是否属于收集器。
  /// Return true if the node belongs to the collector.
  /**
   * \param[in] node_ptr 节点基本接口共享指针。
   * \param[in] node_ptr a node base interface shared pointer.
   * \param[in] weak_groups_to_nodes 用于查找的节点映射。
   * \param[in] weak_groups_to_nodes map to nodes to lookup.
   * \return 返回节点是否属于收集器的布尔值。
   * \return boolean whether a node belongs the collector.
   */
  bool has_node(
      const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
      const WeakCallbackGroupsToNodesMap &weak_groups_to_nodes) const;

  /// 从与执行器关联的节点中添加所有可以被任何执行器自动添加并且尚未与执行器关联的回调组。
  /// Add all callback groups that can be automatically added by any executor
  /// and is not already associated with an executor from nodes
  /// that are associated with executor.
  /**
   * \see rclcpp::Executor::add_callback_groups_from_nodes_associated_to_executor()
   */
  void add_callback_groups_from_nodes_associated_to_executor();

  /// 从映射中填充可执行列表。
  /// Fill the executable list from map.
  void fill_executable_list_from_map(const WeakCallbackGroupsToNodesMap &weak_groups_to_nodes);

  /// 内存策略：用于处理用户定义的内存分配策略的接口。
  /// Memory strategy: an interface for handling user-defined memory allocation strategies.
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy_;

  // 将回调组映射到节点。
  // Maps callback groups to nodes.
  WeakCallbackGroupsToNodesMap weak_groups_associated_with_executor_to_nodes_;
  // 将回调组映射到节点。
  // Maps callback groups to nodes.
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes_associated_with_executor_;

  typedef std::map<
      rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
      const rclcpp::GuardCondition *,
      std::owner_less<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>>
      WeakNodesToGuardConditionsMap;
  WeakNodesToGuardConditionsMap weak_nodes_to_guard_conditions_;

  /// 静态执行器中注册的弱节点列表。
  /// List of weak nodes registered in the static executor.
  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_;

  // 保护新节点向量的互斥锁。
  // Mutex to protect vector of new nodes.
  std::mutex new_nodes_mutex_;
  std::vector<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> new_nodes_;

  /// 用于管理 rmw 层等待的实体的 wait set。
  /// Wait set for managing entities that the rmw layer waits on.
  rcl_wait_set_t *p_wait_set_ = nullptr;

  /// 可执行列表：计时器、订阅者、客户端、服务和可等待对象。
  /// Executable list: timers, subscribers, clients, services and waitables.
  rclcpp::experimental::ExecutableList exec_list_;

  /// 用于检查实体收集器是否已初始化的布尔值。
  /// Bool to check if the entities collector has been initialized.
  bool initialized_ = false;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__STATIC_EXECUTOR_ENTITIES_COLLECTOR_HPP_

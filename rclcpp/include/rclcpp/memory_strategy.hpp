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

#ifndef RCLCPP__MEMORY_STRATEGY_HPP_
#define RCLCPP__MEMORY_STRATEGY_HPP_

#include <list>
#include <map>
#include <memory>

#include "rcl/allocator.h"
#include "rcl/wait.h"
#include "rclcpp/any_executable.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp {
namespace memory_strategy {

/// 内存策略委托，用于处理执行器执行时的内存分配。
/// Delegate for handling memory allocations while the Executor is executing.
/**
 * 默认情况下，内存策略会在执行器等待工作后，根据通过的实体数量动态分配来自 rmw 实现的结构的内存。
 * By default, the memory strategy dynamically allocates memory for structures that come in from
 * the rmw implementation after the executor waits for work, based on the number of entities that
 * come through.
 */
class RCLCPP_PUBLIC MemoryStrategy {
public:
  // 禁止拷贝智能指针定义
  // Smart pointer definitions not copyable
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(MemoryStrategy)
  // 使用弱回调组到节点映射的别名
  // Alias for weak callback groups to nodes map
  using WeakCallbackGroupsToNodesMap = std::map<
      rclcpp::CallbackGroup::WeakPtr,
      rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
      std::owner_less<rclcpp::CallbackGroup::WeakPtr>>;

  // 虚析构函数
  // Virtual destructor
  virtual ~MemoryStrategy() = default;

  // 收集实体的纯虚函数
  // Pure virtual function for collecting entities
  virtual bool collect_entities(const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) = 0;

  // 获取准备好的订阅数
  // Get the number of ready subscriptions
  virtual size_t number_of_ready_subscriptions() const = 0;
  // 获取准备好的服务数
  // Get the number of ready services
  virtual size_t number_of_ready_services() const = 0;
  // 获取准备好的客户端数
  // Get the number of ready clients
  virtual size_t number_of_ready_clients() const = 0;
  // 获取准备好的事件数
  // Get the number of ready events
  virtual size_t number_of_ready_events() const = 0;
  // 获取准备好的定时器数
  // Get the number of ready timers
  virtual size_t number_of_ready_timers() const = 0;
  // 获取保护条件数
  // Get the number of guard conditions
  virtual size_t number_of_guard_conditions() const = 0;
  // 获取可等待对象数
  // Get the number of waitables
  virtual size_t number_of_waitables() const = 0;

  // 添加可等待句柄的纯虚函数
  // Pure virtual function for adding a waitable handle
  virtual void add_waitable_handle(const rclcpp::Waitable::SharedPtr& waitable) = 0;
  // 将句柄添加到等待集的纯虚函数
  // Pure virtual function for adding handles to the wait set
  virtual bool add_handles_to_wait_set(rcl_wait_set_t* wait_set) = 0;
  // 清除句柄的纯虚函数
  // Pure virtual function for clearing handles
  virtual void clear_handles() = 0;
  // 删除空句柄的纯虚函数
  // Pure virtual function for removing null handles
  virtual void remove_null_handles(rcl_wait_set_t* wait_set) = 0;

  // 添加保护条件的纯虚函数
  // Pure virtual function for adding a guard condition
  virtual void add_guard_condition(const rclcpp::GuardCondition& guard_condition) = 0;

  // 删除保护条件的纯虚函数
  // Pure virtual function for removing a guard condition
  virtual void remove_guard_condition(const rclcpp::GuardCondition* guard_condition) = 0;

  // 获取下一个订阅的纯虚函数
  // Pure virtual function for getting the next subscription
  virtual void get_next_subscription(
      rclcpp::AnyExecutable& any_exec,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) = 0;

  // 获取下一个服务的纯虚函数
  // Pure virtual function for getting the next service
  virtual void get_next_service(
      rclcpp::AnyExecutable& any_exec,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) = 0;

  // 获取下一个客户端的纯虚函数
  // Pure virtual function for getting the next client
  virtual void get_next_client(
      rclcpp::AnyExecutable& any_exec,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) = 0;

  // 获取下一个定时器的纯虚函数
  // Pure virtual function for getting the next timer
  virtual void get_next_timer(
      rclcpp::AnyExecutable& any_exec,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) = 0;

  // 获取下一个可等待对象的纯虚函数
  // Pure virtual function for getting the next waitable
  virtual void get_next_waitable(
      rclcpp::AnyExecutable& any_exec,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) = 0;

  // 获取分配器的纯虚函数
  // Pure virtual function for getting the allocator
  virtual rcl_allocator_t get_allocator() = 0;

  // 根据订阅句柄获取订阅的静态函数
  // Static function to get a subscription by its handle
  static rclcpp::SubscriptionBase::SharedPtr get_subscription_by_handle(
      const std::shared_ptr<const rcl_subscription_t>& subscriber_handle,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes);

  // 根据服务句柄获取服务的静态函数
  // Static function to get a service by its handle
  static rclcpp::ServiceBase::SharedPtr get_service_by_handle(
      const std::shared_ptr<const rcl_service_t>& service_handle,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes);

  // 根据客户端句柄获取客户端的静态函数
  // Static function to get a client by its handle
  static rclcpp::ClientBase::SharedPtr get_client_by_handle(
      const std::shared_ptr<const rcl_client_t>& client_handle,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes);

  // 根据定时器句柄获取定时器的静态函数
  // Static function to get a timer by its handle
  static rclcpp::TimerBase::SharedPtr get_timer_by_handle(
      const std::shared_ptr<const rcl_timer_t>& timer_handle,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes);

  // 根据组获取节点的静态函数
  // Static function to get a node by its group
  static rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_by_group(
      const rclcpp::CallbackGroup::SharedPtr& group,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes);

  // 根据订阅获取组的静态函数
  // Static function to get a group by its subscription
  static rclcpp::CallbackGroup::SharedPtr get_group_by_subscription(
      const rclcpp::SubscriptionBase::SharedPtr& subscription,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes);

  // 根据服务获取组的静态函数
  // Static function to get a group by its service
  static rclcpp::CallbackGroup::SharedPtr get_group_by_service(
      const rclcpp::ServiceBase::SharedPtr& service,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes);

  // 根据客户端获取组的静态函数
  // Static function to get a group by its client
  static rclcpp::CallbackGroup::SharedPtr get_group_by_client(
      const rclcpp::ClientBase::SharedPtr& client,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes);

  // 根据定时器获取组的静态函数
  // Static function to get a group by its timer
  static rclcpp::CallbackGroup::SharedPtr get_group_by_timer(
      const rclcpp::TimerBase::SharedPtr& timer,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes);

  // 根据可等待对象获取组的静态函数
  // Static function to get a group by its waitable
  static rclcpp::CallbackGroup::SharedPtr get_group_by_waitable(
      const rclcpp::Waitable::SharedPtr& waitable,
      const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes);
};

}  // namespace memory_strategy
}  // namespace rclcpp

#endif  // RCLCPP__MEMORY_STRATEGY_HPP_

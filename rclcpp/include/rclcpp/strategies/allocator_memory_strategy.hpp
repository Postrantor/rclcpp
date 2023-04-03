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

#ifndef RCLCPP__STRATEGIES__ALLOCATOR_MEMORY_STRATEGY_HPP_
#define RCLCPP__STRATEGIES__ALLOCATOR_MEMORY_STRATEGY_HPP_

#include <memory>
#include <vector>

#include "rcl/allocator.h"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/types.h"

namespace rclcpp {
namespace memory_strategies {
namespace allocator_memory_strategy {

/// 用于处理执行器执行过程中的内存分配的委托。
/// Delegate for handling memory allocations while the Executor is executing.
/**
 * 默认情况下，内存策略会在执行器等待工作后，根据实体的数量动态分配来自 rmw 实现的结构的内存。
 * By default, the memory strategy dynamically allocates memory for structures that come in from
 * the rmw implementation after the executor waits for work, based on the number of entities that
 * come through.
 */
template <typename Alloc = std::allocator<void>>  // 使用默认模板参数为 std::allocator<void>
                                                  // 的类型别名 Alloc
                                                  class AllocatorMemoryStrategy
    : public memory_strategy::MemoryStrategy  // 继承自 memory_strategy::MemoryStrategy 类
{
public:
  // 使用 RCLCPP_SMART_PTR_DEFINITIONS 宏定义智能指针类型
  RCLCPP_SMART_PTR_DEFINITIONS(AllocatorMemoryStrategy<Alloc>)

  // 使用 VoidAllocTraits 定义 void* 类型的内存分配器特性
  // Define the memory allocator traits for void* type using VoidAllocTraits
  using VoidAllocTraits = typename allocator::AllocRebind<void *, Alloc>;
  // 使用 VoidAlloc 定义 void* 类型的内存分配器
  // Define the memory allocator for void* type using VoidAlloc
  using VoidAlloc = typename VoidAllocTraits::allocator_type;

  // 构造函数，接受一个 std::shared_ptr<Alloc> 参数
  // Constructor, takes a std::shared_ptr<Alloc> parameter
  explicit AllocatorMemoryStrategy(std::shared_ptr<Alloc> allocator) {
    // 根据传入的 allocator 创建一个 VoidAlloc 类型的共享指针并赋值给 allocator_
    // Create a shared pointer of type VoidAlloc based on the passed in allocator and assign it to
    // allocator_
    allocator_ = std::make_shared<VoidAlloc>(*allocator.get());
  }

  // 默认构造函数
  // Default constructor
  AllocatorMemoryStrategy() { allocator_ = std::make_shared<VoidAlloc>(); }

  // 添加守护条件（GuardCondition）的方法，覆盖基类中的同名方法
  // Method for adding a GuardCondition, overrides the method with the same name in the base class
  void add_guard_condition(const rclcpp::GuardCondition &guard_condition) override {
    // 遍历已存在的守护条件集合
    // Iterate through the existing guard condition collection
    for (const auto &existing_guard_condition : guard_conditions_) {
      // 如果找到相同的守护条件，则直接返回
      // If the same guard condition is found, return directly
      if (existing_guard_condition == &guard_condition) {
        return;
      }
    }
    // 将新的守护条件添加到集合中
    // Add the new guard condition to the collection
    guard_conditions_.push_back(&guard_condition);
  }

  // 移除守护条件（GuardCondition）的方法，覆盖基类中的同名方法
  // Method for removing a GuardCondition, overrides the method with the same name in the base class
  void remove_guard_condition(const rclcpp::GuardCondition *guard_condition) override {
    // 遍历已存在的守护条件集合
    // Iterate through the existing guard condition collection
    for (auto it = guard_conditions_.begin(); it != guard_conditions_.end(); ++it) {
      // 如果找到需要移除的守护条件，则将其从集合中删除并跳出循环
      // If the guard condition to be removed is found, delete it from the collection and break the
      // loop
      if (*it == guard_condition) {
        guard_conditions_.erase(it);
        break;
      }
    }
  }

  /**
   * @brief 清除所有实体的句柄 (Clear all entity handles)
   */
  void clear_handles() override {
    // 清除订阅句柄列表 (Clear the subscription handles list)
    // 使用 clear() 方法清空容器中的所有元素 (Use the clear() method to remove all elements in the
    // container)
    subscription_handles_.clear();

    // 清除服务句柄列表 (Clear the service handles list)
    service_handles_.clear();

    // 清除客户端句柄列表 (Clear the client handles list)
    client_handles_.clear();

    // 清除定时器句柄列表 (Clear the timer handles list)
    timer_handles_.clear();

    // 清除可等待实体句柄列表 (Clear the waitable handles list)
    waitable_handles_.clear();
  }

  /**
   * @brief 移除空的句柄 (Remove null handles)
   *
   * @param wait_set 指向 rcl_wait_set_t 结构体的指针 (Pointer to an rcl_wait_set_t structure)
   */
  void remove_null_handles(rcl_wait_set_t *wait_set) override {
    // 遍历订阅句柄列表 (Iterate through the subscription handles list)
    for (size_t i = 0; i < subscription_handles_.size(); ++i) {
      // 如果对应的 wait_set 订阅为空 (If the corresponding wait_set subscription is null)
      if (!wait_set->subscriptions[i]) {
        // 重置订阅句柄 (Reset the subscription handle)
        subscription_handles_[i].reset();
      }
    }

    // 遍历服务句柄列表 (Iterate through the service handles list)
    for (size_t i = 0; i < service_handles_.size(); ++i) {
      // 如果对应的 wait_set 服务为空 (If the corresponding wait_set service is null)
      if (!wait_set->services[i]) {
        // 重置服务句柄 (Reset the service handle)
        service_handles_[i].reset();
      }
    }

    // 遍历客户端句柄列表 (Iterate through the client handles list)
    for (size_t i = 0; i < client_handles_.size(); ++i) {
      // 如果对应的 wait_set 客户端为空 (If the corresponding wait_set client is null)
      if (!wait_set->clients[i]) {
        // 重置客户端句柄 (Reset the client handle)
        client_handles_[i].reset();
      }
    }

    // 遍历定时器句柄列表 (Iterate through the timer handles list)
    for (size_t i = 0; i < timer_handles_.size(); ++i) {
      // 如果对应的 wait_set 定时器为空 (If the corresponding wait_set timer is null)
      if (!wait_set->timers[i]) {
        // 重置定时器句柄 (Reset the timer handle)
        timer_handles_[i].reset();
      }
    }

    // 遍历可等待实体句柄列表 (Iterate through the waitable handles list)
    for (size_t i = 0; i < waitable_handles_.size(); ++i) {
      // 如果对应的可等待实体不处于就绪状态 (If the corresponding waitable entity is not ready)
      if (!waitable_handles_[i]->is_ready(wait_set)) {
        // 重置可等待实体句柄 (Reset the waitable handle)
        waitable_handles_[i].reset();
      }
    }

    // 移除空的订阅句柄 (Remove null subscription handles)
    subscription_handles_.erase(
        std::remove(subscription_handles_.begin(), subscription_handles_.end(), nullptr),
        subscription_handles_.end());

    // 移除空的服务句柄 (Remove null service handles)
    service_handles_.erase(
        std::remove(service_handles_.begin(), service_handles_.end(), nullptr),
        service_handles_.end());

    // 移除空的客户端句柄 (Remove null client handles)
    client_handles_.erase(
        std::remove(client_handles_.begin(), client_handles_.end(), nullptr),
        client_handles_.end());

    // 移除空的定时器句柄 (Remove null timer handles)
    timer_handles_.erase(
        std::remove(timer_handles_.begin(), timer_handles_.end(), nullptr), timer_handles_.end());

    // 移除空的可等待实体句柄 (Remove null waitable handles)
    waitable_handles_.erase(
        std::remove(waitable_handles_.begin(), waitable_handles_.end(), nullptr),
        waitable_handles_.end());
  }

  /**
   * @brief 收集实体对象，例如订阅、服务、客户端、计时器和等待对象。
   * @param weak_groups_to_nodes 一个包含弱回调组到节点映射的容器。
   * @return 如果有无效的弱回调组或节点，则返回 true；否则返回 false。
   * @brief Collect entities like subscriptions, services, clients, timers, and waitables.
   * @param weak_groups_to_nodes A container with weak callback groups to nodes mapping.
   * @return True if there are invalid weak groups or nodes, false otherwise.
   */
  bool collect_entities(const WeakCallbackGroupsToNodesMap &weak_groups_to_nodes) override {
    // 声明一个布尔变量，用于检查是否存在无效的弱回调组或节点。
    // Declare a boolean variable to check for invalid weak groups or nodes.
    bool has_invalid_weak_groups_or_nodes = false;

    // 遍历 weak_groups_to_nodes 容器中的每一对元素。
    // Iterate through each pair of elements in the weak_groups_to_nodes container.
    for (const auto &pair : weak_groups_to_nodes) {
      // 尝试锁定并获取弱回调组和节点。
      // Try to lock and obtain the weak callback group and node.
      auto group = pair.first.lock();
      auto node = pair.second.lock();

      // 如果回调组或节点为空，则将 has_invalid_weak_groups_or_nodes 设置为 true 并继续下一次循环。
      // If the callback group or node is null, set has_invalid_weak_groups_or_nodes to true and
      // continue to the next iteration.
      if (group == nullptr || node == nullptr) {
        has_invalid_weak_groups_or_nodes = true;
        continue;
      }

      // 如果回调组不存在或不能从中获取信息，则继续下一次循环。
      // If the callback group is not present or cannot be taken from, continue to the next
      // iteration.
      if (!group || !group->can_be_taken_from().load()) {
        continue;
      }

      // 收集所有实体对象，例如订阅、服务、客户端、计时器和等待对象。
      // Collect all entity objects like subscriptions, services, clients, timers, and waitables.
      group->collect_all_ptrs(
          [this](const rclcpp::SubscriptionBase::SharedPtr &subscription) {
            subscription_handles_.push_back(subscription->get_subscription_handle());
          },
          [this](const rclcpp::ServiceBase::SharedPtr &service) {
            service_handles_.push_back(service->get_service_handle());
          },
          [this](const rclcpp::ClientBase::SharedPtr &client) {
            client_handles_.push_back(client->get_client_handle());
          },
          [this](const rclcpp::TimerBase::SharedPtr &timer) {
            timer_handles_.push_back(timer->get_timer_handle());
          },
          [this](const rclcpp::Waitable::SharedPtr &waitable) {
            waitable_handles_.push_back(waitable);
          });
    }

    // 返回是否有无效的弱回调组或节点的布尔值。
    // Return the boolean value indicating whether there are invalid weak groups or nodes.
    return has_invalid_weak_groups_or_nodes;
  }

  /**
   * @brief 添加等待对象句柄。
   * @param waitable 等待对象的共享指针。
   * @throws std::runtime_error 当等待对象为 nullptr 时抛出异常。
   * @brief Add waitable handle.
   * @param waitable Shared pointer to the waitable object.
   * @throws std::runtime_error Throws exception when waitable object is nullptr.
   */
  void add_waitable_handle(const rclcpp::Waitable::SharedPtr &waitable) override {
    // 如果等待对象为 nullptr，则抛出运行时错误。
    // If the waitable object is nullptr, throw a runtime error.
    if (nullptr == waitable) {
      throw std::runtime_error("waitable object unexpectedly nullptr");
    }

    // 将等待对象添加到 waitable_handles_ 容器中。
    // Add the waitable object to the waitable_handles_ container.
    waitable_handles_.push_back(waitable);
  }

  /**
   * @brief 将句柄添加到等待集合中 (Add handles to the wait set)
   *
   * @param wait_set 指向 rcl_wait_set_t 结构体的指针 (Pointer to an rcl_wait_set_t structure)
   * @return 如果所有句柄都成功添加，则返回 true，否则返回 false (Returns true if all handles were
   * added successfully, false otherwise)
   */
  bool add_handles_to_wait_set(rcl_wait_set_t *wait_set) override {
    // 遍历订阅句柄列表 (Iterate through the list of subscription handles)
    for (const std::shared_ptr<const rcl_subscription_t> &subscription : subscription_handles_) {
      // 尝试将订阅句柄添加到等待集合中 (Attempt to add the subscription handle to the wait set)
      if (rcl_wait_set_add_subscription(wait_set, subscription.get(), NULL) != RCL_RET_OK) {
        // 如果添加失败，记录错误并返回 false (If the addition fails, log the error and return
        // false)
        RCUTILS_LOG_ERROR_NAMED(
            "rclcpp", "Couldn't add subscription to wait set: %s", rcl_get_error_string().str);
        return false;
      }
    }

    // 遍历客户端句柄列表 (Iterate through the list of client handles)
    for (const std::shared_ptr<const rcl_client_t> &client : client_handles_) {
      // 尝试将客户端句柄添加到等待集合中 (Attempt to add the client handle to the wait set)
      if (rcl_wait_set_add_client(wait_set, client.get(), NULL) != RCL_RET_OK) {
        // 如果添加失败，记录错误并返回 false (If the addition fails, log the error and return
        // false)
        RCUTILS_LOG_ERROR_NAMED(
            "rclcpp", "Couldn't add client to wait set: %s", rcl_get_error_string().str);
        return false;
      }
    }

    // 遍历服务句柄列表 (Iterate through the list of service handles)
    for (const std::shared_ptr<const rcl_service_t> &service : service_handles_) {
      // 尝试将服务句柄添加到等待集合中 (Attempt to add the service handle to the wait set)
      if (rcl_wait_set_add_service(wait_set, service.get(), NULL) != RCL_RET_OK) {
        // 如果添加失败，记录错误并返回 false (If the addition fails, log the error and return
        // false)
        RCUTILS_LOG_ERROR_NAMED(
            "rclcpp", "Couldn't add service to wait set: %s", rcl_get_error_string().str);
        return false;
      }
    }

    // 遍历定时器句柄列表 (Iterate through the list of timer handles)
    for (const std::shared_ptr<const rcl_timer_t> &timer : timer_handles_) {
      // 尝试将定时器句柄添加到等待集合中 (Attempt to add the timer handle to the wait set)
      if (rcl_wait_set_add_timer(wait_set, timer.get(), NULL) != RCL_RET_OK) {
        // 如果添加失败，记录错误并返回 false (If the addition fails, log the error and return
        // false)
        RCUTILS_LOG_ERROR_NAMED(
            "rclcpp", "Couldn't add timer to wait set: %s", rcl_get_error_string().str);
        return false;
      }
    }

    // 遍历保护条件列表 (Iterate through the list of guard conditions)
    for (auto guard_condition : guard_conditions_) {
      // 将保护条件添加到 rcl 等待集合中 (Add the guard condition to the rcl wait set)
      detail::add_guard_condition_to_rcl_wait_set(*wait_set, *guard_condition);
    }

    // 遍历可等待句柄列表 (Iterate through the list of waitable handles)
    for (const std::shared_ptr<Waitable> &waitable : waitable_handles_) {
      // 将可等待句柄添加到等待集合中 (Add the waitable handle to the wait set)
      waitable->add_to_wait_set(wait_set);
    }

    // 如果所有句柄都成功添加，返回 true (Return true if all handles were added successfully)
    return true;
  }

  /**
   * @brief 获取下一个订阅 (Get the next subscription)
   *
   * @param[out] any_exec 用于存储找到的可执行订阅信息 (A reference to store the found executable
   * subscription information)
   * @param[in] weak_groups_to_nodes 回调组与节点之间的弱映射关系 (The weak mapping relationship
   * between callback groups and nodes)
   */
  void get_next_subscription(
      rclcpp::AnyExecutable &any_exec,
      const WeakCallbackGroupsToNodesMap &weak_groups_to_nodes) override {
    // 初始化订阅句柄迭代器 (Initialize the subscription handle iterator)
    auto it = subscription_handles_.begin();
    while (it != subscription_handles_.end()) {
      // 根据句柄获取订阅 (Get the subscription by its handle)
      auto subscription = get_subscription_by_handle(*it, weak_groups_to_nodes);
      if (subscription) {
        // 查找此句柄所属的组并检查是否可以服务 (Find the group for this handle and see if it can be
        // serviced)
        auto group = get_group_by_subscription(subscription, weak_groups_to_nodes);
        if (!group) {
          // 找不到组，意味着订阅无效...从准备好的列表中删除它并继续查找 (Group was not found,
          // meaning the subscription is not valid... Remove it from the ready list and continue
          // looking)
          it = subscription_handles_.erase(it);
          continue;
        }
        if (!group->can_be_taken_from().load()) {
          // 组是互斥的且正在使用中，所以暂时跳过它 (Group is mutually exclusive and is being used,
          // so skip it for now) 将其留给下次检查，但继续搜索 (Leave it to be checked next time, but
          // continue searching)
          ++it;
          continue;
        }
        // 否则可以安全地设置并返回 any_exec (Otherwise it is safe to set and return the any_exec)
        any_exec.subscription = subscription;
        any_exec.callback_group = group;
        any_exec.node_base = get_node_by_group(group, weak_groups_to_nodes);
        subscription_handles_.erase(it);
        return;
      }
      // 否则，订阅不再有效，删除它并继续 (Else, the subscription is no longer valid, remove it and
      // continue)
      it = subscription_handles_.erase(it);
    }
  }

  /**
   * @brief 获取下一个服务 (Get the next service)
   *
   * @param[out] any_exec 用于存储找到的可执行服务信息 (A reference to store the found executable
   * service information)
   * @param[in] weak_groups_to_nodes 回调组与节点之间的弱映射关系 (The weak mapping relationship
   * between callback groups and nodes)
   */
  void get_next_service(
      rclcpp::AnyExecutable &any_exec,
      const WeakCallbackGroupsToNodesMap &weak_groups_to_nodes) override {
    // 初始化服务句柄迭代器 (Initialize the service handle iterator)
    auto it = service_handles_.begin();
    while (it != service_handles_.end()) {
      // 根据句柄获取服务 (Get the service by its handle)
      auto service = get_service_by_handle(*it, weak_groups_to_nodes);
      if (service) {
        // 查找此句柄所属的组并检查是否可以服务 (Find the group for this handle and see if it can be
        // serviced)
        auto group = get_group_by_service(service, weak_groups_to_nodes);
        if (!group) {
          // 找不到组，意味着服务无效...从准备好的列表中删除它并继续查找 (Group was not found,
          // meaning the service is not valid... Remove it from the ready list and continue looking)
          it = service_handles_.erase(it);
          continue;
        }
        if (!group->can_be_taken_from().load()) {
          // 组是互斥的且正在使用中，所以暂时跳过它 (Group is mutually exclusive and is being used,
          // so skip it for now) 将其留给下次检查，但继续搜索 (Leave it to be checked next time, but
          // continue searching)
          ++it;
          continue;
        }
        // 否则可以安全地设置并返回 any_exec (Otherwise it is safe to set and return the any_exec)
        any_exec.service = service;
        any_exec.callback_group = group;
        any_exec.node_base = get_node_by_group(group, weak_groups_to_nodes);
        service_handles_.erase(it);
        return;
      }
      // 否则，服务不再有效，删除它并继续 (Else, the service is no longer valid, remove it and
      // continue)
      it = service_handles_.erase(it);
    }
  }

  /**
   * @brief 获取下一个客户端并处理 (Get the next client and handle it)
   *
   * @param any_exec 用于存储可执行实体的引用 (A reference to store the executable entity)
   * @param weak_groups_to_nodes 弱回调组到节点映射 (Weak callback groups to nodes map)
   */
  void get_next_client(
      rclcpp::AnyExecutable &any_exec,
      const WeakCallbackGroupsToNodesMap &weak_groups_to_nodes) override {
    // 初始化客户端句柄迭代器 (Initialize the client handles iterator)
    auto it = client_handles_.begin();

    // 遍历客户端句柄列表 (Iterate through the client handles list)
    while (it != client_handles_.end()) {
      // 根据客户端句柄获取客户端实例 (Get the client instance by the client handle)
      auto client = get_client_by_handle(*it, weak_groups_to_nodes);

      // 如果找到了客户端实例 (If a client instance is found)
      if (client) {
        // 根据客户端实例获取其所属的组 (Find the group for this handle and see if it can be
        // serviced)
        auto group = get_group_by_client(client, weak_groups_to_nodes);

        // 如果没有找到相应的组 (If the group is not found)
        if (!group) {
          // 组未找到，表示服务无效... (Group was not found, meaning the service is not valid...)
          // 从准备好的列表中删除它，并继续查找 (Remove it from the ready list and continue looking)
          it = client_handles_.erase(it);
          continue;
        }

        // 如果组不能被获取 (If the group cannot be taken from)
        if (!group->can_be_taken_from().load()) {
          // 组是互斥的且正在使用中，所以暂时跳过它 (Group is mutually exclusive and is being used,
          // so skip it for now) 将其留给下次检查，但继续搜索 (Leave it to be checked next time, but
          // continue searching)
          ++it;
          continue;
        }

        // 否则可以安全地设置并返回 any_exec (Otherwise, it is safe to set and return the any_exec)
        any_exec.client = client;
        any_exec.callback_group = group;
        any_exec.node_base = get_node_by_group(group, weak_groups_to_nodes);
        client_handles_.erase(it);
        return;
      }
      // 否则，服务不再有效，删除它并继续 (Else, the service is no longer valid, remove it and
      // continue)
      it = client_handles_.erase(it);
    }
  }

  /**
   * @brief 获取下一个定时器并处理 (Get the next timer and handle it)
   *
   * @param any_exec 用于存储可执行实体的引用 (A reference to store the executable entity)
   * @param weak_groups_to_nodes 弱回调组到节点映射 (Weak callback groups to nodes map)
   */
  void get_next_timer(
      rclcpp::AnyExecutable &any_exec,
      const WeakCallbackGroupsToNodesMap &weak_groups_to_nodes) override {
    // 初始化定时器句柄迭代器 (Initialize the timer handles iterator)
    auto it = timer_handles_.begin();

    // 遍历定时器句柄列表 (Iterate through the timer handles list)
    while (it != timer_handles_.end()) {
      // 根据定时器句柄获取定时器实例 (Get the timer instance by the timer handle)
      auto timer = get_timer_by_handle(*it, weak_groups_to_nodes);

      // 如果找到了定时器实例 (If a timer instance is found)
      if (timer) {
        // 根据定时器实例获取其所属的组 (Find the group for this handle and see if it can be
        // serviced)
        auto group = get_group_by_timer(timer, weak_groups_to_nodes);

        // 如果没有找到相应的组 (If the group is not found)
        if (!group) {
          // 组未找到，表示定时器无效... (Group was not found, meaning the timer is not valid...)
          // 从准备好的列表中删除它，并继续查找 (Remove it from the ready list and continue looking)
          it = timer_handles_.erase(it);
          continue;
        }

        // 如果组不能被获取 (If the group cannot be taken from)
        if (!group->can_be_taken_from().load()) {
          // 组是互斥的且正在使用中，所以暂时跳过它 (Group is mutually exclusive and is being used,
          // so skip it for now) 将其留给下次检查，但继续搜索 (Leave it to be checked next time, but
          // continue searching)
          ++it;
          continue;
        }

        // 如果定时器被取消，则跳过它 (If the timer is cancelled, skip it)
        if (!timer->call()) {
          ++it;
          continue;
        }

        // 否则可以安全地设置并返回 any_exec (Otherwise, it is safe to set and return the any_exec)
        any_exec.timer = timer;
        any_exec.callback_group = group;
        any_exec.node_base = get_node_by_group(group, weak_groups_to_nodes);
        timer_handles_.erase(it);
        return;
      }
      // 否则，定时器不再有效，删除它并继续 (Else, the timer is no longer valid, remove it and
      // continue)
      it = timer_handles_.erase(it);
    }
  }

  /**
   * @brief 获取下一个可等待的实体（Get the next waitable entity）
   *
   * @param[out] any_exec 存储找到的可执行实体的输出参数（Output parameter to store the found
   * executable entity）
   * @param[in] weak_groups_to_nodes 弱回调组到节点的映射（Weak callback groups to nodes mapping）
   */
  void get_next_waitable(
      rclcpp::AnyExecutable &any_exec,
      const WeakCallbackGroupsToNodesMap &weak_groups_to_nodes) override {
    // 初始化可等待句柄迭代器（Initialize the waitable handles iterator）
    auto it = waitable_handles_.begin();
    // 遍历所有可等待句柄（Iterate through all waitable handles）
    while (it != waitable_handles_.end()) {
      // 获取当前可等待句柄（Get the current waitable handle）
      std::shared_ptr<Waitable> &waitable = *it;
      if (waitable) {
        // 根据可等待句柄查找对应的组（Find the group corresponding to the waitable handle）
        auto group = get_group_by_waitable(waitable, weak_groups_to_nodes);
        if (!group) {
          // 如果未找到组，说明该等待句柄无效，则从就绪列表中移除并继续查找（If the group is not
          // found, the waitable handle is invalid. Remove it from the ready list and continue
          // searching）
          it = waitable_handles_.erase(it);
          continue;
        }
        if (!group->can_be_taken_from().load()) {
          // 如果组处于互斥状态且正在使用中，则暂时跳过（If the group is mutually exclusive and in
          // use, skip it for now） 保留以便下次检查，但继续搜索（Leave it to be checked next time,
          // but continue searching）
          ++it;
          continue;
        }
        // 否则，设置并返回 any_exec（Otherwise, set and return the any_exec）
        any_exec.waitable = waitable;
        any_exec.callback_group = group;
        any_exec.node_base = get_node_by_group(group, weak_groups_to_nodes);
        waitable_handles_.erase(it);
        return;
      }
      // 否则，可等待句柄不再有效，将其移除并继续（Otherwise, the waitable handle is no longer
      // valid, remove it and continue）
      it = waitable_handles_.erase(it);
    }
  }

  /**
   * @brief 获取分配器（Get the allocator）
   *
   * @return 返回分配器对象（Returns the allocator object）
   */
  rcl_allocator_t get_allocator() override {
    return rclcpp::allocator::get_rcl_allocator<void *, VoidAlloc>(*allocator_.get());
  }

  /**
   * @brief 获取准备好的订阅数量（Get the number of ready subscriptions）
   *
   * @return 返回准备好的订阅数量（Returns the number of ready subscriptions）
   */
  size_t number_of_ready_subscriptions() const override {
    size_t number_of_subscriptions = subscription_handles_.size();
    for (const std::shared_ptr<Waitable> &waitable : waitable_handles_) {
      number_of_subscriptions += waitable->get_number_of_ready_subscriptions();
    }
    return number_of_subscriptions;
  }

  /**
   * @brief 获取准备好的服务数量（Get the number of ready services）
   *
   * @return 返回准备好的服务数量（Returns the number of ready services）
   */
  size_t number_of_ready_services() const override {
    size_t number_of_services = service_handles_.size();
    for (const std::shared_ptr<Waitable> &waitable : waitable_handles_) {
      number_of_services += waitable->get_number_of_ready_services();
    }
    return number_of_services;
  }

  /**
   * @brief 获取已准备好的事件数量 (Get the number of ready events)
   *
   * @return size_t 已准备好的事件数量 (Number of ready events)
   */
  size_t number_of_ready_events() const override {
    // 初始化事件计数为0 (Initialize the event count to 0)
    size_t number_of_events = 0;

    // 遍历可等待对象列表 (Iterate through the list of waitable objects)
    for (const std::shared_ptr<Waitable> &waitable : waitable_handles_) {
      // 累加每个可等待对象的已准备好的事件数量 (Accumulate the number of ready events for each
      // waitable object)
      number_of_events += waitable->get_number_of_ready_events();
    }

    // 返回已准备好的事件总数 (Return the total number of ready events)
    return number_of_events;
  }

  /**
   * @brief 获取已准备好的客户端数量 (Get the number of ready clients)
   *
   * @return size_t 已准备好的客户端数量 (Number of ready clients)
   */
  size_t number_of_ready_clients() const override {
    // 初始化客户端计数为客户端句柄的大小 (Initialize the client count to the size of client
    // handles)
    size_t number_of_clients = client_handles_.size();

    // 遍历可等待对象列表 (Iterate through the list of waitable objects)
    for (const std::shared_ptr<Waitable> &waitable : waitable_handles_) {
      // 累加每个可等待对象的已准备好的客户端数量 (Accumulate the number of ready clients for each
      // waitable object)
      number_of_clients += waitable->get_number_of_ready_clients();
    }

    // 返回已准备好的客户端总数 (Return the total number of ready clients)
    return number_of_clients;
  }

  /**
   * @brief 获取保护条件数量 (Get the number of guard conditions)
   *
   * @return size_t 保护条件数量 (Number of guard conditions)
   */
  size_t number_of_guard_conditions() const override {
    // 初始化保护条件计数为保护条件列表的大小 (Initialize the guard condition count to the size of
    // guard conditions list)
    size_t number_of_guard_conditions = guard_conditions_.size();

    // 遍历可等待对象列表 (Iterate through the list of waitable objects)
    for (const std::shared_ptr<Waitable> &waitable : waitable_handles_) {
      // 累加每个可等待对象的已准备好的保护条件数量 (Accumulate the number of ready guard conditions
      // for each waitable object)
      number_of_guard_conditions += waitable->get_number_of_ready_guard_conditions();
    }

    // 返回保护条件总数 (Return the total number of guard conditions)
    return number_of_guard_conditions;
  }

  /**
   * @brief 获取已准备好的定时器数量 (Get the number of ready timers)
   *
   * @return size_t 已准备好的定时器数量 (Number of ready timers)
   */
  size_t number_of_ready_timers() const override {
    // 初始化定时器计数为定时器句柄的大小 (Initialize the timer count to the size of timer handles)
    size_t number_of_timers = timer_handles_.size();

    // 遍历可等待对象列表 (Iterate through the list of waitable objects)
    for (const std::shared_ptr<Waitable> &waitable : waitable_handles_) {
      // 累加每个可等待对象的已准备好的定时器数量 (Accumulate the number of ready timers for each
      // waitable object)
      number_of_timers += waitable->get_number_of_ready_timers();
    }

    // 返回已准备好的定时器总数 (Return the total number of ready timers)
    return number_of_timers;
  }

  /**
   * @brief 获取可等待对象数量 (Get the number of waitable objects)
   *
   * @return size_t 可等待对象数量 (Number of waitable objects)
   */
  size_t number_of_waitables() const override { return waitable_handles_.size(); }

private:
  /**
   * @brief 使用指定分配器重新绑定的 std::vector 类型别名定义。
   *        Type alias definition for std::vector with a rebinded allocator.
   *
   * @tparam T 向量元素的类型。
   *           The type of the elements in the vector.
   */
  template <typename T>
  using VectorRebind =
      std::vector<T, typename std::allocator_traits<Alloc>::template rebind_alloc<T>>;

  // 用于存储 GuardCondition 类型指针的向量。
  // Vector to store pointers to GuardCondition types.
  VectorRebind<const rclcpp::GuardCondition *> guard_conditions_;

  // 用于存储 rcl_subscription_t 类型共享指针的向量。
  // Vector to store shared pointers to rcl_subscription_t types.
  VectorRebind<std::shared_ptr<const rcl_subscription_t>> subscription_handles_;

  // 用于存储 rcl_service_t 类型共享指针的向量。
  // Vector to store shared pointers to rcl_service_t types.
  VectorRebind<std::shared_ptr<const rcl_service_t>> service_handles_;

  // 用于存储 rcl_client_t 类型共享指针的向量。
  // Vector to store shared pointers to rcl_client_t types.
  VectorRebind<std::shared_ptr<const rcl_client_t>> client_handles_;

  // 用于存储 rcl_timer_t 类型共享指针的向量。
  // Vector to store shared pointers to rcl_timer_t types.
  VectorRebind<std::shared_ptr<const rcl_timer_t>> timer_handles_;

  // 用于存储 Waitable 类型共享指针的向量。
  // Vector to store shared pointers to Waitable types.
  VectorRebind<std::shared_ptr<Waitable>> waitable_handles_;

  // 分配器的共享指针。
  // Shared pointer to the allocator.
  std::shared_ptr<VoidAlloc> allocator_;
};

}  // namespace allocator_memory_strategy
}  // namespace memory_strategies
}  // namespace rclcpp

#endif  // RCLCPP__STRATEGIES__ALLOCATOR_MEMORY_STRATEGY_HPP_

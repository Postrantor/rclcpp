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

#ifndef RCLCPP__WAIT_SET_POLICIES__THREAD_SAFE_SYNCHRONIZATION_HPP_
#define RCLCPP__WAIT_SET_POLICIES__THREAD_SAFE_SYNCHRONIZATION_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

#include "rclcpp/client.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_wait_set_mask.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_result.hpp"
#include "rclcpp/wait_result_kind.hpp"
#include "rclcpp/wait_set_policies/detail/synchronization_policy_common.hpp"
#include "rclcpp/wait_set_policies/detail/write_preferring_read_write_lock.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp {
namespace wait_set_policies {

/// 等待集策略，提供线程安全的等待集同步。
/// WaitSet policy that provides thread-safe synchronization for the wait set.
/**
 * 这个类使用 "写优先的读写锁"，使得添加和移除等待集中的项优先于读取（即等待）。
 * This class uses a "write-preferring RW lock" so that adding items to, and
 * removing items from, the wait set will take priority over reading, i.e.
 * waiting.
 * 这样做是因为添加和移除调用会打断等待集，所以当有许多添加/移除操作排队时，进行“公平”锁定是浪费的。
 * This is done since add and remove calls will interrupt the wait set anyways
 * so it is wasteful to do "fair" locking when there are many add/remove
 * operations queued up.
 *
 * 关于此策略提供的线程安全性，有一些需要注意的事项。
 * There are some things to consider about the thread-safety provided by this
 * policy.
 * 有两类活动：读活动和写活动。
 * There are two categories of activities, reading and writing activities.
 * 写活动包括所有的添加和移除方法，以及 prune_deleted_entities() 方法。
 * The writing activities include all of the add and remove methods, as well as
 * the prune_deleted_entities() method.
 * 读方法包括 wait() 方法和保持 WaitResult 在作用域内。
 * The reading methods include the wait() method and keeping a WaitResult in
 * scope.
 * 读活动和写活动不会同时运行，其中一个会阻塞另一个。
 * The reading and writing activities will not be run at the same time, and one
 * will block the other.
 * 因此，如果在作用域内保持 WaitResult，并尝试同时添加或移除实体，它们将互相阻塞。
 * Therefore, if you are holding a WaitResult in scope, and try to add or
 * remove an entity at the same time, they will block each other.
 * 写活动将尝试通过触发保护条件来中断 wait() 方法，但它们无法让 WaitResult 释放锁。
 * The write activities will try to interrupt the wait() method by triggering
 * a guard condition, but they have no way of causing the WaitResult to release
 * its lock.
 */
class ThreadSafeSynchronization : public detail::SynchronizationPolicyCommon {
protected:
  // 显示构造函数，接收一个 rclcpp::Context::SharedPtr 类型的参数。
  // Explicit constructor, takes a rclcpp::Context::SharedPtr as parameter.
  explicit ThreadSafeSynchronization(rclcpp::Context::SharedPtr context)
      // 初始化列表：创建额外的保护条件和读写锁。
      // Initialization list: create extra guard conditions and read-write lock.
      : extra_guard_conditions_{{std::make_shared<rclcpp::GuardCondition>(context)}},
        wprw_lock_([this]() { this->interrupt_waiting_wait_set(); }) {}
  // 默认析构函数。
  // Default destructor.
  ~ThreadSafeSynchronization() = default;

  /// 返回实现同步策略所需的任何“额外”守护条件。
  /// Return any "extra" guard conditions needed to implement the synchronization policy.
  /**
   * 该策略有一个守护条件，用于在添加和删除实体时中断等待集。
   * This policy has one guard condition which is used to interrupt the wait set when adding and
   * removing entities.
   */
  const std::array<std::shared_ptr<rclcpp::GuardCondition>, 1> &get_extra_guard_conditions() {
    // 返回额外的守护条件数组
    // Return the extra guard conditions array
    return extra_guard_conditions_;
  }

  /// 中断任何等待的等待集。
  /// Interrupt any waiting wait set.
  /**
   * 用于在添加或删除项目时中断等待集。
   * Used to interrupt the wait set when adding or removing items.
   */
  void interrupt_waiting_wait_set() { extra_guard_conditions_[0]->trigger(); }

  /// 添加订阅。
  /// Add subscription.
  /**
   * @param[in] subscription 要添加的订阅（以右值引用传递）
   * @param[in] mask 订阅等待集掩码
   * @param[in] add_subscription_function 添加订阅的函数
   *
   * @param[in] subscription The subscription to be added (passed as an rvalue reference)
   * @param[in] mask Subscription wait set mask
   * @param[in] add_subscription_function Function for adding the subscription
   */
  void sync_add_subscription(
      std::shared_ptr<rclcpp::SubscriptionBase> &&subscription,
      const rclcpp::SubscriptionWaitSetMask &mask,
      std::function<void(
          std::shared_ptr<rclcpp::SubscriptionBase> &&, const rclcpp::SubscriptionWaitSetMask &)>
          add_subscription_function) {
    // 使用 WritePreferringReadWriteLock 类型
    // Use the WritePreferringReadWriteLock type
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;

    // 对写锁进行加锁，以确保同步
    // Lock the write mutex to ensure synchronization
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());

    // 调用添加订阅的函数
    // Call the function to add the subscription
    add_subscription_function(std::move(subscription), mask);
  }

  /// 删除订阅。
  /// Remove subscription.
  /**
   * @param[in] subscription 要删除的订阅（以右值引用传递）
   * @param[in] mask 订阅等待集掩码
   * @param[in] remove_subscription_function 删除订阅的函数
   *
   * @param[in] subscription The subscription to be removed (passed as an rvalue reference)
   * @param[in] mask Subscription wait set mask
   * @param[in] remove_subscription_function Function for removing the subscription
   */
  void sync_remove_subscription(
      std::shared_ptr<rclcpp::SubscriptionBase> &&subscription,
      const rclcpp::SubscriptionWaitSetMask &mask,
      std::function<void(
          std::shared_ptr<rclcpp::SubscriptionBase> &&, const rclcpp::SubscriptionWaitSetMask &)>
          remove_subscription_function) {
    // 使用 WritePreferringReadWriteLock 类型
    // Use the WritePreferringReadWriteLock type
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;

    // 对写锁进行加锁，以确保同步
    // Lock the write mutex to ensure synchronization
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());

    // 调用删除订阅的函数
    // Call the function to remove the subscription
    remove_subscription_function(std::move(subscription), mask);
  }

  /// 添加守护条件。
  /// Add guard condition.
  /**
   * @param[in] guard_condition 要添加的守护条件（以右值引用传递）
   * @param[in] add_guard_condition_function 添加守护条件的函数
   *
   * @param[in] guard_condition The guard condition to be added (passed as an rvalue reference)
   * @param[in] add_guard_condition_function Function for adding the guard condition
   */
  void sync_add_guard_condition(
      std::shared_ptr<rclcpp::GuardCondition> &&guard_condition,
      std::function<void(std::shared_ptr<rclcpp::GuardCondition> &&)>
          add_guard_condition_function) {
    // 使用 WritePreferringReadWriteLock 类型
    // Use the WritePreferringReadWriteLock type
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;

    // 对写锁进行加锁，以确保同步
    // Lock the write mutex to ensure synchronization
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());

    // 调用添加守护条件的函数
    // Call the function to add the guard condition
    add_guard_condition_function(std::move(guard_condition));
  }

  /// 删除保护条件。
  /// Remove guard condition.
  void sync_remove_guard_condition(
      std::shared_ptr<rclcpp::GuardCondition>
          &&guard_condition,  ///< [in] 要删除的保护条件。The guard condition to remove.
      std::function<void(std::shared_ptr<rclcpp::GuardCondition> &&)>
          remove_guard_condition_function)  ///< [in] 用于删除保护条件的函数。Function to remove the
                                            ///< guard condition.
  {
    // 使用读写锁，优先写。
    // Use write-preferring read-write lock.
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    // 获取写锁并锁定。
    // Get and lock the write mutex.
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    // 调用删除保护条件的函数。
    // Call the function to remove the guard condition.
    remove_guard_condition_function(std::move(guard_condition));
  }

  /// 添加定时器。
  /// Add timer.
  void sync_add_timer(
      std::shared_ptr<rclcpp::TimerBase> &&timer,  ///< [in] 要添加的定时器。The timer to add.
      std::function<void(std::shared_ptr<rclcpp::TimerBase> &&)>
          add_timer_function)  ///< [in] 用于添加定时器的函数。Function to add the timer.
  {
    // 使用读写锁，优先写。
    // Use write-preferring read-write lock.
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    // 获取写锁并锁定。
    // Get and lock the write mutex.
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    // 调用添加定时器的函数。
    // Call the function to add the timer.
    add_timer_function(std::move(timer));
  }

  /// 删除定时器。
  /// Remove timer.
  void sync_remove_timer(
      std::shared_ptr<rclcpp::TimerBase> &&timer,  ///< [in] 要删除的定时器。The timer to remove.
      std::function<void(std::shared_ptr<rclcpp::TimerBase> &&)>
          remove_timer_function)  ///< [in] 用于删除定时器的函数。Function to remove the timer.
  {
    // 使用读写锁，优先写。
    // Use write-preferring read-write lock.
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    // 获取写锁并锁定。
    // Get and lock the write mutex.
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    // 调用删除定时器的函数。
    // Call the function to remove the timer.
    remove_timer_function(std::move(timer));
  }

  /// 添加客户端。
  /// Add client.
  void sync_add_client(
      std::shared_ptr<rclcpp::ClientBase> &&client,  ///< [in] 要添加的客户端。The client to add.
      std::function<void(std::shared_ptr<rclcpp::ClientBase> &&)>
          add_client_function)  ///< [in] 用于添加客户端的函数。Function to add the client.
  {
    // 使用读写锁，优先写。
    // Use write-preferring read-write lock.
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    // 获取写锁并锁定。
    // Get and lock the write mutex.
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    // 调用添加客户端的函数。
    // Call the function to add the client.
    add_client_function(std::move(client));
  }

  /// 删除客户端。
  /// Remove client.
  void sync_remove_client(
      std::shared_ptr<rclcpp::ClientBase> &&client,  ///< [in] 要删除的客户端。The client to remove.
      std::function<void(std::shared_ptr<rclcpp::ClientBase> &&)>
          remove_client_function)  ///< [in] 用于删除客户端的函数。Function to remove the client.
  {
    // 使用读写锁，优先写。
    // Use write-preferring read-write lock.
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    // 获取写锁并锁定。
    // Get and lock the write mutex.
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    // 调用删除客户端的函数。
    // Call the function to remove the client.
    remove_client_function(std::move(client));
  }

  /// 添加服务。
  /// Add service.
  void sync_add_service(
      std::shared_ptr<rclcpp::ServiceBase> &&service,  ///< [in] 要添加的服务。The service to add.
      std::function<void(std::shared_ptr<rclcpp::ServiceBase> &&)>
          add_service_function)  ///< [in] 用于添加服务的函数。Function to add the service.
  {
    // 使用读写锁，优先写。
    // Use write-preferring read-write lock.
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    // 获取写锁并锁定。
    // Get and lock the write mutex.
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    // 调用添加服务的函数。
    // Call the function to add the service.
    add_service_function(std::move(service));
  }

  /// 删除服务。
  /// Remove service.
  void sync_remove_service(
      std::shared_ptr<rclcpp::ServiceBase>
          &&service,                ///< [in] 要删除的服务。The service to remove.
      std::function<void(std::shared_ptr<rclcpp::ServiceBase> &&)>
          remove_service_function)  ///< [in] 用于删除服务的函数。Function to remove the service.
  {
    // 使用读写锁，优先写。
    // Use write-preferring read-write lock.
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    // 获取写锁并锁定。
    // Get and lock the write mutex.
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    // 调用删除服务的函数。
    // Call the function to remove the service.
    remove_service_function(std::move(service));
  }

  /// 添加可等待对象。
  /// Add waitable.
  void sync_add_waitable(
      std::shared_ptr<rclcpp::Waitable>
          &&waitable,  ///< [in] 要添加的可等待对象。The waitable to add.
      std::shared_ptr<void> &&
          associated_entity,  ///< [in] 与可等待对象关联的实体。Associated entity with the waitable.
      std::function<void(std::shared_ptr<rclcpp::Waitable> &&, std::shared_ptr<void> &&)>
          add_waitable_function)  ///< [in] 用于添加可等待对象的函数。Function to add the waitable.
  {
    // 使用读写锁，优先写。
    // Use write-preferring read-write lock.
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    // 获取写锁并锁定。
    // Get and lock the write mutex.
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    // 调用添加可等待对象的函数。
    // Call the function to add the waitable.
    add_waitable_function(std::move(waitable), std::move(associated_entity));
  }

  /**
   * @brief 移除等待处理的对象 (Remove a waitable object)
   *
   * @param[in] waitable 需要移除的等待处理对象 (The waitable object to be removed)
   * @param[in] remove_waitable_function 用于移除等待处理对象的函数 (Function to remove the waitable
   * object)
   */
  void sync_remove_waitable(
      std::shared_ptr<rclcpp::Waitable> &&waitable,
      std::function<void(std::shared_ptr<rclcpp::Waitable> &&)> remove_waitable_function) {
    // 使用 WritePreferringReadWriteLock 类型 (Use WritePreferringReadWriteLock type)
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;

    // 获取写锁并加锁 (Acquire the write mutex and lock it)
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());

    // 调用移除等待处理对象的函数 (Call the function to remove the waitable object)
    remove_waitable_function(std::move(waitable));
  }

  /**
   * @brief 清除已删除的实体 (Prune deleted entities)
   *
   * @param[in] prune_deleted_entities_function 用于清除已删除实体的函数 (Function to prune the
   * deleted entities)
   */
  void sync_prune_deleted_entities(std::function<void()> prune_deleted_entities_function) {
    // 使用 WritePreferringReadWriteLock 类型 (Use WritePreferringReadWriteLock type)
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;

    // 获取写锁并加锁 (Acquire the write mutex and lock it)
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());

    // 调用清除已删除实体的函数 (Call the function to prune the deleted entities)
    prune_deleted_entities_function();
  }

  /**
   * @brief 同步等待实现 (Synchronous wait implementation)
   *
   * @tparam WaitResultT 等待结果类型 (Wait result type)
   * @param time_to_wait_ns 等待的时间 (Time to wait)
   * @param rebuild_rcl_wait_set 重新构建 rcl_wait_set 的函数 (Function to rebuild the rcl_wait_set)
   * @param get_rcl_wait_set 获取 rcl_wait_set 的函数 (Function to get the rcl_wait_set)
   * @param create_wait_result 创建等待结果的函数 (Function to create the wait result)
   * @return WaitResultT 等待结果 (Wait result)
   */
  template <class WaitResultT>
  WaitResultT sync_wait(
      std::chrono::nanoseconds time_to_wait_ns,
      std::function<void()> rebuild_rcl_wait_set,
      std::function<rcl_wait_set_t &()> get_rcl_wait_set,
      std::function<WaitResultT(WaitResultKind wait_result_kind)> create_wait_result) {
    // 假设：此函数假定已采取某种措施来确保等待集正在等待的所有实体
    // 不允许超出范围，因此被删除。
    // (Assumption: this function assumes that some measure has been taken to
    // ensure none of the entities being waited on by the wait set are allowed
    // to go out of scope and therefore be deleted.)
    //
    // 在 StaticStorage 策略中，通过在其自身生命周期内保留所有实体的共享所有权来确保这一点。
    // (In the case of the StaticStorage policy, this is ensured because it
    // retains shared ownership of all entites for the duration of its own life.)
    //
    // 在 DynamicStorage
    // 策略中，通过调用此函数的函数在此函数持续期间获取实体的共享所有权来确保这一点。 (In the case
    // of the DynamicStorage policy, this is ensured by the function which calls this function, by
    // acquiring shared ownership of the entites for the duration of this function.)

    // Setup looping predicate.
    // 设置循环条件判断函数
    auto start = std::chrono::steady_clock::now();
    std::function<bool()> should_loop = this->create_loop_predicate(time_to_wait_ns, start);

    // Wait until exit condition is met.
    // 等待直到满足退出条件
    do {
      {
        // We have to prevent the entity sets from being mutated while building
        // the rcl wait set.
        // 在构建 rcl 等待集时，我们必须防止实体集发生变化
        using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
        std::lock_guard<WritePreferringReadWriteLock::ReadMutex> lock(wprw_lock_.get_read_mutex());

        // Rebuild the wait set.
        // This will resize the wait set if needed, due to e.g. adding or removing
        // entities since the last wait, but this should never occur in static
        // storage wait sets since they cannot be changed after construction.
        // This will also clear the wait set and re-add all the entities, which
        // prepares it to be waited on again.
        // 重建等待集
        // 如果需要，这将调整等待集的大小，例如自上次等待以来添加或删除实体，
        // 但这在静态存储等待集中永远不会发生，因为它们在构建后无法更改。
        // 这还将清除等待集并重新添加所有实体，以便再次等待
        rebuild_rcl_wait_set();
      }

      rcl_wait_set_t &rcl_wait_set = get_rcl_wait_set();

      // Wait unconditionally until timeout condition occurs since we assume
      // there are no conditions that would require the wait to stop and reset,
      // like asynchronously adding or removing an entity, i.e. explicitly
      // providing no thread-safety.
      // 无条件等待，直到超时条件发生，因为我们假设
      // 没有需要停止和重置等待的条件，
      // 如异步添加或删除实体，即明确地不提供线程安全

      // Calculate how much time there is left to wait, unless blocking indefinitely.
      // 计算还有多少时间要等待，除非无限期阻塞
      auto time_left_to_wait_ns = this->calculate_time_left_to_wait(time_to_wait_ns, start);

      // Then wait for entities to become ready.
      // 然后等待实体变得可用

      // It is ok to wait while not having the lock acquired, because the state
      // in the rcl wait set will not be updated until this method calls
      // rebuild_rcl_wait_set().
      // 当没有获取锁时等待是可以的，因为在此方法调用
      // rebuild_rcl_wait_set() 之前，rcl 等待集中的状态不会更新
      rcl_ret_t ret = rcl_wait(&rcl_wait_set, time_left_to_wait_ns.count());
      if (RCL_RET_OK == ret) {
        // Something has become ready in the wait set, first check if it was
        // the guard condition added by this class and/or a user defined guard condition.
        // 等待集中有东西变得可用，首先检查它是否是
        // 由这个类添加的保护条件和/或用户定义的保护条件
        const rcl_guard_condition_t *interrupt_guard_condition_ptr =
            &(extra_guard_conditions_[0]->get_rcl_guard_condition());
        bool was_interrupted_by_this_class = false;
        bool any_user_guard_conditions_triggered = false;
        for (size_t index = 0; index < rcl_wait_set.size_of_guard_conditions; ++index) {
          const rcl_guard_condition_t *current = rcl_wait_set.guard_conditions[index];
          if (nullptr != current) {
            // Something is ready.
            // 有东西准备好了
            if (rcl_wait_set.guard_conditions[index] == interrupt_guard_condition_ptr) {
              // This means that this class triggered a guard condition to interrupt this wait.
              // 这意味着这个类触发了一个保护条件来中断这个等待
              was_interrupted_by_this_class = true;
            } else {
              // This means it was a user guard condition.
              // 这意味着它是一个用户保护条件
              any_user_guard_conditions_triggered = true;
            }
          }
        }

        if (!was_interrupted_by_this_class || any_user_guard_conditions_triggered) {
          // In this case we know:
          //   - something was ready
          //   - it was either:
          //     - not interrupted by this class, or
          //     - maybe it was, but there were also user defined guard conditions.
          //
          // We cannot ignore user defined guard conditions, but we can ignore
          // other kinds of user defined entities, because they will still be
          // ready next time we wait, whereas guard conditions are cleared.
          // Therefore we need to create a WaitResult and return it.
          // 在这种情况下，我们知道：
          //   - 有东西准备好了
          //   - 它要么：
          //     - 没有被这个类中断，或者
          //     - 也许它是，但还有用户定义的保护条件。
          //
          // 我们不能忽略用户定义的保护条件，但我们可以忽略
          // 其他类型的用户定义实体，因为它们仍然会
          // 下次等待时准备好，而保护条件已经清除。
          // 因此，我们需要创建一个 WaitResult 并返回它
          return create_wait_result(WaitResultKind::Ready);
        }
        // If we get here the we interrupted the wait set and there were no user
        // guard conditions that needed to be handled.
        // So we will loop and it will re-acquire the lock and rebuild the
        // rcl wait set.
        // 如果我们到达这里，我们中断了等待集，没有用户
        // 需要处理的保护条件。
        // 所以我们会循环，它将重新获取锁并重建
        // rcl 等待集
      } else if (RCL_RET_TIMEOUT == ret) {
        // The wait set timed out, exit the loop.
        // 等待集超时，退出循环
        break;
      } else if (RCL_RET_WAIT_SET_EMPTY == ret) {
        // Wait set was empty, return Empty.
        // 等待集为空，返回 Empty
        return create_wait_result(WaitResultKind::Empty);
      } else {
        // Some other error case, throw.
        // 其他错误情况，抛出异常
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    } while (should_loop());

    // Wait did not result in ready items, return timeout.
    // 等待没有导致准备好的项目，返回超时
    return create_wait_result(WaitResultKind::Timeout);
  }

  /**
   * @brief 同步等待结果获取 (Synchronously wait for result acquisition)
   */
  void sync_wait_result_acquire() {
    // 获取读互斥锁并锁定 (Acquire the read mutex and lock it)
    // 这将阻止其他线程写入数据，但允许多个线程同时读取数据
    // (This will prevent other threads from writing data, but allows multiple threads to read data
    // simultaneously)
    wprw_lock_.get_read_mutex().lock();
  }

  /**
   * @brief 同步等待结果释放 (Synchronously wait for result release)
   */
  void sync_wait_result_release() {
    // 解锁读互斥锁 (Unlock the read mutex)
    // 允许其他线程访问数据 (Allow other threads to access the data)
    wprw_lock_.get_read_mutex().unlock();
  }

protected:
  // 定义一个 std::array 类型的成员变量 extra_guard_conditions_
  // (Define a member variable of type std::array named extra_guard_conditions_)
  // 包含一个共享指针的数组，指向 rclcpp::GuardCondition 类型的对象
  // (An array containing shared pointers pointing to objects of type rclcpp::GuardCondition)
  std::array<std::shared_ptr<rclcpp::GuardCondition>, 1> extra_guard_conditions_;

  // 定义一个 WritePreferringReadWriteLock 类型的成员变量 wprw_lock_
  // (Define a member variable of type WritePreferringReadWriteLock named wprw_lock_)
  // 这是一种偏向写操作的读写锁，用于同步对共享资源的访问
  // (This is a write-preferring read-write lock used for synchronizing access to shared resources)
  rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock wprw_lock_;
};

}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__THREAD_SAFE_SYNCHRONIZATION_HPP_

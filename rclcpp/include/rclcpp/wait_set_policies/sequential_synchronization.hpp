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

#ifndef RCLCPP__WAIT_SET_POLICIES__SEQUENTIAL_SYNCHRONIZATION_HPP_
#define RCLCPP__WAIT_SET_POLICIES__SEQUENTIAL_SYNCHRONIZATION_HPP_

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
#include "rclcpp/waitable.hpp"

namespace rclcpp {
namespace wait_set_policies {

/// @brief WaitSet策略，明确提供无线程同步的功能。
/// @brief WaitSet policy that explicitly provides no thread synchronization.
class SequentialSynchronization : public detail::SynchronizationPolicyCommon {
protected:
  /// @brief 构造函数，接收一个rclcpp::Context::SharedPtr类型的参数。
  /// @brief Constructor, accepts a parameter of type rclcpp::Context::SharedPtr.
  explicit SequentialSynchronization(rclcpp::Context::SharedPtr) {}

  /// @brief 析构函数，默认实现。
  /// @brief Destructor, default implementation.
  ~SequentialSynchronization() = default;

  /// 返回实现同步策略所需的任何“额外”保护条件。
  /// Return any "extra" guard conditions needed to implement the synchronization policy.
  /**
   * 由于此策略不提供线程安全性，因此也不需要额外的保护条件来实现它。
   * Since this policy provides no thread-safety, it also needs no extra guard
   * conditions to implement it.
   */
  const std::array<std::shared_ptr<rclcpp::GuardCondition>, 0> &get_extra_guard_conditions() {
    // 定义一个静态空数组并返回
    // Define a static empty array and return it
    static const std::array<std::shared_ptr<rclcpp::GuardCondition>, 0> empty{};
    return empty;
  }

  /// 添加订阅，不带线程安全性。
  /// Add subscription without thread-safety.
  /**
   * 不会抛出异常，但存储函数可能会抛出异常。
   * Does not throw, but storage function may throw.
   */
  void sync_add_subscription(
      // 输入参数：订阅对象的智能指针
      // Input parameter: Smart pointer of the subscription object
      std::shared_ptr<rclcpp::SubscriptionBase> &&subscription,
      // 输入参数：订阅等待集合掩码
      // Input parameter: Subscription wait set mask
      const rclcpp::SubscriptionWaitSetMask &mask,
      // 输入参数：添加订阅功能的函数对象
      // Input parameter: Function object for adding subscription functionality
      std::function<void(
          std::shared_ptr<rclcpp::SubscriptionBase> &&, const rclcpp::SubscriptionWaitSetMask &)>
          add_subscription_function) {
    // 显式地没有线程同步。
    // Explicitly no thread synchronization.
    // 调用添加订阅功能函数，传入订阅对象和掩码
    // Call the add subscription function with the subscription object and mask
    add_subscription_function(std::move(subscription), mask);
  }

  /// 移除保护条件（不具有线程安全性）。(Remove guard condition without thread-safety.)
  /**
   * 不会抛出异常，但存储函数可能会抛出异常。(Does not throw, but storage function may throw.)
   *
   * @param[in] subscription 需要移除的订阅。(The subscription to be removed.)
   * @param[in] mask 订阅等待集合掩码。(The subscription wait set mask.)
   * @param[in] remove_subscription_function 用于移除订阅的函数。(The function for removing the
   * subscription.)
   */
  void sync_remove_subscription(
      std::shared_ptr<rclcpp::SubscriptionBase> &&subscription,
      const rclcpp::SubscriptionWaitSetMask &mask,
      std::function<void(
          std::shared_ptr<rclcpp::SubscriptionBase> &&, const rclcpp::SubscriptionWaitSetMask &)>
          remove_subscription_function) {
    // 明确无线程同步。(Explicitly no thread synchronization.)
    remove_subscription_function(std::move(subscription), mask);
  }

  /// 添加保护条件（不具有线程安全性）。(Add guard condition without thread-safety.)
  /**
   * 不会抛出异常，但存储函数可能会抛出异常。(Does not throw, but storage function may throw.)
   *
   * @param[in] guard_condition 需要添加的保护条件。(The guard condition to be added.)
   * @param[in] add_guard_condition_function 用于添加保护条件的函数。(The function for adding the
   * guard condition.)
   */
  void sync_add_guard_condition(
      std::shared_ptr<rclcpp::GuardCondition> &&guard_condition,
      std::function<void(std::shared_ptr<rclcpp::GuardCondition> &&)>
          add_guard_condition_function) {
    // 明确无线程同步。(Explicitly no thread synchronization.)
    add_guard_condition_function(std::move(guard_condition));
  }

  /// 移除保护条件（不具有线程安全性）。(Remove guard condition without thread-safety.)
  /**
   * 不会抛出异常，但存储函数可能会抛出异常。(Does not throw, but storage function may throw.)
   *
   * @param[in] guard_condition 需要移除的保护条件。(The guard condition to be removed.)
   * @param[in] remove_guard_condition_function 用于移除保护条件的函数。(The function for removing
   * the guard condition.)
   */
  void sync_remove_guard_condition(
      std::shared_ptr<rclcpp::GuardCondition> &&guard_condition,
      std::function<void(std::shared_ptr<rclcpp::GuardCondition> &&)>
          remove_guard_condition_function) {
    // 明确无线程同步。(Explicitly no thread synchronization.)
    remove_guard_condition_function(std::move(guard_condition));
  }

  /// 添加定时器，不带线程安全性。 (Add timer without thread-safety)
  /**
   * 不抛出异常，但存储函数可能抛出异常。(Does not throw, but storage function may throw)
   *
   * @param[in] timer 指向 rclcpp::TimerBase 的共享指针 (A shared_ptr to rclcpp::TimerBase)
   * @param[in] add_timer_function 添加定时器的回调函数 (Callback function for adding a timer)
   */
  void sync_add_timer(
      std::shared_ptr<rclcpp::TimerBase> &&timer,
      std::function<void(std::shared_ptr<rclcpp::TimerBase> &&)> add_timer_function) {
    // 显式地不进行线程同步。 (Explicitly no thread synchronization)
    add_timer_function(std::move(timer));
  }

  /// 移除定时器，不带线程安全性。 (Remove timer without thread-safety)
  /**
   * 不抛出异常，但存储函数可能抛出异常。(Does not throw, but storage function may throw)
   *
   * @param[in] timer 指向 rclcpp::TimerBase 的共享指针 (A shared_ptr to rclcpp::TimerBase)
   * @param[in] remove_timer_function 移除定时器的回调函数 (Callback function for removing a timer)
   */
  void sync_remove_timer(
      std::shared_ptr<rclcpp::TimerBase> &&timer,
      std::function<void(std::shared_ptr<rclcpp::TimerBase> &&)> remove_timer_function) {
    // 显式地不进行线程同步。 (Explicitly no thread synchronization)
    remove_timer_function(std::move(timer));
  }

  /// 添加客户端，不带线程安全性。 (Add client without thread-safety)
  /**
   * 不抛出异常，但存储函数可能抛出异常。(Does not throw, but storage function may throw)
   *
   * @param[in] client 指向 rclcpp::ClientBase 的共享指针 (A shared_ptr to rclcpp::ClientBase)
   * @param[in] add_client_function 添加客户端的回调函数 (Callback function for adding a client)
   */
  void sync_add_client(
      std::shared_ptr<rclcpp::ClientBase> &&client,
      std::function<void(std::shared_ptr<rclcpp::ClientBase> &&)> add_client_function) {
    // 显式地不进行线程同步。 (Explicitly no thread synchronization)
    add_client_function(std::move(client));
  }

  /// 移除客户端，不提供线程安全。（Remove client without thread-safety.）
  /**
   * 不抛出异常，但存储函数可能会抛出异常。
   * (Does not throw, but storage function may throw.)
   *
   * @param[in] client 需要移除的客户端（Client to be removed）
   * @param[in] remove_client_function 用于移除客户端的函数（Function to remove the client）
   */
  void sync_remove_client(
      std::shared_ptr<rclcpp::ClientBase> &&client,
      std::function<void(std::shared_ptr<rclcpp::ClientBase> &&)> remove_client_function) {
    // 明确表示没有线程同步。（Explicitly no thread synchronization.）
    remove_client_function(std::move(client));
  }

  /// 添加服务，不提供线程安全。（Add service without thread-safety.）
  /**
   * 不抛出异常，但存储函数可能会抛出异常。
   * (Does not throw, but storage function may throw.)
   *
   * @param[in] service 需要添加的服务（Service to be added）
   * @param[in] add_service_function 用于添加服务的函数（Function to add the service）
   */
  void sync_add_service(
      std::shared_ptr<rclcpp::ServiceBase> &&service,
      std::function<void(std::shared_ptr<rclcpp::ServiceBase> &&)> add_service_function) {
    // 明确表示没有线程同步。（Explicitly no thread synchronization.）
    add_service_function(std::move(service));
  }

  /// 移除服务，不提供线程安全。（Remove service without thread-safety.）
  /**
   * 不抛出异常，但存储函数可能会抛出异常。
   * (Does not throw, but storage function may throw.)
   *
   * @param[in] service 需要移除的服务（Service to be removed）
   * @param[in] remove_service_function 用于移除服务的函数（Function to remove the service）
   */
  void sync_remove_service(
      std::shared_ptr<rclcpp::ServiceBase> &&service,
      std::function<void(std::shared_ptr<rclcpp::ServiceBase> &&)> remove_service_function) {
    // 明确表示没有线程同步。（Explicitly no thread synchronization.）
    remove_service_function(std::move(service));
  }

  /// 添加可等待对象，不提供线程安全。（Add waitable without thread-safety.）
  /**
   * 不抛出异常，但存储函数可能会抛出异常。
   * (Does not throw, but storage function may throw.)
   *
   * @param[in] waitable 需要添加的可等待对象（Waitable to be added）
   * @param[in] associated_entity 与可等待对象关联的实体（Associated entity with the waitable）
   * @param[in] add_waitable_function 用于添加可等待对象的函数（Function to add the waitable）
   */
  void sync_add_waitable(
      std::shared_ptr<rclcpp::Waitable> &&waitable,
      std::shared_ptr<void> &&associated_entity,
      std::function<void(std::shared_ptr<rclcpp::Waitable> &&, std::shared_ptr<void> &&)>
          add_waitable_function) {
    // 明确表示没有线程同步。（Explicitly no thread synchronization.）
    add_waitable_function(std::move(waitable), std::move(associated_entity));
  }

  /// 移除 waitable，不具有线程安全性。
  /// Remove waitable without thread-safety.
  /**
   * 不抛出异常，但存储函数可能抛出异常。
   * Does not throw, but storage function may throw.
   */
  void sync_remove_waitable(
      std::shared_ptr<rclcpp::Waitable> &&waitable,
      std::function<void(std::shared_ptr<rclcpp::Waitable> &&)> remove_waitable_function) {
    // 明确没有线程同步。
    // Explicitly no thread synchronization.
    remove_waitable_function(std::move(waitable));
  }

  /// 在不具备线程安全性的情况下修剪已删除的实体。
  /// Prune deleted entities without thread-safety.
  /**
   * 不抛出异常，但存储函数可能抛出异常。
   * Does not throw, but storage function may throw.
   */
  void sync_prune_deleted_entities(std::function<void()> prune_deleted_entities_function) {
    // 明确没有线程同步。
    // Explicitly no thread synchronization.
    prune_deleted_entities_function();
  }

  /// 实现等待，没有任何线程安全性。
  /// Implements wait without any thread-safety.
  template <class WaitResultT>
  WaitResultT sync_wait(
      std::chrono::nanoseconds time_to_wait_ns,
      std::function<void()> rebuild_rcl_wait_set,
      std::function<rcl_wait_set_t &()> get_rcl_wait_set,
      std::function<WaitResultT(WaitResultKind wait_result_kind)> create_wait_result) {
    // 假设：此函数假定已采取某种措施来确保等待集等待的所有实体都不允许超出范围，因此被删除。
    // Assumption: this function assumes that some measure has been taken to
    // ensure none of the entities being waited on by the wait set are allowed
    // to go out of scope and therefore be deleted.
    // 在StaticStorage策略的情况下，这是确保的，因为它在其自身生命周期内保留了所有实体的共享所有权。
    // In the case of the StaticStorage policy, this is ensured because it
    // retains shared ownership of all entites for the duration of its own life.
    // 在DynamicStorage策略的情况下，这是通过调用此函数的函数来确保的，通过获取实体的共享所有权以供此函数使用。
    // In the case of the DynamicStorage policy, this is ensured by the function
    // which calls this function, by acquiring shared ownership of the entites
    // for the duration of this function.

    // 设置循环判断条件。
    // Setup looping predicate.
    auto start = std::chrono::steady_clock::now();
    std::function<bool()> should_loop = this->create_loop_predicate(time_to_wait_ns, start);

    // 等待直到满足退出条件。
    // Wait until exit condition is met.
    do {
      // 重建等待集。
      // Rebuild the wait set.
      // 如果需要，这将调整等待集的大小，例如，在上一次等待之后添加或删除实体，但在静态存储等待集中，这不应该发生，因为它们在构造后无法更改。
      // This will resize the wait set if needed, due to e.g. adding or removing
      // entities since the last wait, but this should never occur in static
      // storage wait sets since they cannot be changed after construction.
      // 这还将清除等待集并重新添加所有实体，这为再次等待做好准备。
      // This will also clear the wait set and re-add all the entities, which
      // prepares it to be waited on again.
      rebuild_rcl_wait_set();

      rcl_wait_set_t &rcl_wait_set = get_rcl_wait_set();

      // 无条件等待，直到发生超时条件，因为我们假设没有需要等待停止和重置的条件，例如异步添加或删除实体，即明确提供不具备线程安全性。
      // Wait unconditionally until timeout condition occurs since we assume
      // there are no conditions that would require the wait to stop and reset,
      // like asynchronously adding or removing an entity, i.e. explicitly
      // providing no thread-safety.

      // 计算剩余等待时间，除非无限期阻塞。
      // Calculate how much time there is left to wait, unless blocking indefinitely.
      auto time_left_to_wait_ns = this->calculate_time_left_to_wait(time_to_wait_ns, start);

      // 然后等待实体变为就绪状态。
      // Then wait for entities to become ready.
      rcl_ret_t ret = rcl_wait(&rcl_wait_set, time_left_to_wait_ns.count());
      if (RCL_RET_OK == ret) {
        // 等待集中的某个东西已经准备好了，而且因为这个类没有添加任何东西，所以它是一个用户实体。
        // Something has become ready in the wait set, and since this class
        // did not add anything to it, it is a user entity that is ready.
        return create_wait_result(WaitResultKind::Ready);
      } else if (RCL_RET_TIMEOUT == ret) {
        // 等待集超时，退出循环。
        // The wait set timed out, exit the loop.
        break;
      } else if (RCL_RET_WAIT_SET_EMPTY == ret) {
        // 等待集为空，返回 Empty。
        // Wait set was empty, return Empty.
        return create_wait_result(WaitResultKind::Empty);
      } else {
        // 其他错误情况，抛出异常。
        // Some other error case, throw.
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    } while (should_loop());

    // 等待并未导致准备好的项目，返回超时。
    // Wait did not result in ready items, return timeout.
    return create_wait_result(WaitResultKind::Timeout);
  }

  /**
   * @brief 同步等待结果获取 (Synchronously wait for result acquisition)
   *
   * 这个函数用于同步等待结果获取，但实际上并没有任何操作。
   * (This function is used for synchronously waiting for result acquisition, but actually does
   * nothing.)
   */
  void sync_wait_result_acquire() {
    // 显式地什么都不做 (Explicitly do nothing)
  }

  /**
   * @brief 同步等待结果释放 (Synchronously wait for result release)
   *
   * 这个函数用于同步等待结果释放，但实际上并没有任何操作。
   * (This function is used for synchronously waiting for result release, but actually does
   * nothing.)
   */
  void sync_wait_result_release() {
    // 显式地什么都不做 (Explicitly do nothing)
  }
};

}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__SEQUENTIAL_SYNCHRONIZATION_HPP_

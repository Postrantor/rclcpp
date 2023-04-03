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

#ifndef RCLCPP__WAIT_SET_TEMPLATE_HPP_
#define RCLCPP__WAIT_SET_TEMPLATE_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "rcl/wait.h"
#include "rclcpp/client.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_wait_set_mask.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_result.hpp"
#include "rclcpp/waitable.hpp"
#include "rcpputils/scope_exit.hpp"

namespace rclcpp {

/// 封装了一组可等待的项目，可以作为一个组进行等待。
/// Encapsulates sets of waitable items which can be waited on as a group.
/**
 * 本类使用 rcl_wait_set_t 作为存储，但它还有助于管理相关的 rclcpp 类型的所有权。
 * This class uses the rcl_wait_set_t as storage, but it also helps manage the
 * ownership of associated rclcpp types.
 */
template <class SynchronizationPolicy, class StoragePolicy>
class WaitSetTemplate final : private SynchronizationPolicy, private StoragePolicy {
public:
  // 不可复制的智能指针定义
  // Smart pointer definitions not copyable
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(WaitSetTemplate)

  // 定义订阅条目类型
  // Define SubscriptionEntry type
  using typename StoragePolicy::SubscriptionEntry;

  // 定义可等待条目类型
  // Define WaitableEntry type
  using typename StoragePolicy::WaitableEntry;

  /// 使用可选的初始 waitable 实体和可选的自定义上下文构造一个 wait set。
  /// Construct a wait set with optional initial waitable entities and optional custom context.
  /**
   * 对于
   * waitables，它们还有一个“关联”实体，你可以在此类中的这些类型的添加和删除函数中了解更多关于它的信息。
   * For the waitables, they have additionally an "associated" entity, which
   * you can read more about in the add and remove functions for those types
   * in this class.
   *
   * \param[in] subscriptions 要添加的订阅向量。Vector of subscriptions to be added.
   * \param[in] guard_conditions 要添加的保护条件向量。Vector of guard conditions to be added.
   * \param[in] timers 要添加的定时器向量。Vector of timers to be added.
   * \param[in] clients 要添加的客户端及其关联实体向量。Vector of clients and their associated
   * entity to be added. \param[in] services 要添加的服务及其关联实体向量。Vector of services and
   * their associated entity to be added. \param[in] waitables 要添加的 waitables
   * 及其关联实体向量。Vector of waitables and their associated entity to be added. \param[in]
   * context 要使用的自定义上下文，默认为全局默认值。Custom context to be used, defaults to global
   * default. \throws std::invalid_argument 如果上下文是 nullptr。If context is nullptr.
   */
  explicit WaitSetTemplate(
      const typename StoragePolicy::SubscriptionsIterable& subscriptions = {},
      const typename StoragePolicy::GuardConditionsIterable& guard_conditions = {},
      const typename StoragePolicy::TimersIterable& timers = {},
      const typename StoragePolicy::ClientsIterable& clients = {},
      const typename StoragePolicy::ServicesIterable& services = {},
      const typename StoragePolicy::WaitablesIterable& waitables = {},
      rclcpp::Context::SharedPtr context = rclcpp::contexts::get_global_default_context())
      : SynchronizationPolicy(context),
        StoragePolicy(
            subscriptions,
            guard_conditions,
            // 此方法来自 SynchronizationPolicy。This method comes from the SynchronizationPolicy.
            this->get_extra_guard_conditions(),
            timers,
            clients,
            services,
            waitables,
            context) {}

  /// 返回内部的 rcl wait set 对象。
  /// Return the internal rcl wait set object.
  /**
   * 访问此结构时，此方法不提供线程安全性。
   * This method provides no thread-safety when accessing this structure.
   * 此结构的状态可以随时通过诸如 wait()、add_*()、remove_*() 等方法进行更新。
   * The state of this structure can be updated at anytime by methods like
   * wait(), add_*(), remove_*(), etc.
   */
  const rcl_wait_set_t& get_rcl_wait_set() const {
    // 此方法来自 StoragePolicy。This method comes from the StoragePolicy.
    return this->storage_get_rcl_wait_set();
  }

  /// 添加订阅到此等待集。
  /// Add a subscription to this wait set.
  /**
   * \sa add_guard_condition() for details of how this method works.
   *
   * 此外，除了add_guard_condition的文档，此方法还有一个mask参数，
   * 允许您控制将订阅的哪些部分添加到此调用的等待集中。
   * 例如，您可能希望将实际订阅包含在此等待集中，但将内部进程可等待项添加到另一个等待集中。
   * 如果禁用了内部进程，则不会发生错误，只会跳过。
   * Additionally to the documentation for add_guard_condition, this method
   * has a mask parameter which allows you to control which parts of the
   * subscription is added to the wait set with this call.
   * For example, you might want to include the actual subscription to this
   * wait set, but add the intra-process waitable to another wait set.
   * If intra-process is disabled, no error will occur, it will just be skipped.
   *
   * 当在等待后进行自省时，此订阅的共享指针将是Waitable（内部进程或QoS事件）的“关联实体”
   * 指针，以便更容易地弄清楚哪个订阅哪个可等待项随后与之对应。
   * When introspecting after waiting, this subscription's shared pointer will
   * be the Waitable's (intra-process or the QoS Events) "associated entity"
   * pointer, for more easily figuring out which subscription which waitable
   * goes with afterwards.
   *
   * \param[in] subscription 要添加的订阅。
   * \param[in] mask 一个控制要添加哪些订阅部分的类。
   * \throws std::invalid_argument 如果订阅是nullptr。
   * \throws std::runtime_error 如果订阅已经被添加或与其他等待集关联。
   * \throws 基于使用的策略的异常。
   * \param[in] subscription Subscription to be added.
   * \param[in] mask A class which controls which parts of the subscription to add.
   * \throws std::invalid_argument if subscription is nullptr.
   * \throws std::runtime_error if subscription has already been added or is
   *   associated with another wait set.
   * \throws exceptions based on the policies used.
   */
  void add_subscription(
      std::shared_ptr<rclcpp::SubscriptionBase> subscription,
      rclcpp::SubscriptionWaitSetMask mask = {}) {
    // 如果订阅是nullptr，抛出无效参数异常
    // If the subscription is nullptr, throw an invalid argument exception
    if (nullptr == subscription) {
      throw std::invalid_argument("subscription is nullptr");
    }
    // 此方法来自SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    this->sync_add_subscription(
        std::move(subscription), mask,
        [this](
            std::shared_ptr<rclcpp::SubscriptionBase>&& inner_subscription,
            const rclcpp::SubscriptionWaitSetMask& mask) {
          // 这些方法来自StoragePolicy，对于固定大小的存储策略可能不存在。
          // 如果订阅已经被添加，它将抛出异常。
          // These methods come from the StoragePolicy, and may not exist for
          // fixed sized storage policies.
          // It will throw if the subscription has already been added.
          // 如果包含订阅者 (If including subscriptions)
          if (mask.include_subscription) {
            // 获取本地订阅者引用 (Get local subscription reference)
            auto local_subscription = inner_subscription;
            // 检查订阅者是否已经被 wait set 使用 (Check if the subscription is already in use by a
            // wait set)
            bool already_in_use = local_subscription->exchange_in_use_by_wait_set_state(
                local_subscription.get(), true);
            // 如果订阅者已经被 wait set 使用，抛出异常 (If the subscription is already in use,
            // throw an exception)
            if (already_in_use) {
              throw std::runtime_error("subscription already associated with a wait set");
            }
            // 将订阅者添加到存储器中 (Add subscription to storage)
            this->storage_add_subscription(std::move(local_subscription));
          }

          // 如果包含事件处理器 (If including event handlers)
          if (mask.include_events) {
            // 遍历订阅者的事件处理器 (Iterate through the subscription's event handlers)
            for (auto key_event_pair : inner_subscription->get_event_handlers()) {
              // 获取事件处理器 (Get the event handler)
              auto event = key_event_pair.second;
              // 获取本地订阅者引用 (Get local subscription reference)
              auto local_subscription = inner_subscription;
              // 检查事件处理器是否已经被 wait set 使用 (Check if the event handler is already in
              // use by a wait set)
              bool already_in_use =
                  local_subscription->exchange_in_use_by_wait_set_state(event.get(), true);
              // 如果事件处理器已经被 wait set 使用，抛出异常 (If the event handler is already in
              // use, throw an exception)
              if (already_in_use) {
                throw std::runtime_error("subscription event already associated with a wait set");
              }
              // 将事件处理器和订阅者添加到存储器中 (Add event handler and subscription to storage)
              this->storage_add_waitable(std::move(event), std::move(local_subscription));
            }
          }

          // 如果包含内部进程等待对象 (If including intra-process waitables)
          if (mask.include_intra_process_waitable) {
            // 获取本地订阅者引用 (Get local subscription reference)
            auto local_subscription = inner_subscription;
            // 获取内部进程等待对象 (Get the intra-process waitable)
            auto waitable = inner_subscription->get_intra_process_waitable();
            // 如果内部进程等待对象不为空 (If the intra-process waitable is not null)
            if (nullptr != waitable) {
              // 检查内部进程等待对象是否已经被 wait set 使用 (Check if the intra-process waitable
              // is already in use by a wait set)
              bool already_in_use =
                  local_subscription->exchange_in_use_by_wait_set_state(waitable.get(), true);
              // 如果内部进程等待对象已经被 wait set 使用，抛出异常 (If the intra-process waitable
              // is already in use, throw an exception)
              if (already_in_use) {
                throw std::runtime_error(
                    "subscription intra-process waitable already associated with a wait set");
              }
              // 将内部进程等待对象和订阅者添加到存储器中 (Add intra-process waitable and
              // subscription to storage)
              this->storage_add_waitable(
                  std::move(inner_subscription->get_intra_process_waitable()),
                  std::move(local_subscription));
            }
          }
        });
  }

  /// 从此等待集中移除订阅。
  /// Remove a subscription from this wait set.
  /**
   * \sa remove_guard_condition() for details of how this method works.
   *
   * 此外，除了add_guard_condition的文档，此方法还有一个mask参数，
   * 允许您控制将订阅的哪些部分从此调用的等待集中移除。
   * 您可以按照与添加时不同的顺序从等待集中选择性地删除项目。
   * Additionally to the documentation for add_guard_condition, this method
   * has a mask parameter which allows you to control which parts of the
   * subscription is added to the wait set with this call.
   * You may remove items selectively from the wait set in a different order
   * than they were added.
   *
   * \param[in] subscription 要移除的订阅。
   * \param[in] mask 一个控制要移除哪些订阅部分的类。
   * \throws std::invalid_argument 如果订阅是nullptr。
   * \throws std::runtime_error 如果订阅不是等待集的一部分。
   * \throws 基于使用的策略的异常。
   * \param[in] subscription Subscription to be removed.
   * \param[in] mask A class which controls which parts of the subscription to remove.
   * \throws std::invalid_argument if subscription is nullptr.
   * \throws std::runtime_error if subscription is not part of the wait set.
   * \throws exceptions based on the policies used.
   */
  void remove_subscription(
      std::shared_ptr<rclcpp::SubscriptionBase> subscription,
      rclcpp::SubscriptionWaitSetMask mask = {}) {
    // 如果订阅是nullptr，抛出无效参数异常
    // If the subscription is nullptr, throw an invalid argument exception
    if (nullptr == subscription) {
      throw std::invalid_argument("subscription is nullptr");
    }
    // 此方法来自SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    this->sync_remove_subscription(
        std::move(subscription), mask,
        [this](
            std::shared_ptr<rclcpp::SubscriptionBase>&& inner_subscription,
            const rclcpp::SubscriptionWaitSetMask& mask) {
          // 此方法来自StoragePolicy，对于固定大小的存储策略可能不存在。
          // 如果订阅不在等待集中，它将抛出异常。
          // This method comes from the StoragePolicy, and it may not exist for
          // fixed sized storage policies.
          // It will throw if the subscription is not in the wait set.
          if (mask.include_subscription) {
            // 创建一个局部订阅变量，用于操作（Create a local subscription variable for
            // manipulation）
            auto local_subscription = inner_subscription;

            // 更改订阅的等待集状态（Change the wait set state of the subscription）
            local_subscription->exchange_in_use_by_wait_set_state(local_subscription.get(), false);

            // 从存储中移除订阅（Remove the subscription from storage）
            this->storage_remove_subscription(std::move(local_subscription));
          }
          if (mask.include_events) {
            // 遍历订阅的事件处理器（Iterate through the event handlers of the subscription）
            for (auto key_event_pair : inner_subscription->get_event_handlers()) {
              // 获取当前事件（Get the current event）
              auto event = key_event_pair.second;

              // 创建一个局部订阅变量，用于操作（Create a local subscription variable for
              // manipulation）
              auto local_subscription = inner_subscription;

              // 更改事件的等待集状态（Change the wait set state of the event）
              local_subscription->exchange_in_use_by_wait_set_state(event.get(), false);

              // 从存储中移除等待事件（Remove the waitable event from storage）
              this->storage_remove_waitable(std::move(event));
            }
          }
          if (mask.include_intra_process_waitable) {
            // 获取内部处理可等待对象（Get the intra process waitable object）
            auto local_waitable = inner_subscription->get_intra_process_waitable();

            // 如果内部处理可等待对象不为空（If the intra process waitable object is not null）
            if (nullptr != local_waitable) {
              // 当订阅启用内部处理时，这是情况。
              // This is the case when intra process is enabled for the subscription.

              // 更改内部处理可等待对象的等待集状态（Change the wait set state of the intra process
              // waitable object）
              inner_subscription->exchange_in_use_by_wait_set_state(local_waitable.get(), false);

              // 从存储中移除内部处理可等待对象（Remove the intra process waitable object from
              // storage）
              this->storage_remove_waitable(std::move(local_waitable));
            }
          }
        });
  }

  /// 向此等待集添加一个守卫条件。
  /// Add a guard condition to this wait set.
  /**
   * 守卫条件被添加到等待集中，并在等待期间持有共享所有权。
   * Guard condition is added to the wait set, and shared ownership is held while waiting.
   * 然而，如果在调用 wait() 之间，守卫条件的引用计数变为零，则它将在下一次调用 wait()
   * 时被隐式移除。 However, if between calls to wait() the guard condition's reference count goes
   * to zero, it will be implicitly removed on the next call to wait().
   *
   * 除了固定大小存储的情况，在该情况下，构造后等待集的更改无法发生，此时它一直保持共享所有权，直到等待集被销毁，但是这个方法在固定大小的等待集上也不存在。
   * Except in the case of a fixed sized storage, where changes to the wait set cannot occur after
   * construction, in which case it holds shared ownership at all times until the wait set is
   * destroy, but this method also does not exist on a fixed sized wait set.
   *
   * 此函数是否线程安全取决于与此类一起使用的 SynchronizationPolicy。
   * This function may be thread-safe depending on the SynchronizationPolicy used with this class.
   * 使用 ThreadSafeWaitSetPolicy 将确保在此函数添加守卫条件之前中断 wait() 并返回。
   * Using the ThreadSafeWaitSetPolicy will ensure that wait() is interrupted and returns before
   * this function adds the guard condition. 否则，不建议同时调用此函数和 wait()。 Otherwise, it is
   * not safe to call this function concurrently with wait().
   *
   * 如果 StoragePolicy 不允许在初始化后编辑等待集，则此功能将不被启用（不可使用）。
   * This function will not be enabled (will not be available) if the StoragePolicy does not allow
   * editing of the wait set after initialization.
   *
   * \param[in] guard_condition 要添加的守卫条件。
   * \param[in] guard_condition Guard condition to be added.
   * \throws std::invalid_argument 如果 guard_condition 为 nullptr。
   * \throws std::invalid_argument if guard_condition is nullptr.
   * \throws std::runtime_error 如果 guard_condition 已经被添加或与其他等待集关联。
   * \throws std::runtime_error if guard_condition has already been added or is associated with
   * another wait set. \throws 基于所使用策略的异常。 \throws exceptions based on the policies used.
   */
  void add_guard_condition(std::shared_ptr<rclcpp::GuardCondition> guard_condition) {
    // 如果 guard_condition 为空指针，则抛出无效参数异常
    // If guard_condition is nullptr, throw an invalid_argument exception
    if (nullptr == guard_condition) {
      throw std::invalid_argument("guard_condition is nullptr");
    }
    // 此方法来自 SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    this->sync_add_guard_condition(
        std::move(guard_condition),
        [this](std::shared_ptr<rclcpp::GuardCondition>&& inner_guard_condition) {
          bool already_in_use = inner_guard_condition->exchange_in_use_by_wait_set_state(true);
          // 如果守卫条件已经被另一个等待集使用，则抛出运行时异常
          // If the guard condition is already in use by another wait set, throw a runtime_error
          // exception
          if (already_in_use) {
            throw std::runtime_error("guard condition already in use by another wait set");
          }
          // 此方法来自 StoragePolicy，对于固定大小的存储策略可能不存在。
          // This method comes from the StoragePolicy, and it may not exist for fixed sized storage
          // policies. 如果守卫条件已经被添加，则抛出异常。 It will throw if the guard condition has
          // already been added.
          this->storage_add_guard_condition(std::move(inner_guard_condition));
        });
  }

  /// 从此等待集中移除一个守卫条件。
  /// Remove a guard condition from this wait set.
  /**
   * 守卫条件从等待集中移除，如果需要则释放共享所有权。
   * Guard condition is removed from the wait set, and if needed the shared ownership is released.
   *
   * 此函数是否线程安全取决于与此类一起使用的 SynchronizationPolicy。
   * This function may be thread-safe depending on the SynchronizationPolicy used with this class.
   * 使用 ThreadSafeWaitSetPolicy 将确保在此函数移除守卫条件之前中断 wait() 并返回。
   * Using the ThreadSafeWaitSetPolicy will ensure that wait() is interrupted and returns before
   * this function removes the guard condition. 否则，不建议同时调用此函数和 wait()。 Otherwise, it
   * is not safe to call this function concurrently with wait().
   *
   * 如果 StoragePolicy 不允许在初始化后编辑等待集，则此功能将不被启用（不可使用）。
   * This function will not be enabled (will not be available) if the StoragePolicy does not allow
   * editing of the wait set after initialization.
   *
   * \param[in] guard_condition 要移除的守卫条件。
   * \param[in] guard_condition Guard condition to be removed.
   * \throws std::invalid_argument 如果 guard_condition 为 nullptr。
   * \throws std::invalid_argument if guard_condition is nullptr.
   * \throws std::runtime_error 如果 guard_condition 不是等待集的一部分。
   * \throws std::runtime_error if guard_condition is not part of the wait set.
   * \throws 基于所使用策略的异常。
   * \throws exceptions based on the policies used.
   */
  void remove_guard_condition(std::shared_ptr<rclcpp::GuardCondition> guard_condition) {
    // 如果 guard_condition 为空指针，则抛出无效参数异常
    // If guard_condition is nullptr, throw an invalid_argument exception
    if (nullptr == guard_condition) {
      throw std::invalid_argument("guard_condition is nullptr");
    }
    // 此方法来自 SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    this->sync_remove_guard_condition(
        std::move(guard_condition),
        [this](std::shared_ptr<rclcpp::GuardCondition>&& inner_guard_condition) {
          inner_guard_condition->exchange_in_use_by_wait_set_state(false);
          // 此方法来自 StoragePolicy，对于固定大小的存储策略可能不存在。
          // This method comes from the StoragePolicy, and it may not exist for fixed sized storage
          // policies. 如果守卫条件不在等待集中，则抛出异常。 It will throw if the guard condition
          // is not in the wait set.
          this->storage_remove_guard_condition(std::move(inner_guard_condition));
        });
  }

  /// 向此等待集添加计时器。
  /// Add a timer to this wait set.
  /**
   * \sa add_guard_condition() 了解此方法的工作方式。
   * \sa add_guard_condition() for details of how this method works.
   *
   * \param[in] timer 要添加的计时器。
   * \param[in] timer Timer to be added.
   * \throws std::invalid_argument 如果计时器为nullptr。
   * \throws std::invalid_argument if timer is nullptr.
   * \throws std::runtime_error 如果计时器已经被添加或与其他等待集关联。
   * \throws std::runtime_error if timer has already been added or is
   *   associated with another wait set.
   * \throws 基于所使用策略的异常。
   * \throws exceptions based on the policies used.
   */
  void add_timer(std::shared_ptr<rclcpp::TimerBase> timer) {
    // 如果计时器为空，则抛出异常。
    // If the timer is null, throw an exception.
    if (nullptr == timer) {
      throw std::invalid_argument("timer is nullptr");
    }
    // 此方法来自 SynchronizationPolicy。
    // This method comes from the SynchronizationPolicy.
    this->sync_add_timer(
        std::move(timer), [this](std::shared_ptr<rclcpp::TimerBase>&& inner_timer) {
          // 检查计时器是否已经被其他等待集使用。
          // Check if the timer is already in use by another wait set.
          bool already_in_use = inner_timer->exchange_in_use_by_wait_set_state(true);
          // 如果计时器已经被其他等待集使用，则抛出异常。
          // If the timer is already in use by another wait set, throw an exception.
          if (already_in_use) {
            throw std::runtime_error("timer already in use by another wait set");
          }
          // 此方法来自 StoragePolicy，对于固定大小的存储策略可能不存在。
          // This method comes from the StoragePolicy, and it may not exist for
          // fixed sized storage policies.
          // 如果计时器已经被添加，则抛出异常。
          // It will throw if the timer has already been added.
          this->storage_add_timer(std::move(inner_timer));
        });
  }

  /// 从此等待集中移除计时器。
  /// Remove a timer from this wait set.
  /**
   * \sa remove_guard_condition() 了解此方法的工作方式。
   * \sa remove_guard_condition() for details of how this method works.
   *
   * \param[in] timer 要移除的计时器。
   * \param[in] timer Timer to be removed.
   * \throws std::invalid_argument 如果计时器为nullptr。
   * \throws std::invalid_argument if timer is nullptr.
   * \throws std::runtime_error 如果计时器不是等待集的一部分。
   * \throws std::runtime_error if timer is not part of the wait set.
   * \throws 基于所使用策略的异常。
   * \throws exceptions based on the policies used.
   */
  void remove_timer(std::shared_ptr<rclcpp::TimerBase> timer) {
    // 如果计时器为空，则抛出异常。
    // If the timer is null, throw an exception.
    if (nullptr == timer) {
      throw std::invalid_argument("timer is nullptr");
    }
    // 此方法来自 SynchronizationPolicy。
    // This method comes from the SynchronizationPolicy.
    this->sync_remove_timer(
        std::move(timer), [this](std::shared_ptr<rclcpp::TimerBase>&& inner_timer) {
          // 将计时器的等待集使用状态设置为false。
          // Set the timer's wait set usage state to false.
          inner_timer->exchange_in_use_by_wait_set_state(false);
          // 此方法来自 StoragePolicy，对于固定大小的存储策略可能不存在。
          // This method comes from the StoragePolicy, and it may not exist for
          // fixed sized storage policies.
          // 如果计时器不在等待集中，则抛出异常。
          // It will throw if the timer is not in the wait set.
          this->storage_remove_timer(std::move(inner_timer));
        });
  }

  /// 向此等待集添加客户端。
  /// Add a client to this wait set.
  /**
   * \sa add_guard_condition() 了解此方法的工作方式。
   * \sa add_guard_condition() for details of how this method works.
   *
   * \param[in] client 要添加的客户端。
   * \param[in] client Client to be added.
   * \throws std::invalid_argument 如果客户端为nullptr。
   * \throws std::invalid_argument if client is nullptr.
   * \throws std::runtime_error 如果客户端已经被添加或与其他等待集关联。
   * \throws std::runtime_error if client has already been added or is
   *   associated with another wait set.
   * \throws 基于所使用策略的异常。
   * \throws exceptions based on the policies used.
   */
  void add_client(std::shared_ptr<rclcpp::ClientBase> client) {
    // 如果客户端为空，则抛出异常。
    // If the client is null, throw an exception.
    if (nullptr == client) {
      throw std::invalid_argument("client is nullptr");
    }
    // 此方法来自 SynchronizationPolicy。
    // This method comes from the SynchronizationPolicy.
    this->sync_add_client(
        std::move(client), [this](std::shared_ptr<rclcpp::ClientBase>&& inner_client) {
          // 检查客户端是否已经被其他等待集使用。
          // Check if the client is already in use by another wait set.
          bool already_in_use = inner_client->exchange_in_use_by_wait_set_state(true);
          // 如果客户端已经被其他等待集使用，则抛出异常。
          // If the client is already in use by another wait set, throw an exception.
          if (already_in_use) {
            throw std::runtime_error("client already in use by another wait set");
          }
          // 此方法来自 StoragePolicy，对于固定大小的存储策略可能不存在。
          // This method comes from the StoragePolicy, and it may not exist for
          // fixed sized storage policies.
          // 如果客户端已经被添加，则抛出异常。
          // It will throw if the client has already been added.
          this->storage_add_client(std::move(inner_client));
        });
  }

  /// 从此等待集中删除一个客户端。
  /// Remove a client from this wait set.
  /**
   * \sa remove_guard_condition() for details of how this method works.
   *
   * \param[in] client 要删除的客户端。Client to be removed.
   * \throws std::invalid_argument 如果客户端为nullptr。if client is nullptr.
   * \throws std::runtime_error 如果客户端不是等待集的一部分。if client is not part of the wait set.
   * \throws 根据所使用的策略抛出异常。exceptions based on the policies used.
   */
  void remove_client(std::shared_ptr<rclcpp::ClientBase> client) {
    // 如果客户端为空，则抛出无效参数异常
    // If the client is null, throw an invalid argument exception
    if (nullptr == client) {
      throw std::invalid_argument("client is nullptr");
    }
    // 此方法来自 SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    this->sync_remove_client(
        std::move(client), [this](std::shared_ptr<rclcpp::ClientBase>&& inner_client) {
          // 改变内部客户端在等待集中的使用状态
          // Change the in-use state of the inner client in the wait set
          inner_client->exchange_in_use_by_wait_set_state(false);
          // 此方法来自 StoragePolicy，对于固定大小的存储策略可能不存在
          // This method comes from the StoragePolicy, and it may not exist for
          // fixed sized storage policies.
          // 如果客户端不在等待集中，则抛出异常
          // It will throw if the client is not in the wait set.
          this->storage_remove_client(std::move(inner_client));
        });
  }

  /// 将服务添加到此等待集。
  /// Add a service to this wait set.
  /**
   * \sa add_guard_condition() for details of how this method works.
   *
   * \param[in] service 要添加的服务。Service to be added.
   * \throws std::invalid_argument 如果服务为nullptr。if service is nullptr.
   * \throws std::runtime_error 如果服务已经被添加或与其他等待集关联。if service has already been
   * added or is associated with another wait set. \throws 根据所使用的策略抛出异常。exceptions
   * based on the policies used.
   */
  void add_service(std::shared_ptr<rclcpp::ServiceBase> service) {
    // 如果服务为空，则抛出无效参数异常
    // If the service is null, throw an invalid argument exception
    if (nullptr == service) {
      throw std::invalid_argument("service is nullptr");
    }
    // 此方法来自 SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    this->sync_add_service(
        std::move(service), [this](std::shared_ptr<rclcpp::ServiceBase>&& inner_service) {
          // 改变内部服务在等待集中的使用状态，如果已经在使用则抛出运行时异常
          // Change the in-use state of the inner service in the wait set, and throw a runtime
          // exception if it's already in use
          bool already_in_use = inner_service->exchange_in_use_by_wait_set_state(true);
          if (already_in_use) {
            throw std::runtime_error("service already in use by another wait set");
          }
          // 此方法来自 StoragePolicy，对于固定大小的存储策略可能不存在
          // This method comes from the StoragePolicy, and it may not exist for
          // fixed sized storage policies.
          // 如果服务已经被添加，则抛出异常
          // It will throw if the service has already been added.
          this->storage_add_service(std::move(inner_service));
        });
  }

  /// 从此等待集中删除一个服务。
  /// Remove a service from this wait set.
  /**
   * \sa remove_guard_condition() for details of how this method works.
   *
   * \param[in] service 要删除的服务。Service to be removed.
   * \throws std::invalid_argument 如果服务为nullptr。if service is nullptr.
   * \throws std::runtime_error 如果服务不是等待集的一部分。if service is not part of the wait set.
   * \throws 根据所使用的策略抛出异常。exceptions based on the policies used.
   */
  void remove_service(std::shared_ptr<rclcpp::ServiceBase> service) {
    // 如果服务为空，则抛出无效参数异常
    // If the service is null, throw an invalid argument exception
    if (nullptr == service) {
      throw std::invalid_argument("service is nullptr");
    }
    // 此方法来自 SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    this->sync_remove_service(
        std::move(service), [this](std::shared_ptr<rclcpp::ServiceBase>&& inner_service) {
          // 改变内部服务在等待集中的使用状态
          // Change the in-use state of the inner service in the wait set
          inner_service->exchange_in_use_by_wait_set_state(false);
          // 此方法来自 StoragePolicy，对于固定大小的存储策略可能不存在
          // This method comes from the StoragePolicy, and it may not exist for
          // fixed sized storage policies.
          // 如果服务不在等待集中，则抛出异常
          // It will throw if the service is not in the wait set.
          this->storage_remove_service(std::move(inner_service));
        });
  }

  /// 将可等待项添加到此等待集。
  /// Add a waitable to this wait set.
  /**
   * \sa add_guard_condition() for details of how this method works.
   *
   * Additionally, this function has an optional parameter which can be used to
   * more quickly associate this waitable with an entity when it is ready, and
   * so that the ownership maybe held in order to keep the waitable's parent in
   * scope while waiting.
   * If it is set to nullptr it will be ignored.
   * The destruction of the associated entity's shared pointer will not cause
   * the waitable to be removed, but it will cause the associated entity pointer
   * to be nullptr when introspecting this waitable after waiting.
   *
   * 注意 rclcpp::QOSEventHandlerBase 只是 rclcpp::Waitable 的一个特例，可以使用此功能添加。
   * Note that rclcpp::QOSEventHandlerBase are just a special case of
   * rclcpp::Waitable and can be added with this function.
   *
   * \param[in] waitable 要添加的可等待项。Waitable to be added.
   * \param[in] associated_entity 与可等待项关联的类型擦除共享指针。Type erased shared pointer
   * associated with the waitable. This may be nullptr. \throws std::invalid_argument
   * 如果可等待项为nullptr。if waitable is nullptr. \throws std::runtime_error
   * 如果可等待项已经被添加或与其他等待集关联。if waitable has already been added or is associated
   * with another wait set. \throws 根据所使用的策略抛出异常。exceptions based on the policies used.
   */
  void add_waitable(
      std::shared_ptr<rclcpp::Waitable> waitable,
      std::shared_ptr<void> associated_entity = nullptr) {
    // 如果可等待项为空，则抛出无效参数异常
    // If the waitable is null, throw an invalid argument exception
    if (nullptr == waitable) {
      throw std::invalid_argument("waitable is nullptr");
    }
    // 此方法来自 SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    this->sync_add_waitable(
        std::move(waitable), std::move(associated_entity),
        [this](
            std::shared_ptr<rclcpp::Waitable>&& inner_waitable,
            std::shared_ptr<void>&& associated_entity) {
          // 改变内部可等待项在等待集中的使用状态，如果已经在使用则抛出运行时异常
          // Change the in-use state of the inner waitable in the wait set, and throw a runtime
          // exception if it's already in use
          bool already_in_use = inner_waitable->exchange_in_use_by_wait_set_state(true);
          if (already_in_use) {
            throw std::runtime_error("waitable already in use by another wait set");
          }
          // 此方法来自 StoragePolicy，对于固定大小的存储策略可能不存在
          // This method comes from the StoragePolicy, and it may not exist for
          // fixed sized storage policies.
          // 如果可等待项已经被添加，则抛出异常
          // It will throw if the waitable has already been added.
          this->storage_add_waitable(std::move(inner_waitable), std::move(associated_entity));
        });
  }

  /// 从此等待集中删除一个 waitable。
  /// Remove a waitable from this wait set.
  /**
   * \sa remove_guard_condition() for details of how this method works.
   *
   * \param[in] waitable 要删除的 Waitable。
   * \param[in] waitable Waitable to be removed.
   * \throws std::invalid_argument 如果 waitable 是 nullptr。
   * \throws std::invalid_argument if waitable is nullptr.
   * \throws std::runtime_error 如果 waitable 不是等待集的一部分。
   * \throws std::runtime_error if waitable is not part of the wait set.
   * \throws 基于使用的策略的异常。
   * \throws exceptions based on the policies used.
   */
  void remove_waitable(std::shared_ptr<rclcpp::Waitable> waitable) {
    // 如果 waitable 是 nullptr，则抛出异常
    // If waitable is nullptr, throw an exception
    if (nullptr == waitable) {
      throw std::invalid_argument("waitable is nullptr");
    }
    // 此方法来自 SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    this->sync_remove_waitable(
        std::move(waitable), [this](std::shared_ptr<rclcpp::Waitable>&& inner_waitable) {
          // 改变内部 waitable 的状态
          // Change the state of the inner waitable
          inner_waitable->exchange_in_use_by_wait_set_state(false);
          // 此方法来自 StoragePolicy，对于固定大小的存储策略可能不存在
          // This method comes from the StoragePolicy, and it may not exist for
          // fixed sized storage policies.
          // 如果 waitable 不在等待集中，它将抛出异常
          // It will throw if the waitable is not in the wait set.
          this->storage_remove_waitable(std::move(inner_waitable));
        });
  }

  /// 从等待集中删除任何已销毁的实体。
  /// Remove any destroyed entities from the wait set.
  /**
   * 当存储策略在等待集的生命周期内不保持共享所有权时，例如 DynamicStorage 策略，
   * 实体可能会在此等待集注意到之前超出范围并被删除。因此，需要定期清除此等待集中的弱引用。
   * When the storage policy does not maintain shared ownership for the life
   * of the wait set, e.g. the DynamicStorage policy, it is possible for an
   * entity to go out of scope and be deleted without this wait set noticing.
   * Therefore there are weak references in this wait set which need to be
   * periodically cleared.
   * 此函数执行该清理操作。
   * This function performs that clean up.
   *
   * 由于这涉及从等待集中删除实体，并且只有在等待集不保持添加的实体的所有权时才需要，
   * 因此静态存储策略不需要此功能，因此不提供此功能。
   * Since this involves removing entities from the wait set, and is only
   * needed if the wait set does not keep ownership of the added entities, the
   * storage policies which are static will not need this function and therefore
   * do not provide this function.
   *
   * \throws 基于使用的策略的异常。
   * \throws exceptions based on the policies used.
   */
  void prune_deleted_entities() {
    // 此方法来自 SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    this->sync_prune_deleted_entities([this]() {
      // 此方法来自 StoragePolicy，对于固定大小的存储策略可能不存在
      // This method comes from the StoragePolicy, and it may not exist for
      // fixed sized storage policies.
      this->storage_prune_deleted_entities();
    });
  }

  /// 等待等待集中的任何实体准备好，或者经过一段时间。
  /// Wait for any of the entities in the wait set to be ready, or a period of time to pass.
  /**
   * 当此等待集内的一个实体已准备好，或者经过一段时间时（以先到者为准），此函数将返回。
   * 对于不同的实体，“准备好”意味着不同的事情，但通常来说，这意味着此函数等待的某种异步条件得到满足。
   * This function will return when either one of the entities within this wait
   * set is ready, or a period of time has passed, which ever is first.
   * The term "ready" means different things for different entities, but
   * generally it means some condition is met asynchronously for which this
   * function waits.
   *
   * 此函数可以等待一段时间、不等待（非阻塞）或无限期等待，所有这些都取决于 time_to_wait 参数的值。
   * 等待始终是基于 std::chrono::steady_clock 进行测量的。如果无限期等待，Timeout 结果不可能。
   * 此类上没有“取消等待”的功能，但是如果您想无限期等待但有一种异步中断此方法的方法，
   * 则可以使用专用的 rclcpp::GuardCondition 来实现该目的。
   * This function can either wait for a period of time, do no waiting
   * (non-blocking), or wait indefinitely, all based on the value of the
   * time_to_wait parameter.
   * Waiting is always measured against the std::chrono::steady_clock.
   * If waiting indefinitely, the Timeout result is not possible.
   * There is no "cancel wait" function on this class, but if you want to wait
   * indefinitely but have a way to asynchronously interrupt this method, then
   * you can use a dedicated rclcpp::GuardCondition for that purpose.
   *
   * 此函数将修改内部 rcl_wait_set_t，因此在调用 wait 期间内省 wait 集永远是安全的。
   * 您应该始终等待，然后进行内省，然后，只有在完成内省时，再次等待。
   * This function will modify the internal rcl_wait_set_t, so introspecting
   * the wait set during a call to wait is never safe.
   * You should always wait, then introspect, and then, only when done
   * introspecting, wait again.
   *
   * 根据使用的 SynchronizationPolicy，与此函数并发添加和删除等待集中的实体可能是线程安全的。
   * 使用 rclcpp::wait_set_policies::ThreadSafeSynchronization
   * 策略时，此函数将停止等待以允许添加或删除实体， 然后恢复等待，只要超时尚未到达。 It may be
   * thread-safe to add and remove entities to the wait set concurrently with this function,
   * depending on the SynchronizationPolicy that is used. With the
   * rclcpp::wait_set_policies::ThreadSafeSynchronization policy this function will stop waiting to
   * allow add or remove of an entity, and then resume waiting, so long as the timeout has not been
   * reached.
   *
   * \param[in] time_to_wait 如果 > 0，则等待实体准备好的时间，
   *   如果 == 0，在不阻塞的情况下检查是否有任何准备好的事物，
   *   或者如果 < 0，则无限期等待直到其中一个项目准备好。
   *   默认值为 -1，因此无限期等待。
   * \param[in] time_to_wait If > 0, time to wait for entities to be ready,
   *   if == 0, check if anything is ready without blocking, or
   *   if < 0, wait indefinitely until one of the items is ready.
   *   Default is -1, so wait indefinitely.
   * \returns 当其中一个实体准备好时，返回 Ready，或者
   * \returns 当给定的等待时间超过时，返回 Timeout，当 time_to_wait < 0 时不可能，或者
   * \returns 如果等待集为空，返回 Empty，避免在空等待集上无限期等待的可能性。
   * \returns Ready when one of the entities is ready, or
   * \returns Timeout when the given time to wait is exceeded, not possible
   *   when time_to_wait is < 0, or
   * \returns Empty if the wait set is empty, avoiding the possibility of
   *   waiting indefinitely on an empty wait set.
   * \throws rclcpp::exceptions::RCLError 对于未处理的 rcl 错误，或者，
   * \throws std::runtime_error 如果 WaitResultKind 未知
   * \throws rclcpp::exceptions::RCLError on unhandled rcl errors or,
   * \throws std::runtime_error if unknown WaitResultKind
   */
  template <class Rep = int64_t, class Period = std::milli>
  RCUTILS_WARN_UNUSED WaitResult<WaitSetTemplate> wait(
      std::chrono::duration<Rep, Period> time_to_wait = std::chrono::duration<Rep, Period>(-1)) {
    // 将 time_to_wait 转换为纳秒
    // Convert time_to_wait to nanoseconds
    auto time_to_wait_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time_to_wait);

    // 确保在等待期间等待集中实体的所有权是共享的
    // Ensure the ownership of the entities in the wait set is shared for the duration of wait
    this->storage_acquire_ownerships();
    RCPPUTILS_SCOPE_EXIT({ this->storage_release_ownerships(); });

    // 此方法来自 SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    return this->template sync_wait<WaitResult<WaitSetTemplate>>(
        // 将 time_to_wait 作为纳秒传递
        // Pass along the time_to_wait duration as nanoseconds
        time_to_wait_ns,
        // 此方法提供了根据需要重建等待集的功能
        // This method provides the ability to rebuild the wait set, if needed
        [this]() {
          // 此方法来自 StoragePolicy
          // This method comes from the StoragePolicy
          this->storage_rebuild_rcl_wait_set(
              // 此方法来自 SynchronizationPolicy
              // This method comes from the SynchronizationPolicy
              this->get_extra_guard_conditions());
        },
        // 此方法提供了访问 rcl 等待集的方法
        // This method provides access to the rcl wait set
        [this]() -> rcl_wait_set_t& {
          // 此方法来自 StoragePolicy
          // This method comes from the StoragePolicy
          return this->storage_get_rcl_wait_set();
        },
        // 此方法提供了一种创建 WaitResult 的方法
        // This method provides a way to create the WaitResult
        [this](WaitResultKind wait_result_kind) -> WaitResult<WaitSetTemplate> {
          // 将结果转换为 WaitResult
          // Convert the result into a WaitResult
          switch (wait_result_kind) {
            case WaitResultKind::Ready:
              return WaitResult<WaitSetTemplate>::from_ready_wait_result_kind(*this);
            case WaitResultKind::Timeout:
              return WaitResult<WaitSetTemplate>::from_timeout_wait_result_kind();
            case WaitResultKind::Empty:
              return WaitResult<WaitSetTemplate>::from_empty_wait_result_kind();
            default:
              auto msg = "unknown WaitResultKind with value: " + std::to_string(wait_result_kind);
              throw std::runtime_error(msg);
          }
        });
  }

private:
  // 将 WaitResult 类型作为友元，以便在初始化和析构 WaitResult 时分别调用私有方法获取和释放资源。
  // Add WaitResult type as a friend so it can call private methods for
  // acquiring and releasing resources as the WaitResult is initialized and
  // destructed, respectively.
  friend WaitResult<WaitSetTemplate>;

  /// 被 WaitResult 的构造函数调用，用于在拥有权和线程安全方面设置保持状态。
  /// Called by the WaitResult's constructor to place a hold on ownership and thread-safety.
  /**
   * 只应与 wait_result_release() 成对调用。
   * Should only be called in pairs with wait_result_release().
   *
   * \throws std::runtime_error 如果在 wait_result_release() 之前调用两次。
   * \throws std::runtime_error If called twice before wait_result_release().
   */
  void wait_result_acquire() {
    // 如果已经处于保持状态，则抛出运行时错误
    // Throw a runtime error if already holding
    if (wait_result_holding_) {
      throw std::runtime_error("wait_result_acquire() called while already holding");
    }
    wait_result_holding_ = true;
    // 这个方法来自 SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    this->sync_wait_result_acquire();
    // 这个方法来自 StoragePolicy
    // This method comes from the StoragePolicy
    this->storage_acquire_ownerships();
  }

  /// 被 WaitResult 的析构函数调用，以释放资源。
  /// Called by the WaitResult's destructor to release resources.
  /**
   * 只有在调用过 wait_result_acquire() 后才应调用此方法。
   * Should only be called if wait_result_acquire() has been called.
   *
   * \throws std::runtime_error 如果在调用 wait_result_acquire() 之前调用。
   * \throws std::runtime_error If called before wait_result_acquire().
   */
  void wait_result_release() {
    // 如果没有保持状态，则抛出运行时错误
    // Throw a runtime error if not holding
    if (!wait_result_holding_) {
      throw std::runtime_error("wait_result_release() called while not holding");
    }
    wait_result_holding_ = false;
    // 这个方法来自 StoragePolicy
    // This method comes from the StoragePolicy
    this->storage_release_ownerships();
    // 这个方法来自 SynchronizationPolicy
    // This method comes from the SynchronizationPolicy
    this->sync_wait_result_release();
  }

  // 初始化为不保持状态
  // Initialize as not holding
  bool wait_result_holding_ = false;
};

}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_TEMPLATE_HPP_

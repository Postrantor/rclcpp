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

#ifndef RCLCPP__WAIT_SET_POLICIES__DYNAMIC_STORAGE_HPP_
#define RCLCPP__WAIT_SET_POLICIES__DYNAMIC_STORAGE_HPP_

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/client.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_wait_set_mask.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_set_policies/detail/storage_policy_common.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp {
namespace wait_set_policies {

/**
 * @class DynamicStorage
 * @brief WaitSet policy that provides dynamically sized storage.
 * @details 该类是一个 WaitSet 策略，提供动态大小的存储。
 */
class DynamicStorage : public rclcpp::wait_set_policies::detail::StoragePolicyCommon<false> {
protected:
  /**
   * @typedef is_mutable
   * @brief A type trait indicating if the storage can be modified.
   * @details 一个类型特征，表示存储是否可以被修改。
   */
  using is_mutable = std::true_type;

  /**
   * @class SubscriptionEntry
   * @brief A class representing a subscription entry in a wait set.
   * @details 该类表示 wait set 中的订阅项。
   */
  class SubscriptionEntry {
    // (wjwwood): indent of 'public:' is weird, I know. uncrustify is dumb.

  public:
    /**
     * @brief A shared pointer to the subscription base object.
     * @details 一个指向订阅基础对象的共享指针。
     */
    std::shared_ptr<rclcpp::SubscriptionBase> subscription;

    /**
     * @brief A mask for the subscription wait set.
     * @details 订阅等待集的掩码。
     */
    rclcpp::SubscriptionWaitSetMask mask;

    /**
     * @brief Conversion constructor, which is intentionally not marked explicit.
     * @param subscription_in A shared pointer to the subscription base object (default is nullptr).
     * @param mask_in A mask for the subscription wait set (default is empty).
     * @details 转换构造函数，故意未标记为 explicit。
     */
    SubscriptionEntry(
        std::shared_ptr<rclcpp::SubscriptionBase> subscription_in = nullptr,
        const rclcpp::SubscriptionWaitSetMask& mask_in = {})
        : subscription(std::move(subscription_in)), mask(mask_in) {}

    /**
     * @brief Reset the subscription entry.
     * @details 重置订阅项。
     */
    void reset() noexcept { subscription.reset(); }
  };

  /**
   * @class WeakSubscriptionEntry
   * @brief A class representing a weak subscription entry in a wait set.
   * @details 该类表示 wait set 中的弱订阅项。
   */
  class WeakSubscriptionEntry {
  public:
    /**
     * @brief A weak pointer to the subscription base object.
     * @details 一个指向订阅基础对象的弱指针。
     */
    std::weak_ptr<rclcpp::SubscriptionBase> subscription;

    /**
     * @brief A mask for the subscription wait set.
     * @details 订阅等待集的掩码。
     */
    rclcpp::SubscriptionWaitSetMask mask;

    /**
     * @brief Explicit constructor for WeakSubscriptionEntry.
     * @param subscription_in A shared pointer to the subscription base object.
     * @param mask_in A mask for the subscription wait set.
     * @details 显式构造函数。
     */
    explicit WeakSubscriptionEntry(
        const std::shared_ptr<rclcpp::SubscriptionBase>& subscription_in,
        const rclcpp::SubscriptionWaitSetMask& mask_in) noexcept
        : subscription(subscription_in), mask(mask_in) {}

    /**
     * @brief Explicit constructor for WeakSubscriptionEntry from a SubscriptionEntry.
     * @param other Another SubscriptionEntry object.
     * @details 根据另一个 SubscriptionEntry 对象的显式构造函数。
     */
    explicit WeakSubscriptionEntry(const SubscriptionEntry& other)
        : subscription(other.subscription), mask(other.mask) {}

    /**
     * @brief Lock the weak pointer and return a shared pointer to the subscription base object.
     * @return A shared pointer to the subscription base object, or nullptr if expired.
     * @details 锁定弱指针并返回指向订阅基础对象的共享指针。
     */
    std::shared_ptr<rclcpp::SubscriptionBase> lock() const { return subscription.lock(); }

    /**
     * @brief Check if the weak pointer is expired.
     * @return True if expired, false otherwise.
     * @details 检查弱指针是否过期。
     */
    bool expired() const noexcept { return subscription.expired(); }
  };

  // 弱订阅条目的序列（A sequence of weak subscription entries）
  using SequenceOfWeakSubscriptions = std::vector<WeakSubscriptionEntry>;
  // 可迭代的订阅条目（An iterable of subscription entries）
  using SubscriptionsIterable = std::vector<SubscriptionEntry>;

  // 弱保护条件的序列（A sequence of weak guard conditions）
  using SequenceOfWeakGuardConditions = std::vector<std::weak_ptr<rclcpp::GuardCondition>>;
  // 可迭代的保护条件（An iterable of guard conditions）
  using GuardConditionsIterable = std::vector<std::shared_ptr<rclcpp::GuardCondition>>;

  // 弱定时器的序列（A sequence of weak timers）
  using SequenceOfWeakTimers = std::vector<std::weak_ptr<rclcpp::TimerBase>>;
  // 可迭代的定时器（An iterable of timers）
  using TimersIterable = std::vector<std::shared_ptr<rclcpp::TimerBase>>;

  // 弱客户端的序列（A sequence of weak clients）
  using SequenceOfWeakClients = std::vector<std::weak_ptr<rclcpp::ClientBase>>;
  // 可迭代的客户端（An iterable of clients）
  using ClientsIterable = std::vector<std::shared_ptr<rclcpp::ClientBase>>;

  // 弱服务的序列（A sequence of weak services）
  using SequenceOfWeakServices = std::vector<std::weak_ptr<rclcpp::ServiceBase>>;
  // 可迭代的服务（An iterable of services）
  using ServicesIterable = std::vector<std::shared_ptr<rclcpp::ServiceBase>>;

  /**
   * @class WaitableEntry
   * @brief 一个包含可等待对象和关联实体的类。(A class containing a waitable object and an
   * associated entity.)
   */
  class WaitableEntry {
  public:
    // 可等待对象指针 (Waitable object pointer)
    std::shared_ptr<rclcpp::Waitable> waitable;
    // 关联实体指针 (Associated entity pointer)
    std::shared_ptr<void> associated_entity;

    /**
     * @brief 转换构造函数，故意不标记为 explicit。 (Conversion constructor, which is intentionally
     * not marked explicit.)
     * @param waitable_in 可等待对象 (Waitable object)
     * @param associated_entity_in 关联实体 (Associated entity)
     */
    WaitableEntry(
        std::shared_ptr<rclcpp::Waitable> waitable_in = nullptr,
        std::shared_ptr<void> associated_entity_in = nullptr) noexcept
        : waitable(std::move(waitable_in)), associated_entity(std::move(associated_entity_in)) {}

    /**
     * @brief 重置可等待对象和关联实体的指针。(Reset pointers to the waitable object and the
     * associated entity.)
     */
    void reset() noexcept {
      waitable.reset();
      associated_entity.reset();
    }
  };

  /**
   * @class WeakWaitableEntry
   * @brief 一个包含弱引用可等待对象和关联实体的类。(A class containing weak references to a
   * waitable object and an associated entity.)
   */
  class WeakWaitableEntry {
  public:
    // 弱引用可等待对象指针 (Weak reference to the waitable object pointer)
    std::weak_ptr<rclcpp::Waitable> waitable;
    // 弱引用关联实体指针 (Weak reference to the associated entity pointer)
    std::weak_ptr<void> associated_entity;

    /**
     * @brief 显式构造函数。(Explicit constructor.)
     * @param waitable_in 可等待对象 (Waitable object)
     * @param associated_entity_in 关联实体 (Associated entity)
     */
    explicit WeakWaitableEntry(
        const std::shared_ptr<rclcpp::Waitable>& waitable_in,
        const std::shared_ptr<void>& associated_entity_in) noexcept
        : waitable(waitable_in), associated_entity(associated_entity_in) {}

    /**
     * @brief 从另一个 WaitableEntry 对象构造。(Construct from another WaitableEntry object.)
     * @param other 另一个 WaitableEntry 对象 (Another WaitableEntry object)
     */
    explicit WeakWaitableEntry(const WaitableEntry& other)
        : waitable(other.waitable), associated_entity(other.associated_entity) {}

    /**
     * @brief 锁定并返回可等待对象的共享指针。(Lock and return a shared pointer to the waitable
     * object.)
     * @return 可等待对象的共享指针 (Shared pointer to the waitable object)
     */
    std::shared_ptr<rclcpp::Waitable> lock() const { return waitable.lock(); }

    /**
     * @brief 检查可等待对象是否过期。(Check if the waitable object has expired.)
     * @return 如果过期则返回 true，否则返回 false。(True if expired, false otherwise.)
     */
    bool expired() const noexcept { return waitable.expired(); }
  };
  // 用于存储弱引用等待对象的序列 (Sequence for storing weakly referenced waitable objects)
  using SequenceOfWeakWaitables = std::vector<WeakWaitableEntry>;
  // 用于存储可迭代的等待对象 (Iterable sequence for storing waitable objects)
  using WaitablesIterable = std::vector<WaitableEntry>;

  /**
   * @brief 动态存储类 (Dynamic storage class)
   *
   * @tparam ArrayOfExtraGuardConditions 额外守护条件数组类型 (Type of the array of extra guard
   * conditions)
   * @param[in] subscriptions 订阅迭代器 (Subscriptions iterable)
   * @param[in] guard_conditions 守护条件迭代器 (Guard conditions iterable)
   * @param[in] extra_guard_conditions 额外守护条件迭代器 (Extra guard conditions iterable)
   * @param[in] timers 计时器迭代器 (Timers iterable)
   * @param[in] clients 客户端迭代器 (Clients iterable)
   * @param[in] services 服务迭代器 (Services iterable)
   * @param[in] waitables 可等待对象迭代器 (Waitables iterable)
   * @param[in] context ROS2 上下文共享指针 (Shared pointer to ROS2 context)
   */
  template <class ArrayOfExtraGuardConditions>
  explicit DynamicStorage(
      const SubscriptionsIterable& subscriptions,  // 订阅集合 (Subscription collection)
      const GuardConditionsIterable& guard_conditions,  // 守护条件集合 (Guard condition collection)
      const ArrayOfExtraGuardConditions&
          extra_guard_conditions,    // 额外守护条件集合 (Extra guard condition collection)
      const TimersIterable& timers,  // 计时器集合 (Timer collection)
      const ClientsIterable& clients,      // 客户端集合 (Client collection)
      const ServicesIterable& services,    // 服务集合 (Service collection)
      const WaitablesIterable& waitables,  // 可等待对象集合 (Waitable collection)
      rclcpp::Context::SharedPtr context)  // ROS2 上下文共享指针 (Shared pointer to ROS2 context)
      : StoragePolicyCommon(
            subscriptions,
            guard_conditions,
            extra_guard_conditions,
            timers,
            clients,
            services,
            waitables,
            context),
        subscriptions_(
            subscriptions.cbegin(),
            subscriptions.cend()),  // 初始化订阅列表 (Initialize subscription list)
        shared_subscriptions_(
            subscriptions_.size()),  // 初始化共享订阅列表 (Initialize shared subscription list)
        guard_conditions_(
            guard_conditions.cbegin(),
            guard_conditions.cend()),  // 初始化守护条件列表 (Initialize guard condition list)
        shared_guard_conditions_(
            guard_conditions_
                .size()),  // 初始化共享守护条件列表 (Initialize shared guard condition list)
        timers_(timers.cbegin(), timers.cend()),  // 初始化计时器列表 (Initialize timer list)
        shared_timers_(timers_.size()),  // 初始化共享计时器列表 (Initialize shared timer list)
        clients_(clients.cbegin(), clients.cend()),  // 初始化客户端列表 (Initialize client list)
        shared_clients_(clients_.size()),  // 初始化共享客户端列表 (Initialize shared client list)
        services_(services.cbegin(), services.cend()),  // 初始化服务列表 (Initialize service list)
        shared_services_(services_.size()),  // 初始化共享服务列表 (Initialize shared service list)
        waitables_(
            waitables.cbegin(),
            waitables.cend()),  // 初始化可等待对象列表 (Initialize waitable list)
        shared_waitables_(
            waitables_.size())  // 初始化共享可等待对象列表 (Initialize shared waitable list)
  {}

  /**
   * @brief 动态存储类析构函数 (DynamicStorage class destructor)
   */
  ~DynamicStorage() = default;

  /**
   * @brief 重建 rcl_wait_set 的存储
   * @param extra_guard_conditions 额外的守卫条件数组
   *
   * @details 用提供的实体集合重建 rcl_wait_set
   * (Rebuild the storage for rcl_wait_set with the provided sets of entities)
   *
   * @tparam ArrayOfExtraGuardConditions 额外守卫条件数组类型
   * (Type of the array containing extra guard conditions)
   */
  template <class ArrayOfExtraGuardConditions>
  void storage_rebuild_rcl_wait_set(const ArrayOfExtraGuardConditions& extra_guard_conditions) {
    this->storage_rebuild_rcl_wait_set_with_sets(
        subscriptions_, guard_conditions_, extra_guard_conditions, timers_, clients_, services_,
        waitables_);
  }

  /**
   * @brief 检查实体是否在实体序列中
   * @param entity 要检查的实体
   * @param entities 实体序列
   * @return 如果找到实体，则返回 true，否则返回 false
   *
   * @details 检查给定的实体是否存在于提供的实体序列中
   * (Check if the given entity exists in the provided sequence of entities)
   *
   * @tparam EntityT 要检查的实体类型
   * @tparam SequenceOfEntitiesT 实体序列类型
   */
  template <class EntityT, class SequenceOfEntitiesT>
  static bool storage_has_entity(const EntityT& entity, const SequenceOfEntitiesT& entities) {
    return std::any_of(entities.cbegin(), entities.cend(), [&entity](const auto& inner) {
      return &entity == inner.lock().get();
    });
  }

  /**
   * @brief 在实体序列中查找实体
   * @param entity 要查找的实体
   * @param entities 实体序列
   * @return 如果找到实体，则返回该实体在序列中的迭代器，否则返回序列的结束迭代器
   *
   * @details 查找给定的实体在提供的实体序列中的位置
   * (Find the position of the given entity in the provided sequence of entities)
   *
   * @tparam EntityT 要查找的实体类型
   * @tparam SequenceOfEntitiesT 实体序列类型
   */
  template <class EntityT, class SequenceOfEntitiesT>
  static auto storage_find_entity(const EntityT& entity, const SequenceOfEntitiesT& entities) {
    return std::find_if(entities.cbegin(), entities.cend(), [&entity](const auto& inner) {
      return &entity == inner.lock().get();
    });
  }

  /**
   * @brief 添加订阅到存储器 (Add a subscription to the storage)
   *
   * @param subscription 要添加的订阅对象的共享指针 (A shared pointer to the subscription object to
   * be added)
   */
  void storage_add_subscription(std::shared_ptr<rclcpp::SubscriptionBase>&& subscription) {
    // 检查订阅是否已在存储器中 (Check if the subscription is already in the storage)
    if (this->storage_has_entity(*subscription, subscriptions_)) {
      throw std::runtime_error("subscription already in wait set");
    }

    // 创建一个弱订阅条目 (Create a weak subscription entry)
    WeakSubscriptionEntry weak_entry{std::move(subscription), {}};

    // 将弱订阅条目添加到订阅列表中 (Add the weak subscription entry to the subscriptions list)
    subscriptions_.push_back(std::move(weak_entry));

    // 标记存储器需要调整大小 (Flag the storage for resizing)
    this->storage_flag_for_resize();
  }

  /**
   * @brief 从存储器中移除订阅 (Remove a subscription from the storage)
   *
   * @param subscription 要移除的订阅对象的共享指针 (A shared pointer to the subscription object to
   * be removed)
   */
  void storage_remove_subscription(std::shared_ptr<rclcpp::SubscriptionBase>&& subscription) {
    // 查找要移除的订阅对象在订阅列表中的位置 (Find the position of the subscription object to be
    // removed in the subscriptions list)
    auto it = this->storage_find_entity(*subscription, subscriptions_);

    // 如果没有找到订阅对象 (If the subscription object is not found)
    if (subscriptions_.cend() == it) {
      throw std::runtime_error("subscription not in wait set");
    }

    // 从订阅列表中移除订阅对象 (Remove the subscription object from the subscriptions list)
    subscriptions_.erase(it);

    // 标记存储器需要调整大小 (Flag the storage for resizing)
    this->storage_flag_for_resize();
  }

  /**
   * @brief 添加守护条件到存储器 (Add a guard condition to the storage)
   *
   * @param guard_condition 要添加的守护条件对象的共享指针 (A shared pointer to the guard condition
   * object to be added)
   */
  void storage_add_guard_condition(std::shared_ptr<rclcpp::GuardCondition>&& guard_condition) {
    // 检查守护条件是否已在存储器中 (Check if the guard condition is already in the storage)
    if (this->storage_has_entity(*guard_condition, guard_conditions_)) {
      throw std::runtime_error("guard_condition already in wait set");
    }

    // 将守护条件添加到守护条件列表中 (Add the guard condition to the guard conditions list)
    guard_conditions_.push_back(std::move(guard_condition));

    // 标记存储器需要调整大小 (Flag the storage for resizing)
    this->storage_flag_for_resize();
  }

  /**
   * @brief 从存储器中移除守护条件 (Remove a guard condition from the storage)
   *
   * @param guard_condition 要移除的守护条件对象的共享指针 (A shared pointer to the guard condition
   * object to be removed)
   */
  void storage_remove_guard_condition(std::shared_ptr<rclcpp::GuardCondition>&& guard_condition) {
    // 查找要移除的守护条件对象在守护条件列表中的位置 (Find the position of the guard condition
    // object to be removed in the guard conditions list)
    auto it = this->storage_find_entity(*guard_condition, guard_conditions_);

    // 如果没有找到守护条件对象 (If the guard condition object is not found)
    if (guard_conditions_.cend() == it) {
      throw std::runtime_error("guard_condition not in wait set");
    }

    // 从守护条件列表中移除守护条件对象 (Remove the guard condition object from the guard conditions
    // list)
    guard_conditions_.erase(it);

    // 标记存储器需要调整大小 (Flag the storage for resizing)
    this->storage_flag_for_resize();
  }

  /**
   * @brief 添加定时器到存储容器中 (Add a timer to the storage container)
   *
   * @param timer 需要添加的定时器 (The timer to be added)
   */
  void storage_add_timer(std::shared_ptr<rclcpp::TimerBase>&& timer) {
    // 检查定时器是否已经在存储容器中 (Check if the timer is already in the storage container)
    if (this->storage_has_entity(*timer, timers_)) {
      // 如果已经存在，则抛出异常 (If it exists, throw an exception)
      throw std::runtime_error("timer already in wait set");
    }
    // 将定时器添加到存储容器中 (Add the timer to the storage container)
    timers_.push_back(std::move(timer));
    // 标记存储容器需要调整大小 (Flag the storage container for resizing)
    this->storage_flag_for_resize();
  }

  /**
   * @brief 从存储容器中移除定时器 (Remove a timer from the storage container)
   *
   * @param timer 需要移除的定时器 (The timer to be removed)
   */
  void storage_remove_timer(std::shared_ptr<rclcpp::TimerBase>&& timer) {
    // 查找定时器在存储容器中的位置 (Find the position of the timer in the storage container)
    auto it = this->storage_find_entity(*timer, timers_);
    // 如果定时器不在存储容器中，则抛出异常 (If the timer is not in the storage container, throw an
    // exception)
    if (timers_.cend() == it) {
      throw std::runtime_error("timer not in wait set");
    }
    // 从存储容器中移除定时器 (Remove the timer from the storage container)
    timers_.erase(it);
    // 标记存储容器需要调整大小 (Flag the storage container for resizing)
    this->storage_flag_for_resize();
  }

  /**
   * @brief 添加客户端到存储容器中 (Add a client to the storage container)
   *
   * @param client 需要添加的客户端 (The client to be added)
   */
  void storage_add_client(std::shared_ptr<rclcpp::ClientBase>&& client) {
    // 检查客户端是否已经在存储容器中 (Check if the client is already in the storage container)
    if (this->storage_has_entity(*client, clients_)) {
      // 如果已经存在，则抛出异常 (If it exists, throw an exception)
      throw std::runtime_error("client already in wait set");
    }
    // 将客户端添加到存储容器中 (Add the client to the storage container)
    clients_.push_back(std::move(client));
    // 标记存储容器需要调整大小 (Flag the storage container for resizing)
    this->storage_flag_for_resize();
  }

  /**
   * @brief 从存储容器中移除客户端 (Remove a client from the storage container)
   *
   * @param client 需要移除的客户端 (The client to be removed)
   */
  void storage_remove_client(std::shared_ptr<rclcpp::ClientBase>&& client) {
    // 查找客户端在存储容器中的位置 (Find the position of the client in the storage container)
    auto it = this->storage_find_entity(*client, clients_);
    // 如果客户端不在存储容器中，则抛出异常 (If the client is not in the storage container, throw an
    // exception)
    if (clients_.cend() == it) {
      throw std::runtime_error("client not in wait set");
    }
    // 从存储容器中移除客户端 (Remove the client from the storage container)
    clients_.erase(it);
    // 标记存储容器需要调整大小 (Flag the storage container for resizing)
    this->storage_flag_for_resize();
  }

  /**
   * @brief 添加服务到存储器 (Add a service to the storage)
   *
   * @param service 要添加的服务对象的智能指针 (A shared pointer to the service object to be added)
   */
  void storage_add_service(std::shared_ptr<rclcpp::ServiceBase>&& service) {
    // 检查服务是否已经在存储器中 (Check if the service is already in the storage)
    if (this->storage_has_entity(*service, services_)) {
      throw std::runtime_error("service already in wait set");
    }
    // 将服务添加到服务列表中 (Add the service to the list of services)
    services_.push_back(std::move(service));
    // 标记存储器需要调整大小 (Flag the storage for resizing)
    this->storage_flag_for_resize();
  }

  /**
   * @brief 从存储器中移除服务 (Remove a service from the storage)
   *
   * @param service 要移除的服务对象的智能指针 (A shared pointer to the service object to be
   * removed)
   */
  void storage_remove_service(std::shared_ptr<rclcpp::ServiceBase>&& service) {
    // 在服务列表中查找要移除的服务 (Find the service to be removed in the list of services)
    auto it = this->storage_find_entity(*service, services_);
    // 如果未找到服务，抛出异常 (If the service is not found, throw an exception)
    if (services_.cend() == it) {
      throw std::runtime_error("service not in wait set");
    }
    // 从服务列表中删除服务 (Erase the service from the list of services)
    services_.erase(it);
    // 标记存储器需要调整大小 (Flag the storage for resizing)
    this->storage_flag_for_resize();
  }

  /**
   * @brief 添加等待对象到存储器 (Add a waitable object to the storage)
   *
   * @param waitable 要添加的等待对象的智能指针 (A shared pointer to the waitable object to be
   * added)
   * @param associated_entity 与等待对象关联的实体的智能指针 (A shared pointer to the entity
   * associated with the waitable object)
   */
  void storage_add_waitable(
      std::shared_ptr<rclcpp::Waitable>&& waitable, std::shared_ptr<void>&& associated_entity) {
    // 检查等待对象是否已经在存储器中 (Check if the waitable object is already in the storage)
    if (this->storage_has_entity(*waitable, waitables_)) {
      throw std::runtime_error("waitable already in wait set");
    }
    // 创建一个弱等待条目 (Create a weak waitable entry)
    WeakWaitableEntry weak_entry(std::move(waitable), std::move(associated_entity));
    // 将弱等待条目添加到等待对象列表中 (Add the weak waitable entry to the list of waitable
    // objects)
    waitables_.push_back(std::move(weak_entry));
    // 标记存储器需要调整大小 (Flag the storage for resizing)
    this->storage_flag_for_resize();
  }

  /**
   * @brief 从存储器中移除等待对象 (Remove a waitable object from the storage)
   *
   * @param waitable 要移除的等待对象的智能指针 (A shared pointer to the waitable object to be
   * removed)
   */
  void storage_remove_waitable(std::shared_ptr<rclcpp::Waitable>&& waitable) {
    // 在等待对象列表中查找要移除的等待对象 (Find the waitable object to be removed in the list of
    // waitable objects)
    auto it = this->storage_find_entity(*waitable, waitables_);
    // 如果未找到等待对象，抛出异常 (If the waitable object is not found, throw an exception)
    if (waitables_.cend() == it) {
      throw std::runtime_error("waitable not in wait set");
    }
    // 从等待对象列表中删除等待对象 (Erase the waitable object from the list of waitable objects)
    waitables_.erase(it);
    // 标记存储器需要调整大小 (Flag the storage for resizing)
    this->storage_flag_for_resize();
  }

  /**
   * @brief 清理已删除实体的存储 (Prune storage of deleted entities)
   *
   * 这个函数是 noexcept 的原因：
   *   - std::weak_ptr::expired 是 noexcept
   *   - erase-remove 惯用法是 noexcept，因为我们没有使用 ExecutionPolicy 版本
   *   - 如果 T 的赋值运算符也是 noexcept，则 std::vector::erase 是 noexcept
   *     - 而且，std::weak_ptr 的 operator= 是 noexcept
   */
  void storage_prune_deleted_entities() noexcept {
    // 可重用（模板化）lambda，用于移除谓词 (Reusable (templated) lambda for removal predicate)
    auto p = [](const auto& weak_ptr) {
      // 移除已过期的条目 (Remove entries which have expired)
      return weak_ptr.expired();
    };
    // 删除已被删除的 guard_conditions_ (Remove guard conditions which have been deleted)
    guard_conditions_.erase(
        std::remove_if(guard_conditions_.begin(), guard_conditions_.end(), p),
        guard_conditions_.end());
    // 删除已被删除的 timers_ (Remove timers which have been deleted)
    timers_.erase(std::remove_if(timers_.begin(), timers_.end(), p), timers_.end());
    // 删除已被删除的 clients_ (Remove clients which have been deleted)
    clients_.erase(std::remove_if(clients_.begin(), clients_.end(), p), clients_.end());
    // 删除已被删除的 services_ (Remove services which have been deleted)
    services_.erase(std::remove_if(services_.begin(), services_.end(), p), services_.end());
    // 删除已被删除的 waitables_ (Remove waitables which have been deleted)
    waitables_.erase(std::remove_if(waitables_.begin(), waitables_.end(), p), waitables_.end());
  }

  /**
   * @brief 以 Doxygen 支持的格式为代码段添加参数列表说明。
   * @brief Add parameter list description for the code segment in Doxygen supported format.
   *
   * @note 此函数没有参数。This function has no parameters.
   */
  void storage_acquire_ownerships() {
    // 如果 ownership_reference_counter_ 大于等于1，自增后直接返回，避免重复锁定。
    // If ownership_reference_counter_ is greater than or equal to 1, increment it and return
    // directly to avoid redundant locking.
    if (++ownership_reference_counter_ > 1) {
      return;
    }

    // 设置通用锁定功能的 lambda 函数。
    // Set up a lambda function for the common locking feature.
    auto lock_all = [](const auto& weak_ptrs, auto& shared_ptrs) {
      // 调整 shared_ptrs 的大小以匹配 weak_ptrs 的大小。
      // Resize the shared_ptrs to match the size of weak_ptrs.
      shared_ptrs.resize(weak_ptrs.size());

      size_t index = 0;
      // 遍历 weak_ptrs，并将其锁定为 shared_ptrs。
      // Iterate through weak_ptrs and lock them as shared_ptrs.
      for (const auto& weak_ptr : weak_ptrs) {
        shared_ptrs[index++] = weak_ptr.lock();
      }
    };

    // 锁定所有弱指针并保持它们，直到释放。
    // Lock all the weak pointers and hold them until released.
    lock_all(guard_conditions_, shared_guard_conditions_);
    lock_all(timers_, shared_timers_);
    lock_all(clients_, shared_clients_);
    lock_all(services_, shared_services_);

    // 我们需要一个专门用于 waitables 的此版本。
    // We need a specialized version of this for waitables.
    auto lock_all_waitables = [](const auto& weak_ptrs, auto& shared_ptrs) {
      // 调整 shared_ptrs 的大小以匹配 weak_ptrs 的大小。
      // Resize the shared_ptrs to match the size of weak_ptrs.
      shared_ptrs.resize(weak_ptrs.size());

      size_t index = 0;
      // 遍历 weak_ptrs，并将其锁定为 WaitableEntry 类型的 shared_ptrs。
      // Iterate through weak_ptrs and lock them as WaitableEntry type shared_ptrs.
      for (const auto& weak_ptr : weak_ptrs) {
        shared_ptrs[index++] =
            WaitableEntry{weak_ptr.waitable.lock(), weak_ptr.associated_entity.lock()};
      }
    };

    // 锁定所有 waitables 并保持它们，直到释放。
    // Lock all waitables and hold them until released.
    lock_all_waitables(waitables_, shared_waitables_);
  }

  /**
   * @brief 释放所有权 (Release ownerships)
   *
   * 该函数用于释放所有权，当引用计数器为0时，会重置所有共享指针。
   * The function is used to release ownerships. When the reference counter reaches 0, it will reset
   * all shared pointers.
   */
  void storage_release_ownerships() {
    // 如果引用计数器大于0，则递减并避免释放所有权 (If the reference counter is greater than 0,
    // decrement it and avoid releasing ownership)
    if (--ownership_reference_counter_ > 0) {
      // 避免在引用计数为0之前释放所有权 (Avoid releasing ownership until reference count is 0)
      return;
    }

    // 通过重置它们来"解锁"所有共享指针 ("Unlock" all shared pointers by resetting them)
    auto reset_all = [](auto& shared_ptrs) {
      // 遍历每个共享指针并重置它 (Iterate through each shared pointer and reset it)
      for (auto& shared_ptr : shared_ptrs) {
        shared_ptr.reset();
      }
    };

    // 重置所有共享的 Guard Conditions (Reset all shared Guard Conditions)
    reset_all(shared_guard_conditions_);
    // 重置所有共享的 Timers (Reset all shared Timers)
    reset_all(shared_timers_);
    // 重置所有共享的 Clients (Reset all shared Clients)
    reset_all(shared_clients_);
    // 重置所有共享的 Services (Reset all shared Services)
    reset_all(shared_services_);
    // 重置所有共享的 Waitables (Reset all shared Waitables)
    reset_all(shared_waitables_);
  }

  // 所有权引用计数器 (Ownership reference counter)
  size_t ownership_reference_counter_ = 0;

  // 弱订阅序列 (Sequence of weak subscriptions)
  SequenceOfWeakSubscriptions subscriptions_;
  // 共享订阅迭代 (Shared subscriptions iterable)
  SubscriptionsIterable shared_subscriptions_;

  // 弱 Guard Conditions 序列 (Sequence of weak Guard Conditions)
  SequenceOfWeakGuardConditions guard_conditions_;
  // 共享 Guard Conditions 迭代 (Shared Guard Conditions iterable)
  GuardConditionsIterable shared_guard_conditions_;

  // 弱 Timers 序列 (Sequence of weak Timers)
  SequenceOfWeakTimers timers_;
  // 共享 Timers 迭代 (Shared Timers iterable)
  TimersIterable shared_timers_;

  // 弱 Clients 序列 (Sequence of weak Clients)
  SequenceOfWeakClients clients_;
  // 共享 Clients 迭代 (Shared Clients iterable)
  ClientsIterable shared_clients_;

  // 弱 Services 序列 (Sequence of weak Services)
  SequenceOfWeakServices services_;
  // 共享 Services 迭代 (Shared Services iterable)
  ServicesIterable shared_services_;

  // 弱 Waitables 序列 (Sequence of weak Waitables)
  SequenceOfWeakWaitables waitables_;
  // 共享 Waitables 迭代 (Shared Waitables iterable)
  WaitablesIterable shared_waitables_;
};

}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DYNAMIC_STORAGE_HPP_

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

#ifndef RCLCPP__WAIT_SET_POLICIES__STATIC_STORAGE_HPP_
#define RCLCPP__WAIT_SET_POLICIES__STATIC_STORAGE_HPP_

#include <array>
#include <memory>
#include <utility>

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

/// 等待集策略，只提供固定大小的存储。 (WaitSet policy that explicitly provides fixed sized storage
/// only.)
/**
 * 注意底层的 rcl_wait_set_t 仍然是动态分配的，但只在构造时分配一次，在析构时释放一次。
 * (Note the underlying rcl_wait_set_t is still dynamically allocated, but only
 * once during construction, and deallocated once during destruction.)
 */
template <
    std::size_t NumberOfSubscriptions,   // 订阅数量 (Number of subscriptions)
    std::size_t NumberOfGuardCondtions,  // 守卫条件数量 (Number of guard conditions)
    std::size_t NumberOfTimers,          // 计时器数量 (Number of timers)
    std::size_t NumberOfClients,         // 客户端数量 (Number of clients)
    std::size_t NumberOfServices,        // 服务数量 (Number of services)
    std::size_t NumberOfWaitables>       // 可等待对象数量 (Number of waitables)
class StaticStorage : public rclcpp::wait_set_policies::detail::StoragePolicyCommon<true> {
protected:
  // 表示存储是否可变的类型，这里为 false_type，表示存储不可变。
  // (Type indicating whether the storage is mutable or not, here it's std::false_type, meaning the
  // storage is immutable.)
  using is_mutable = std::false_type;

  /**
   * @class SubscriptionEntry
   * @brief 一个包含订阅信息的类，用于管理订阅对象和其相关的掩码。 (A class containing subscription
   * information for managing subscription objects and their associated masks.)
   */
  class SubscriptionEntry {
  public:
    /// 指向 rclcpp::SubscriptionBase 的共享指针。 (Shared pointer to rclcpp::SubscriptionBase)
    std::shared_ptr<rclcpp::SubscriptionBase> subscription;

    /// 订阅等待集合掩码。 (Subscription wait set mask)
    rclcpp::SubscriptionWaitSetMask mask;

    /**
     * @brief 转换构造函数，有意不将其标记为 explicit。
     *        (Conversion constructor, which is intentionally not marked explicit.)
     *
     * @param subscription_in 默认值为 nullptr，指向 rclcpp::SubscriptionBase 的共享指针。
     *                        (Default value is nullptr, shared pointer to
     * rclcpp::SubscriptionBase.)
     * @param mask_in 默认值为空，rclcpp::SubscriptionWaitSetMask 类型的变量。
     *                (Default value is empty, variable of type rclcpp::SubscriptionWaitSetMask.)
     */
    SubscriptionEntry(
        std::shared_ptr<rclcpp::SubscriptionBase> subscription_in = nullptr,
        rclcpp::SubscriptionWaitSetMask mask_in = {})
        : subscription(std::move(subscription_in)),  // 移动语义赋值订阅。 (Move semantics
                                                     // assignment for subscription.)
          mask(mask_in)                              // 赋值掩码。 (Assignment for mask.)
    {}
  };
  using ArrayOfSubscriptions = std::array<SubscriptionEntry, NumberOfSubscriptions>;
  using SubscriptionsIterable = ArrayOfSubscriptions;

  using ArrayOfGuardConditions =
      std::array<std::shared_ptr<rclcpp::GuardCondition>, NumberOfGuardCondtions>;
  using GuardConditionsIterable = ArrayOfGuardConditions;

  using ArrayOfTimers = std::array<std::shared_ptr<rclcpp::TimerBase>, NumberOfTimers>;
  using TimersIterable = ArrayOfTimers;

  using ArrayOfClients = std::array<std::shared_ptr<rclcpp::ClientBase>, NumberOfClients>;
  using ClientsIterable = ArrayOfClients;

  using ArrayOfServices = std::array<std::shared_ptr<rclcpp::ServiceBase>, NumberOfServices>;
  using ServicesIterable = ArrayOfServices;

  /**
   * @brief WaitableEntry 结构体，用于存储 waitable 对象及其关联的实体 (WaitableEntry structure,
   * used to store waitable objects and their associated entities)
   */
  struct WaitableEntry {
    /**
     * @brief 转换构造函数，故意不标记为 explicit (Conversion constructor, which is intentionally
     * not marked explicit)
     *
     * @param[in] waitable_in 共享指针，指向 rclcpp::Waitable 类型的对象，默认为空 (Shared pointer
     * pointing to an object of type rclcpp::Waitable, default is nullptr)
     * @param[in] associated_entity_in 共享指针，指向与 waitable 相关联的实体，默认为空 (Shared
     * pointer pointing to the associated entity with the waitable, default is nullptr)
     */
    WaitableEntry(
        std::shared_ptr<rclcpp::Waitable> waitable_in = nullptr,
        std::shared_ptr<void> associated_entity_in = nullptr) noexcept
        : waitable(std::move(waitable_in)), associated_entity(std::move(associated_entity_in)) {
      // 移动语义将传入的共享指针赋值给 waitable 和 associated_entity 成员变量 (Move semantics
      // assign the incoming shared pointers to the waitable and associated_entity member variables)
    }

    // waitable 成员变量，共享指针，指向 rclcpp::Waitable 类型的对象 (waitable member variable,
    // shared pointer pointing to an object of type rclcpp::Waitable)
    std::shared_ptr<rclcpp::Waitable> waitable;

    // associated_entity 成员变量，共享指针，指向与 waitable 相关联的实体 (associated_entity member
    // variable, shared pointer pointing to the associated entity with the waitable)
    std::shared_ptr<void> associated_entity;
  };
  using ArrayOfWaitables = std::array<WaitableEntry, NumberOfWaitables>;
  using WaitablesIterable = ArrayOfWaitables;

  /**
   * @brief 构造函数，用于创建 StaticStorage 对象 (Constructor for creating a StaticStorage object)
   *
   * @tparam ArrayOfExtraGuardConditions 额外的 GuardCondition 数组类型 (Type of the array of extra
   * GuardConditions)
   * @param[in] subscriptions 订阅者数组 (Array of subscriptions)
   * @param[in] guard_conditions GuardCondition 数组 (Array of GuardConditions)
   * @param[in] extra_guard_conditions 额外的 GuardCondition 数组 (Array of extra GuardConditions)
   * @param[in] timers 定时器数组 (Array of timers)
   * @param[in] clients 客户端数组 (Array of clients)
   * @param[in] services 服务数组 (Array of services)
   * @param[in] waitables 可等待对象数组 (Array of Waitables)
   * @param[in] context 共享指针，指向 rclcpp::Context 对象 (SharedPtr pointing to an
   * rclcpp::Context object)
   */
  template <class ArrayOfExtraGuardConditions>
  explicit StaticStorage(
      const ArrayOfSubscriptions& subscriptions,
      const ArrayOfGuardConditions& guard_conditions,
      const ArrayOfExtraGuardConditions& extra_guard_conditions,
      const ArrayOfTimers& timers,
      const ArrayOfClients& clients,
      const ArrayOfServices& services,
      const ArrayOfWaitables& waitables,
      rclcpp::Context::SharedPtr context)
      : StoragePolicyCommon(
            subscriptions,
            guard_conditions,
            extra_guard_conditions,
            timers,
            clients,
            services,
            waitables,
            context),
        // 初始化成员变量 (Initialize member variables)
        subscriptions_(subscriptions),
        guard_conditions_(guard_conditions),
        timers_(timers),
        clients_(clients),
        services_(services),
        waitables_(waitables) {}

  /**
   * @brief 析构函数 (Destructor)
   */
  ~StaticStorage() = default;

  /**
   * @brief 重建 rcl_wait_set_t 结构 (Rebuild the rcl_wait_set_t structure)
   *
   * @tparam ArrayOfExtraGuardConditions 额外的 GuardCondition 数组类型 (Type of the array of extra
   * GuardConditions)
   * @param[in] extra_guard_conditions 额外的 GuardCondition 数组 (Array of extra GuardConditions)
   */
  template <class ArrayOfExtraGuardConditions>
  void storage_rebuild_rcl_wait_set(const ArrayOfExtraGuardConditions& extra_guard_conditions) {
    // 使用给定的集合重建 rcl_wait_set_t 结构 (Rebuild the rcl_wait_set_t structure with the given
    // sets)
    this->storage_rebuild_rcl_wait_set_with_sets(
        subscriptions_, guard_conditions_, extra_guard_conditions, timers_, clients_, services_,
        waitables_);
  }

  // storage_add_subscription() explicitly not declared here
  // storage_remove_subscription() explicitly not declared here
  // storage_add_guard_condition() explicitly not declared here
  // storage_remove_guard_condition() explicitly not declared here
  // storage_add_timer() explicitly not declared here
  // storage_remove_timer() explicitly not declared here
  // storage_add_client() explicitly not declared here
  // storage_remove_client() explicitly not declared here
  // storage_add_service() explicitly not declared here
  // storage_remove_service() explicitly not declared here
  // storage_add_waitable() explicitly not declared here
  // storage_remove_waitable() explicitly not declared here
  // storage_prune_deleted_entities() explicitly not declared here

  /**
   * @brief storage_acquire_ownerships() 函数声明，该函数在当前实现中不执行任何操作。
   * @brief storage_acquire_ownerships() function declaration, this function does nothing in the
   * current implementation.
   */
  void storage_acquire_ownerships();

  /**
   * @brief storage_release_ownerships() 函数声明，该函数在当前实现中不执行任何操作。
   * @brief storage_release_ownerships() function declaration, this function does nothing in the
   * current implementation.
   */
  void storage_release_ownerships();

  // 订阅列表 (Subscription list)
  const ArrayOfSubscriptions subscriptions_;

  // 守护条件列表 (Guard condition list)
  const ArrayOfGuardConditions guard_conditions_;

  // 定时器列表 (Timer list)
  const ArrayOfTimers timers_;

  // 客户端列表 (Client list)
  const ArrayOfClients clients_;

  // 服务列表 (Service list)
  const ArrayOfServices services_;

  // 可等待对象列表 (Waitable objects list)
  const ArrayOfWaitables waitables_;
};

}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__STATIC_STORAGE_HPP_

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

#ifndef RCLCPP__WAIT_SET_POLICIES__DETAIL__STORAGE_POLICY_COMMON_HPP_
#define RCLCPP__WAIT_SET_POLICIES__DETAIL__STORAGE_POLICY_COMMON_HPP_

#include <memory>
#include <stdexcept>
#include <utility>

#include "rcl/wait.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp {
namespace wait_set_policies {
namespace detail {

/// @brief 存储策略通用结构，提供 rcl 等待集访问。
/// @brief Common structure for storage policies, which provides rcl wait set access.
template <bool HasStrongOwnership>
class StoragePolicyCommon {
protected:
  /// @brief 构造函数，用于初始化存储策略。
  /// @brief Constructor for initializing the storage policy.
  /// @param[in] subscriptions 订阅列表。
  /// @param[in] guard_conditions 守护条件列表。
  /// @param[in] extra_guard_conditions 额外守护条件列表。
  /// @param[in] timers 定时器列表。
  /// @param[in] clients 客户端列表。
  /// @param[in] services 服务列表。
  /// @param[in] waitables 可等待对象列表。
  /// @param[in] context 上下文共享指针。
  ///
  /// @param[in] subscriptions A list of subscriptions.
  /// @param[in] guard_conditions A list of guard conditions.
  /// @param[in] extra_guard_conditions A list of extra guard conditions.
  /// @param[in] timers A list of timers.
  /// @param[in] clients A list of clients.
  /// @param[in] services A list of services.
  /// @param[in] waitables A list of waitable objects.
  /// @param[in] context A shared pointer to the context.
  template <
      class SubscriptionsIterable,
      class GuardConditionsIterable,
      class ExtraGuardConditionsIterable,
      class TimersIterable,
      class ClientsIterable,
      class ServicesIterable,
      class WaitablesIterable>
  explicit StoragePolicyCommon(
      const SubscriptionsIterable &subscriptions,
      const GuardConditionsIterable &guard_conditions,
      const ExtraGuardConditionsIterable &extra_guard_conditions,
      const TimersIterable &timers,
      const ClientsIterable &clients,
      const ServicesIterable &services,
      const WaitablesIterable &waitables,
      rclcpp::Context::SharedPtr context)
      : rcl_wait_set_(rcl_get_zero_initialized_wait_set()), context_(context) {
    // 检查上下文是否为空指针。
    // Check if the context is a nullptr.
    if (nullptr == context) {
      throw std::invalid_argument("context is nullptr");
    }
    // 计算可等待对象的总贡献。
    // Accumulate total contributions from waitables.
    size_t subscriptions_from_waitables = 0;
    size_t guard_conditions_from_waitables = 0;
    size_t timers_from_waitables = 0;
    size_t clients_from_waitables = 0;
    size_t services_from_waitables = 0;
    size_t events_from_waitables = 0;
    for (const auto &waitable_entry : waitables) {
      auto waitable_ptr_pair = get_raw_pointer_from_smart_pointer(waitable_entry.waitable);
      if (nullptr == waitable_ptr_pair.second) {
        if (HasStrongOwnership) {
          throw std::runtime_error("unexpected condition, fixed storage policy needs pruning");
        }
        // 设置修剪标志。
        // Set flag for pruning.
        needs_pruning_ = true;
        continue;
      }

      rclcpp::Waitable &waitable = *waitable_ptr_pair.second;
      subscriptions_from_waitables += waitable.get_number_of_ready_subscriptions();
      guard_conditions_from_waitables += waitable.get_number_of_ready_guard_conditions();
      timers_from_waitables += waitable.get_number_of_ready_timers();
      clients_from_waitables += waitable.get_number_of_ready_clients();
      services_from_waitables += waitable.get_number_of_ready_services();
      events_from_waitables += waitable.get_number_of_ready_events();
    }
    // 使用初始输入初始化等待集。
    // Initialize the wait set using initial inputs.
    rcl_ret_t ret = rcl_wait_set_init(
        &rcl_wait_set_, subscriptions.size() + subscriptions_from_waitables,
        guard_conditions.size() + extra_guard_conditions.size() + guard_conditions_from_waitables,
        timers.size() + timers_from_waitables, clients.size() + clients_from_waitables,
        services.size() + services_from_waitables, events_from_waitables,
        context_->get_rcl_context().get(),
        // TODO(wjwwood): 支持自定义分配器，可能限制为多态分配器
        // TODO(wjwwood): support custom allocator, maybe restrict to polymorphic allocator
        rcl_get_default_allocator());
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }

    // 首次(重)构建 wait set。
    // (Re)build the wait set for the first time.
    this->storage_rebuild_rcl_wait_set_with_sets(
        subscriptions, guard_conditions, extra_guard_conditions, timers, clients, services,
        waitables);
  }

  /**
   * @brief 析构函数，用于释放 rcl_wait_set_ 资源 (Destructor, used to release the resources of
   * rcl_wait_set_)
   */
  ~StoragePolicyCommon() {
    // 释放 rcl_wait_set_ 资源 (Release the resources of rcl_wait_set_)
    rcl_ret_t ret = rcl_wait_set_fini(&rcl_wait_set_);
    // 判断资源释放是否成功 (Check if the resource release is successful)
    if (RCL_RET_OK != ret) {
      try {
        // 抛出异常 (Throw an exception)
        rclcpp::exceptions::throw_from_rcl_error(ret);
      } catch (const std::exception &exception) {
        // 记录错误日志 (Record error log)
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"), "Error in destruction of rcl wait set: %s",
            exception.what());
      }
    }
  }

  /**
   * @brief 获取智能指针中的原始指针 (Get the raw pointer from the smart pointer)
   *
   * @tparam EntityT 模板类型参数 (Template type parameter)
   * @param shared_pointer 共享指针 (Shared pointer)
   * @return std::pair<void *, EntityT *> 包含 nullptr 和原始指针的 pair 对象 (A pair object
   * containing nullptr and the raw pointer)
   */
  template <class EntityT>
  std::pair<void *, EntityT *> get_raw_pointer_from_smart_pointer(
      const std::shared_ptr<EntityT> &shared_pointer) {
    // 返回包含 nullptr 和原始指针的 pair 对象 (Return a pair object containing nullptr and the raw
    // pointer)
    return {nullptr, shared_pointer.get()};
  }

  /**
   * @brief 获取智能指针中的原始指针 (Get the raw pointer from the smart pointer)
   *
   * @tparam EntityT 模板类型参数 (Template type parameter)
   * @param weak_pointer 弱指针 (Weak pointer)
   * @return std::pair<std::shared_ptr<EntityT>, EntityT *> 包含共享指针和原始指针的 pair 对象 (A
   * pair object containing shared pointer and the raw pointer)
   */
  template <class EntityT>
  std::pair<std::shared_ptr<EntityT>, EntityT *> get_raw_pointer_from_smart_pointer(
      const std::weak_ptr<EntityT> &weak_pointer) {
    // 尝试从弱指针获取共享指针 (Try to get a shared pointer from the weak pointer)
    auto shared_pointer = weak_pointer.lock();
    // 返回包含共享指针和原始指针的 pair 对象 (Return a pair object containing shared pointer and
    // the raw pointer)
    return {shared_pointer, shared_pointer.get()};
  }

  /// 重新构建 wait set，为下一次 wait 调用做准备。
  /// Rebuild the wait set, preparing it for the next wait call.
  /**
   * 通过以下方式重建 wait set：
   * The wait set is rebuilt by:
   *
   *   - 如果需要，调整 wait set 的大小；
   *   - resizing the wait set if needed,
   *   - 如果没有通过调整大小来清除，则清除 wait set；
   *   - clearing the wait set if not already done by resizing, and
   *   - 重新添加实体。
   *   - re-adding the entities.
   */
  template <
      class SubscriptionsIterable,
      class GuardConditionsIterable,
      class ExtraGuardConditionsIterable,
      class TimersIterable,
      class ClientsIterable,
      class ServicesIterable,
      class WaitablesIterable>
  void storage_rebuild_rcl_wait_set_with_sets(
      const SubscriptionsIterable &subscriptions,                  // 订阅列表
      const GuardConditionsIterable &guard_conditions,             // 守护条件列表
      const ExtraGuardConditionsIterable &extra_guard_conditions,  // 额外的守护条件列表
      const TimersIterable &timers,                                // 计时器列表
      const ClientsIterable &clients,                              // 客户端列表
      const ServicesIterable &services,                            // 服务列表
      const WaitablesIterable &waitables                           // 可等待对象列表
  ) {
    bool was_resized = false;                                      // 标记是否已调整大小
    // 如果需要，调整 wait set 的大小。
    // Resize the wait set, but only if it needs to be.
    if (needs_resize_) {
      // 如果没有发生变化，使用 rcl_wait_set_resize() 调整大小是一个空操作，
      // 但在此类中跟踪调整大小的需求可以避免每次等待循环时不必要的库调用（rcl
      // 很可能是一个单独的共享库）。 另外，由于静态存储 wait set
      // 永远不需要调整大小，因此它会完全避免对该函数的冗余调用。 累计来自 waitables 的总贡献。
      size_t subscriptions_from_waitables = 0;
      size_t guard_conditions_from_waitables = 0;
      size_t timers_from_waitables = 0;
      size_t clients_from_waitables = 0;
      size_t services_from_waitables = 0;
      size_t events_from_waitables = 0;
      for (const auto &waitable_entry : waitables) {
        auto waitable_ptr_pair = get_raw_pointer_from_smart_pointer(waitable_entry.waitable);
        if (nullptr == waitable_ptr_pair.second) {
          // 在这种情况下，它可能被存储为 weak_ptr，但现在锁定为 nullptr。
          // In this case it was probably stored as a weak_ptr, but is now locking to nullptr.
          if (HasStrongOwnership) {
            // 在固定大小存储中不会发生这种情况，因为它一直保持共享所有权，从不需要修剪。
            // This will not happen in fixed sized storage, as it holds
            // shared ownership the whole time and is never in need of pruning.
            throw std::runtime_error("unexpected condition, fixed storage policy needs pruning");
          }
          // 标记为修剪。
          // Flag for pruning.
          needs_pruning_ = true;
          continue;
        }
        rclcpp::Waitable &waitable = *waitable_ptr_pair.second;
        subscriptions_from_waitables += waitable.get_number_of_ready_subscriptions();
        guard_conditions_from_waitables += waitable.get_number_of_ready_guard_conditions();
        timers_from_waitables += waitable.get_number_of_ready_timers();
        clients_from_waitables += waitable.get_number_of_ready_clients();
        services_from_waitables += waitable.get_number_of_ready_services();
        events_from_waitables += waitable.get_number_of_ready_events();
      }
      rcl_ret_t ret = rcl_wait_set_resize(
          &rcl_wait_set_, subscriptions.size() + subscriptions_from_waitables,
          guard_conditions.size() + extra_guard_conditions.size() + guard_conditions_from_waitables,
          timers.size() + timers_from_waitables, clients.size() + clients_from_waitables,
          services.size() + services_from_waitables, events_from_waitables);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
      was_resized = true;
      // 假设：调用代码确保在将此变量设置为 true 的函数与本函数并发时，不会调用此函数，
      // 无论是使用文档（如 SequentialSychronization 策略的情况），还是使用同步原语（如
      // ThreadSafeSynchronization 策略的情况）。 Assumption: the calling code ensures this function
      // is not called concurrently with functions that set this variable to true, either with
      // documentation (as is the case for the SequentialSychronization policy), or with
      // synchronization primitives (as is the case with the ThreadSafeSynchronization policy).
      needs_resize_ = false;
    }

    // 现在清除 wait set，但仅当它没有调整大小时，因为调整大小也会清除 wait set。
    // Now clear the wait set, but only if it was not resized, as resizing also
    // clears the wait set.
    if (!was_resized) {
      rcl_ret_t ret = rcl_wait_set_clear(&rcl_wait_set_);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }

    // 添加订阅。
    // Add subscriptions.
    for (const auto &subscription_entry : subscriptions) {
      auto subscription_ptr_pair =
          get_raw_pointer_from_smart_pointer(subscription_entry.subscription);
      if (nullptr == subscription_ptr_pair.second) {
        // 在这种情况下，它可能被存储为 weak_ptr，但现在锁定为 nullptr。
        // In this case it was probably stored as a weak_ptr, but is now locking to nullptr.
        if (HasStrongOwnership) {
          // 在固定大小存储中不会发生这种情况，因为它一直保持共享所有权，从不需要修剪。
          // This will not happen in fixed sized storage, as it holds
          // shared ownership the whole time and is never in need of pruning.
          throw std::runtime_error("unexpected condition, fixed storage policy needs pruning");
        }
        // 标记为修剪。
        // Flag for pruning.
        needs_pruning_ = true;
        continue;
      }
      rcl_ret_t ret = rcl_wait_set_add_subscription(
          &rcl_wait_set_, subscription_ptr_pair.second->get_subscription_handle().get(), nullptr);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }

    // 设置通用代码以添加守护条件。
    // Setup common code to add guard_conditions.
    auto add_guard_conditions = [this](const auto &inner_guard_conditions) {
      for (const auto &guard_condition : inner_guard_conditions) {
        auto guard_condition_ptr_pair = get_raw_pointer_from_smart_pointer(guard_condition);
        if (nullptr == guard_condition_ptr_pair.second) {
          // 在这种情况下，它可能被存储为 weak_ptr，但现在锁定为 nullptr。
          // In this case it was probably stored as a weak_ptr, but is now locking to nullptr.
          if (HasStrongOwnership) {
            // 在固定大小存储中不会发生这种情况，因为它一直保持共享所有权，从不需要修剪。
            // This will not happen in fixed sized storage, as it holds
            // shared ownership the whole time and is never in need of pruning.
            throw std::runtime_error("unexpected condition, fixed storage policy needs pruning");
          }
          // 标记为修剪。
          // Flag for pruning.
          needs_pruning_ = true;
          continue;
        }
        rcl_ret_t ret = rcl_wait_set_add_guard_condition(
            &rcl_wait_set_, &guard_condition_ptr_pair.second->get_rcl_guard_condition(), nullptr);
        if (RCL_RET_OK != ret) {
          rclcpp::exceptions::throw_from_rcl_error(ret);
        }
      }
    };

    // 添加守护条件。
    // Add guard conditions.
    add_guard_conditions(guard_conditions);

    // 添加额外的守护条件。
    // Add extra guard conditions.
    add_guard_conditions(extra_guard_conditions);

    // 添加计时器。
    // Add timers.
    for (const auto &timer : timers) {
      auto timer_ptr_pair = get_raw_pointer_from_smart_pointer(timer);
      if (nullptr == timer_ptr_pair.second) {
        // 在这种情况下，它可能被存储为 weak_ptr，但现在锁定为 nullptr。
        // In this case it was probably stored as a weak_ptr, but is now locking to nullptr.
        if (HasStrongOwnership) {
          // 在固定大小存储中不会发生这种情况，因为它一直保持共享所有权，从不需要修剪。
          // This will not happen in fixed sized storage, as it holds
          // shared ownership the whole time and is never in need of pruning.
          throw std::runtime_error("unexpected condition, fixed storage policy needs pruning");
        }
        // 标记为修剪。
        // Flag for pruning.
        needs_pruning_ = true;
        continue;
      }
      rcl_ret_t ret = rcl_wait_set_add_timer(
          &rcl_wait_set_, timer_ptr_pair.second->get_timer_handle().get(), nullptr);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }

    // 添加客户端。
    // Add clients.
    for (const auto &client : clients) {
      // 获取智能指针中的原始指针
      // Get raw pointer from smart pointer
      auto client_ptr_pair = get_raw_pointer_from_smart_pointer(client);

      // 判断原始指针是否为空
      // Check if the raw pointer is nullptr
      if (nullptr == client_ptr_pair.second) {
        // 在这种情况下，它可能被存储为一个 weak_ptr，但现在锁定为 nullptr。
        // In this case it was probably stored as a weak_ptr, but is now locking to nullptr.

        if (HasStrongOwnership) {
          // 这不会发生在固定大小的存储中，因为它一直持有共享所有权，从不需要修剪。
          // This will not happen in fixed sized storage, as it holds
          // shared ownership the whole time and is never in need of pruning.

          throw std::runtime_error("unexpected condition, fixed storage policy needs pruning");
        }

        // 标记为需要修剪
        // Flag for pruning.
        needs_pruning_ = true;
        continue;
      }

      // 将客户端添加到等待集
      // Add client to the wait set
      rcl_ret_t ret = rcl_wait_set_add_client(
          &rcl_wait_set_, client_ptr_pair.second->get_client_handle().get(), nullptr);

      // 检查返回值是否为 RCL_RET_OK
      // Check if return value is RCL_RET_OK
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }

    // 添加服务。
    // Add services.
    for (const auto &service : services) {
      // 获取智能指针中的原始指针
      // Get raw pointer from smart pointer
      auto service_ptr_pair = get_raw_pointer_from_smart_pointer(service);

      // 判断原始指针是否为空
      // Check if the raw pointer is nullptr
      if (nullptr == service_ptr_pair.second) {
        // 在这种情况下，它可能被存储为一个 weak_ptr，但现在锁定为 nullptr。
        // In this case it was probably stored as a weak_ptr, but is now locking to nullptr.

        if (HasStrongOwnership) {
          // 这不会发生在固定大小的存储中，因为它一直持有共享所有权，从不需要修剪。
          // This will not happen in fixed sized storage, as it holds
          // shared ownership the whole time and is never in need of pruning.

          throw std::runtime_error("unexpected condition, fixed storage policy needs pruning");
        }

        // 标记为需要修剪
        // Flag for pruning.
        needs_pruning_ = true;
        continue;
      }

      // 将服务添加到等待集
      // Add service to the wait set
      rcl_ret_t ret = rcl_wait_set_add_service(
          &rcl_wait_set_, service_ptr_pair.second->get_service_handle().get(), nullptr);

      // 检查返回值是否为 RCL_RET_OK
      // Check if return value is RCL_RET_OK
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }

    // 添加 waitables（等待对象）。
    // Add waitables.
    for (auto &waitable_entry : waitables) {
      // 从智能指针中获取原始指针。
      // Get the raw pointer from the smart pointer.
      auto waitable_ptr_pair = get_raw_pointer_from_smart_pointer(waitable_entry.waitable);

      // 如果第二个元素是空指针，说明该对象可能已经被销毁。
      // If the second element is nullptr, it means that the object might have been destroyed.
      if (nullptr == waitable_ptr_pair.second) {
        // 在这种情况下，它可能存储为 weak_ptr，但现在锁定为空指针。
        // In this case, it was probably stored as a weak_ptr, but is now locking to nullptr.

        // 如果拥有强引用权。
        // If has strong ownership.
        if (HasStrongOwnership) {
          // 这种情况不会发生在固定大小的存储中，因为它一直持有共享所有权，永远不需要修剪。
          // This will not happen in fixed sized storage, as it holds
          // shared ownership the whole time and is never in need of pruning.
          throw std::runtime_error("unexpected condition, fixed storage policy needs pruning");
        }

        // 标记为需要修剪。
        // Flag for pruning.
        needs_pruning_ = true;
        continue;
      }

      // 获取 waitable 引用。
      // Get the waitable reference.
      rclcpp::Waitable &waitable = *waitable_ptr_pair.second;

      // 将 waitable 添加到 wait set 中。
      // Add the waitable to the wait set.
      waitable.add_to_wait_set(&rcl_wait_set_);
    }
  }

  /**
   * @brief 获取一个常量引用到 rcl_wait_set_t 结构体的实例。
   * Get a const reference to an instance of the rcl_wait_set_t struct.
   *
   * @return 返回一个 rcl_wait_set_t 的常量引用。
   *         Return a const reference to an rcl_wait_set_t.
   */
  const rcl_wait_set_t &storage_get_rcl_wait_set() const {
    // 返回 rcl_wait_set_ 成员变量的引用
    // Return a reference to the rcl_wait_set_ member variable
    return rcl_wait_set_;
  }

  /**
   * @brief 获取一个引用到 rcl_wait_set_t 结构体的实例。
   * Get a reference to an instance of the rcl_wait_set_t struct.
   *
   * @return 返回一个 rcl_wait_set_t 的引用。
   *         Return a reference to an rcl_wait_set_t.
   */
  rcl_wait_set_t &storage_get_rcl_wait_set() {
    // 返回 rcl_wait_set_ 成员变量的引用
    // Return a reference to the rcl_wait_set_ member variable
    return rcl_wait_set_;
  }

  /**
   * @brief 设置需要调整 wait set 大小的标志。
   * Set the flag for needing to resize the wait set.
   */
  void storage_flag_for_resize() {
    // 将 needs_resize_ 标志设置为 true
    // Set the needs_resize_ flag to true
    needs_resize_ = true;
  }

  // 定义 rcl_wait_set_t 类型的成员变量
  // Define a member variable of type rcl_wait_set_t
  rcl_wait_set_t rcl_wait_set_;

  // 定义一个指向 rclcpp::Context 类型的共享指针成员变量
  // Define a shared pointer member variable to the rclcpp::Context type
  rclcpp::Context::SharedPtr context_;

  // 定义一个布尔型成员变量，表示是否需要进行修剪，默认为 false
  // Define a boolean member variable indicating whether pruning is needed, default to false
  bool needs_pruning_ = false;

  // 定义一个布尔型成员变量，表示是否需要调整 wait set 大小，默认为 false
  // Define a boolean member variable indicating whether resizing the wait set is needed, default to
  // false
  bool needs_resize_ = false;
};

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DETAIL__STORAGE_POLICY_COMMON_HPP_

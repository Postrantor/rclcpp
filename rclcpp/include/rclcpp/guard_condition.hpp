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

#ifndef RCLCPP__GUARD_CONDITION_HPP_
#define RCLCPP__GUARD_CONDITION_HPP_

#include <atomic>

#include "rcl/guard_condition.h"
#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/// \class GuardCondition
/// \brief 一个可以在单个等待集中等待并异步触发的条件。
/// A condition that can be waited on in a single wait set and asynchronously triggered.
class GuardCondition {
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(GuardCondition)

  // TODO(wjwwood): 支持自定义分配器，可能限制为多态分配器
  // TODO(wjwwood): support custom allocator, maybe restrict to polymorphic allocator

  /// \brief 构造守卫条件，可选地指定要使用的上下文。
  /// Construct the guard condition, optionally specifying which Context to use.
  /**
   * \param[in] context 可选的自定义上下文。
   *   默认使用全局默认上下文单例。
   *   与守卫条件共享上下文的所有权，直到析构。
   * \param[in] guard_condition_options 要使用的可选守卫条件选项。
   *   默认使用默认的守卫条件选项。
   * \throws std::invalid_argument 如果上下文为 nullptr。
   * \throws rclcpp::exceptions::RCLError 当底层 rcl 函数失败时抛出基于 RCLError 的异常。
   */
  /// \param[in] context Optional custom context to be used.
  ///   Defaults to using the global default context singleton.
  ///   Shared ownership of the context is held with the guard condition until
  ///   destruction.
  /// \param[in] guard_condition_options Optional guard condition options to be used.
  ///   Defaults to using the default guard condition options.
  /// \throws std::invalid_argument if the context is nullptr.
  /// \throws rclcpp::exceptions::RCLError based exceptions when underlying
  ///   rcl functions fail.
  RCLCPP_PUBLIC
  explicit GuardCondition(
      rclcpp::Context::SharedPtr context = rclcpp::contexts::get_global_default_context(),
      rcl_guard_condition_options_t guard_condition_options =
          rcl_guard_condition_get_default_options());

  /// \brief 守卫条件的虚拟析构函数。
  /// Virtual destructor of the guard condition.
  RCLCPP_PUBLIC
  virtual ~GuardCondition();

  /// \brief 返回创建此守卫条件时使用的上下文。
  /// Return the context used when creating this guard condition.
  RCLCPP_PUBLIC
  rclcpp::Context::SharedPtr get_context() const;

  /// \brief 返回底层的 rcl 守卫条件结构。
  /// Return the underlying rcl guard condition structure.
  RCLCPP_PUBLIC
  rcl_guard_condition_t& get_rcl_guard_condition();

  /// \brief 返回底层的 rcl 守卫条件结构。
  /// Return the underlying rcl guard condition structure.
  RCLCPP_PUBLIC
  const rcl_guard_condition_t& get_rcl_guard_condition() const;

  /// \brief 表示条件已满足，通知等待集和侦听器（如果有）。
  /// Signal that the condition has been met, notifying both the wait set and listeners, if any.
  /**
   * 此函数是线程安全的，并且可以与在等待集中等待此守卫条件同时调用。
   *
   * \throws rclcpp::exceptions::RCLError 当底层 rcl 函数失败时抛出基于 RCLError 的异常。
   */
  /// This function is thread-safe, and may be called concurrently with waiting
  /// on this guard condition in a wait set.
  ///
  /// \throws rclcpp::exceptions::RCLError based exceptions when underlying
  ///   rcl functions fail.
  RCLCPP_PUBLIC
  void trigger();

  /// \brief 交换此守卫条件的“由等待集使用”的状态。
  /// Exchange the "in use by wait set" state for this guard condition.
  /**
   * 此操作用于确保此守卫条件不会同时被多个等待集使用。
   *
   * \param[in] in_use_state 要交换到状态的新状态，true 表示现在由等待集使用，false
   * 表示不再由等待集使用。 \returns 之前的状态。
   */
  /// This is used to ensure this guard condition is not used by multiple
  /// wait sets at the same time.
  ///
  /// \param[in] in_use_state the new state to exchange into the state, true
  ///   indicates it is now in use by a wait set, and false is that it is no
  ///   longer in use by a wait set.
  /// \returns the previous state.
  RCLCPP_PUBLIC
  bool exchange_in_use_by_wait_set_state(bool in_use_state);

  /// \brief 将守卫条件添加到等待集
  /// Adds the guard condition to a waitset
  /**
   * 此函数是线程安全的。
   * \param[in] wait_set 要将守卫条件添加到的等待集的指针
   */
  /// This function is thread-safe.
  /// \param[in] wait_set pointer to a wait set where to add the guard condition
  RCLCPP_PUBLIC
  void add_to_wait_set(rcl_wait_set_t* wait_set);

  /// \brief 设置一个回调，当守卫条件被触发时调用。
  /// Set a callback to be called whenever the guard condition is triggered.
  /**
   * 回调接收一个 size_t 类型，表示自上次调用此回调以来守卫条件被触发的次数。
   * 通常为 1，但如果在设置任何回调之前触发了守卫条件，则可以 > 1。
   *
   * 再次调用将清除之前设置的任何回调。
   *
   * 此函数是线程安全的。
   *
   * 如果您希望在回调中使用更多信息，例如守卫条件或其他信息，您可以使用带捕获的 lambda 或
   * std::bind。
   *
   * \param[in] callback 守卫条件被触发时要调用的函数对象
   */
  RCLCPP_PUBLIC
  void set_on_trigger_callback(std::function<void(size_t)> callback);

protected:
  rclcpp::Context::SharedPtr context_;  ///< 上下文的共享指针。Shared pointer to the context.
  rcl_guard_condition_t rcl_guard_condition_;  ///< rcl 守卫条件结构。rcl guard condition structure.
  ///< 是否正在被等待集使用的原子布尔值。Atomic boolean for in-use by wait set state.
  std::atomic<bool> in_use_by_wait_set_{false};
  std::recursive_mutex reentrant_mutex_;  ///< 可重入互斥锁。Reentrant mutex.
  ///< 触发回调函数。On trigger callback function.
  std::function<void(size_t)> on_trigger_callback_{nullptr};
  size_t unread_count_{0};             ///< 未读计数。Unread count.
  rcl_wait_set_t* wait_set_{nullptr};  ///< 等待集指针。Pointer to the wait set.
};

}  // namespace rclcpp

#endif  // RCLCPP__GUARD_CONDITION_HPP_

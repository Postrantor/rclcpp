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

#include "rclcpp/callback_group.hpp"

#include <algorithm>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "rclcpp/client.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/waitable.hpp"

using rclcpp::CallbackGroup;
using rclcpp::CallbackGroupType;

/**
 * @brief CallbackGroup 构造函数
 * @param group_type 回调组类型（CallbackGroupType）
 * @param automatically_add_to_executor_with_node 是否自动将回调组添加到节点的执行器中（bool）
 *
 * @brief CallbackGroup constructor
 * @param group_type The callback group type (CallbackGroupType)
 * @param automatically_add_to_executor_with_node Whether to automatically add the callback group to
 * the node's executor (bool)
 */
CallbackGroup::CallbackGroup(
    CallbackGroupType group_type, bool automatically_add_to_executor_with_node)
    : type_(group_type),
      associated_with_executor_(false),
      can_be_taken_from_(true),
      automatically_add_to_executor_with_node_(automatically_add_to_executor_with_node) {}

/**
 * @brief CallbackGroup 析构函数，触发通知保护条件
 *
 * @brief CallbackGroup destructor, triggers notify guard condition
 */
CallbackGroup::~CallbackGroup() { trigger_notify_guard_condition(); }

/**
 * @brief 获取 can_be_taken_from_ 的原子引用
 * @return std::atomic_bool& 原子布尔引用
 *
 * @brief Get atomic reference of can_be_taken_from_
 * @return std::atomic_bool& Atomic boolean reference
 */
std::atomic_bool &CallbackGroup::can_be_taken_from() { return can_be_taken_from_; }

/**
 * @brief 获取回调组类型
 * @return const CallbackGroupType& 回调组类型常量引用
 *
 * @brief Get the callback group type
 * @return const CallbackGroupType& Constant reference to the callback group type
 */
const CallbackGroupType &CallbackGroup::type() const { return type_; }

/**
 * @brief 收集所有指针并对其执行相应的函数
 * @param sub_func 订阅指针的函数
 * @param service_func 服务指针的函数
 * @param client_func 客户端指针的函数
 * @param timer_func 计时器指针的函数
 * @param waitable_func 可等待对象指针的函数
 *
 * @brief Collect all pointers and execute the corresponding functions on them
 * @param sub_func Function for subscription pointers
 * @param service_func Function for service pointers
 * @param client_func Function for client pointers
 * @param timer_func Function for timer pointers
 * @param waitable_func Function for waitable object pointers
 */
void CallbackGroup::collect_all_ptrs(
    std::function<void(const rclcpp::SubscriptionBase::SharedPtr &)> sub_func,
    std::function<void(const rclcpp::ServiceBase::SharedPtr &)> service_func,
    std::function<void(const rclcpp::ClientBase::SharedPtr &)> client_func,
    std::function<void(const rclcpp::TimerBase::SharedPtr &)> timer_func,
    std::function<void(const rclcpp::Waitable::SharedPtr &)> waitable_func) const {
  std::lock_guard<std::mutex> lock(mutex_);

  // 遍历订阅指针并执行 sub_func 函数
  // Iterate through subscription pointers and execute sub_func function
  for (const rclcpp::SubscriptionBase::WeakPtr &weak_ptr : subscription_ptrs_) {
    rclcpp::SubscriptionBase::SharedPtr ref_ptr = weak_ptr.lock();
    if (ref_ptr) {
      sub_func(ref_ptr);
    }
  }

  // 遍历服务指针并执行 service_func 函数
  // Iterate through service pointers and execute service_func function
  for (const rclcpp::ServiceBase::WeakPtr &weak_ptr : service_ptrs_) {
    rclcpp::ServiceBase::SharedPtr ref_ptr = weak_ptr.lock();
    if (ref_ptr) {
      service_func(ref_ptr);
    }
  }

  // 遍历客户端指针并执行 client_func 函数
  // Iterate through client pointers and execute client_func function
  for (const rclcpp::ClientBase::WeakPtr &weak_ptr : client_ptrs_) {
    rclcpp::ClientBase::SharedPtr ref_ptr = weak_ptr.lock();
    if (ref_ptr) {
      client_func(ref_ptr);
    }
  }

  // 遍历计时器指针并执行 timer_func 函数
  // Iterate through timer pointers and execute timer_func function
  for (const rclcpp::TimerBase::WeakPtr &weak_ptr : timer_ptrs_) {
    rclcpp::TimerBase::SharedPtr ref_ptr = weak_ptr.lock();
    if (ref_ptr) {
      timer_func(ref_ptr);
    }
  }

  // 遍历可等待对象指针并执行 waitable_func 函数
  // Iterate through waitable object pointers and execute waitable_func function
  for (const rclcpp::Waitable::WeakPtr &weak_ptr : waitable_ptrs_) {
    rclcpp::Waitable::SharedPtr ref_ptr = weak_ptr.lock();
    if (ref_ptr) {
      waitable_func(ref_ptr);
    }
  }
}

/**
 * @brief 获取与执行器关联的原子引用
 * @return std::atomic_bool& 原子布尔引用
 *
 * @brief Get atomic reference associated with the executor
 * @return std::atomic_bool& Atomic boolean reference
 */
std::atomic_bool &CallbackGroup::get_associated_with_executor_atomic() {
  return associated_with_executor_;
}

/**
 * @brief 检查回调组是否自动添加到节点的执行器中
 * @return bool 是否自动添加到节点的执行器中
 *
 * @brief Check if the callback group is automatically added to the node's executor
 * @return bool Whether it is automatically added to the node's executor
 */
bool CallbackGroup::automatically_add_to_executor_with_node() const {
  return automatically_add_to_executor_with_node_;
}

/**
 * @brief 获取通知保护条件
 * @param context_ptr 上下文智能指针
 * @return rclcpp::GuardCondition::SharedPtr 保护条件智能指针
 *
 * @brief Get notify guard condition
 * @param context_ptr Shared pointer to the context
 * @return rclcpp::GuardCondition::SharedPtr Guard condition shared pointer
 */
rclcpp::GuardCondition::SharedPtr CallbackGroup::get_notify_guard_condition(
    const rclcpp::Context::SharedPtr context_ptr) {
  std::lock_guard<std::recursive_mutex> lock(notify_guard_condition_mutex_);
  if (notify_guard_condition_ && context_ptr != notify_guard_condition_->get_context()) {
    if (associated_with_executor_) {
      trigger_notify_guard_condition();
    }
    notify_guard_condition_ = nullptr;
  }

  if (!notify_guard_condition_) {
    notify_guard_condition_ = std::make_shared<rclcpp::GuardCondition>(context_ptr);
  }

  return notify_guard_condition_;
}

/**
 * @brief 触发通知保护条件
 *
 * @brief Trigger notify guard condition
 */
void CallbackGroup::trigger_notify_guard_condition() {
  std::lock_guard<std::recursive_mutex> lock(notify_guard_condition_mutex_);
  if (notify_guard_condition_) {
    notify_guard_condition_->trigger();
  }
}

// 下面的函数都遵循类似的模式，向回调组添加或删除订阅、计时器、服务、客户端和可等待对象。
// 请参考 add_subscription 函数的注释以了解其工作原理。
//
// The following functions follow a similar pattern, adding or removing subscriptions, timers,
// services, clients, and waitables to/from the callback group.
// Refer to the comments on the add_subscription function for an understanding of how they work.

/**
 * @brief 向回调组添加订阅
 * @param subscription_ptr 订阅的智能指针（rclcpp::SubscriptionBase::SharedPtr）
 *
 * @brief Add subscription to the callback group
 * @param subscription_ptr Shared pointer to the subscription (rclcpp::SubscriptionBase::SharedPtr)
 */
void CallbackGroup::add_subscription(const rclcpp::SubscriptionBase::SharedPtr subscription_ptr) {
  // 使用互斥锁保护订阅指针列表，防止多线程问题
  // Use a mutex lock to protect the subscription pointers list, preventing multi-threading issues
  std::lock_guard<std::mutex> lock(mutex_);

  // 将新的订阅指针添加到订阅指针列表中
  // Add the new subscription pointer to the subscription pointers list
  subscription_ptrs_.push_back(subscription_ptr);

  // 移除已过期的订阅指针
  // Remove expired subscription pointers
  subscription_ptrs_.erase(
      std::remove_if(
          subscription_ptrs_.begin(), subscription_ptrs_.end(),
          [](rclcpp::SubscriptionBase::WeakPtr x) { return x.expired(); }),  // 检查订阅指针是否过期
      subscription_ptrs_.end());
}

/**
 * @brief 添加定时器到回调组 (Add a timer to the callback group)
 *
 * @param timer_ptr 指向需要添加的定时器的共享指针 (Shared pointer to the timer to be added)
 */
void CallbackGroup::add_timer(const rclcpp::TimerBase::SharedPtr timer_ptr) {
  // 使用 std::lock_guard 对互斥锁上锁，保证线程安全
  // Lock the mutex using std::lock_guard to ensure thread safety
  std::lock_guard<std::mutex> lock(mutex_);

  // 将传入的定时器共享指针添加到 timer_ptrs_ 容器中
  // Add the incoming timer shared pointer to the timer_ptrs_ container
  timer_ptrs_.push_back(timer_ptr);

  // 删除 timer_ptrs_ 中已过期的弱指针
  // Remove expired weak pointers from timer_ptrs_
  timer_ptrs_.erase(
      // 使用 std::remove_if 算法寻找并删除过期的弱指针
      // Use the std::remove_if algorithm to find and remove expired weak pointers
      std::remove_if(
          timer_ptrs_.begin(), timer_ptrs_.end(),
          // 使用 lambda 表达式作为谓词函数，检查弱指针是否过期
          // Use a lambda expression as a predicate function to check if the weak pointer has
          // expired
          [](rclcpp::TimerBase::WeakPtr x) { return x.expired(); }),
      // 更新 timer_ptrs_ 的末尾迭代器
      // Update the end iterator of timer_ptrs_
      timer_ptrs_.end());
}

/**
 * @brief 添加一个服务到回调组中 (Add a service to the callback group)
 *
 * @param service_ptr 一个共享指针，指向要添加的服务对象 (A shared pointer pointing to the service
 * object to be added)
 */
void CallbackGroup::add_service(const rclcpp::ServiceBase::SharedPtr service_ptr) {
  // 使用 std::lock_guard 对互斥量进行加锁，以保证线程安全
  // (Lock the mutex using std::lock_guard to ensure thread safety)
  std::lock_guard<std::mutex> lock(mutex_);

  // 将服务指针添加到服务指针列表中
  // (Add the service pointer to the list of service pointers)
  service_ptrs_.push_back(service_ptr);

  // 移除服务指针列表中已经过期（失效）的弱指针，并重新计算列表末尾
  // (Remove expired (invalid) weak pointers from the list of service pointers and recalculate the
  // end of the list)
  service_ptrs_.erase(
      // 从服务指针列表的开始到结束，使用 std::remove_if 算法移除过期的弱指针
      // (From the beginning to the end of the service pointer list, use the std::remove_if
      // algorithm to remove expired weak pointers)
      std::remove_if(
          service_ptrs_.begin(), service_ptrs_.end(),
          // 使用 lambda 函数作为谓词，检查每个弱指针是否已经过期（失效）
          // (Use a lambda function as a predicate to check if each weak pointer has expired
          // (invalid))
          [](rclcpp::ServiceBase::WeakPtr x) { return x.expired(); }),
      // 更新服务指针列表的结束位置
      // (Update the end position of the service pointer list)
      service_ptrs_.end());
}

void CallbackGroup::add_client(const rclcpp::ClientBase::SharedPtr client_ptr) {
  std::lock_guard<std::mutex> lock(mutex_);
  client_ptrs_.push_back(client_ptr);
  client_ptrs_.erase(
      std::remove_if(
          client_ptrs_.begin(), client_ptrs_.end(),
          [](rclcpp::ClientBase::WeakPtr x) { return x.expired(); }),
      client_ptrs_.end());
}

void CallbackGroup::add_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr) {
  std::lock_guard<std::mutex> lock(mutex_);
  waitable_ptrs_.push_back(waitable_ptr);
  waitable_ptrs_.erase(
      std::remove_if(
          waitable_ptrs_.begin(), waitable_ptrs_.end(),
          [](rclcpp::Waitable::WeakPtr x) { return x.expired(); }),
      waitable_ptrs_.end());
}

void CallbackGroup::remove_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto iter = waitable_ptrs_.begin(); iter != waitable_ptrs_.end(); ++iter) {
    const auto shared_ptr = iter->lock();
    if (shared_ptr.get() == waitable_ptr.get()) {
      waitable_ptrs_.erase(iter);
      break;
    }
  }
}

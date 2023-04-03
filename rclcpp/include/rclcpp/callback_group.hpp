// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CALLBACK_GROUP_HPP_
#define RCLCPP__CALLBACK_GROUP_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

#include "rclcpp/client.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp {

// 前向声明，用于类 CallbackGroup 中的 friend 语句
// Forward declarations for friend statement in class CallbackGroup
namespace node_interfaces {
// NodeServices 类的前向声明
// Forward declaration of the NodeServices class
class NodeServices;

// NodeTimers 类的前向声明
// Forward declaration of the NodeTimers class
class NodeTimers;

// NodeTopics 类的前向声明
// Forward declaration of the NodeTopics class
class NodeTopics;

// NodeWaitables 类的前向声明
// Forward declaration of the NodeWaitables class
}  // namespace node_interfaces

// 定义一个枚举类 CallbackGroupType，包含两个成员：MutuallyExclusive 和 Reentrant
// Define an enumeration class CallbackGroupType with two members: MutuallyExclusive and Reentrant
enum class CallbackGroupType { MutuallyExclusive, Reentrant };

class CallbackGroup {
  friend class rclcpp::node_interfaces::NodeServices;
  friend class rclcpp::node_interfaces::NodeTimers;
  friend class rclcpp::node_interfaces::NodeTopics;
  friend class rclcpp::node_interfaces::NodeWaitables;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(CallbackGroup)

  /// 构造函数，用于创建 CallbackGroup。
  /// Constructor for CallbackGroup.
  /**
   * Callback Group 有两种类型：'Mutually Exclusive（互斥）' 和 'Reentrant（可重入）'，
   * 在创建时必须指定类型。
   * Callback Groups have a type, either 'Mutually Exclusive' or 'Reentrant'
   * and when creating one the type must be specified.
   *
   * 可重入 Callback Group 中的回调函数必须满足以下条件：
   * Callbacks in Reentrant Callback Groups must be able to:
   *   - 同时运行多次（可重入）
   *   - run at the same time as themselves (reentrant)
   *   - 与组内其他回调同时运行
   *   - run at the same time as other callbacks in their group
   *   - 与其他组中的回调同时运行
   *   - run at the same time as other callbacks in other groups
   *
   * 互斥 Callback Group 中的回调函数：
   * Callbacks in Mutually Exclusive Callback Groups:
   *   - 不会同时运行多次（非重入）
   *   - will not be run multiple times simultaneously (non-reentrant)
   *   - 不会与组内其他回调同时运行
   *   - will not be run at the same time as other callbacks in their group
   *   - 必须与其他组中的回调同时运行
   *   - but must run at the same time as callbacks in other groups
   *
   * 此外，callback group 还具有一个属性，该属性决定了它们是否在与关联节点一起自动添加到执行器中。
   * Additionally, callback groups have a property which determines whether or
   * not they are added to an executor with their associated node automatically.
   * 在创建 callback group 时，automatically_add_to_executor_with_node 参数决定了这种行为，
   * 如果为 true，则在使用 Executor::add_node 方法时，新创建的 callback group
   * 将与节点一起添加到执行器中。 When creating a callback group the
   * automatically_add_to_executor_with_node argument determines this behavior, and if true it will
   * cause the newly created callback group to be added to an executor with the node when the
   * Executor::add_node method is used.
   * 如果为 false，则此 callback group 不会自动添加，需要使用 Executor::add_callback_group
   * 方法手动将其添加到执行器中。 If false, this callback group will not be added automatically and
   * would have to be added to an executor manually using the Executor::add_callback_group method.
   *
   * 在创建 callback group 之前或之后将节点添加到执行器中都无关紧要；在这两种情况下，callback group
   * 都会自动添加到执行器中。 Whether the node was added to the executor before creating the
   * callback group, or after, is irrelevant; the callback group will be automatically added to the
   * executor in either case.
   *
   * \param[in] group_type 回调组的类型。
   * \param[in] group_type The type of the callback group.
   * \param[in] automatically_add_to_executor_with_node
   * 一个布尔值，用于确定回调组是否与关联节点一起自动添加到执行器中。 \param[in]
   * automatically_add_to_executor_with_node A boolean that determines whether a callback group is
   * automatically added to an executor with the node with which it is associated.
   */
  RCLCPP_PUBLIC
  explicit CallbackGroup(
      CallbackGroupType group_type, bool automatically_add_to_executor_with_node = true);

  /// Default destructor.
  RCLCPP_PUBLIC
  ~CallbackGroup();

  /**
   * @brief 查找符合条件的订阅指针 (Find subscription pointers that meet the condition)
   *
   * @tparam Function 函数类型，用于判断是否满足条件 (Function type for determining if the condition
   * is met)
   * @param func 用于判断是否满足条件的函数 (Function to determine if the condition is met)
   * @return rclcpp::SubscriptionBase::SharedPtr 返回符合条件的订阅指针 (Return subscription pointer
   * that meets the condition)
   */
  template <typename Function>
  rclcpp::SubscriptionBase::SharedPtr find_subscription_ptrs_if(Function func) const {
    // 调用实现函数，传入订阅指针列表 (Call the implementation function and pass in the subscription
    // pointer list)
    return _find_ptrs_if_impl<rclcpp::SubscriptionBase, Function>(func, subscription_ptrs_);
  }

  /**
   * @brief 查找符合条件的定时器指针 (Find timer pointers that meet the condition)
   *
   * @tparam Function 函数类型，用于判断是否满足条件 (Function type for determining if the condition
   * is met)
   * @param func 用于判断是否满足条件的函数 (Function to determine if the condition is met)
   * @return rclcpp::TimerBase::SharedPtr 返回符合条件的定时器指针 (Return timer pointer that meets
   * the condition)
   */
  template <typename Function>
  rclcpp::TimerBase::SharedPtr find_timer_ptrs_if(Function func) const {
    // 调用实现函数，传入定时器指针列表 (Call the implementation function and pass in the timer
    // pointer list)
    return _find_ptrs_if_impl<rclcpp::TimerBase, Function>(func, timer_ptrs_);
  }

  /**
   * @brief 查找符合条件的服务指针 (Find service pointers that meet the condition)
   *
   * @tparam Function 函数类型，用于判断是否满足条件 (Function type for determining if the condition
   * is met)
   * @param func 用于判断是否满足条件的函数 (Function to determine if the condition is met)
   * @return rclcpp::ServiceBase::SharedPtr 返回符合条件的服务指针 (Return service pointer that
   * meets the condition)
   */
  template <typename Function>
  rclcpp::ServiceBase::SharedPtr find_service_ptrs_if(Function func) const {
    // 调用实现函数，传入服务指针列表 (Call the implementation function and pass in the service
    // pointer list)
    return _find_ptrs_if_impl<rclcpp::ServiceBase, Function>(func, service_ptrs_);
  }

  /**
   * @brief 查找符合条件的客户端指针 (Find client pointers that meet the condition)
   *
   * @tparam Function 函数类型，用于判断是否满足条件 (Function type for determining if the condition
   * is met)
   * @param func 用于判断是否满足条件的函数 (Function to determine if the condition is met)
   * @return rclcpp::ClientBase::SharedPtr 返回符合条件的客户端指针 (Return client pointer that
   * meets the condition)
   */
  template <typename Function>
  rclcpp::ClientBase::SharedPtr find_client_ptrs_if(Function func) const {
    // 调用实现函数，传入客户端指针列表 (Call the implementation function and pass in the client
    // pointer list)
    return _find_ptrs_if_impl<rclcpp::ClientBase, Function>(func, client_ptrs_);
  }

  /**
   * @brief 查找符合条件的可等待对象指针 (Find waitable pointers that meet the condition)
   *
   * @tparam Function 函数类型，用于判断是否满足条件 (Function type for determining if the condition
   * is met)
   * @param func 用于判断是否满足条件的函数 (Function to determine if the condition is met)
   * @return rclcpp::Waitable::SharedPtr 返回符合条件的可等待对象指针 (Return waitable pointer that
   * meets the condition)
   */
  template <typename Function>
  rclcpp::Waitable::SharedPtr find_waitable_ptrs_if(Function func) const {
    // 调用实现函数，传入可等待对象指针列表 (Call the implementation function and pass in the
    // waitable pointer list)
    return _find_ptrs_if_impl<rclcpp::Waitable, Function>(func, waitable_ptrs_);
  }

  /**
   * @brief 获取 can_be_taken_from 的引用
   * @return std::atomic_bool& - 可以从中获取的原子布尔值的引用
   *
   * @brief Get the reference of can_be_taken_from
   * @return std::atomic_bool& - The reference to the atomic boolean that can be taken from
   */
  RCLCPP_PUBLIC
  std::atomic_bool &can_be_taken_from();

  /**
   * @brief 获取回调组类型
   * @return const CallbackGroupType& - 回调组类型的常量引用
   *
   * @brief Get the callback group type
   * @return const CallbackGroupType& - The constant reference to the callback group type
   */
  RCLCPP_PUBLIC
  const CallbackGroupType &type() const;

  /**
   * @brief 收集所有指针并对每个指针执行相应的函数
   * @param[in] sub_func - 一个函数，将在每个订阅指针上调用
   * @param[in] service_func - 一个函数，将在每个服务指针上调用
   * @param[in] client_func - 一个函数，将在每个客户端指针上调用
   * @param[in] timer_func - 一个函数，将在每个计时器指针上调用
   * @param[in] waitable_func - 一个函数，将在每个可等待指针上调用
   *
   * @brief Collect all pointers and perform the corresponding function on each pointer
   * @param[in] sub_func - A function that will be called on each subscription pointer
   * @param[in] service_func - A function that will be called on each service pointer
   * @param[in] client_func - A function that will be called on each client pointer
   * @param[in] timer_func - A function that will be called on each timer pointer
   * @param[in] waitable_func - A function that will be called on each waitable pointer
   */
  RCLCPP_PUBLIC
  void collect_all_ptrs(
      std::function<void(const rclcpp::SubscriptionBase::SharedPtr &)> sub_func,
      std::function<void(const rclcpp::ServiceBase::SharedPtr &)> service_func,
      std::function<void(const rclcpp::ClientBase::SharedPtr &)> client_func,
      std::function<void(const rclcpp::TimerBase::SharedPtr &)> timer_func,
      std::function<void(const rclcpp::Waitable::SharedPtr &)> waitable_func) const;

  /// 返回一个指向“与执行器关联”的原子布尔值的引用。
  /// Return a reference to the 'associated with executor' atomic boolean.
  /**
   * 当回调组被添加到执行器时，检查此布尔值
   * 以确保它尚未添加到另一个执行器。
   * 如果尚未添加，则将此布尔值设置为 true，表示现在
   * 与执行器关联。
   * When a callback group is added to an executor, this boolean is checked
   * to ensure it has not already been added to another executor.
   * If it has not been, then this boolean is set to true to indicate it is
   * now associated with an executor.
   *
   * 当从执行器中删除回调组时，此原子布尔值
   * 被设置回 false。
   * When the callback group is removed from the executor, this atomic boolean
   * is set back to false.
   *
   * \return “与执行器关联”的原子布尔值
   * \return the 'associated with executor' atomic boolean
   */
  RCLCPP_PUBLIC
  std::atomic_bool &get_associated_with_executor_atomic();

  /// 如果节点应自动将此回调组添加到执行器，则返回 true。
  /// Return true if this callback group should be automatically added to an executor by the node.
  /**
   * \return 如果在添加关联节点时，此回调组应自动添加
   *   到执行器，则为 true，否则为 false。
   * \return boolean true if this callback group should be automatically added
   *   to an executor when the associated node is added, otherwise false.
   */
  RCLCPP_PUBLIC
  bool automatically_add_to_executor_with_node() const;

  /// 推迟创建通知保护条件并返回它。
  /// Defer creating the notify guard condition and return it.
  RCLCPP_PUBLIC
  rclcpp::GuardCondition::SharedPtr get_notify_guard_condition(
      const rclcpp::Context::SharedPtr context_ptr);

  /**
   * @brief 触发通知保护条件 (Trigger the notify guard condition)
   *
   * 这个函数用于触发通知保护条件，它在 ROS2 项目的 rclcpp 中使用。
   * (This function is used to trigger the notify guard condition, and it is used in the ROS2
   * project's rclcpp.)
   */
  RCLCPP_PUBLIC
  void trigger_notify_guard_condition();

protected:
  /// \class CallbackGroup
  /// \brief 禁用 CallbackGroup 类的拷贝功能 (Disable copy functionality for the CallbackGroup
  /// class)
  RCLCPP_DISABLE_COPY(CallbackGroup)

  /// \fn void add_publisher(const rclcpp::PublisherBase::SharedPtr publisher_ptr)
  /// \brief 添加一个发布者到回调组 (Add a publisher to the callback group)
  /// \param publisher_ptr 发布者的共享指针 (Shared pointer to the publisher)
  RCLCPP_PUBLIC
  void add_publisher(const rclcpp::PublisherBase::SharedPtr publisher_ptr);

  /// \fn void add_subscription(const rclcpp::SubscriptionBase::SharedPtr subscription_ptr)
  /// \brief 添加一个订阅者到回调组 (Add a subscription to the callback group)
  /// \param subscription_ptr 订阅者的共享指针 (Shared pointer to the subscription)
  RCLCPP_PUBLIC
  void add_subscription(const rclcpp::SubscriptionBase::SharedPtr subscription_ptr);

  /// \fn void add_timer(const rclcpp::TimerBase::SharedPtr timer_ptr)
  /// \brief 添加一个定时器到回调组 (Add a timer to the callback group)
  /// \param timer_ptr 定时器的共享指针 (Shared pointer to the timer)
  RCLCPP_PUBLIC
  void add_timer(const rclcpp::TimerBase::SharedPtr timer_ptr);

  /// \fn void add_service(const rclcpp::ServiceBase::SharedPtr service_ptr)
  /// \brief 添加一个服务到回调组 (Add a service to the callback group)
  /// \param service_ptr 服务的共享指针 (Shared pointer to the service)
  RCLCPP_PUBLIC
  void add_service(const rclcpp::ServiceBase::SharedPtr service_ptr);

  /// \fn void add_client(const rclcpp::ClientBase::SharedPtr client_ptr)
  /// \brief 添加一个客户端到回调组 (Add a client to the callback group)
  /// \param client_ptr 客户端的共享指针 (Shared pointer to the client)
  RCLCPP_PUBLIC
  void add_client(const rclcpp::ClientBase::SharedPtr client_ptr);

  /// \fn void add_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr)
  /// \brief 添加一个可等待对象到回调组 (Add a waitable object to the callback group)
  /// \param waitable_ptr 可等待对象的共享指针 (Shared pointer to the waitable object)
  RCLCPP_PUBLIC
  void add_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr);

  /// \fn void remove_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr) noexcept
  /// \brief 从回调组中移除一个可等待对象 (Remove a waitable object from the callback group)
  /// \param waitable_ptr 要移除的可等待对象的共享指针 (Shared pointer to the waitable object to be
  /// removed)
  RCLCPP_PUBLIC
  void remove_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr) noexcept;

  /**
   * @brief CallbackGroup 类的成员变量定义
   *
   * @param type_ 回调组类型 (Callback group type)
   * @param mutex_ 保护以下指针向量的互斥锁 (Mutex to protect the subsequent vectors of pointers)
   * @param associated_with_executor_ 原子布尔值，表示是否与执行器关联 (Atomic bool indicating
   * association with an executor)
   * @param subscription_ptrs_ 订阅者对象弱指针向量 (Vector of weak pointers to Subscription
   * objects)
   * @param timer_ptrs_ 定时器对象弱指针向量 (Vector of weak pointers to Timer objects)
   * @param service_ptrs_ 服务对象弱指针向量 (Vector of weak pointers to Service objects)
   * @param client_ptrs_ 客户端对象弱指针向量 (Vector of weak pointers to Client objects)
   * @param waitable_ptrs_ 可等待对象弱指针向量 (Vector of weak pointers to Waitable objects)
   * @param can_be_taken_from_ 原子布尔值，表示是否可以从中获取 (Atomic bool indicating if it can be
   * taken from)
   * @param automatically_add_to_executor_with_node_ 布尔值，表示是否在节点添加到执行器时自动添加
   * (Bool indicating automatic addition when a node is added to an executor)
   * @param notify_guard_condition_ 通知保护条件的共享指针，延迟创建 (Shared pointer to the notify
   * guard condition, deferring creation)
   * @param notify_guard_condition_mutex_ 通知保护条件的递归互斥锁 (Recursive mutex for the notify
   * guard condition)
   */
  CallbackGroupType type_;
  mutable std::mutex
      mutex_;  // 互斥锁，用于保护以下指针向量 (Mutex to protect the subsequent vectors of pointers)
  std::atomic_bool associated_with_executor_;  // 原子布尔值，表示是否与执行器关联 (Atomic bool
                                               // indicating association with an executor)
  std::vector<rclcpp::SubscriptionBase::WeakPtr>
      subscription_ptrs_;  // 订阅者对象弱指针向量 (Vector of weak pointers to Subscription objects)
  std::vector<rclcpp::TimerBase::WeakPtr>
      timer_ptrs_;    // 定时器对象弱指针向量 (Vector of weak pointers to Timer objects)
  std::vector<rclcpp::ServiceBase::WeakPtr>
      service_ptrs_;  // 服务对象弱指针向量 (Vector of weak pointers to Service objects)
  std::vector<rclcpp::ClientBase::WeakPtr>
      client_ptrs_;   // 客户端对象弱指针向量 (Vector of weak pointers to Client objects)
  std::vector<rclcpp::Waitable::WeakPtr>
      waitable_ptrs_;  // 可等待对象弱指针向量 (Vector of weak pointers to Waitable objects)
  std::atomic_bool can_be_taken_from_;  // 原子布尔值，表示是否可以从中获取 (Atomic bool indicating
                                        // if it can be taken from)
  const bool
      automatically_add_to_executor_with_node_;  // 布尔值，表示是否在节点添加到执行器时自动添加
                                                 // (Bool indicating automatic addition when
                                                 // a node is added to an executor)
  std::shared_ptr<rclcpp::GuardCondition> notify_guard_condition_ =
      nullptr;  // 通知保护条件的共享指针，延迟创建 (Shared pointer to the notify guard condition,
                // deferring creation)
  std::recursive_mutex notify_guard_condition_mutex_;  // 通知保护条件的递归互斥锁 (Recursive mutex
                                                       // for the notify guard condition)

private:
  /**
   * @brief 在给定的弱指针向量中查找满足特定条件的共享指针 (Find a shared pointer that meets
   * specific conditions in the given weak pointer vector)
   *
   * @tparam TypeT 类型参数，用于表示弱指针和共享指针的类型 (Type parameter, used to represent the
   * type of weak pointer and shared pointer)
   * @tparam Function 函数类型参数，用于表示传入的函数对象 (Function type parameter, used to
   * represent the incoming function object)
   * @param func 一个函数对象，用于判断给定的共享指针是否满足特定条件 (A function object, used to
   * determine whether the given shared pointer meets specific conditions)
   * @param vect_ptrs 一个包含弱指针的向量 (A vector containing weak pointers)
   * @return 如果找到满足条件的共享指针，则返回该共享指针；否则返回空共享指针 (If a shared pointer
   * that meets the conditions is found, return the shared pointer; otherwise, return an empty
   * shared pointer)
   */
  template <typename TypeT, typename Function>
  typename TypeT::SharedPtr _find_ptrs_if_impl(
      Function func, const std::vector<typename TypeT::WeakPtr> &vect_ptrs) const {
    // 使用 lock_guard 对互斥量加锁以确保线程安全 (Use lock_guard to lock the mutex to ensure thread
    // safety)
    std::lock_guard<std::mutex> lock(mutex_);

    // 遍历向量中的每个弱指针 (Iterate through each weak pointer in the vector)
    for (auto &weak_ptr : vect_ptrs) {
      // 尝试将弱指针升级为共享指针 (Try to upgrade the weak pointer to a shared pointer)
      auto ref_ptr = weak_ptr.lock();

      // 如果共享指针不为空且满足传入的函数条件，则返回该共享指针 (If the shared pointer is not
      // empty and meets the conditions of the incoming function, return the shared pointer)
      if (ref_ptr && func(ref_ptr)) {
        return ref_ptr;
      }
    }

    // 如果没有找到满足条件的共享指针，则返回空共享指针 (If no shared pointer that meets the
    // conditions is found, return an empty shared pointer)
    return typename TypeT::SharedPtr();
  }
};

}  // namespace rclcpp

#endif  // RCLCPP__CALLBACK_GROUP_HPP_

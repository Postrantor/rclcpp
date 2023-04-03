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

#ifndef RCLCPP__WAIT_SET_HPP_
#define RCLCPP__WAIT_SET_HPP_

#include <memory>

#include "rcl/wait.h"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_set_policies/dynamic_storage.hpp"
#include "rclcpp/wait_set_policies/sequential_synchronization.hpp"
#include "rclcpp/wait_set_policies/static_storage.hpp"
#include "rclcpp/wait_set_policies/thread_safe_synchronization.hpp"
#include "rclcpp/wait_set_template.hpp"

namespace rclcpp {

/// 最常见的用户配置 WaitSet，它是动态的但不是线程安全的。
/// Most common user configuration of a WaitSet, which is dynamic but not thread-safe.
/**
 * 此 wait set 允许您动态添加和删除项目，每次调用 wait() 或 prune_destroyed_entities() 时，
 * 它会自动删除超出范围的项目。
 * This wait set allows you to add and remove items dynamically, and it will
 * automatically remove items that are let out of scope each time wait() or
 * prune_destroyed_entities() is called.
 *
 * 然而，它不会在等待时为添加和删除实体提供线程安全性。
 * It will not, however, provide thread-safety for adding and removing entities
 * while waiting.
 *
 * \sa rclcpp::WaitSetTemplate for API documentation
 */
using WaitSet = rclcpp::WaitSetTemplate<
    rclcpp::wait_set_policies::SequentialSynchronization,
    rclcpp::wait_set_policies::DynamicStorage>;

/// WaitSet 配置在构造后不允许更改。
/// WaitSet configuration which does not allow changes after construction.
/**
 * 此 wait set 要求您在构造时指定所有实体，并阻止您调用典型的添加和删除功能。
 * This wait set requires that you specify all entities at construction, and
 * prevents you from calling the typical add and remove functions.
 * 它还要求您将每个项目的数量作为模板参数指定。
 * It also requires that you specify how many of each item there will be as a
 * template argument.
 *
 * 它将共享实体的所有权，直到被销毁，因此只要 wait set 存在，即使用户让他们的共享指针超出范围，
 * 它也会阻止实体的销毁。
 * It will share ownership of the entities until destroyed, therefore it will
 * prevent the destruction of entities so long as the wait set exists, even if
 * the user lets their copy of the shared pointer to the entity go out of scope.
 *
 * 由于 wait set 无法更改，因此不需要是线程安全的。
 * Since the wait set cannot be mutated, it does not need to be thread-safe.
 *
 * \sa rclcpp::WaitSetTemplate for API documentation
 */
template <
    std::size_t NumberOfSubscriptions,
    std::size_t NumberOfGuardCondtions,
    std::size_t NumberOfTimers,
    std::size_t NumberOfClients,
    std::size_t NumberOfServices,
    std::size_t NumberOfWaitables>
using StaticWaitSet = rclcpp::WaitSetTemplate<
    rclcpp::wait_set_policies::SequentialSynchronization,
    rclcpp::wait_set_policies::StaticStorage<
        NumberOfSubscriptions,
        NumberOfGuardCondtions,
        NumberOfTimers,
        NumberOfClients,
        NumberOfServices,
        NumberOfWaitables>>;

/// 与 WaitSet 类似，此配置是动态的，但还具有线程安全性。
/// Like WaitSet, this configuration is dynamic, but is also thread-safe.
/**
 * 此 wait set 允许您动态添加和删除项目，每次调用 wait() 或 prune_destroyed_entities() 时，
 * 它会自动删除超出范围的项目。
 * This wait set allows you to add and remove items dynamically, and it will
 * automatically remove items that are let out of scope each time wait() or
 * prune_destroyed_entities() is called.
 *
 * 它还将确保以线程安全的方式从 wait set 中显式添加和删除项目，保护并发添加和删除，
 * 以及在 wait() 期间的添加和删除。此线程安全性需要一些开销和使用线程同步原语。
 * It will also ensure that adding and removing items explicitly from the
 * wait set is done in a thread-safe way, protecting against concurrent add and
 * deletes, as well as add and deletes during a wait().
 * This thread-safety comes at some overhead and the use of thread
 * synchronization primitives.
 *
 * \sa rclcpp::WaitSetTemplate for API documentation
 */
using ThreadSafeWaitSet = rclcpp::WaitSetTemplate<
    rclcpp::wait_set_policies::ThreadSafeSynchronization,
    rclcpp::wait_set_policies::DynamicStorage>;

}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_HPP_

// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__WAITABLE_HPP_
#define RCLCPP__WAITABLE_HPP_

#include <atomic>
#include <functional>
#include <memory>

#include "rcl/wait.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/*!
 * \class Waitable
 * \brief 等待类，用于处理订阅、定时器、客户端、事件和服务等对象的等待。
 *        Waitable class, used for handling waiting for objects such as subscriptions, timers,
 * clients, events and services.
 */
class Waitable {
public:
  // 定义智能指针类型，不可复制
  // Define smart pointer types, not copyable
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Waitable)

  // 公共虚析构函数
  // Public virtual destructor
  RCLCPP_PUBLIC
  virtual ~Waitable() = default;

  /*!
   * \brief 获取就绪订阅的数量
   *        Get the number of ready subscriptions
   * \return 默认返回0。如果 Waitable 包含一个或多个订阅，则应重写此方法。
   *         Returns a value of 0 by default. This should be overridden if the Waitable contains one
   * or more subscriptions.
   */
  RCLCPP_PUBLIC
  virtual size_t get_number_of_ready_subscriptions();

  /*!
   * \brief 获取就绪定时器的数量
   *        Get the number of ready timers
   * \return 默认返回0。如果 Waitable 包含一个或多个定时器，则应重写此方法。
   *         Returns a value of 0 by default. This should be overridden if the Waitable contains one
   * or more timers.
   */
  RCLCPP_PUBLIC
  virtual size_t get_number_of_ready_timers();

  /*!
   * \brief 获取就绪客户端的数量
   *        Get the number of ready clients
   * \return 默认返回0。如果 Waitable 包含一个或多个客户端，则应重写此方法。
   *         Returns a value of 0 by default. This should be overridden if the Waitable contains one
   * or more clients.
   */
  RCLCPP_PUBLIC
  virtual size_t get_number_of_ready_clients();

  /*!
   * \brief 获取就绪事件的数量
   *        Get the number of ready events
   * \return 默认返回0。如果 Waitable 包含一个或多个事件，则应重写此方法。
   *         Returns a value of 0 by default. This should be overridden if the Waitable contains one
   * or more events.
   */
  RCLCPP_PUBLIC
  virtual size_t get_number_of_ready_events();

  /*!
   * \brief 获取就绪服务的数量
   *        Get the number of ready services
   * \return 默认返回0。如果 Waitable 包含一个或多个服务，则应重写此方法。
   *         Returns a value of 0 by default. This should be overridden if the Waitable contains one
   * or more services.
   */
  RCLCPP_PUBLIC
  virtual size_t get_number_of_ready_services();

  /*!
   * \brief 获取就绪保护条件的数量
   *        Get the number of ready guard_conditions
   * \return 默认返回0。如果 Waitable 包含一个或多个保护条件，则应重写此方法。
   *         Returns a value of 0 by default. This should be overridden if the Waitable contains one
   * or more guard_conditions.
   */
  RCLCPP_PUBLIC
  virtual size_t get_number_of_ready_guard_conditions();

  /*!
   * \brief 将 Waitable 添加到等待集。
   *        Add the Waitable to a wait set.
   * \param[in] wait_set 要将 Waitable 添加到的等待集句柄。
   *            A handle to the wait set to add the Waitable to.
   * \throws rclcpp::execptions::RCLError from rcl_wait_set_add_*()
   */
  RCLCPP_PUBLIC
  virtual void add_to_wait_set(rcl_wait_set_t* wait_set) = 0;

  /*!
   * \brief 检查 Waitable 是否就绪。
   *        Check if the Waitable is ready.
   * \param[in] wait_set 先前用于调用 `add_to_wait_set()` 的等待集句柄，且已在 `rcl_wait()` 上等待。
   *            A handle to the wait set the Waitable was previously added to and that has been
   * waited on. \return 如果 Waitable 就绪，则返回 `true`，否则返回 `false`。 `true` if the Waitable
   * is ready, `false` otherwise.
   */
  RCLCPP_PUBLIC
  virtual bool is_ready(rcl_wait_set_t* wait_set) = 0;

  // ... 更多方法注释 ...
  // ... More method comments ...

private:
  // 原子布尔变量，表示是否由等待集使用
  // Atomic boolean variable indicating whether it is in use by a wait set
  std::atomic<bool> in_use_by_wait_set_{false};
};  // class Waitable

}  // namespace rclcpp

#endif  // RCLCPP__WAITABLE_HPP_

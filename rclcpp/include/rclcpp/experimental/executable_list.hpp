// Copyright 2019 Nobleo Technology
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

#ifndef RCLCPP__EXPERIMENTAL__EXECUTABLE_LIST_HPP_
#define RCLCPP__EXPERIMENTAL__EXECUTABLE_LIST_HPP_

#include <vector>

#include "rclcpp/client.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp {
namespace experimental {

/// \brief 此类包含 SubscriptionBase, TimerBase 等，可用于运行回调。
///        This class contains SubscriptionBase, TimerBase, etc. which can be used to run callbacks.
class ExecutableList final {
public:
  /// \brief 构造函数
  ///        Constructor
  RCLCPP_PUBLIC
  ExecutableList();

  /// \brief 析构函数
  ///        Destructor
  RCLCPP_PUBLIC
  ~ExecutableList();

  /// \brief 清除列表中的所有元素
  ///        Clear all elements in the list
  RCLCPP_PUBLIC
  void clear();

  /// \brief 添加订阅
  /// \param[in] subscription 订阅指针
  ///        Add subscription
  /// \param[in] subscription Shared pointer to the subscription
  RCLCPP_PUBLIC
  void add_subscription(rclcpp::SubscriptionBase::SharedPtr subscription);

  /// \brief 添加定时器
  /// \param[in] timer 定时器指针
  ///        Add timer
  /// \param[in] timer Shared pointer to the timer
  RCLCPP_PUBLIC
  void add_timer(rclcpp::TimerBase::SharedPtr timer);

  /// \brief 添加服务
  /// \param[in] service 服务指针
  ///        Add service
  /// \param[in] service Shared pointer to the service
  RCLCPP_PUBLIC
  void add_service(rclcpp::ServiceBase::SharedPtr service);

  /// \brief 添加客户端
  /// \param[in] client 客户端指针
  ///        Add client
  /// \param[in] client Shared pointer to the client
  RCLCPP_PUBLIC
  void add_client(rclcpp::ClientBase::SharedPtr client);

  /// \brief 添加等待对象
  /// \param[in] waitable 等待对象指针
  ///        Add waitable
  /// \param[in] waitable Shared pointer to the waitable
  RCLCPP_PUBLIC
  void add_waitable(rclcpp::Waitable::SharedPtr waitable);

  // 向执行器添加的所有订阅的 SubscriptionBase 的向量。
  // Vector containing the SubscriptionBase of all the subscriptions added to the executor.
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscription;
  // 已添加订阅的计数
  // Contains the count of added subscriptions
  size_t number_of_subscriptions;
  // 向执行器添加的所有定时器的 TimerBase 的向量。
  // Vector containing the TimerBase of all the timers added to the executor.
  std::vector<rclcpp::TimerBase::SharedPtr> timer;
  // 已添加定时器的计数
  // Contains the count of added timers
  size_t number_of_timers;
  // 向执行器添加的所有服务的 ServiceBase 的向量。
  // Vector containing the ServiceBase of all the services added to the executor.
  std::vector<rclcpp::ServiceBase::SharedPtr> service;
  // 已添加服务的计数
  // Contains the count of added services
  size_t number_of_services;
  // Vector containing the ClientBase of all the clients added to the executor.
  std::vector<rclcpp::ClientBase::SharedPtr> client;
  // Contains the count of added clients
  size_t number_of_clients;
  // Vector containing all the waitables added to the executor.
  std::vector<rclcpp::Waitable::SharedPtr> waitable;
  // Contains the count of added waitables
  size_t number_of_waitables;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__EXECUTABLE_LIST_HPP_

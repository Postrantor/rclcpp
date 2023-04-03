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

#include "rclcpp/experimental/executable_list.hpp"

#include <utility>

using rclcpp::experimental::ExecutableList;

/**
 * @class ExecutableList
 * @brief 构造函数 (Constructor)
 */
ExecutableList::ExecutableList()
    : number_of_subscriptions(0),
      number_of_timers(0),
      number_of_services(0),
      number_of_clients(0),
      number_of_waitables(0) {
  // 初始化各种资源计数器为0 (Initialize all resource counters to 0)
}

/**
 * @class ExecutableList
 * @brief 析构函数 (Destructor)
 */
ExecutableList::~ExecutableList() {}

/**
 * @brief 清除所有列表和计数器 (Clears all lists and counters)
 */
void ExecutableList::clear() {
  // 清除定时器列表，计数器置0 (Clear the timer list and set the counter to 0)
  this->timer.clear();
  this->number_of_timers = 0;

  // 清除订阅列表，计数器置0 (Clear the subscription list and set the counter to 0)
  this->subscription.clear();
  this->number_of_subscriptions = 0;

  // 清除服务列表，计数器置0 (Clear the service list and set the counter to 0)
  this->service.clear();
  this->number_of_services = 0;

  // 清除客户端列表，计数器置0 (Clear the client list and set the counter to 0)
  this->client.clear();
  this->number_of_clients = 0;

  // 清除可等待对象列表，计数器置0 (Clear the waitable list and set the counter to 0)
  this->waitable.clear();
  this->number_of_waitables = 0;
}

/**
 * @brief 添加订阅到订阅列表 (Adds a subscription to the subscription list)
 * @param subscription 订阅对象 (Subscription object)
 */
void ExecutableList::add_subscription(rclcpp::SubscriptionBase::SharedPtr subscription) {
  // 将订阅添加到列表中 (Add the subscription to the list)
  this->subscription.push_back(std::move(subscription));
  // 增加订阅计数器 (Increase the subscription counter)
  this->number_of_subscriptions++;
}

/**
 * @brief 添加定时器到定时器列表 (Adds a timer to the timer list)
 * @param timer 定时器对象 (Timer object)
 */
void ExecutableList::add_timer(rclcpp::TimerBase::SharedPtr timer) {
  // 将定时器添加到列表中 (Add the timer to the list)
  this->timer.push_back(std::move(timer));
  // 增加定时器计数器 (Increase the timer counter)
  this->number_of_timers++;
}

/**
 * @brief 添加服务到服务列表 (Adds a service to the service list)
 * @param service 服务对象 (Service object)
 */
void ExecutableList::add_service(rclcpp::ServiceBase::SharedPtr service) {
  // 将服务添加到列表中 (Add the service to the list)
  this->service.push_back(std::move(service));
  // 增加服务计数器 (Increase the service counter)
  this->number_of_services++;
}

/**
 * @brief 添加客户端到客户端列表 (Adds a client to the client list)
 * @param client 客户端对象 (Client object)
 */
void ExecutableList::add_client(rclcpp::ClientBase::SharedPtr client) {
  // 将客户端添加到列表中 (Add the client to the list)
  this->client.push_back(std::move(client));
  // 增加客户端计数器 (Increase the client counter)
  this->number_of_clients++;
}

/**
 * @brief 添加可等待对象到可等待对象列表 (Adds a waitable to the waitable list)
 * @param waitable 可等待对象 (Waitable object)
 */
void ExecutableList::add_waitable(rclcpp::Waitable::SharedPtr waitable) {
  // 将可等待对象添加到列表中 (Add the waitable to the list)
  this->waitable.push_back(std::move(waitable));
  // 增加可等待对象计数器 (Increase the waitable counter)
  this->number_of_waitables++;
}

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

#include "rclcpp/waitable.hpp"

#include <stdexcept>

/// \class Waitable
/// \brief Waitable 类是一个抽象基类，用于表示可等待的实体（如订阅器、定时器等）。
using rclcpp::Waitable;

/// \brief 获取已准备好的订阅器数量。
///
/// \return 返回已准备好的订阅器数量。默认值为 0。
size_t Waitable::get_number_of_ready_subscriptions() { return 0u; }

/// \brief 获取已准备好的定时器数量。
///
/// \return 返回已准备好的定时器数量。默认值为 0。
size_t Waitable::get_number_of_ready_timers() { return 0u; }

/// \brief 获取已准备好的客户端数量。
///
/// \return 返回已准备好的客户端数量。默认值为 0。
size_t Waitable::get_number_of_ready_clients() { return 0u; }

/// \brief 获取已准备好的事件数量。
///
/// \return 返回已准备好的事件数量。默认值为 0。
size_t Waitable::get_number_of_ready_events() { return 0u; }

/// \brief 获取已准备好的服务数量。
///
/// \return 返回已准备好的服务数量。默认值为 0。
size_t Waitable::get_number_of_ready_services() { return 0u; }

/// \brief 获取已准备好的保护条件数量。
///
/// \return 返回已准备好的保护条件数量。默认值为 0。
size_t Waitable::get_number_of_ready_guard_conditions() { return 0u; }

/**
 * @brief 通过实体ID获取数据
 *
 * @param id 实体ID
 * @return std::shared_ptr<void> 返回指向数据的共享指针
 * @throws std::runtime_error 如果自定义可等待对象没有覆盖此方法，则抛出运行时错误
 */
std::shared_ptr<void> Waitable::take_data_by_entity_id(size_t id) {
  (void)id;
  // 抛出运行时错误，提示用户应该覆盖此方法以使用它
  throw std::runtime_error(
      "Custom waitables should override take_data_by_entity_id "
      "if they want to use it.");
}

/**
 * @brief 交换等待集合中的使用状态
 *
 * @param in_use_state 新的使用状态
 * @return bool 返回旧的使用状态
 */
bool Waitable::exchange_in_use_by_wait_set_state(bool in_use_state) {
  // 使用原子操作交换使用状态，并返回旧状态
  return in_use_by_wait_set_.exchange(in_use_state);
}

/**
 * @brief 设置就绪回调函数
 *
 * @param callback 就绪回调函数，接受两个参数：size_t 和 int
 * @throws std::runtime_error 如果自定义可等待对象没有覆盖此方法，则抛出运行时错误
 */
void Waitable::set_on_ready_callback(std::function<void(size_t, int)> callback) {
  (void)callback;
  // 抛出运行时错误，提示用户应该覆盖此方法以使用它
  throw std::runtime_error(
      "Custom waitables should override set_on_ready_callback "
      "if they want to use it.");
}

/**
 * @brief 清除就绪回调函数
 *
 * @throws std::runtime_error 如果自定义可等待对象没有覆盖此方法，则抛出运行时错误
 */
void Waitable::clear_on_ready_callback() {
  // 抛出运行时错误，提示用户应该覆盖此方法以使用它，并确保在可等待对象的析构函数中调用它
  throw std::runtime_error(
      "Custom waitables should override clear_on_ready_callback if they "
      "want to use it and make sure to call it on the waitable destructor.");
}

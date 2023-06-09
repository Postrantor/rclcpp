// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EVENT_HPP_
#define RCLCPP__EVENT_HPP_

#include <atomic>
#include <memory>

#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/**
 * @class Event
 * @brief 事件类，用于表示一个简单的二值状态（true/false）。
 */
class Event {
public:
  /// 定义智能指针类型，禁止拷贝
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Event)

  /// 默认构造函数
  /**
   * 将默认值设置为 false
   */
  RCLCPP_PUBLIC
  Event();

  /// 将事件状态值设置为 true
  /**
   * \return 调用前的状态值。
   */
  RCLCPP_PUBLIC
  bool set();

  /// 获取事件的状态值
  /**
   * \return 事件状态值
   */
  RCLCPP_PUBLIC
  bool check();

  /// 获取事件的状态值并设置为 false
  /**
   * \return 事件状态值
   */
  RCLCPP_PUBLIC
  bool check_and_clear();

private:
  /// 禁止拷贝
  RCLCPP_DISABLE_COPY(Event)

  /// 原子布尔型状态变量
  std::atomic_bool state_;
};

}  // namespace rclcpp

#endif  // RCLCPP__EVENT_HPP_

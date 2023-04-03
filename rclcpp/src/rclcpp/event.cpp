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

#include "rclcpp/event.hpp"

namespace rclcpp {

/**
 * @brief 事件类的构造函数，初始化事件状态为 false（未设置）。
 * @details Event class constructor, initializes the event state to false (unset).
 */
Event::Event() : state_(false) {}

/**
 * @brief 设置事件状态为 true（已设置）。
 * @details Set the event state to true (set).
 * @return 返回之前的事件状态。Returns the previous event state.
 */
bool Event::set() { return state_.exchange(true); }

/**
 * @brief 检查事件状态。
 * @details Check the event state.
 * @return 返回当前的事件状态。Returns the current event state.
 */
bool Event::check() { return state_.load(); }

/**
 * @brief 检查并清除（重置）事件状态。
 * @details Check and clear (reset) the event state.
 * @return 返回之前的事件状态。Returns the previous event state.
 */
bool Event::check_and_clear() { return state_.exchange(false); }

}  // namespace rclcpp

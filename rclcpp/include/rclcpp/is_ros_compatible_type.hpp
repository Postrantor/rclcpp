// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__IS_ROS_COMPATIBLE_TYPE_HPP_
#define RCLCPP__IS_ROS_COMPATIBLE_TYPE_HPP_

#include "rclcpp/type_adapter.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rclcpp {

/**
 * @brief 判断一个类型是否与 ROS2 兼容的结构体 (A struct to determine if a type is compatible with
 * ROS2)
 *
 * @tparam T 要检查的类型 (The type to check for compatibility)
 */
template <typename T>
struct is_ros_compatible_type {
  /**
   * @brief 标志变量，表示类型 T 是否与 ROS2 兼容
   *
   * 如果 T 是 ROS 消息类型或者 T 的 TypeAdapter 特化，则 value 为 true，否则为 false。
   */
  static constexpr bool value = rosidl_generator_traits::is_message<T>::value ||
                                rclcpp::TypeAdapter<T>::is_specialized::value;
};

}  // namespace rclcpp

#endif  // RCLCPP__IS_ROS_COMPATIBLE_TYPE_HPP_

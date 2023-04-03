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

#ifndef RCLCPP__GET_MESSAGE_TYPE_SUPPORT_HANDLE_HPP_
#define RCLCPP__GET_MESSAGE_TYPE_SUPPORT_HANDLE_HPP_

#include <type_traits>

#include "rclcpp/type_adapter.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

/// Versions of rosidl_typesupport_cpp::get_message_type_support_handle that handle adapted types.

namespace rclcpp {

#ifdef DOXYGEN_ONLY

/// 返回给定 `MessageT` 类型的消息类型支持。 (Returns the message type support for the given
/// `MessageT` type.)
/**
 * \tparam MessageT 实际的 ROS 消息类型或使用 `rclcpp::TypeAdapter` 的适配类型
 *                  (an actual ROS message type or an adapted type using `rclcpp::TypeAdapter`)
 */
template <typename MessageT>
constexpr const rosidl_message_type_support_t& get_message_type_support_handle();

#else

// 若 MessageT 为 ROS 消息类型，则返回对应的消息类型支持句柄。
// (If MessageT is a ROS message type, return the corresponding message type support handle.)
template <typename MessageT>
constexpr typename std::enable_if_t<
    rosidl_generator_traits::is_message<MessageT>::value,
    const rosidl_message_type_support_t &>
get_message_type_support_handle() {
  // 获取 MessageT 类型的消息类型支持句柄。 (Get the message type support handle for MessageT type.)
  auto handle = rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>();
  // 如果句柄为空，则抛出异常。 (If the handle is nullptr, throw an exception.)
  if (!handle) {
    throw std::runtime_error("Type support handle unexpectedly nullptr");
  }
  // 返回消息类型支持句柄。 (Return the message type support handle.)
  return *handle;
}

// 若 AdaptedType 不是 ROS 消息类型且已特化，则返回对应的消息类型支持句柄。
// (If AdaptedType is not a ROS message type and is specialized, return the corresponding message
// type support handle.)
template <typename AdaptedType>
constexpr typename std::enable_if_t<
    !rosidl_generator_traits::is_message<AdaptedType>::value &&
        rclcpp::TypeAdapter<AdaptedType>::is_specialized::value,
    const rosidl_message_type_support_t &>
get_message_type_support_handle() {
  // 获取 AdaptedType 类型的消息类型支持句柄。 (Get the message type support handle for AdaptedType
  // type.)
  auto handle = rosidl_typesupport_cpp::get_message_type_support_handle<
      typename TypeAdapter<AdaptedType>::ros_message_type>();
  // 如果句柄为空，则抛出异常。 (If the handle is nullptr, throw an exception.)
  if (!handle) {
    throw std::runtime_error("Type support handle unexpectedly nullptr");
  }
  // 返回消息类型支持句柄。 (Return the message type support handle.)
  return *handle;
}

// 这个特化是一个通过运行时检查的通道，允许更好的 static_assert 捕获此问题。
// 实际上，这个特化永远不会被调用，纯粹是防御性的。
// (This specialization is a pass through runtime check, which allows a better
// static_assert to catch this issue further down the line.
// This should never get to be called in practice, and is purely defensive.)
template <typename AdaptedType>
constexpr typename std::enable_if_t<
    !rosidl_generator_traits::is_message<AdaptedType>::value &&
        !TypeAdapter<AdaptedType>::is_specialized::value,
    const rosidl_message_type_support_t &>
get_message_type_support_handle() {
  // 抛出运行时异常，表示这个特化不应该被调用。 (Throw a runtime exception, indicating that this
  // specialization should not be called.)
  throw std::runtime_error(
      "this specialization of rclcpp::get_message_type_support_handle() "
      "should never be called");
}

#endif

}  // namespace rclcpp

#endif  // RCLCPP__GET_MESSAGE_TYPE_SUPPORT_HANDLE_HPP_

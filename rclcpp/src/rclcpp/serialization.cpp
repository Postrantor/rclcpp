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

#include "rclcpp/serialization.hpp"

#include <memory>
#include <string>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/asserts.hpp"
#include "rmw/rmw.h"

namespace rclcpp {

/**
 * @brief 构造函数 (Constructor)
 *
 * @param type_support 指向 ROS 消息类型支持的指针 (Pointer to the ROS message type support)
 */
SerializationBase::SerializationBase(const rosidl_message_type_support_t* type_support)
    : type_support_(
          type_support)  // 初始化成员变量 type_support_ (Initialize member variable type_support_)
{
  // 检查 type_support 是否为空指针 (Check if type_support is a nullpointer)
  rcpputils::check_true(nullptr != type_support, "Typesupport is nullpointer.");
}

/**
 * @brief 序列化 ROS 消息 (Serialize ROS message)
 *
 * @param ros_message 指向要序列化的 ROS 消息的指针 (Pointer to the ROS message to be serialized)
 * @param serialized_message 指向序列化后消息的指针 (Pointer to the serialized message)
 */
void SerializationBase::serialize_message(
    const void* ros_message, SerializedMessage* serialized_message) const {
  // 检查 type_support_, ros_message 和 serialized_message 是否为空指针 (Check if type_support_,
  // ros_message and serialized_message are nullpointers)
  rcpputils::check_true(nullptr != type_support_, "Typesupport is nullpointer.");
  rcpputils::check_true(nullptr != ros_message, "ROS message is nullpointer.");
  rcpputils::check_true(nullptr != serialized_message, "Serialized message is nullpointer.");

  // 调用 rmw_serialize 函数进行序列化 (Call rmw_serialize function for serialization)
  const auto ret =
      rmw_serialize(ros_message, type_support_, &serialized_message->get_rcl_serialized_message());

  // 判断序列化是否成功 (Check if serialization is successful)
  if (ret != RMW_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to serialize ROS message.");
  }
}

/**
 * @brief 反序列化 ROS 消息 (Deserialize ROS message)
 *
 * @param serialized_message 指向序列化后的消息的指针 (Pointer to the serialized message)
 * @param ros_message 指向要反序列化的 ROS 消息的指针 (Pointer to the ROS message to be
 * deserialized)
 */
void SerializationBase::deserialize_message(
    const SerializedMessage* serialized_message, void* ros_message) const {
  // 检查 type_support_, serialized_message 和 ros_message 是否为空指针 (Check if type_support_,
  // serialized_message and ros_message are nullpointers)
  rcpputils::check_true(nullptr != type_support_, "Typesupport is nullpointer.");
  rcpputils::check_true(nullptr != serialized_message, "Serialized message is nullpointer.");

  // 检查 serialized_message 的 capacity 和 size 是否为零 (Check if the capacity and size of
  // serialized_message are zero)
  rcpputils::check_true(
      0u != serialized_message->capacity(),
      "Wrongly initialized. Serialized message has a capacity of zero.");
  rcpputils::check_true(
      0u != serialized_message->size(),
      "Wrongly initialized. Serialized message has a size of zero.");

  // 检查 ros_message 是否为空指针 (Check if ros_message is a nullpointer)
  rcpputils::check_true(nullptr != ros_message, "ROS message is a nullpointer.");

  // 调用 rmw_deserialize 函数进行反序列化 (Call rmw_deserialize function for deserialization)
  const auto ret = rmw_deserialize(
      &serialized_message->get_rcl_serialized_message(), type_support_, ros_message);

  // 判断反序列化是否成功 (Check if deserialization is successful)
  if (ret != RMW_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to deserialize ROS message.");
  }
}

}  // namespace rclcpp

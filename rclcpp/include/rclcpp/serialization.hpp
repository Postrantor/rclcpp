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

#ifndef RCLCPP__SERIALIZATION_HPP_
#define RCLCPP__SERIALIZATION_HPP_

#include <memory>
#include <stdexcept>
#include <string>

#include "rcl/types.h"
#include "rclcpp/visibility_control.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

namespace rclcpp {

class SerializedMessage;

// 序列化特征命名空间
// Serialization traits namespace
namespace serialization_traits {
// 检查类型是否为面向对象的序列化消息
// Trait to check if type is the object-oriented serialized message
template <typename T>
struct is_serialized_message_class : std::false_type {};

// 特化SerializedMessage类型的is_serialized_message_class
// Specialize is_serialized_message_class for SerializedMessage type
template <>
struct is_serialized_message_class<SerializedMessage> : std::true_type {};
}  // namespace serialization_traits

// (反)序列化消息的接口
// Interface to (de)serialize a message
class RCLCPP_PUBLIC_TYPE SerializationBase {
public:
  // SerializationBase的构造函数
  // Constructor of SerializationBase
  /**
   * \param[in] type_support 用于序列化和反序列化的消息类型支持句柄
   * \param[in] type_support handle for the message type support
   * to be used for serialization and deserialization.
   */
  explicit SerializationBase(const rosidl_message_type_support_t* type_support);

  // SerializationBase的析构函数
  // Destructor of SerializationBase
  virtual ~SerializationBase() = default;

  // 将ROS2消息序列化到序列化流中
  // Serialize a ROS2 message to a serialized stream
  /**
   * \param[in] ros_message 被rmw读取并序列化的ROS2消息
   * \param[in] ros_message The ROS2 message which is read and serialized by rmw.
   * \param[out] serialized_message 序列化后的消息
   * \param[out] serialized_message The serialized message.
   */
  void serialize_message(const void* ros_message, SerializedMessage* serialized_message) const;

  // 将序列化流反序列化为ROS消息
  // Deserialize a serialized stream to a ROS message
  /**
   * \param[in] serialized_message 要被rmw转换为ROS2的序列化消息
   * \param[in] serialized_message The serialized message to be converted to ROS2 by rmw.
   * \param[out] ros_message 反序列化后的ROS2消息
   * \param[out] ros_message The deserialized ROS2 message.
   */
  void deserialize_message(const SerializedMessage* serialized_message, void* ros_message) const;

private:
  // 消息类型支持句柄
  // Message type support handle
  const rosidl_message_type_support_t* type_support_;
};

// 使用rmw_(de)serialize对消息进行(反)序列化的默认实现
// Default implementation to (de)serialize a message by using rmw_(de)serialize
template <typename MessageT>
class Serialization : public SerializationBase {
public:
  // Serialization的构造函数
  // Constructor of Serialization
  Serialization()
      : SerializationBase(rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>()) {
    // 静态断言，不允许将序列化消息序列化为序列化消息
    // Static assertion, serialization of serialized message to serialized message is not possible.
    static_assert(
        !serialization_traits::is_serialized_message_class<MessageT>::value,
        "Serialization of serialized message to serialized message is not possible.");
  }
};

}  // namespace rclcpp

#endif  // RCLCPP__SERIALIZATION_HPP_

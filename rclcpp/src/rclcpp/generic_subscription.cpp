// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2021, Apex.AI Inc.
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

#include "rclcpp/generic_subscription.hpp"

#include <memory>
#include <string>

#include "rcl/subscription.h"
#include "rclcpp/exceptions.hpp"

namespace rclcpp {

/**
 * @brief 创建一条消息
 * @return 返回一个共享指针，指向创建的消息
 *
 * @brief Create a message
 * @return A shared pointer pointing to the created message
 */
std::shared_ptr<void> GenericSubscription::create_message() {
  // 调用 create_serialized_message() 函数创建序列化消息
  // Call create_serialized_message() function to create a serialized message
  return create_serialized_message();
}

/**
 * @brief 创建一条序列化消息
 * @return 返回一个共享指针，指向创建的序列化消息
 *
 * @brief Create a serialized message
 * @return A shared pointer pointing to the created serialized message
 */
std::shared_ptr<rclcpp::SerializedMessage> GenericSubscription::create_serialized_message() {
  // 使用 std::make_shared 构造一个空的 SerializedMessage 对象并返回
  // Construct an empty SerializedMessage object with std::make_shared and return it
  return std::make_shared<rclcpp::SerializedMessage>(0);
}

/**
 * @brief 处理接收到的消息
 * @param[in] message 指向接收到的消息的共享指针
 * @param[in] message_info 接收到的消息的信息
 * @throw rclcpp::exceptions::UnimplementedError 抛出未实现错误异常
 *
 * @brief Handle received message
 * @param[in] message Shared pointer pointing to the received message
 * @param[in] message_info Information about the received message
 * @throw rclcpp::exceptions::UnimplementedError Throw unimplemented error exception
 */
void GenericSubscription::handle_message(std::shared_ptr<void> &, const rclcpp::MessageInfo &) {
  // 抛出一个未实现错误异常，因为这个函数在 GenericSubscription 中没有实现
  // Throw an unimplemented error exception, because this function is not implemented in
  // GenericSubscription
  throw rclcpp::exceptions::UnimplementedError(
      "handle_message is not implemented for GenericSubscription");
}

/**
 * @brief 处理接收到的序列化消息
 * @param[in] message 指向接收到的序列化消息的共享指针
 * @param[in] message_info 接收到的序列化消息的信息
 *
 * @brief Handle received serialized message
 * @param[in] message Shared pointer pointing to the received serialized message
 * @param[in] message_info Information about the received serialized message
 */
void GenericSubscription::handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage> &message, const rclcpp::MessageInfo &) {
  // 调用回调函数处理接收到的序列化消息
  // Call the callback function to handle the received serialized message
  callback_(message);
}

/**
 * @brief 处理接收到的借用消息
 * @param[in] message 指向接收到的借用消息的指针
 * @param[in] message_info 接收到的借用消息的信息
 * @throw rclcpp::exceptions::UnimplementedError 抛出未实现错误异常
 *
 * @brief Handle received loaned message
 * @param[in] message Pointer pointing to the received loaned message
 * @param[in] message_info Information about the received loaned message
 * @throw rclcpp::exceptions::UnimplementedError Throw unimplemented error exception
 */
void GenericSubscription::handle_loaned_message(
    void *message, const rclcpp::MessageInfo &message_info) {
  // 忽略未使用的参数
  // Ignore unused parameters
  (void)message;
  (void)message_info;

  // 抛出一个未实现错误异常，因为这个函数在 GenericSubscription 中没有实现
  // Throw an unimplemented error exception, because this function is not implemented in
  // GenericSubscription
  throw rclcpp::exceptions::UnimplementedError(
      "handle_loaned_message is not implemented for GenericSubscription");
}

/**
 * @brief 返回消息
 * @param[in,out] message 指向消息的共享指针
 *
 * @brief Return message
 * @param[in,out] message Shared pointer pointing to the message
 */
void GenericSubscription::return_message(std::shared_ptr<void> &message) {
  // 将消息的类型从 void 转换为 SerializedMessage 类型
  // Convert the message type from void to SerializedMessage type
  auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);

  // 调用 return_serialized_message() 函数返回序列化消息
  // Call return_serialized_message() function to return the serialized message
  return_serialized_message(typed_message);
}

/**
 * @brief 返回序列化消息
 * @param[in,out] message 指向序列化消息的共享指针
 *
 * @brief Return serialized message
 * @param[in,out] message Shared pointer pointing to the serialized message
 */
void GenericSubscription::return_serialized_message(
    std::shared_ptr<rclcpp::SerializedMessage> &message) {
  // 重置序列化消息的共享指针
  // Reset the shared pointer of the serialized message
  message.reset();
}

}  // namespace rclcpp

Y  // Copyright 2018, Bosch Software Innovations GmbH.
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

#include "rclcpp/generic_publisher.hpp"

#include <memory>
#include <string>

    namespace rclcpp {

  /**
   * @brief 发布序列化的消息 (Publish a serialized message)
   *
   * @param message 要发布的序列化消息 (The serialized message to publish)
   */
  void GenericPublisher::publish(const rclcpp::SerializedMessage& message) {
    // 使用rcl_publish_serialized_message函数发布序列化的消息
    // (Publish the serialized message using the rcl_publish_serialized_message function)
    auto return_code = rcl_publish_serialized_message(
        get_publisher_handle().get(), &message.get_rcl_serialized_message(), NULL);

    // 如果发布不成功，抛出异常 (If the publishing is not successful, throw an exception)
    if (return_code != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to publish serialized message");
    }
  }

  /**
   * @brief 以借用的消息形式发布序列化的消息 (Publish a serialized message as a loaned message)
   *
   * @param message 要发布的序列化消息 (The serialized message to publish)
   */
  void GenericPublisher::publish_as_loaned_msg(const rclcpp::SerializedMessage& message) {
    // 借用一条消息 (Borrow a message)
    auto loaned_message = borrow_loaned_message();
    // 将序列化的消息反序列化为借用的消息 (Deserialize the serialized message into the borrowed
    // message)
    deserialize_message(message.get_rcl_serialized_message(), loaned_message);
    // 发布借用的消息 (Publish the borrowed message)
    publish_loaned_message(loaned_message);
  }

  /**
   * @brief 借用一条消息 (Borrow a message)
   *
   * @return void* 指向借用消息的指针 (A pointer to the borrowed message)
   */
  void* GenericPublisher::borrow_loaned_message() {
    void* loaned_message = nullptr;
    // 使用rcl_borrow_loaned_message函数借用一条消息
    // (Borrow a message using the rcl_borrow_loaned_message function)
    auto return_code =
        rcl_borrow_loaned_message(get_publisher_handle().get(), &type_support_, &loaned_message);

    // 如果借用不成功，抛出异常 (If borrowing is not successful, throw an exception)
    if (return_code != RMW_RET_OK) {
      if (return_code == RCL_RET_UNSUPPORTED) {
        rclcpp::exceptions::throw_from_rcl_error(
            return_code, "current middleware cannot support loan messages");
      } else {
        rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to borrow loaned msg");
      }
    }
    return loaned_message;
  }

  /**
   * @brief 反序列化消息 (Deserialize a message)
   *
   * @param serialized_message 序列化的消息 (The serialized message)
   * @param deserialized_msg 反序列化后的消息 (The deserialized message after deserialization)
   */
  void GenericPublisher::deserialize_message(
      const rmw_serialized_message_t& serialized_message, void* deserialized_msg) {
    // 使用rmw_deserialize函数反序列化消息 (Deserialize the message using the rmw_deserialize
    // function)
    auto return_code = rmw_deserialize(&serialized_message, &type_support_, deserialized_msg);
    // 如果反序列化不成功，抛出异常 (If deserialization is not successful, throw an exception)
    if (return_code != RMW_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to deserialize msg");
    }
  }

  /**
   * @brief 发布借用的消息 (Publish a loaned message)
   *
   * @param loaned_message 要发布的借用消息 (The loaned message to publish)
   */
  void GenericPublisher::publish_loaned_message(void* loaned_message) {
    // 使用rcl_publish_loaned_message函数发布借用的消息
    // (Publish the loaned message using the rcl_publish_loaned_message function)
    auto return_code =
        rcl_publish_loaned_message(get_publisher_handle().get(), loaned_message, NULL);

    // 如果发布不成功，抛出异常 (If the publishing is not successful, throw an exception)
    if (return_code != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to publish loaned message");
    }
  }

}  // namespace rclcpp

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

#ifndef RCLCPP__EXPERIMENTAL__ROS_MESSAGE_INTRA_PROCESS_BUFFER_HPP_
#define RCLCPP__EXPERIMENTAL__ROS_MESSAGE_INTRA_PROCESS_BUFFER_HPP_

#include <memory>
#include <string>

#include "rcl/error_handling.h"
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp {
namespace experimental {

/**
 * @brief 模板类，用于实现 ROS2 Intra-Process 通信的缓冲区
 * @tparam RosMessageT ROS 消息类型
 * @tparam Alloc 分配器，默认为 std::allocator<void>
 * @tparam Deleter 删除器，默认为 std::default_delete<void>
 *
 * Template class for implementing a buffer for ROS2 Intra-Process Communication.
 * @tparam RosMessageT The type of the ROS message
 * @tparam Alloc Allocator, default is std::allocator<void>
 * @tparam Deleter Deleter, default is std::default_delete<void>
 */
template <
    typename RosMessageT,
    typename Alloc = std::allocator<void>,
    typename Deleter = std::default_delete<void>>
class SubscriptionROSMsgIntraProcessBuffer : public SubscriptionIntraProcessBase {
public:
  // 重新绑定分配器类型
  // Rebind the allocator type
  using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<RosMessageT, Alloc>;
  using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
  using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, RosMessageT>;

  // 定义常量共享指针和唯一指针
  // Define const shared pointer and unique pointer
  using ConstMessageSharedPtr = std::shared_ptr<const RosMessageT>;
  using MessageUniquePtr = std::unique_ptr<RosMessageT, ROSMessageTypeDeleter>;

  /**
   * @brief 构造函数
   * @param context ROS2 上下文共享指针
   * @param topic_name 主题名称
   * @param qos_profile QoS 配置
   *
   * Constructor
   * @param context Shared pointer to ROS2 context
   * @param topic_name Name of the topic
   * @param qos_profile QoS profile configuration
   */
  SubscriptionROSMsgIntraProcessBuffer(
      rclcpp::Context::SharedPtr context,
      const std::string& topic_name,
      const rclcpp::QoS& qos_profile)
      : SubscriptionIntraProcessBase(context, topic_name, qos_profile) {}

  // 虚析构函数
  // Virtual destructor
  virtual ~SubscriptionROSMsgIntraProcessBuffer() {}

  /**
   * @brief 提供 Intra-Process 消息（常量共享指针）
   * @param message 常量共享指针类型的消息
   *
   * Provide an Intra-Process message (const shared pointer)
   * @param message Message in the form of a const shared pointer
   */
  virtual void provide_intra_process_message(ConstMessageSharedPtr message) = 0;

  /**
   * @brief 提供 Intra-Process 消息（唯一指针）
   * @param message 唯一指针类型的消息
   *
   * Provide an Intra-Process message (unique pointer)
   * @param message Message in the form of a unique pointer
   */
  virtual void provide_intra_process_message(MessageUniquePtr message) = 0;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__ROS_MESSAGE_INTRA_PROCESS_BUFFER_HPP_

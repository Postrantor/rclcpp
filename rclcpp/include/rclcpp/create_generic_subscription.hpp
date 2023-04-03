// Copyright 2020, Apex.AI Inc.
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

#ifndef RCLCPP__CREATE_GENERIC_SUBSCRIPTION_HPP_
#define RCLCPP__CREATE_GENERIC_SUBSCRIPTION_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rcl/subscription.h"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/typesupport_helpers.hpp"

namespace rclcpp {

/// 创建并返回一个 GenericSubscription（创建并返回一个通用订阅）。
/**
 * 返回的指针永远不会为空，但是此函数可能抛出各种异常，例如
 * 当无法在 AMENT_PREFIX_PATH 上找到消息的软件包时。
 *
 * \param topics_interface 用于部分设置的 NodeTopicsInterface 指针。
 * \param topic_name 主题名称
 * \param topic_type 主题类型
 * \param qos QoS 设置
 * \param callback 用于接收新消息的序列化形式的回调
 * \param options 发布者选项。
 * 并非所有发布者选项都受到尊重，对于此发布者，唯一相关的选项是
 * `event_callbacks`、`use_default_callbacks` 和 `callback_group`。
 */
template <typename AllocatorT = std::allocator<void>>
std::shared_ptr<GenericSubscription> create_generic_subscription(
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
        topics_interface,           // 节点主题接口共享指针
    const std::string& topic_name,  // 主题名称
    const std::string& topic_type,  // 主题类型
    const rclcpp::QoS& qos,         // QoS 设置
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,  // 序列化消息的回调
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options =  // 订阅选项（带分配器）
    (rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>())) {
  // 获取类型支持库
  auto ts_lib = rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");

  // 创建通用订阅对象
  auto subscription = std::make_shared<GenericSubscription>(
      topics_interface->get_node_base_interface(),  // 获取节点基本接口
      std::move(ts_lib),                            // 移动类型支持库
      topic_name,                                   // 主题名称
      topic_type,                                   // 主题类型
      qos,                                          // QoS 设置
      callback,                                     // 序列化消息的回调
      options);                                     // 订阅选项

  // 向节点主题接口添加订阅
  topics_interface->add_subscription(subscription, options.callback_group);

  // 返回创建的通用订阅
  return subscription;
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_GENERIC_SUBSCRIPTION_HPP_

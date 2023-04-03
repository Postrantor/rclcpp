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

#ifndef RCLCPP__CREATE_GENERIC_PUBLISHER_HPP_
#define RCLCPP__CREATE_GENERIC_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/typesupport_helpers.hpp"

namespace rclcpp {

/// 创建并返回一个 GenericPublisher.
/**
 * 返回的指针永远不会为空，但这个函数可能抛出各种异常，例如当消息的包在 AMENT_PREFIX_PATH
 * 上找不到时。
 * The returned pointer will never be empty, but this function can throw various
 * exceptions, for instance when the message's package can not be found on the AMENT_PREFIX_PATH.
 *
 * \param topics_interface 用于部分设置的 NodeTopicsInterface 指针
 * \param topic_name 主题名称
 * \param topic_type 主题类型
 * \param qos QoS 设置
 * \param options 发布者选项
 * 并非所有发布者选项都受到尊重，对于此发布者来说，唯一相关的选项是
 * `event_callbacks`、`use_default_callbacks` 和 `%callback_group`。
 * Not all publisher options are currently respected, the only relevant options for this publisher
 * are `event_callbacks`, `use_default_callbacks`, and `%callback_group`.
 */
template <typename AllocatorT = std::allocator<void>>
std::shared_ptr<GenericPublisher> create_generic_publisher(
    // 节点主题接口共享指针
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    const std::string& topic_name,  // 主题名称
    const std::string& topic_type,  // 主题类型
    const rclcpp::QoS& qos,         // QoS 设置
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT>&
        options =                   // 发布者选项，带分配器的发布者选项
    (rclcpp::PublisherOptionsWithAllocator<AllocatorT>())) {
  // 获取类型支持库
  auto ts_lib = rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
  // 创建 GenericPublisher 共享指针
  auto pub = std::make_shared<GenericPublisher>(
      topics_interface->get_node_base_interface(),  // 获取节点基本接口
      std::move(ts_lib),                            // 移动类型支持库
      topic_name,                                   // 主题名称
      topic_type,                                   // 主题类型
      qos,                                          // QoS 设置
      options);                                     // 发布者选项
  // 向节点主题接口添加发布者
  topics_interface->add_publisher(pub, options.callback_group);

  return pub;  // 返回创建的 GenericPublisher 共享指针
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_GENERIC_PUBLISHER_HPP_

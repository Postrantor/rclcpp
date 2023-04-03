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

#ifndef RCLCPP__CREATE_PUBLISHER_HPP_
#define RCLCPP__CREATE_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/detail/qos_parameters.hpp"
#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher_factory.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_overriding_options.hpp"
#include "rmw/qos_profiles.h"

namespace rclcpp {

namespace detail {
/**
 * @brief 创建并返回给定 MessageT 类型的发布者。
 * @tparam MessageT 消息类型
 * @tparam AllocatorT 分配器类型，默认为 std::allocator<void>
 * @tparam PublisherT 发布者类型，默认为 rclcpp::Publisher<MessageT, AllocatorT>
 * @tparam NodeParametersT 节点参数类型
 * @tparam NodeTopicsT 节点主题类型
 * @param node_parameters 节点参数引用
 * @param node_topics 节点主题引用
 * @param topic_name 主题名称
 * @param qos 服务质量配置
 * @param options 发布者选项，带有分配器，默认为 rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
 * @return 返回创建的发布者共享指针
 */
template <
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>,
    typename NodeParametersT,
    typename NodeTopicsT>
std::shared_ptr<PublisherT> create_publisher(
    NodeParametersT& node_parameters,
    NodeTopicsT& node_topics,
    const std::string& topic_name,
    const rclcpp::QoS& qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options =
        (rclcpp::PublisherOptionsWithAllocator<AllocatorT>())) {
  // 获取节点主题接口
  auto node_topics_interface = rclcpp::node_interfaces::get_node_topics_interface(node_topics);
  // 确定实际的 QoS 设置
  const rclcpp::QoS& actual_qos = options.qos_overriding_options.get_policy_kinds().size()
                                      ? rclcpp::detail::declare_qos_parameters(
                                            options.qos_overriding_options, node_parameters,
                                            node_topics_interface->resolve_topic_name(topic_name),
                                            qos, rclcpp::detail::PublisherQosParametersTraits{})
                                      : qos;
  // 创建发布者
  auto pub = node_topics_interface->create_publisher(
      topic_name,                                                                   //
      rclcpp::create_publisher_factory<MessageT, AllocatorT, PublisherT>(options),  //
      actual_qos);
  // 将发布者添加到节点主题接口
  node_topics_interface->add_publisher(pub, options.callback_group);

  // 返回创建的发布者共享指针
  return std::dynamic_pointer_cast<PublisherT>(pub);
}
}  // namespace detail

/// 创建并返回给定 MessageT 类型的发布者。
/// Create and return a publisher of the given MessageT type.
/**
 * NodeT 类型只需要具有一个名为 get_node_topics_interface() 的方法，
 * 该方法返回一个指向 NodeTopicsInterface 的 shared_ptr。
 * The NodeT type only needs to have a method called get_node_topics_interface(),
 * which returns a shared_ptr to a NodeTopicsInterface.
 *
 * 如果 `options.qos_overriding_options` 启用了 qos 参数覆盖，
 * 那么 NodeT 还必须具有一个名为 get_node_parameters_interface() 的方法，
 * 该方法返回一个指向 NodeParametersInterface 的 shared_ptr。
 * In case `options.qos_overriding_options` is enabling qos parameter overrides,
 * NodeT must also have a method called get_node_parameters_interface(),
 * which returns a shared_ptr to a NodeParametersInterface.
 */
template <
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>,
    typename NodeT>
std::shared_ptr<PublisherT> create_publisher(
    NodeT&& node,
    const std::string& topic_name,
    const rclcpp::QoS& qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options =
        (rclcpp::PublisherOptionsWithAllocator<AllocatorT>())) {
  // 调用 detail 命名空间中的 create_publisher 函数，并传入相应的参数。
  // Call the create_publisher function in the detail namespace with the given parameters.
  return detail::create_publisher<MessageT, AllocatorT, PublisherT>(
      node, node, topic_name, qos, options);
}

/// 创建并返回给定 MessageT 类型的发布者。
/// Create and return a publisher of the given MessageT type.
template <
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>>
std::shared_ptr<PublisherT> create_publisher(
    // 传入节点参数接口的 shared_ptr 引用。
    // Pass in a reference to the shared_ptr of the node parameters interface.
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_parameters,
    // 传入节点主题接口的 shared_ptr 引用。
    // Pass in a reference to the shared_ptr of the node topics interface.
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& node_topics,
    const std::string& topic_name,
    const rclcpp::QoS& qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options =
        (rclcpp::PublisherOptionsWithAllocator<AllocatorT>())) {
  // 调用 detail 命名空间中的 create_publisher 函数，并传入相应的参数。
  // Call the create_publisher function in the detail namespace with the given parameters.
  return detail::create_publisher<MessageT, AllocatorT, PublisherT>(
      node_parameters, node_topics, topic_name, qos, options);
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_PUBLISHER_HPP_

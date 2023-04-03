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

#ifndef RCLCPP__NODE_INTERFACES__NODE_TOPICS_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_TOPICS_HPP_

#include <string>

#include "rcl/publisher.h"
#include "rcl/subscription.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/publisher_factory.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/// 实现 Node API 的 NodeTopics 部分。 (Implementation of the NodeTopics part of the Node API.)
class NodeTopics : public NodeTopicsInterface {
public:
  // 只使用智能指针别名。 (Only use smart pointer aliases.)
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTopicsInterface)

  // 公共构造函数。 (Public constructor.)
  RCLCPP_PUBLIC
  NodeTopics(
      rclcpp::node_interfaces::NodeBaseInterface* node_base,
      rclcpp::node_interfaces::NodeTimersInterface* node_timers);

  // 公共析构函数。 (Public destructor.)
  RCLCPP_PUBLIC
  ~NodeTopics() override;

  /// 创建发布者。 (Create a publisher.)
  /**
   * @param[in] topic_name 要发布的主题名称。 (The name of the topic to publish.)
   * @param[in] publisher_factory 用于创建发布者实例的工厂。 (Factory used to create publisher
   * instances.)
   * @param[in] qos 要使用的服务质量配置。 (Quality of service configuration to use.)
   */
  RCLCPP_PUBLIC
  rclcpp::PublisherBase::SharedPtr create_publisher(
      const std::string& topic_name,
      const rclcpp::PublisherFactory& publisher_factory,
      const rclcpp::QoS& qos) override;

  /// 添加发布者。 (Add a publisher.)
  /**
   * @param[in] publisher 要添加的发布者。 (The publisher to add.)
   * @param[in] callback_group 与发布者关联的回调组。 (Callback group associated with the
   * publisher.)
   */
  RCLCPP_PUBLIC
  void add_publisher(
      rclcpp::PublisherBase::SharedPtr publisher,
      rclcpp::CallbackGroup::SharedPtr callback_group) override;

  /// 创建订阅者。 (Create a subscription.)
  /**
   * @param[in] topic_name 要订阅的主题名称。 (The name of the topic to subscribe to.)
   * @param[in] subscription_factory 用于创建订阅者实例的工厂。 (Factory used to create subscription
   * instances.)
   * @param[in] qos 要使用的服务质量配置。 (Quality of service configuration to use.)
   */
  RCLCPP_PUBLIC
  rclcpp::SubscriptionBase::SharedPtr create_subscription(
      const std::string& topic_name,
      const rclcpp::SubscriptionFactory& subscription_factory,
      const rclcpp::QoS& qos) override;

  /// 添加订阅者。 (Add a subscription.)
  /**
   * @param[in] subscription 要添加的订阅者。 (The subscription to add.)
   * @param[in] callback_group 与订阅者关联的回调组。 (Callback group associated with the
   * subscription.)
   */
  RCLCPP_PUBLIC
  void add_subscription(
      rclcpp::SubscriptionBase::SharedPtr subscription,
      rclcpp::CallbackGroup::SharedPtr callback_group) override;

  // 获取节点基本接口。 (Get the node base interface.)
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface* get_node_base_interface() const override;

  // 获取节点计时器接口。 (Get the node timers interface.)
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeTimersInterface* get_node_timers_interface() const override;

  /// 解析主题名称。 (Resolve a topic name.)
  /**
   * @param[in] name 要解析的主题名称。 (The topic name to resolve.)
   * @param[in] only_expand 是否仅展开名称。 (Whether to only expand the name.)
   */
  RCLCPP_PUBLIC
  std::string resolve_topic_name(const std::string& name, bool only_expand = false) const override;

private:
  // 禁用复制。 (Disable copy.)
  RCLCPP_DISABLE_COPY(NodeTopics)

  // 节点基本接口指针。 (Node base interface pointer.)
  rclcpp::node_interfaces::NodeBaseInterface* node_base_;
  // 节点计时器接口指针。 (Node timers interface pointer.)
  rclcpp::node_interfaces::NodeTimersInterface* node_timers_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_TOPICS_HPP_

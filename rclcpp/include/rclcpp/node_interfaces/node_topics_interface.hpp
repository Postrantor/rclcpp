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

#ifndef RCLCPP__NODE_INTERFACES__NODE_TOPICS_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_TOPICS_INTERFACE_HPP_

#include <string>

#include "rcl/publisher.h"
#include "rcl/subscription.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/detail/node_interfaces_helpers.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_factory.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/// 纯虚接口类，用于实现 Node API 的 NodeTopics 部分。
class NodeTopicsInterface {
public:
  // 智能指针别名定义
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTopicsInterface)

  /// 默认析构函数
  RCLCPP_PUBLIC
  virtual ~NodeTopicsInterface() = default;

  /// 创建发布者
  /**
   * \param[in] topic_name 要发布的主题名称
   * \param[in] publisher_factory 发布者工厂对象
   * \param[in] qos 指定服务质量（Quality of Service）配置
   * \return 返回创建的发布者共享指针
   */
  RCLCPP_PUBLIC
  virtual rclcpp::PublisherBase::SharedPtr create_publisher(
      const std::string& topic_name,
      const rclcpp::PublisherFactory& publisher_factory,
      const rclcpp::QoS& qos) = 0;

  /// 添加发布者
  /**
   * \param[in] publisher 要添加的发布者共享指针
   * \param[in] callback_group 回调组共享指针
   */
  RCLCPP_PUBLIC
  virtual void add_publisher(
      rclcpp::PublisherBase::SharedPtr publisher,
      rclcpp::CallbackGroup::SharedPtr callback_group) = 0;

  /// 创建订阅者
  /**
   * \param[in] topic_name 要订阅的主题名称
   * \param[in] subscription_factory 订阅者工厂对象
   * \param[in] qos 指定服务质量（Quality of Service）配置
   * \return 返回创建的订阅者共享指针
   */
  RCLCPP_PUBLIC
  virtual rclcpp::SubscriptionBase::SharedPtr create_subscription(
      const std::string& topic_name,
      const rclcpp::SubscriptionFactory& subscription_factory,
      const rclcpp::QoS& qos) = 0;

  /// 添加订阅者
  /**
   * \param[in] subscription 要添加的订阅者共享指针
   * \param[in] callback_group 回调组共享指针
   */
  RCLCPP_PUBLIC
  virtual void add_subscription(
      rclcpp::SubscriptionBase::SharedPtr subscription,
      rclcpp::CallbackGroup::SharedPtr callback_group) = 0;

  /// 获取节点基础接口指针
  /**
   * \return 返回节点基础接口指针
   */
  RCLCPP_PUBLIC
  virtual rclcpp::node_interfaces::NodeBaseInterface* get_node_base_interface() const = 0;

  /// 获取节点定时器接口指针
  /**
   * \return 返回节点定时器接口指针
   */
  RCLCPP_PUBLIC
  virtual rclcpp::node_interfaces::NodeTimersInterface* get_node_timers_interface() const = 0;

  /// 根据输入名称获取重映射和扩展的主题名称
  /**
   * \param[in] name 输入的主题名称
   * \param[in] only_expand 是否仅进行扩展，默认为 false
   * \return 返回解析后的主题名称
   */
  RCLCPP_PUBLIC
  virtual std::string resolve_topic_name(
      const std::string& name, bool only_expand = false) const = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeTopicsInterface, topics)

#endif  // RCLCPP__NODE_INTERFACES__NODE_TOPICS_INTERFACE_HPP_

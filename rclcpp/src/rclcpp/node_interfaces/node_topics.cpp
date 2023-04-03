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

#include "rclcpp/node_interfaces/node_topics.hpp"

#include <stdexcept>
#include <string>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/publisher_factory.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_factory.hpp"

using rclcpp::exceptions::throw_from_rcl_error;

using rclcpp::node_interfaces::NodeTopics;

/**
 * @brief 构造函数，初始化 NodeTopics 对象
 * @param node_base 指向 NodeBaseInterface 的指针
 * @param node_timers 指向 NodeTimersInterface 的指针
 *
 * @brief Constructor, initializes the NodeTopics object
 * @param node_base Pointer to NodeBaseInterface
 * @param node_timers Pointer to NodeTimersInterface
 */
NodeTopics::NodeTopics(
    rclcpp::node_interfaces::NodeBaseInterface* node_base,
    rclcpp::node_interfaces::NodeTimersInterface* node_timers)
    : node_base_(node_base), node_timers_(node_timers) {}

/// 析构函数
NodeTopics::~NodeTopics() {}

/**
 * @brief 创建一个发布器实例
 * @param topic_name 要发布的主题名称
 * @param publisher_factory 发布器工厂对象
 * @param qos Quality of Service 配置
 * @return 返回一个指向 PublisherBase 的共享指针
 */
rclcpp::PublisherBase::SharedPtr NodeTopics::create_publisher(
    const std::string& topic_name,
    const rclcpp::PublisherFactory& publisher_factory,
    const rclcpp::QoS& qos) {
  // 使用工厂创建特定于 MessageT 的发布器，但将其作为 PublisherBase 返回。
  return publisher_factory.create_typed_publisher(node_base_, topic_name, qos);
}

/**
 * @brief 将发布器添加到回调组
 * @param publisher 指向 PublisherBase 的共享指针
 * @param callback_group 指向 CallbackGroup 的共享指针
 */
void NodeTopics::add_publisher(
    rclcpp::PublisherBase::SharedPtr publisher,  //
    rclcpp::CallbackGroup::SharedPtr callback_group) {
  // 分配给一个组。
  if (callback_group) {
    if (!node_base_->callback_group_in_node(callback_group)) {
      throw std::runtime_error("Cannot create publisher, callback group not in node.");
    }
  } else {
    callback_group = node_base_->get_default_callback_group();
  }

  // 遍历发布器的事件处理程序，并将它们添加到回调组。
  for (auto& key_event_pair : publisher->get_event_handlers()) {
    auto publisher_event = key_event_pair.second;
    callback_group->add_waitable(publisher_event);
  }

  // 使用父节点通知执行器创建了一个新的发布器。
  auto& node_gc = node_base_->get_notify_guard_condition();
  try {
    node_gc.trigger();
    callback_group->trigger_notify_guard_condition();
  } catch (const rclcpp::exceptions::RCLError& ex) {
    throw std::runtime_error(
        std::string("failed to notify wait set on publisher creation: ") + ex.what());
  }
}

/**
 * @brief 创建一个订阅者 (Create a subscription)
 *
 * @param topic_name 订阅的主题名称 (Topic name to subscribe)
 * @param subscription_factory 用于创建订阅者的工厂对象 (Subscription factory for creating the
 * subscriber)
 * @param qos 服务质量配置 (Quality of Service configuration)
 * @return rclcpp::SubscriptionBase::SharedPtr 创建的订阅者 (Created subscription)
 */
rclcpp::SubscriptionBase::SharedPtr NodeTopics::create_subscription(
    const std::string& topic_name,
    const rclcpp::SubscriptionFactory& subscription_factory,
    const rclcpp::QoS& qos) {
  // 使用工厂创建特定类型的订阅，但返回一个 SubscriptionBase 类型
  return subscription_factory.create_typed_subscription(node_base_, topic_name, qos);
}

/**
 * @brief 添加订阅者到回调组 (Add a subscription to a callback group)
 *
 * @param subscription 要添加的订阅者 (Subscription to add)
 * @param callback_group 回调组 (Callback group)
 */
void NodeTopics::add_subscription(
    rclcpp::SubscriptionBase::SharedPtr subscription,
    rclcpp::CallbackGroup::SharedPtr callback_group) {
  // 将订阅分配给回调组 (Assign the subscription to a callback group)
  if (callback_group) {
    if (!node_base_->callback_group_in_node(callback_group)) {
      // TODO(jacquelinekay): 使用自定义异常 (Use custom exception)
      throw std::runtime_error("Cannot create subscription, callback group not in node.");
    }
  } else {
    callback_group = node_base_->get_default_callback_group();
  }

  // 将订阅添加到回调组 (Add the subscription to the callback group)
  callback_group->add_subscription(subscription);

  // 遍历订阅的事件处理程序并添加到回调组 (Iterate through the event handlers of the subscription
  // and add them to the callback group)
  for (auto& key_event_pair : subscription->get_event_handlers()) {
    auto subscription_event = key_event_pair.second;
    callback_group->add_waitable(subscription_event);
  }

  // 获取订阅的内部进程可等待对象 (Get the intra-process waitable object of the subscription)
  auto intra_process_waitable = subscription->get_intra_process_waitable();
  if (nullptr != intra_process_waitable) {
    // 添加到回调组以便通知内部进程消息 (Add to the callback group to be notified about
    // intra-process messages)
    callback_group->add_waitable(intra_process_waitable);
  }

  // 使用父节点通知执行器已创建新订阅 (Notify the executor that a new subscription was created using
  // the parent Node)
  auto& node_gc = node_base_->get_notify_guard_condition();
  try {
    node_gc.trigger();
    callback_group->trigger_notify_guard_condition();
  } catch (const rclcpp::exceptions::RCLError& ex) {
    throw std::runtime_error(
        std::string("failed to notify wait set on subscription creation: ") + ex.what());
  }
}

/**
 * @brief 获取节点基本接口 (Get node base interface)
 *
 * @return rclcpp::node_interfaces::NodeBaseInterface* 返回节点基本接口指针 (Return pointer to node
 * base interface)
 */
rclcpp::node_interfaces::NodeBaseInterface* NodeTopics::get_node_base_interface() const {
  // 返回节点基本接口指针 (Return pointer to node base interface)
  return node_base_;
}

/**
 * @brief 获取节点定时器接口 (Get node timers interface)
 *
 * @return rclcpp::node_interfaces::NodeTimersInterface* 返回节点定时器接口指针 (Return pointer to
 * node timers interface)
 */
rclcpp::node_interfaces::NodeTimersInterface* NodeTopics::get_node_timers_interface() const {
  // 返回节点定时器接口指针 (Return pointer to node timers interface)
  return node_timers_;
}

/**
 * @brief 解析主题名称 (Resolve topic name)
 *
 * @param name 输入的主题名称 (Input topic name)
 * @param only_expand 是否仅扩展命名空间 (Whether to only expand namespace)
 * @return std::string 返回解析后的主题名称 (Return the resolved topic name)
 */
std::string NodeTopics::resolve_topic_name(const std::string& name, bool only_expand) const {
  // 调用节点基本接口的 resolve_topic_or_service_name 方法来解析主题或服务名称
  // (Call the resolve_topic_or_service_name method of the node base interface to resolve the topic
  // or service name)
  return node_base_->resolve_topic_or_service_name(name, false, only_expand);
}

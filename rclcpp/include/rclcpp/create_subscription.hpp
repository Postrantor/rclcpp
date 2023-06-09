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

#ifndef RCLCPP__CREATE_SUBSCRIPTION_HPP_
#define RCLCPP__CREATE_SUBSCRIPTION_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/detail/resolve_enable_topic_statistics.hpp"
#include "rclcpp/node_interfaces/get_node_timers_interface.hpp"
#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/topic_statistics/subscription_topic_statistics.hpp"
#include "rmw/qos_profiles.h"

namespace rclcpp {

namespace detail {
/**
 * @brief 创建一个订阅者 (Create a subscription)
 *
 * @tparam MessageT 消息类型 (Message type)
 * @tparam CallbackT 回调函数类型 (Callback function type)
 * @tparam AllocatorT 内存分配器类型 (Allocator type)
 * @tparam SubscriptionT 订阅者类型 (Subscription type)
 * @tparam MessageMemoryStrategyT 消息内存策略类型 (Message memory strategy
 * type)
 * @tparam NodeParametersT 节点参数类型 (Node parameters type)
 * @tparam NodeTopicsT 节点主题类型 (Node topics type)
 * @tparam ROSMessageType ROS消息类型，默认为SubscriptionT::ROSMessageType (ROS
 * message type, default to SubscriptionT::ROSMessageType)
 * @param node_parameters 节点参数引用 (Node parameters reference)
 * @param node_topics 节点主题引用 (Node topics reference)
 * @param topic_name 主题名称 (Topic name)
 * @param qos 服务质量配置 (Quality of Service configuration)
 * @param callback 回调函数 (Callback function)
 * @param options 订阅选项，带有分配器的默认值 (Subscription options with
 * allocator defaults)
 * @param msg_mem_strat 消息内存策略共享指针，默认为创建默认值 (Message memory
 * strategy shared pointer, default to create_default)
 * @return std::shared_ptr<SubscriptionT> 创建的订阅者指针 (Created subscription
 * pointer)
 */
template <
    typename MessageT,
    typename CallbackT,
    typename AllocatorT,
    typename SubscriptionT,
    typename MessageMemoryStrategyT,
    typename NodeParametersT,
    typename NodeTopicsT,
    typename ROSMessageType = typename SubscriptionT::ROSMessageType>
typename std::shared_ptr<SubscriptionT> create_subscription(
    NodeParametersT& node_parameters,
    NodeTopicsT& node_topics,
    const std::string& topic_name,
    const rclcpp::QoS& qos,
    CallbackT&& callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options =
        (rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()),
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strat =
        (MessageMemoryStrategyT::create_default())) {
  // 获取节点主题接口 (Get the node topics interface)
  using rclcpp::node_interfaces::get_node_topics_interface;
  auto node_topics_interface = get_node_topics_interface(node_topics);

  // 定义订阅主题统计指针并初始化为空 (Define subscription topic statistics
  // pointer and initialize to nullptr)
  std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics<ROSMessageType>>
      subscription_topic_stats = nullptr;

  // 如果启用了主题统计功能 (If topic statistics feature is enabled)
  if (rclcpp::detail::resolve_enable_topic_statistics(
          options, *node_topics_interface->get_node_base_interface())) {
    // 检查发布周期是否大于0，否则抛出异常 (Check if publish period is greater
    // than 0, otherwise throw an exception)
    if (options.topic_stats_options.publish_period <= std::chrono::milliseconds(0)) {
      throw std::invalid_argument(
          "topic_stats_options.publish_period must be greater than 0, "
          "specified value of " +
          std::to_string(options.topic_stats_options.publish_period.count()) + " ms");
    }

    // 创建统计消息的发布者 (Create a publisher for statistics messages)
    std::shared_ptr<Publisher<statistics_msgs::msg::MetricsMessage>> publisher =
        rclcpp::detail::create_publisher<statistics_msgs::msg::MetricsMessage>(
            node_parameters, node_topics_interface, options.topic_stats_options.publish_topic, qos);

    // 创建订阅主题统计实例 (Create a subscription topic statistics instance)
    subscription_topic_stats =
        std::make_shared<rclcpp::topic_statistics::SubscriptionTopicStatistics<ROSMessageType>>(
            node_topics_interface->get_node_base_interface()->get_name(), publisher);

    // 创建弱指针以捕获订阅主题统计 (Create a weak pointer to capture the
    // subscription topic statistics)
    std::weak_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics<ROSMessageType>>
        weak_subscription_topic_stats(subscription_topic_stats);

    // 定义回调函数，用于发布消息并重置测量值 (Define a callback function for
    // publishing messages and resetting measurements)
    auto sub_call_back = [weak_subscription_topic_stats]() {
      auto subscription_topic_stats = weak_subscription_topic_stats.lock();
      if (subscription_topic_stats) {
        subscription_topic_stats->publish_message_and_reset_measurements();
      }
    };

    // 获取节点定时器接口 (Get the node timers interface)
    auto node_timer_interface = node_topics_interface->get_node_timers_interface();

    // 创建墙钟定时器 (Create a wall timer)
    auto timer = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            options.topic_stats_options.publish_period),
        sub_call_back, options.callback_group, node_topics_interface->get_node_base_interface(),
        node_timer_interface);

    // 设置发布者定时器 (Set the publisher timer)
    subscription_topic_stats->set_publisher_timer(timer);
  }

  // 创建订阅工厂 (Create a subscription factory)
  auto factory = rclcpp::create_subscription_factory<MessageT>(
      std::forward<CallbackT>(callback), options, msg_mem_strat, subscription_topic_stats);

  // 获取实际的QoS配置，如果有QoS覆盖选项，则声明QoS参数
  const rclcpp::QoS& actual_qos = options.qos_overriding_options.get_policy_kinds().size()
                                      ? rclcpp::detail::declare_qos_parameters(
                                            options.qos_overriding_options, node_parameters,
                                            node_topics_interface->resolve_topic_name(topic_name),
                                            qos, rclcpp::detail::SubscriptionQosParametersTraits{})
                                      : qos;

  // 使用工厂创建订阅 (Create a subscription using the factory)
  auto sub = node_topics_interface->create_subscription(topic_name, factory, actual_qos);

  // 将订阅添加到节点主题接口 (Add the subscription to the node topics
  // interface)
  node_topics_interface->add_subscription(sub, options.callback_group);

  // 返回创建的订阅者指针 (Return the created subscription pointer)
  return std::dynamic_pointer_cast<SubscriptionT>(sub);
}

}  // namespace detail

/// 创建并返回给定 MessageT 类型的订阅。
/// Create and return a subscription of the given MessageT type.
/**
 * NodeT 类型只需要有一个名为 get_node_topics_interface() 的方法，
 * 该方法返回一个指向 NodeTopicsInterface 的 shared_ptr，或者本身就是一个
 * NodeTopicsInterface 指针。 The NodeT type only needs to have a method called
 * get_node_topics_interface() which returns a shared_ptr to a
 * NodeTopicsInterface, or be a NodeTopicsInterface pointer itself.
 *
 * 如果 `options.qos_overriding_options` 启用了 qos 参数覆盖，
 * NodeT 还必须有一个名为 get_node_parameters_interface() 的方法，
 * 该方法返回一个指向 NodeParametersInterface 的 shared_ptr。
 * In case `options.qos_overriding_options` is enabling qos parameter overrides,
 * NodeT must also have a method called get_node_parameters_interface()
 * which returns a shared_ptr to a NodeParametersInterface.
 *
 * \tparam MessageT
 * \tparam CallbackT
 * \tparam AllocatorT
 * \tparam SubscriptionT
 * \tparam MessageMemoryStrategyT
 * \tparam NodeT
 * \param node
 * \param topic_name
 * \param qos
 * \param callback
 * \param options
 * \param msg_mem_strat
 * \return 创建的订阅
 * \throws std::invalid_argument 如果启用了主题统计信息并且发布周期小于等于零。
 */
template <
    typename MessageT,
    typename CallbackT,
    typename AllocatorT = std::allocator<void>,
    typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
    typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType,
    typename NodeT>
typename std::shared_ptr<SubscriptionT> create_subscription(
    NodeT& node,
    const std::string& topic_name,
    const rclcpp::QoS& qos,
    CallbackT&& callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options =
        (rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()),
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strat =
        (MessageMemoryStrategyT::create_default())) {
  // 调用 detail 命名空间中的 create_subscription 函数来创建订阅。
  // Call the create_subscription function in the detail namespace to create the
  // subscription.
  return rclcpp::detail::create_subscription<
      MessageT, CallbackT, AllocatorT, SubscriptionT, MessageMemoryStrategyT>(
      node, node, topic_name, qos, std::forward<CallbackT>(callback), options, msg_mem_strat);
}

/// 创建并返回指定 MessageT 类型的订阅。
/**
 * 参见 \ref create_subscription()。
 */
template <
    typename MessageT,   // 消息类型，用于订阅的数据类型
    typename CallbackT,  // 回调类型，当接收到消息时将调用此回调
    typename AllocatorT = std::allocator<void>,  // 分配器类型，默认为
                                                 // std::allocator<void>
    typename SubscriptionT = rclcpp::Subscription<
        MessageT,
        AllocatorT>,                // 订阅类型，默认为
                                    // rclcpp::Subscription<MessageT,
                                    // AllocatorT>
    typename MessageMemoryStrategyT = typename SubscriptionT::
        MessageMemoryStrategyType>  // 消息内存策略类型，默认为 SubscriptionT 的
                                    // MessageMemoryStrategyType
typename std::shared_ptr<SubscriptionT> create_subscription(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr&
        node_parameters,  // 节点参数接口共享指针，用于访问节点参数
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr&
        node_topics,      // 节点主题接口共享指针，用于管理节点的发布和订阅
    const std::string& topic_name,  // 主题名称，订阅的主题名
    const rclcpp::QoS& qos,         // QoS 设置，用于配置订阅的质量保证
    CallbackT&& callback,           // 回调函数，当接收到消息时将调用此回调
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>&
        options =                   // 订阅选项，包含订阅的各种配置
    (rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()),
    typename MessageMemoryStrategyT::SharedPtr
        msg_mem_strat =  // 消息内存策略共享指针，用于处理消息内存分配和回收
    (MessageMemoryStrategyT::create_default())) {
  // 调用 rclcpp::detail::create_subscription 创建并返回订阅实例
  return rclcpp::detail::create_subscription<
      MessageT, CallbackT, AllocatorT, SubscriptionT, MessageMemoryStrategyT>(
      node_parameters, node_topics, topic_name, qos, std::forward<CallbackT>(callback), options,
      msg_mem_strat);
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_SUBSCRIPTION_HPP_

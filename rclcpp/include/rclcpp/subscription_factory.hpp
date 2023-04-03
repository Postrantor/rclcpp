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

#ifndef RCLCPP__SUBSCRIPTION_FACTORY_HPP_
#define RCLCPP__SUBSCRIPTION_FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rcl/subscription.h"
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/get_message_type_support_handle.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp/topic_statistics/subscription_topic_statistics.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

namespace rclcpp {

/// Factory 包含用于创建 Subscription<MessageT> 的函数。
/**
 * 此工厂类用于封装在非模板类中创建 Message 类型特定订阅期间使用的模板生成函数。
 *
 * 它是通过 create_subscription_factory 函数创建的，通常从 Node 类的带有模板的 "create_subscription"
 * 方法中调用， 并传递给 NodeTopics 类的非模板 "create_subscription"
 * 方法，在那里它用于创建和设置订阅。
 *
 * 它还处理 Subscriptions 的两步构建过程，先调用构造函数，然后调用 post_init_setup() 方法。
 */
struct SubscriptionFactory {
  // 创建一个 Subscription<MessageT> 对象并将其作为 SubscriptionBase 返回。
  // Creates a Subscription<MessageT> object and returns it as a SubscriptionBase.
  using SubscriptionFactoryFunction = std::function<rclcpp::SubscriptionBase::SharedPtr(
      rclcpp::node_interfaces::NodeBaseInterface* node_base,
      const std::string& topic_name,
      const rclcpp::QoS& qos)>;

  const SubscriptionFactoryFunction create_typed_subscription;
};

/// 返回一个设置好的 SubscriptionFactory，以创建 SubscriptionT<MessageT, AllocatorT>。
/**
 * \param[in] callback 用户定义的回调函数，用于接收消息
 * \param[in] options 创建订阅的其他选项
 * \param[in] msg_mem_strat 用于分配消息的消息内存策略
 * \param[in] subscription_topic_stats 可选的主题统计回调
 */
template <
    typename MessageT,
    typename CallbackT,
    typename AllocatorT,
    typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
    typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType,
    typename ROSMessageType = typename SubscriptionT::ROSMessageType>
SubscriptionFactory create_subscription_factory(
    CallbackT&& callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options,
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strat,
    std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics<ROSMessageType>>
        subscription_topic_stats = nullptr) {
  auto allocator = options.get_allocator();

  using rclcpp::AnySubscriptionCallback;
  AnySubscriptionCallback<MessageT, AllocatorT> any_subscription_callback(*allocator);
  any_subscription_callback.set(std::forward<CallbackT>(callback));

  SubscriptionFactory factory{
      // 工厂函数，用于创建特定于 MessageT 的 SubscriptionT
      [options, msg_mem_strat, any_subscription_callback, subscription_topic_stats](
          rclcpp::node_interfaces::NodeBaseInterface* node_base, const std::string& topic_name,
          const rclcpp::QoS& qos) -> rclcpp::SubscriptionBase::SharedPtr {
        using rclcpp::Subscription;
        using rclcpp::SubscriptionBase;

        auto sub = Subscription<MessageT, AllocatorT>::make_shared(
            node_base, rclcpp::get_message_type_support_handle<MessageT>(), topic_name, qos,
            any_subscription_callback, options, msg_mem_strat, subscription_topic_stats);
        // 这用于设置诸如内部进程通信之类的东西，
        // 需要调用 this->shared_from_this()，而不能从构造函数中调用。
        sub->post_init_setup(node_base, qos, options);
        auto sub_base_ptr = std::dynamic_pointer_cast<SubscriptionBase>(sub);
        return sub_base_ptr;
      }};

  // 返回填充好的工厂
  return factory;
}

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_FACTORY_HPP_

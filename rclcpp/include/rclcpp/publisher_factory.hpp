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

#ifndef RCLCPP__PUBLISHER_FACTORY_HPP_
#define RCLCPP__PUBLISHER_FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rcl/publisher.h"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

namespace rclcpp {

/// Factory with functions used to create a MessageT specific PublisherT.
/**
 * 这个工厂类用于封装在非模板类中创建特定于消息类型的发布器时使用的模板生成函数。
 * This factory class is used to encapsulate the template generated functions
 * which are used during the creation of a Message type specific publisher
 * within a non-templated class.
 *
 * 它是通过 create_publisher_factory 函数创建的，通常是从 Node 类上的模板化 "create_publisher"
 * 方法调用的， 并传递给 NodeTopics 类上的非模板化 "create_publisher"
 * 方法，在那里它被用来创建和设置发布器。 It is created using the create_publisher_factory function,
 * which is usually called from a templated "create_publisher" method on the Node class, and is
 * passed to the non-templated "create_publisher" method on the NodeTopics class where it is used to
 * create and setup the Publisher.
 *
 * 它还处理了发布器的两步构建过程，首先调用构造函数，然后调用 post_init_setup() 方法。
 * It also handles the two step construction of Publishers, first calling
 * the constructor and then the post_init_setup() method.
 */
struct PublisherFactory {
  // 创建一个 PublisherT<MessageT, ...> 发布器对象并将其作为 PublisherBase 返回。
  using PublisherFactoryFunction = std::function<rclcpp::PublisherBase::SharedPtr(
      rclcpp::node_interfaces::NodeBaseInterface* node_base,
      const std::string& topic_name,
      const rclcpp::QoS& qos)>;

  const PublisherFactoryFunction create_typed_publisher;
};

/// 返回一个带有为创建 PublisherT<MessageT, AllocatorT> 设置的函数的 PublisherFactory。
template <typename MessageT, typename AllocatorT, typename PublisherT>
PublisherFactory create_publisher_factory(
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options) {
  PublisherFactory factory{
      // 创建特定于 MessageT 的 PublisherT 的工厂函数
      [options](
          rclcpp::node_interfaces::NodeBaseInterface* node_base, const std::string& topic_name,
          const rclcpp::QoS& qos) -> std::shared_ptr<PublisherT> {
        // 使用给定参数创建新的 PublisherT 实例
        auto publisher = std::make_shared<PublisherT>(node_base, topic_name, qos, options);
        // 这用于设置诸如内部进程通信之类的东西，它需要调用 this->shared_from_this()，
        // 该方法不能从构造函数中调用。
        publisher->post_init_setup(node_base, topic_name, qos, options);
        return publisher;
      }};

  // 现在已经填充了工厂，返回它
  return factory;
}

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_FACTORY_HPP_

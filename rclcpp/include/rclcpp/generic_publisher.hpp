// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2021, Apex.AI Inc.
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

#ifndef RCLCPP__GENERIC_PUBLISHER_HPP_
#define RCLCPP__GENERIC_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcpputils/shared_library.hpp"
#include "rmw/rmw.h"

namespace rclcpp {

/// 用于发布在编译时类型未知的序列化消息的发布器。
/// Publisher for serialized messages whose type is not known at compile time.
/**
 * 由于类型在编译时未知，这不是模板，需要根据类型名称识别和加载包含类型支持信息的动态库。
 * Since the type is not known at compile time, this is not a template, and the dynamic library
 * containing type support information has to be identified and loaded based on the type name.
 *
 * 它不支持进程内处理。
 * It does not support intra-process handling.
 */
class GenericPublisher : public rclcpp::PublisherBase {
public:
  // cppcheck-suppress unknownMacro
  RCLCPP_SMART_PTR_DEFINITIONS(GenericPublisher)

  /// 构造函数。
  /// Constructor.
  /**
   * 为了正确地发布到一个主题，需要将此发布器添加到传递给此构造函数的节点的node_topic_interface中。
   * In order to properly publish to a topic, this publisher needs to be added to
   * the node_topic_interface of the node passed into this constructor.
   *
   * 参见rclcpp::Node::create_generic_publisher()或rclcpp::create_generic_publisher()，用于
   * 创建此类的实例并将其添加到node_topic_interface。
   * \sa rclcpp::Node::create_generic_publisher() or rclcpp::create_generic_publisher() for
   * creating an instance of this class and adding it to the node_topic_interface.
   *
   * \param node_base 指向父节点NodeBaseInterface的指针
   * \param ts_lib 类型支持库，需要与topic_type相对应
   * \param topic_name 主题名称
   * \param topic_type 主题类型
   * \param qos QoS设置
   * \param options 发布器选项。
   * 目前并非所有发布器选项都受到尊重，对于此发布器来说，唯一相关的选项是`event_callbacks`、
   * `use_default_callbacks`和`callback_group`。
   * \param options %Publisher options.
   * Not all publisher options are currently respected, the only relevant options for this
   * publisher are `event_callbacks`, `use_default_callbacks`, and `%callback_group`.
   */
  template <typename AllocatorT = std::allocator<void>>
  GenericPublisher(
      rclcpp::node_interfaces::NodeBaseInterface* node_base,
      std::shared_ptr<rcpputils::SharedLibrary> ts_lib,
      const std::string& topic_name,
      const std::string& topic_type,
      const rclcpp::QoS& qos,
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options)
      : rclcpp::PublisherBase(
            node_base,
            topic_name,
            *rclcpp::get_typesupport_handle(topic_type, "rosidl_typesupport_cpp", *ts_lib),
            options.template to_rcl_publisher_options<rclcpp::SerializedMessage>(qos),
            // NOTE(methylDragon): Passing these args separately is necessary for event binding
            options.event_callbacks,
            options.use_default_callbacks),
        ts_lib_(ts_lib) {}

  // 公共成员函数
  RCLCPP_PUBLIC
  // 虚析构函数
  virtual ~GenericPublisher() = default;

  /// 发布一个rclcpp::SerializedMessage。
  /// Publish a rclcpp::SerializedMessage.
  RCLCPP_PUBLIC
  void publish(const rclcpp::SerializedMessage& message);

  /**
   * 在反序列化后通过借用的消息发布rclcpp::SerializedMessage。
   * Publish a rclcpp::SerializedMessage via loaned message after de-serialization.
   *
   * \param message 一个序列化的消息
   * \param message a serialized message
   * \throws 任何rclcpp::exceptions::throw_from_rcl_error能显示的内容
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can show
   */
  RCLCPP_PUBLIC
  void publish_as_loaned_msg(const rclcpp::SerializedMessage& message);

private:
  // 类型支持库应保持加载状态，因此将其存储在GenericPublisher中
  // The type support library should stay loaded, so it is stored in the GenericPublisher
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;

  void* borrow_loaned_message();
  void deserialize_message(
      const rmw_serialized_message_t& serialized_message, void* deserialized_msg);
  void publish_loaned_message(void* loaned_message);
};

}  // namespace rclcpp

#endif  // RCLCPP__GENERIC_PUBLISHER_HPP_

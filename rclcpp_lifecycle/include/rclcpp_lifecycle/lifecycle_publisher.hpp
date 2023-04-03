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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_PUBLISHER_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"

namespace rclcpp_lifecycle {
/// \brief 子类，继承自 rclcpp Publisher 类。 (Child class of rclcpp Publisher class.)
/**
 * 覆盖所有发布器功能以检查启用/禁用状态。 (Overrides all publisher functions to check for
 * enabled/disabled state.)
 */
template <typename MessageT, typename Alloc = std::allocator<void>>
class LifecyclePublisher : public SimpleManagedEntity, public rclcpp::Publisher<MessageT, Alloc> {
public:
  // 定义智能指针类型 (Define smart pointer types)
  RCLCPP_SMART_PTR_DEFINITIONS(LifecyclePublisher)

  // 定义消息分配器相关类型 (Define message allocator related types)
  using MessageAllocTraits = rclcpp::allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;

  /// \brief 构造函数 (Constructor)
  /**
   * \param[in] node_base ROS2 节点基础接口指针 (Pointer to the ROS2 node base interface)
   * \param[in] topic 发布的主题名称 (Topic name to publish)
   * \param[in] qos 消息质量参数 (Quality of Service parameters for the messages)
   * \param[in] options 发布器选项 (Options for the publisher)
   */
  LifecyclePublisher(
      rclcpp::node_interfaces::NodeBaseInterface* node_base,
      const std::string& topic,
      const rclcpp::QoS& qos,
      const rclcpp::PublisherOptionsWithAllocator<Alloc>& options)
      : rclcpp::Publisher<MessageT, Alloc>(node_base, topic, qos, options),
        should_log_(true),
        logger_(rclcpp::get_logger("LifecyclePublisher")) {}

  /// \brief 析构函数 (Destructor)
  ~LifecyclePublisher() {}

  /// 生命周期发布器的 publish 函数 (LifecyclePublisher publish function)
  /**
   * publish 函数检查通信是否被启用或禁用，并将消息转发到实际的 rclcpp Publisher 基类
   * (The publish function checks whether the communication
   * was enabled or disabled and forwards the message
   * to the actual rclcpp Publisher base class)
   *
   * @param msg 一个具有 MessageDeleter 删除器的 std::unique_ptr 类型的消息对象
   *            (A std::unique_ptr of a message object with a MessageDeleter deleter)
   */
  virtual void publish(std::unique_ptr<MessageT, MessageDeleter> msg) {
    // 如果生命周期发布器未激活，则记录日志并返回
    // (If the LifecyclePublisher is not activated, log it and return)
    if (!this->is_activated()) {
      log_publisher_not_enabled();
      return;
    }
    // 将消息转发到实际的 rclcpp Publisher 基类
    // (Forward the message to the actual rclcpp Publisher base class)
    rclcpp::Publisher<MessageT, Alloc>::publish(std::move(msg));
  }

  /// 生命周期发布器的 publish 函数 (LifecyclePublisher publish function)
  /**
   * publish 函数检查通信是否被启用或禁用，并将消息转发到实际的 rclcpp Publisher 基类
   * (The publish function checks whether the communication
   * was enabled or disabled and forwards the message
   * to the actual rclcpp Publisher base class)
   *
   * @param msg 消息对象的常量引用 (A const reference to a message object)
   */
  virtual void publish(const MessageT& msg) {
    // 如果生命周期发布器未激活，则记录日志并返回
    // (If the LifecyclePublisher is not activated, log it and return)
    if (!this->is_activated()) {
      log_publisher_not_enabled();
      return;
    }
    // 将消息转发到实际的 rclcpp Publisher 基类
    // (Forward the message to the actual rclcpp Publisher base class)
    rclcpp::Publisher<MessageT, Alloc>::publish(msg);
  }

  // 当激活时调用此函数 (Called when activating)
  void on_activate() override {
    // 调用 SimpleManagedEntity 的 on_activate 函数
    // (Call the on_activate function of SimpleManagedEntity)
    SimpleManagedEntity::on_activate();

    // 设置 should_log_ 为 true，表示在激活状态下允许记录日志
    // (Set should_log_ to true, indicating that logging is allowed when activated)
    should_log_ = true;
  }

private:
  /// 生命周期发布者日志辅助函数 (LifecyclePublisher log helper function)
  /**
   * @brief 辅助函数，用于记录发布者无法发布的消息，因为它没有启用。(Helper function that logs a
   * message saying that publisher can't publish because it's not enabled.)
   */
  void log_publisher_not_enabled() {
    // 如果我们不打算记录日志，则不执行任何操作 (Nothing to do if we are not meant to log)
    if (!should_log_) {
      return;
    }

    // 记录消息 (Log the message)
    RCLCPP_WARN(
        logger_,
        "尝试在主题 '%s' 上发布消息，但发布者未激活",  // (Trying to publish message on the topic
                                                       // '%s', but the publisher is not activated)
        this->get_topic_name());

    // 在标志再次启用之前停止记录日志 (We stop logging until the flag gets enabled again)
    should_log_ = false;
  }

  // 是否应记录日志的标志，默认为true (Flag for whether or not to log, default is true)
  bool should_log_ = true;

  // rclcpp::Logger实例，用于记录日志信息 (rclcpp::Logger instance for logging messages)
  rclcpp::Logger logger_;
};

}  // namespace rclcpp_lifecycle

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_PUBLISHER_HPP_

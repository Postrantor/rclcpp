// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BUFFER_HPP_
#define RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BUFFER_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/guard_condition.h"
#include "rcl/wait.h"
#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/create_intra_process_buffer.hpp"
#include "rclcpp/experimental/ros_message_intra_process_buffer.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/type_support_decl.hpp"

namespace rclcpp {
namespace experimental {

/**
 * @brief 订阅内部进程缓冲区类（Subscription Intra-Process Buffer class）
 *
 * @tparam SubscribedType 订阅的数据类型（Subscribed data type）
 * @tparam Alloc 分配器类型，默认为 std::allocator<SubscribedType>（Allocator type, default is
 * std::allocator<SubscribedType>）
 * @tparam Deleter 删除器类型，默认为 std::default_delete<SubscribedType>（Deleter type, default is
 * std::default_delete<SubscribedType>）
 * @tparam ROSMessageType 消息类型，如果 MessageT 是 TypeAdapter，则为
 * MessageT::ros_message_type，否则为 MessageT（Message type, MessageT::ros_message_type if MessageT
 * is a TypeAdapter, otherwise just MessageT）
 */
template <
    typename SubscribedType,
    typename Alloc = std::allocator<SubscribedType>,
    typename Deleter = std::default_delete<SubscribedType>,
    typename ROSMessageType = SubscribedType>
class SubscriptionIntraProcessBuffer
    : public SubscriptionROSMsgIntraProcessBuffer<
          ROSMessageType,
          typename allocator::AllocRebind<ROSMessageType, Alloc>::allocator_type,
          allocator::Deleter<
              typename allocator::AllocRebind<ROSMessageType, Alloc>::allocator_type,
              ROSMessageType>> {
public:
  // 定义智能指针类型（Define smart pointer types）
  RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionIntraProcessBuffer)

  using SubscribedTypeAllocatorTraits = allocator::AllocRebind<SubscribedType, Alloc>;
  using SubscribedTypeAllocator = typename SubscribedTypeAllocatorTraits::allocator_type;
  using SubscribedTypeDeleter = allocator::Deleter<SubscribedTypeAllocator, SubscribedType>;

  using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<ROSMessageType, Alloc>;
  using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
  using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;

  using ConstMessageSharedPtr = std::shared_ptr<const ROSMessageType>;
  using MessageUniquePtr = std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>;

  using ConstDataSharedPtr = std::shared_ptr<const SubscribedType>;
  using SubscribedTypeUniquePtr = std::unique_ptr<SubscribedType, SubscribedTypeDeleter>;

  using BufferUniquePtr = typename rclcpp::experimental::buffers::
      IntraProcessBuffer<SubscribedType, Alloc, SubscribedTypeDeleter>::UniquePtr;

  /**
   * @brief 构造函数，创建一个 SubscriptionIntraProcessBuffer 对象
   * Constructor, creates a SubscriptionIntraProcessBuffer object
   *
   * @param allocator 分配器对象，用于分配内存
   *        Allocator object for memory allocation
   * @param context ROS2 上下文对象
   *        ROS2 context object
   * @param topic_name 订阅的话题名称
   *        Topic name to subscribe
   * @param qos_profile QoS 配置文件
   *        QoS profile
   * @param buffer_type 缓冲区类型
   *        Buffer type
   */
  SubscriptionIntraProcessBuffer(
      std::shared_ptr<Alloc> allocator,
      rclcpp::Context::SharedPtr context,
      const std::string& topic_name,
      const rclcpp::QoS& qos_profile,
      rclcpp::IntraProcessBufferType buffer_type)
      : SubscriptionROSMsgIntraProcessBuffer<
            ROSMessageType,
            ROSMessageTypeAllocator,
            ROSMessageTypeDeleter>(context, topic_name, qos_profile),
        subscribed_type_allocator_(*allocator) {
    // 设置分配器以供删除器使用
    // Set allocator for the deleter
    allocator::set_allocator_for_deleter(&subscribed_type_deleter_, &subscribed_type_allocator_);

    // 创建进程内缓冲区
    // Create the intra-process buffer
    buffer_ = rclcpp::experimental::create_intra_process_buffer<
        SubscribedType, Alloc, SubscribedTypeDeleter>(
        buffer_type, qos_profile, std::make_shared<Alloc>(subscribed_type_allocator_));
  }

  /**
   * @brief 检查是否准备好从 wait_set 中读取数据
   * Check if it's ready to read data from the wait_set
   *
   * @param wait_set 等待集合
   *        Wait set
   * @return 是否准备好读取数据
   *         Whether it's ready to read data
   */
  bool is_ready(rcl_wait_set_t* wait_set) override {
    (void)wait_set;
    return buffer_->has_data();
  }

  /**
   * @brief 将 ROS 消息转换为订阅类型的唯一指针
   *        Convert ROS message to unique pointer of subscribed type
   *
   * @param msg ROS 消息
   *             ROS message
   * @return 订阅类型的唯一指针
   *         Unique pointer of subscribed type
   */
  SubscribedTypeUniquePtr convert_ros_message_to_subscribed_type_unique_ptr(
      const ROSMessageType& msg) {
    // 如果 SubscribedType 和 ROSMessageType 不是相同的类型，那么执行以下代码块
    // If SubscribedType and ROSMessageType are not the same type, execute the following code block
    if constexpr (!std::is_same<SubscribedType, ROSMessageType>::value) {
      // 使用 subscribed_type_allocator_ 分配内存空间，并将指针赋值给 ptr
      // Allocate memory space using subscribed_type_allocator_ and assign the pointer to ptr
      auto ptr = SubscribedTypeAllocatorTraits::allocate(subscribed_type_allocator_, 1);

      // 使用 subscribed_type_allocator_ 在分配的内存空间上构造 SubscribedType 对象
      // Construct a SubscribedType object on the allocated memory space using
      // subscribed_type_allocator_
      SubscribedTypeAllocatorTraits::construct(subscribed_type_allocator_, ptr);

      // 调用 TypeAdapter 的 convert_to_custom 方法将 ROSMessageType 转换为 SubscribedType
      // Call the convert_to_custom method of TypeAdapter to convert ROSMessageType to
      // SubscribedType
      rclcpp::TypeAdapter<SubscribedType, ROSMessageType>::convert_to_custom(msg, *ptr);

      // 返回一个 SubscribedType 类型的唯一指针，使用 subscribed_type_deleter_ 作为删除器
      // Return a unique pointer of SubscribedType type, using subscribed_type_deleter_ as the
      // deleter
      return SubscribedTypeUniquePtr(ptr, subscribed_type_deleter_);
    } else {
      // 如果没有 TypeAdapter，抛出运行时错误
      // Throw a runtime error if there is no TypeAdapter
      throw std::runtime_error(
          "convert_ros_message_to_subscribed_type_unique_ptr "
          "unexpectedly called without TypeAdapter");
    }
  }

  /**
   * @brief 提供进程内消息
   * Provide intra-process message
   *
   * @param message 常量消息共享指针
   *        Const message shared pointer
   */
  void provide_intra_process_message(ConstMessageSharedPtr message) override {
    if constexpr (std::is_same<SubscribedType, ROSMessageType>::value) {
      buffer_->add_shared(std::move(message));
      trigger_guard_condition();
    } else {
      buffer_->add_shared(convert_ros_message_to_subscribed_type_unique_ptr(*message));
      trigger_guard_condition();
    }
    this->invoke_on_new_message();
  }

  /**
   * @brief 提供内部进程消息 (Provide intra-process message)
   *
   * @param message 消息的独特指针 (MessageUniquePtr, unique pointer to the message)
   */
  void provide_intra_process_message(MessageUniquePtr message) override {
    // 如果订阅类型与ROS消息类型相同 (If SubscribedType is the same as ROSMessageType)
    if constexpr (std::is_same<SubscribedType, ROSMessageType>::value) {
      // 将唯一的消息添加到缓冲区中 (Add the unique message to the buffer)
      buffer_->add_unique(std::move(message));
      // 触发守护条件 (Trigger the guard condition)
      trigger_guard_condition();
    } else {
      // 将ROS消息转换为订阅类型的唯一指针，并将其添加到缓冲区中 (Convert the ROS message to a
      // unique pointer of SubscribedType and add it to the buffer)
      buffer_->add_unique(convert_ros_message_to_subscribed_type_unique_ptr(*message));
      // 触发守护条件 (Trigger the guard condition)
      trigger_guard_condition();
    }
    // 调用新消息处理函数 (Invoke the new message handling function)
    this->invoke_on_new_message();
  }

  /**
   * @brief 提供内部进程数据 (Provide intra-process data)
   *
   * @param message 数据的共享指针 (ConstDataSharedPtr, shared pointer to the data)
   */
  void provide_intra_process_data(ConstDataSharedPtr message) {
    // 将共享的数据添加到缓冲区中 (Add the shared data to the buffer)
    buffer_->add_shared(std::move(message));
    // 触发守护条件 (Trigger the guard condition)
    trigger_guard_condition();
    // 调用新消息处理函数 (Invoke the new message handling function)
    this->invoke_on_new_message();
  }

  /**
   * @brief 提供内部进程数据 (Provide intra-process data)
   *
   * @param message 数据的独特指针 (SubscribedTypeUniquePtr, unique pointer to the data)
   */
  void provide_intra_process_data(SubscribedTypeUniquePtr message) {
    // 将唯一的数据添加到缓冲区中 (Add the unique data to the buffer)
    buffer_->add_unique(std::move(message));
    // 触发守护条件 (Trigger the guard condition)
    trigger_guard_condition();
    // 调用新消息处理函数 (Invoke the new message handling function)
    this->invoke_on_new_message();
  }

  /**
   * @brief 使用 take_shared 方法获取数据
   * @details 此函数检查是否应使用 take_shared 方法从缓冲区中获取数据。
   *
   * @return bool 如果应使用 take_shared 方法，则返回 true，否则返回 false。
   *
   * @brief Use the take_shared method to get data
   * @details This function checks whether the take_shared method should be used to get data from
   * the buffer.
   *
   * @return bool Returns true if the take_shared method should be used, otherwise returns false.
   */
  bool use_take_shared_method() const override { return buffer_->use_take_shared_method(); }

protected:
  /**
   * @brief 触发保护条件
   * @details 此函数触发 gc_ 成员变量的保护条件。
   *
   * @brief Trigger guard condition
   * @details This function triggers the guard condition of the gc_ member variable.
   */
  void trigger_guard_condition() override { this->gc_.trigger(); }

  // 缓冲区对象的智能指针（用于存储消息）
  // Smart pointer to a buffer object (used for storing messages)
  BufferUniquePtr buffer_;

  // 订阅类型分配器，负责为订阅的消息类型分配内存
  // Subscribed type allocator, responsible for allocating memory for the subscribed message types
  SubscribedTypeAllocator subscribed_type_allocator_;

  // 订阅类型删除器，负责删除订阅的消息类型实例
  // Subscribed type deleter, responsible for deleting instances of the subscribed message types
  SubscribedTypeDeleter subscribed_type_deleter_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BUFFER_HPP_

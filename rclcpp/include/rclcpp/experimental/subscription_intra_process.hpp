// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_HPP_
#define RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_HPP_

#include <rmw/types.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "rcl/types.h"
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/subscription_intra_process_buffer.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp {
namespace experimental {

/**
 * @brief ROS2 订阅内部进程的模板类 (Template class for ROS2 subscription intra-process)
 *
 * @tparam MessageT 消息类型 (Message type)
 * @tparam SubscribedType 订阅消息类型 (Subscribed message type)
 * @tparam SubscribedTypeAlloc 订阅消息类型分配器，默认为 std::allocator<SubscribedType> (Allocator
 * for subscribed message type, default is std::allocator<SubscribedType>)
 * @tparam SubscribedTypeDeleter 订阅消息类型删除器，默认为 std::default_delete<SubscribedType>
 * (Deleter for subscribed message type, default is std::default_delete<SubscribedType>)
 * @tparam ROSMessageType ROS 消息类型，默认为 SubscribedType (ROS message type, default is
 * SubscribedType)
 * @tparam Alloc 分配器类型，默认为 std::allocator<void> (Allocator type, default is
 * std::allocator<void>)
 */
template <
    typename MessageT,
    typename SubscribedType,
    typename SubscribedTypeAlloc = std::allocator<SubscribedType>,
    typename SubscribedTypeDeleter = std::default_delete<SubscribedType>,
    typename ROSMessageType = SubscribedType,
    typename Alloc = std::allocator<void>>
class SubscriptionIntraProcess : public SubscriptionIntraProcessBuffer<
                                     SubscribedType,
                                     SubscribedTypeAlloc,
                                     SubscribedTypeDeleter,
                                     ROSMessageType> {
  // 使用 SubscriptionIntraProcessBuffer 类型别名，以便简化代码 (Using the
  // SubscriptionIntraProcessBuffer type alias for code simplification)
  using SubscriptionIntraProcessBufferT = SubscriptionIntraProcessBuffer<
      SubscribedType,
      SubscribedTypeAlloc,
      SubscribedTypeDeleter,
      ROSMessageType>;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionIntraProcess)

  using MessageAllocTraits =
      typename SubscriptionIntraProcessBufferT::SubscribedTypeAllocatorTraits;
  using MessageAlloc = typename SubscriptionIntraProcessBufferT::SubscribedTypeAllocator;
  using ConstMessageSharedPtr = typename SubscriptionIntraProcessBufferT::ConstDataSharedPtr;
  using MessageUniquePtr = typename SubscriptionIntraProcessBufferT::SubscribedTypeUniquePtr;
  using BufferUniquePtr = typename SubscriptionIntraProcessBufferT::BufferUniquePtr;

  /**
   * @brief 构造一个用于处理内部进程通信的订阅器 (Construct a subscription for intra-process
   * communication)
   *
   * @tparam MessageT 消息类型 (Message type)
   * @tparam Alloc 分配器类型 (Allocator type)
   * @param callback 当收到消息时触发的回调函数 (Callback function to be triggered when a message is
   * received)
   * @param allocator 分配器实例，用于分配和释放内存 (Allocator instance for allocating and
   * deallocating memory)
   * @param context ROS2 上下文 (ROS2 context)
   * @param topic_name 订阅的话题名称 (Topic name to subscribe to)
   * @param qos_profile 服务质量配置 (Quality of Service profile)
   * @param buffer_type 缓冲区类型 (Buffer type)
   */
  SubscriptionIntraProcess(
      AnySubscriptionCallback<MessageT, Alloc> callback,
      std::shared_ptr<Alloc> allocator,
      rclcpp::Context::SharedPtr context,
      const std::string &topic_name,
      const rclcpp::QoS &qos_profile,
      rclcpp::IntraProcessBufferType buffer_type)
      : SubscriptionIntraProcessBuffer<
            SubscribedType,
            SubscribedTypeAlloc,
            SubscribedTypeDeleter,
            ROSMessageType>(
            // 使用给定的分配器创建一个新的 SubscribedTypeAlloc 实例 (Create a new
            // SubscribedTypeAlloc instance using the given allocator)
            std::make_shared<SubscribedTypeAlloc>(*allocator),
            context,
            topic_name,
            qos_profile,
            buffer_type),
        any_callback_(callback)  // 初始化回调函数成员变量 (Initialize the callback member variable)
  {
    // 注册跟踪点 (Register tracepoint)
    TRACEPOINT(
        rclcpp_subscription_callback_added, static_cast<const void *>(this),
        static_cast<const void *>(&any_callback_));

    // 若回调对象被复制，且在此点之前注册（例如在 `AnySubscriptionCallback::set()`
    // 中），则其地址将不匹配后续跟踪点中使用的任何地址 (If the callback object gets copied and
    // registration is done too early/before this point, its address won't match any address used
    // later in subsequent tracepoints)
#ifndef TRACETOOLS_DISABLED
    any_callback_.register_callback_for_tracing();
#endif
  }

  /**
   * @class SubscriptionIntraProcess
   * @brief 订阅内部进程类 (Subscription intra-process class)
   *
   * @details 该类负责处理订阅的内部进程相关操作。(This class is responsible for handling
   * subscription intra-process related operations.)
   */
  virtual ~SubscriptionIntraProcess() = default;

  /// @brief 获取数据 (Take data)
  /// @return std::shared_ptr<void> 返回共享指针类型的数据 (Return shared pointer type of data)
  std::shared_ptr<void> take_data() override {
    // 声明一个常量消息共享指针 (Declare a constant message shared pointer)
    ConstMessageSharedPtr shared_msg;
    // 声明一个消息唯一指针 (Declare a message unique pointer)
    MessageUniquePtr unique_msg;

    // 判断是否使用共享方法 (Determine if the shared method is used)
    if (any_callback_.use_take_shared_method()) {
      // 从缓冲区中消费共享消息 (Consume shared message from buffer)
      shared_msg = this->buffer_->consume_shared();
      // 如果共享消息为空，返回空指针 (If shared message is empty, return nullptr)
      if (!shared_msg) {
        return nullptr;
      }
    } else {
      // 从缓冲区中消费唯一消息 (Consume unique message from buffer)
      unique_msg = this->buffer_->consume_unique();
      // 如果唯一消息为空，返回空指针 (If unique message is empty, return nullptr)
      if (!unique_msg) {
        return nullptr;
      }
    }
    // 返回静态指针类型的数据 (Return static pointer type of data)
    return std::static_pointer_cast<void>(
        std::make_shared<std::pair<ConstMessageSharedPtr, MessageUniquePtr>>(
            std::pair<ConstMessageSharedPtr, MessageUniquePtr>(shared_msg, std::move(unique_msg))));
  }

  /// @brief 执行操作 (Execute operation)
  /// @param data 共享指针类型的数据 (Shared pointer type of data)
  void execute(std::shared_ptr<void> &data) override { execute_impl<SubscribedType>(data); }

protected:
  /// @brief 执行实现 (Execute implementation)
  /// @tparam T 类型参数 (Type parameter)
  /// @param data 共享指针类型的数据 (Shared pointer type of data)
  template <typename T>
  typename std::enable_if<std::is_same<T, rcl_serialized_message_t>::value, void>::type
  execute_impl(std::shared_ptr<void> &data) {
    // 未使用的参数 (Unused parameter)
    (void)data;
    // 抛出运行时错误 (Throw runtime error)
    throw std::runtime_error("Subscription intra-process can't handle serialized messages");
  }

  /**
   * @brief execute_impl 函数模板，用于处理非 rcl_serialized_message_t 类型的消息
   * @tparam T 消息类型
   * @param data 存储消息的智能指针
   *
   * @note 该函数在内部处理 rmw_message_info_t 信息，并根据 any_callback_ 的配置执行相应的回调
   */
  template <class T>
  typename std::enable_if<!std::is_same<T, rcl_serialized_message_t>::value, void>::type
  execute_impl(std::shared_ptr<void> &data) {
    // 如果数据为空，则直接返回
    // If the data is empty, return directly
    if (!data) {
      return;
    }

    // 初始化 msg_info 对象
    // Initialize the msg_info object
    rmw_message_info_t msg_info;
    msg_info.publisher_gid = {0, {0}};
    msg_info.from_intra_process = true;

    // 将 data 转换为存储 ConstMessageSharedPtr 和 MessageUniquePtr 的智能指针
    // Convert data to a smart pointer that stores ConstMessageSharedPtr and MessageUniquePtr
    auto shared_ptr =
        std::static_pointer_cast<std::pair<ConstMessageSharedPtr, MessageUniquePtr>>(data);

    // 根据 any_callback_ 的配置选择使用 take_shared 方法或者不使用
    // Use the take_shared method or not according to the configuration of any_callback_
    if (any_callback_.use_take_shared_method()) {
      // 使用共享指针方式处理消息
      // Process the message using shared pointer method
      ConstMessageSharedPtr shared_msg = shared_ptr->first;
      any_callback_.dispatch_intra_process(shared_msg, msg_info);
    } else {
      // 使用独占指针方式处理消息
      // Process the message using unique pointer method
      MessageUniquePtr unique_msg = std::move(shared_ptr->second);
      any_callback_.dispatch_intra_process(std::move(unique_msg), msg_info);
    }

    // 重置 shared_ptr，释放资源
    // Reset shared_ptr to release resources
    shared_ptr.reset();
  }

  // 定义一个 AnySubscriptionCallback 对象，用于存储回调函数
  // Define an AnySubscriptionCallback object to store callback functions
  AnySubscriptionCallback<MessageT, Alloc> any_callback_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_HPP_

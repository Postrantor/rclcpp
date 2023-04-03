// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_
#define RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_

#include <functional>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>  // NOLINT[build/include_order]

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/detail/subscription_callback_type_helper.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/type_adapter.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "tracetools/tracetools.h"
#include "tracetools/utils.hpp"

/**
 * @brief 一个模板变量，用于在编译时期产生一个始终为 false 的布尔值。 (A template variable that
 * generates a boolean value that is always false at compile time.)
 *
 * @tparam T 类型参数，无实际意义。 (Type parameter, no actual meaning.)
 */
template <class T>
inline constexpr bool always_false_v = false;

namespace rclcpp {

namespace detail {

/**
 * @brief 消息删除器助手模板类 (Message Deleter Helper template class)
 *
 * @tparam MessageT 消息类型 (Message type)
 * @tparam AllocatorT 分配器类型 (Allocator type)
 */
template <typename MessageT, typename AllocatorT>
struct MessageDeleterHelper {
  // 使用 AllocTraits 重新绑定分配器 (Rebind the allocator using AllocTraits)
  using AllocTraits = allocator::AllocRebind<MessageT, AllocatorT>;

  // 定义分配器类型 (Define the allocator type)
  using Alloc = typename AllocTraits::allocator_type;

  // 定义删除器类型 (Define the deleter type)
  using Deleter = allocator::Deleter<Alloc, MessageT>;
};

/**
 * @brief 包含所有可能的回调签名的结构体，可以使用或不使用 TypeAdapter。
 * @tparam MessageT 消息类型
 * @tparam AllocatorT 内存分配器类型
 *
 * Struct which contains all possible callback signatures, with or without a TypeAdapter.
 */
template <typename MessageT, typename AllocatorT>
struct AnySubscriptionCallbackPossibleTypes {
  /// 使用 MessageT::custom_type（如果 MessageT 是 TypeAdapter），否则使用 MessageT。
  /// Use MessageT::custom_type if MessageT is a TypeAdapter, otherwise just MessageT.
  using SubscribedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;
  /// 使用 MessageT::ros_message_type（如果 MessageT 是 TypeAdapter），否则使用 MessageT。
  /// Use MessageT::ros_message_type if MessageT is a TypeAdapter, otherwise just MessageT.
  using ROSMessageType = typename rclcpp::TypeAdapter<MessageT>::ros_message_type;

  // 定义删除器类型
  // Define deleter types
  using SubscribedMessageDeleter =
      typename MessageDeleterHelper<SubscribedType, AllocatorT>::Deleter;
  using ROSMessageDeleter = typename MessageDeleterHelper<ROSMessageType, AllocatorT>::Deleter;
  using SerializedMessageDeleter =
      typename MessageDeleterHelper<rclcpp::SerializedMessage, AllocatorT>::Deleter;

  // 定义回调函数类型
  // Define callback function types
  using ConstRefCallback = std::function<void(const SubscribedType &)>;
  using ConstRefROSMessageCallback = std::function<void(const ROSMessageType &)>;
  using ConstRefWithInfoCallback =
      std::function<void(const SubscribedType &, const rclcpp::MessageInfo &)>;
  using ConstRefWithInfoROSMessageCallback =
      std::function<void(const ROSMessageType &, const rclcpp::MessageInfo &)>;
  using ConstRefSerializedMessageCallback = std::function<void(const rclcpp::SerializedMessage &)>;
  using ConstRefSerializedMessageWithInfoCallback =
      std::function<void(const rclcpp::SerializedMessage &, const rclcpp::MessageInfo &)>;

  // 定义 unique_ptr 回调函数类型
  // Define unique_ptr callback function types
  using UniquePtrCallback =
      std::function<void(std::unique_ptr<SubscribedType, SubscribedMessageDeleter>)>;
  using UniquePtrROSMessageCallback =
      std::function<void(std::unique_ptr<ROSMessageType, ROSMessageDeleter>)>;
  using UniquePtrWithInfoCallback = std::function<void(
      std::unique_ptr<SubscribedType, SubscribedMessageDeleter>, const rclcpp::MessageInfo &)>;
  using UniquePtrWithInfoROSMessageCallback = std::function<void(
      std::unique_ptr<ROSMessageType, ROSMessageDeleter>, const rclcpp::MessageInfo &)>;
  using UniquePtrSerializedMessageCallback =
      std::function<void(std::unique_ptr<rclcpp::SerializedMessage, SerializedMessageDeleter>)>;
  using UniquePtrSerializedMessageWithInfoCallback = std::function<void(
      std::unique_ptr<rclcpp::SerializedMessage, SerializedMessageDeleter>,
      const rclcpp::MessageInfo &)>;

  // 定义 shared_ptr 回调函数类型
  // Define shared_ptr callback function types
  using SharedConstPtrCallback = std::function<void(std::shared_ptr<const SubscribedType>)>;
  using SharedConstPtrROSMessageCallback =
      std::function<void(std::shared_ptr<const ROSMessageType>)>;
  using SharedConstPtrWithInfoCallback =
      std::function<void(std::shared_ptr<const SubscribedType>, const rclcpp::MessageInfo &)>;
  using SharedConstPtrWithInfoROSMessageCallback =
      std::function<void(std::shared_ptr<const ROSMessageType>, const rclcpp::MessageInfo &)>;
  using SharedConstPtrSerializedMessageCallback =
      std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>)>;
  using SharedConstPtrSerializedMessageWithInfoCallback = std::function<void(
      std::shared_ptr<const rclcpp::SerializedMessage>, const rclcpp::MessageInfo &)>;

  // 定义 const shared_ptr 引用回调函数类型
  // Define const shared_ptr reference callback function types
  using ConstRefSharedConstPtrCallback =
      std::function<void(const std::shared_ptr<const SubscribedType> &)>;
  using ConstRefSharedConstPtrROSMessageCallback =
      std::function<void(const std::shared_ptr<const ROSMessageType> &)>;
  using ConstRefSharedConstPtrWithInfoCallback = std::function<void(
      const std::shared_ptr<const SubscribedType> &, const rclcpp::MessageInfo &)>;
  using ConstRefSharedConstPtrWithInfoROSMessageCallback = std::function<void(
      const std::shared_ptr<const ROSMessageType> &, const rclcpp::MessageInfo &)>;
  using ConstRefSharedConstPtrSerializedMessageCallback =
      std::function<void(const std::shared_ptr<const rclcpp::SerializedMessage> &)>;
  using ConstRefSharedConstPtrSerializedMessageWithInfoCallback = std::function<void(
      const std::shared_ptr<const rclcpp::SerializedMessage> &, const rclcpp::MessageInfo &)>;

  // 已弃用的签名：
  // Deprecated signatures:
  using SharedPtrCallback = std::function<void(std::shared_ptr<SubscribedType>)>;
  using SharedPtrROSMessageCallback = std::function<void(std::shared_ptr<ROSMessageType>)>;
  using SharedPtrWithInfoCallback =
      std::function<void(std::shared_ptr<SubscribedType>, const rclcpp::MessageInfo &)>;
  using SharedPtrWithInfoROSMessageCallback =
      std::function<void(std::shared_ptr<ROSMessageType>, const rclcpp::MessageInfo &)>;
  using SharedPtrSerializedMessageCallback =
      std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)>;
  using SharedPtrSerializedMessageWithInfoCallback =
      std::function<void(std::shared_ptr<rclcpp::SerializedMessage>, const rclcpp::MessageInfo &)>;
};

/// \brief 模板辅助器，根据 MessageT 是否是 TypeAdapter 来选择 variant 类型。
/// Template helper to select the variant type based on whether or not MessageT is a TypeAdapter.
template <
    typename MessageT,
    typename AllocatorT,
    bool is_adapted_type = rclcpp::TypeAdapter<MessageT>::is_specialized::value>
struct AnySubscriptionCallbackHelper;

/// \brief 当 MessageT 不是 TypeAdapter 时的特化。
/// Specialization for when MessageT is not a TypeAdapter.
template <typename MessageT, typename AllocatorT>
struct AnySubscriptionCallbackHelper<MessageT, AllocatorT, false> {
  // 定义 CallbackTypes 为 AnySubscriptionCallbackPossibleTypes 的实例。
  // Define CallbackTypes as an instance of AnySubscriptionCallbackPossibleTypes.
  using CallbackTypes = AnySubscriptionCallbackPossibleTypes<MessageT, AllocatorT>;

  // 定义 variant_type 为一个 std::variant，包含了所有可能的回调类型。
  // Define variant_type as a std::variant containing all possible callback types.
  using variant_type = std::variant<
      // 回调接收常量引用消息
      typename CallbackTypes::ConstRefCallback,
      // 回调接收常量引用消息和额外的消息信息
      typename CallbackTypes::ConstRefWithInfoCallback,

      // 回调接收序列化后的常量引用消息
      typename CallbackTypes::ConstRefSerializedMessageCallback,
      // 回调接收序列化后的常量引用消息和额外的消息信息
      typename CallbackTypes::ConstRefSerializedMessageWithInfoCallback,

      // 回调接收独占指针(unique_ptr)消息
      typename CallbackTypes::UniquePtrCallback,
      // 回调接收独占指针(unique_ptr)消息和额外的消息信息
      typename CallbackTypes::UniquePtrWithInfoCallback,

      // 回调接收序列化后的独占指针(unique_ptr)消息
      typename CallbackTypes::UniquePtrSerializedMessageCallback,
      // 回调接收序列化后的独占指针(unique_ptr)消息和额外的消息信息
      typename CallbackTypes::UniquePtrSerializedMessageWithInfoCallback,

      // 回调接收共享常量指针(shared_ptr<const>)消息
      typename CallbackTypes::SharedConstPtrCallback,
      // 回调接收共享常量指针(shared_ptr<const>)消息和额外的消息信息
      typename CallbackTypes::SharedConstPtrWithInfoCallback,

      // 回调接收序列化后的共享常量指针(shared_ptr<const>)消息
      typename CallbackTypes::SharedConstPtrSerializedMessageCallback,
      // 回调接收序列化后的共享常量指针(shared_ptr<const>)消息和额外的消息信息
      typename CallbackTypes::SharedConstPtrSerializedMessageWithInfoCallback,

      // 回调接收共享常量指针的常量引用(const&)消息
      typename CallbackTypes::ConstRefSharedConstPtrCallback,
      // 回调接收共享常量指针的常量引用(const&)消息和额外的消息信息
      typename CallbackTypes::ConstRefSharedConstPtrWithInfoCallback,

      // 回调接收序列化后的共享常量指针的常量引用(const&)消息
      typename CallbackTypes::ConstRefSharedConstPtrSerializedMessageCallback,
      // 回调接收序列化后的共享常量指针的常量引用(const&)消息和额外的消息信息
      typename CallbackTypes::ConstRefSharedConstPtrSerializedMessageWithInfoCallback,

      // 回调接收共享指针(shared_ptr)消息
      typename CallbackTypes::SharedPtrCallback,
      // 回调接收共享指针(shared_ptr)消息和额外的消息信息
      typename CallbackTypes::SharedPtrWithInfoCallback,

      // 回调接收序列化后的共享指针(shared_ptr)消息
      typename CallbackTypes::SharedPtrSerializedMessageCallback,
      // 回调接收序列化后的共享指针(shared_ptr)消息和额外的消息信息
      typename CallbackTypes::SharedPtrSerializedMessageWithInfoCallback>;
};

/// \brief 特化，当 MessageT 是 TypeAdapter 时的 AnySubscriptionCallbackHelper 结构体定义。
///        (Specialization for when MessageT is a TypeAdapter in the AnySubscriptionCallbackHelper
///        struct definition.)
template <typename MessageT, typename AllocatorT>
struct AnySubscriptionCallbackHelper<MessageT, AllocatorT, true> {
  /// \brief 定义 CallbackTypes 类型为 AnySubscriptionCallbackPossibleTypes 模板实例。
  ///        (Define CallbackTypes type as an instance of the AnySubscriptionCallbackPossibleTypes
  ///        template.)
  using CallbackTypes = AnySubscriptionCallbackPossibleTypes<MessageT, AllocatorT>;

  /// \brief 定义 variant_type 类型，包含各种可能的回调类型。
  ///        (Define variant_type type containing various possible callback types.)
  using variant_type = std::variant<
      // 常量引用回调 (ConstRefCallback)
      typename CallbackTypes::ConstRefCallback,
      // 常量引用 ROS 消息回调 (ConstRefROSMessageCallback)
      typename CallbackTypes::ConstRefROSMessageCallback,
      // 常量引用带信息回调 (ConstRefWithInfoCallback)
      typename CallbackTypes::ConstRefWithInfoCallback,
      // 常量引用带信息 ROS 消息回调 (ConstRefWithInfoROSMessageCallback)
      typename CallbackTypes::ConstRefWithInfoROSMessageCallback,
      // 常量引用序列化消息回调 (ConstRefSerializedMessageCallback)
      typename CallbackTypes::ConstRefSerializedMessageCallback,
      // 常量引用序列化消息带信息回调 (ConstRefSerializedMessageWithInfoCallback)
      typename CallbackTypes::ConstRefSerializedMessageWithInfoCallback,
      // 独占指针回调 (UniquePtrCallback)
      typename CallbackTypes::UniquePtrCallback,
      // 独占指针 ROS 消息回调 (UniquePtrROSMessageCallback)
      typename CallbackTypes::UniquePtrROSMessageCallback,
      // 独占指针带信息回调 (UniquePtrWithInfoCallback)
      typename CallbackTypes::UniquePtrWithInfoCallback,
      // 独占指针带信息 ROS 消息回调 (UniquePtrWithInfoROSMessageCallback)
      typename CallbackTypes::UniquePtrWithInfoROSMessageCallback,
      // 独占指针序列化消息回调 (UniquePtrSerializedMessageCallback)
      typename CallbackTypes::UniquePtrSerializedMessageCallback,
      // 独占指针序列化消息带信息回调 (UniquePtrSerializedMessageWithInfoCallback)
      typename CallbackTypes::UniquePtrSerializedMessageWithInfoCallback,
      // 共享常量指针回调 (SharedConstPtrCallback)
      typename CallbackTypes::SharedConstPtrCallback,
      // 共享常量指针 ROS 消息回调 (SharedConstPtrROSMessageCallback)
      typename CallbackTypes::SharedConstPtrROSMessageCallback,
      // 共享常量指针带信息回调 (SharedConstPtrWithInfoCallback)
      typename CallbackTypes::SharedConstPtrWithInfoCallback,
      // 共享常量指针带信息 ROS 消息回调 (SharedConstPtrWithInfoROSMessageCallback)
      typename CallbackTypes::SharedConstPtrWithInfoROSMessageCallback,
      // 共享常量指针序列化消息回调 (SharedConstPtrSerializedMessageCallback)
      typename CallbackTypes::SharedConstPtrSerializedMessageCallback,
      // 共享常量指针序列化消息带信息回调 (SharedConstPtrSerializedMessageWithInfoCallback)
      typename CallbackTypes::SharedConstPtrSerializedMessageWithInfoCallback,
      // 常量引用共享常量指针回调 (ConstRefSharedConstPtrCallback)
      typename CallbackTypes::ConstRefSharedConstPtrCallback,
      // 常量引用共享常量指针 ROS 消息回调 (ConstRefSharedConstPtrROSMessageCallback)
      typename CallbackTypes::ConstRefSharedConstPtrROSMessageCallback,
      // 常量引用共享常量指针带信息回调 (ConstRefSharedConstPtrWithInfoCallback)
      typename CallbackTypes::ConstRefSharedConstPtrWithInfoCallback,
      // 常量引用共享常量指针带信息 ROS 消息回调 (ConstRefSharedConstPtrWithInfoROSMessageCallback)
      typename CallbackTypes::ConstRefSharedConstPtrWithInfoROSMessageCallback,
      // 常量引用共享常量指针序列化消息回调 (ConstRefSharedConstPtrSerializedMessageCallback)
      typename CallbackTypes::ConstRefSharedConstPtrSerializedMessageCallback,
      // 常量引用共享常量指针序列化消息带信息回调
      // (ConstRefSharedConstPtrSerializedMessageWithInfoCallback)
      typename CallbackTypes::ConstRefSharedConstPtrSerializedMessageWithInfoCallback,
      // 共享指针回调 (SharedPtrCallback)
      typename CallbackTypes::SharedPtrCallback,
      // 共享指针 ROS 消息回调 (SharedPtrROSMessageCallback)
      typename CallbackTypes::SharedPtrROSMessageCallback,
      // 共享指针带信息回调 (SharedPtrWithInfoCallback)
      typename CallbackTypes::SharedPtrWithInfoCallback,
      // 共享指针带信息 ROS 消息回调 (SharedPtrWithInfoROSMessageCallback)
      typename CallbackTypes::SharedPtrWithInfoROSMessageCallback,
      // 共享指针序列化消息回调 (SharedPtrSerializedMessageCallback)
      typename CallbackTypes::SharedPtrSerializedMessageCallback,
      // 共享指针序列化消息带信息回调 (SharedPtrSerializedMessageWithInfoCallback)
      typename CallbackTypes::SharedPtrSerializedMessageWithInfoCallback>;
};

}  // namespace detail

template <typename MessageT, typename AllocatorT = std::allocator<void> >
class AnySubscriptionCallback {
private:
  /// \tparam MessageT 消息类型 (Message type)
  /// \tparam AllocatorT 分配器类型 (Allocator type)
  /// 如果 MessageT 是 TypeAdapter，则为 MessageT::custom_type，否则为 MessageT。
  /// (MessageT::custom_type if MessageT is a TypeAdapter, otherwise just MessageT.)
  using SubscribedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;

  /// 如果 MessageT 是 TypeAdapter，则为 MessageT::ros_message_type，否则为 MessageT。
  /// (MessageT::ros_message_type if MessageT is a TypeAdapter, otherwise just MessageT.)
  using ROSMessageType = typename rclcpp::TypeAdapter<MessageT>::ros_message_type;

  // 使用 HelperT 类型定义 rclcpp::detail::AnySubscriptionCallbackHelper 的实例
  // (Define an instance of rclcpp::detail::AnySubscriptionCallbackHelper using HelperT type)
  using HelperT = typename rclcpp::detail::AnySubscriptionCallbackHelper<MessageT, AllocatorT>;

  // 定义 SubscribedTypeDeleterHelper 类型
  // (Define SubscribedTypeDeleterHelper type)
  using SubscribedTypeDeleterHelper =
      rclcpp::detail::MessageDeleterHelper<SubscribedType, AllocatorT>;
  using SubscribedTypeAllocatorTraits = typename SubscribedTypeDeleterHelper::AllocTraits;
  using SubscribedTypeAllocator = typename SubscribedTypeDeleterHelper::Alloc;
  using SubscribedTypeDeleter = typename SubscribedTypeDeleterHelper::Deleter;

  // 定义 ROSMessageTypeDeleterHelper 类型
  // (Define ROSMessageTypeDeleterHelper type)
  using ROSMessageTypeDeleterHelper =
      rclcpp::detail::MessageDeleterHelper<ROSMessageType, AllocatorT>;
  using ROSMessageTypeAllocatorTraits = typename ROSMessageTypeDeleterHelper::AllocTraits;
  using ROSMessageTypeAllocator = typename ROSMessageTypeDeleterHelper::Alloc;
  using ROSMessageTypeDeleter = typename ROSMessageTypeDeleterHelper::Deleter;

  // 定义 SerializedMessageDeleterHelper 类型
  // (Define SerializedMessageDeleterHelper type)
  using SerializedMessageDeleterHelper =
      rclcpp::detail::MessageDeleterHelper<rclcpp::SerializedMessage, AllocatorT>;
  using SerializedMessageAllocatorTraits = typename SerializedMessageDeleterHelper::AllocTraits;
  using SerializedMessageAllocator = typename SerializedMessageDeleterHelper::Alloc;
  using SerializedMessageDeleter = typename SerializedMessageDeleterHelper::Deleter;

  // 请参阅 AnySubscriptionCallbackPossibleTypes 以了解这些类型。
  // See AnySubscriptionCallbackPossibleTypes for the types of these.
  using CallbackTypes = detail::AnySubscriptionCallbackPossibleTypes<MessageT, AllocatorT>;

  // 定义常量引用回调类型
  // Define the ConstRefCallback type
  using ConstRefCallback = typename CallbackTypes::ConstRefCallback;

  // 定义常量引用 ROS 消息回调类型
  // Define the ConstRefROSMessageCallback type
  using ConstRefROSMessageCallback = typename CallbackTypes::ConstRefROSMessageCallback;

  // 定义带有信息的常量引用回调类型
  // Define the ConstRefWithInfoCallback type
  using ConstRefWithInfoCallback = typename CallbackTypes::ConstRefWithInfoCallback;

  // 定义带有信息的常量引用 ROS 消息回调类型
  // Define the ConstRefWithInfoROSMessageCallback type
  using ConstRefWithInfoROSMessageCallback =
      typename CallbackTypes::ConstRefWithInfoROSMessageCallback;

  // 定义常量引用序列化消息回调类型
  // Define the ConstRefSerializedMessageCallback type
  using ConstRefSerializedMessageCallback =
      typename CallbackTypes::ConstRefSerializedMessageCallback;

  // 定义带有信息的常量引用序列化消息回调类型
  // Define the ConstRefSerializedMessageWithInfoCallback type
  using ConstRefSerializedMessageWithInfoCallback =
      typename CallbackTypes::ConstRefSerializedMessageWithInfoCallback;

  // 定义独占指针回调类型
  // Define the UniquePtrCallback type
  using UniquePtrCallback = typename CallbackTypes::UniquePtrCallback;

  // 定义独占指针 ROS 消息回调类型
  // Define the UniquePtrROSMessageCallback type
  using UniquePtrROSMessageCallback = typename CallbackTypes::UniquePtrROSMessageCallback;

  // 定义带有信息的独占指针回调类型
  // Define the UniquePtrWithInfoCallback type
  using UniquePtrWithInfoCallback = typename CallbackTypes::UniquePtrWithInfoCallback;

  // 定义带有信息的独占指针 ROS 消息回调类型
  // Define the UniquePtrWithInfoROSMessageCallback type
  using UniquePtrWithInfoROSMessageCallback =
      typename CallbackTypes::UniquePtrWithInfoROSMessageCallback;

  // 定义独占指针序列化消息回调类型
  // Define the UniquePtrSerializedMessageCallback type
  using UniquePtrSerializedMessageCallback =
      typename CallbackTypes::UniquePtrSerializedMessageCallback;

  // 定义带有信息的独占指针序列化消息回调类型
  // Define the UniquePtrSerializedMessageWithInfoCallback type
  using UniquePtrSerializedMessageWithInfoCallback =
      typename CallbackTypes::UniquePtrSerializedMessageWithInfoCallback;

  // 定义共享常量指针回调类型
  // Define the SharedConstPtrCallback type
  using SharedConstPtrCallback = typename CallbackTypes::SharedConstPtrCallback;

  // 定义共享常量指针 ROS 消息回调类型
  // Define the SharedConstPtrROSMessageCallback type
  using SharedConstPtrROSMessageCallback = typename CallbackTypes::SharedConstPtrROSMessageCallback;

  // 定义带有信息的共享常量指针回调类型
  // Define the SharedConstPtrWithInfoCallback type
  using SharedConstPtrWithInfoCallback = typename CallbackTypes::SharedConstPtrWithInfoCallback;

  // 定义带有信息的共享常量指针 ROS 消息回调类型
  // Define the SharedConstPtrWithInfoROSMessageCallback type
  using SharedConstPtrWithInfoROSMessageCallback =
      typename CallbackTypes::SharedConstPtrWithInfoROSMessageCallback;

  // 定义共享常量指针序列化消息回调类型
  // Define the SharedConstPtrSerializedMessageCallback type
  using SharedConstPtrSerializedMessageCallback =
      typename CallbackTypes::SharedConstPtrSerializedMessageCallback;

  // 定义带有信息的共享常量指针序列化消息回调类型
  // Define the SharedConstPtrSerializedMessageWithInfoCallback type
  using SharedConstPtrSerializedMessageWithInfoCallback =
      typename CallbackTypes::SharedConstPtrSerializedMessageWithInfoCallback;

  // 定义常量引用共享常量指针回调类型
  // Define the ConstRefSharedConstPtrCallback type
  using ConstRefSharedConstPtrCallback = typename CallbackTypes::ConstRefSharedConstPtrCallback;

  // 定义常量引用共享常量指针 ROS 消息回调类型
  // Define the ConstRefSharedConstPtrROSMessageCallback type
  using ConstRefSharedConstPtrROSMessageCallback =
      typename CallbackTypes::ConstRefSharedConstPtrROSMessageCallback;

  // 定义带有信息的常量引用共享常量指针回调类型
  // Define the ConstRefSharedConstPtrWithInfoCallback type
  using ConstRefSharedConstPtrWithInfoCallback =
      typename CallbackTypes::ConstRefSharedConstPtrWithInfoCallback;

  // 定义带有信息的常量引用共享常量指针 ROS 消息回调类型
  // Define the ConstRefSharedConstPtrWithInfoROSMessageCallback type
  using ConstRefSharedConstPtrWithInfoROSMessageCallback =
      typename CallbackTypes::ConstRefSharedConstPtrWithInfoROSMessageCallback;

  // 使用ConstRefSharedConstPtrSerializedMessageCallback类型定义回调类型。
  // Define callback type using ConstRefSharedConstPtrSerializedMessageCallback.
  using ConstRefSharedConstPtrSerializedMessageCallback =
      typename CallbackTypes::ConstRefSharedConstPtrSerializedMessageCallback;

  // 使用ConstRefSharedConstPtrSerializedMessageWithInfoCallback类型定义回调类型。
  // Define callback type using ConstRefSharedConstPtrSerializedMessageWithInfoCallback.
  using ConstRefSharedConstPtrSerializedMessageWithInfoCallback =
      typename CallbackTypes::ConstRefSharedConstPtrSerializedMessageWithInfoCallback;

  // 使用SharedPtrCallback类型定义回调类型。
  // Define callback type using SharedPtrCallback.
  using SharedPtrCallback = typename CallbackTypes::SharedPtrCallback;

  // 使用SharedPtrROSMessageCallback类型定义回调类型。
  // Define callback type using SharedPtrROSMessageCallback.
  using SharedPtrROSMessageCallback = typename CallbackTypes::SharedPtrROSMessageCallback;

  // 使用SharedPtrWithInfoCallback类型定义回调类型。
  // Define callback type using SharedPtrWithInfoCallback.
  using SharedPtrWithInfoCallback = typename CallbackTypes::SharedPtrWithInfoCallback;

  // 使用SharedPtrWithInfoROSMessageCallback类型定义回调类型。
  // Define callback type using SharedPtrWithInfoROSMessageCallback.
  using SharedPtrWithInfoROSMessageCallback =
      typename CallbackTypes::SharedPtrWithInfoROSMessageCallback;

  // 使用SharedPtrSerializedMessageCallback类型定义回调类型。
  // Define callback type using SharedPtrSerializedMessageCallback.
  using SharedPtrSerializedMessageCallback =
      typename CallbackTypes::SharedPtrSerializedMessageCallback;

  // 使用SharedPtrSerializedMessageWithInfoCallback类型定义回调类型。
  // Define callback type using SharedPtrSerializedMessageWithInfoCallback.
  using SharedPtrSerializedMessageWithInfoCallback =
      typename CallbackTypes::SharedPtrSerializedMessageWithInfoCallback;

  /**
   * @brief 一个模板结构体，用于确保指针不为空 (A template struct to ensure the pointer is not null)
   * @tparam T 类型参数 (Type parameter)
   */
  template <typename T>
  struct NotNull {
    /**
     * @brief 构造函数，接收一个指针和一条错误信息 (Constructor, accepts a pointer and an error
     * message)
     * @param pointer_in 输入的指针 (Input pointer)
     * @param msg 当指针为空时抛出的错误信息 (Error message thrown when the pointer is null)
     */
    NotNull(const T *pointer_in, const char *msg) : pointer(pointer_in) {
      // 如果指针为空，则抛出 std::invalid_argument 异常
      // If the pointer is null, throw a std::invalid_argument exception
      if (pointer == nullptr) {
        throw std::invalid_argument(msg);
      }
    }

    const T *pointer;  ///< 非空指针 (Non-null pointer)
  };

public:
  /// \brief 构造函数，用于初始化订阅回调对象。Constructor for initializing the subscription
  /// callback object. \param allocator 分配器，默认为 AllocatorT 类型。Allocator, default is of
  /// type AllocatorT.
  explicit AnySubscriptionCallback(
      const AllocatorT &allocator = AllocatorT())  // NOLINT[runtime/explicit]
      : subscribed_type_allocator_(allocator), ros_message_type_allocator_(allocator) {
    // 设置订阅类型删除器的分配器。Set the allocator for the subscribed type deleter.
    allocator::set_allocator_for_deleter(&subscribed_type_deleter_, &subscribed_type_allocator_);
    // 设置 ROS 消息类型删除器的分配器。Set the allocator for the ROS message type deleter.
    allocator::set_allocator_for_deleter(&ros_message_type_deleter_, &ros_message_type_allocator_);
  }

  /// \brief 拷贝构造函数。Copy constructor.
  AnySubscriptionCallback(const AnySubscriptionCallback &) = default;

  /// \brief 通用函数，用于设置回调。Generic function for setting the callback.
  /**
   * 有一些特化重载此函数，以便弃用某些回调签名，并在使用 lambda 函数时解决 shared_ptr 和 unique_ptr
   * 回调签名之间的歧义。 There are specializations that overload this in order to deprecate some
   * callback signatures, and also to fix ambiguity between shared_ptr and
   * unique_ptr callback signatures when using them with lambda functions.
   */
  template <typename CallbackT>
  AnySubscriptionCallback<MessageT, AllocatorT> set(CallbackT callback) {
    // 使用 SubscriptionCallbackTypeHelper 确定 CallbackT 的实际类型，以 std::function<...>
    // 的形式，这在使用可互相转换的参数（如 shared_ptr 和 unique_ptr）的 lambda 函数时不会自动发生。
    // Use the SubscriptionCallbackTypeHelper to determine the actual type of
    // the CallbackT, in terms of std::function<...>, which does not happen
    // automatically with lambda functions in cases where the arguments can be
    // converted to one another, e.g. shared_ptr and unique_ptr.
    using scbth = detail::SubscriptionCallbackTypeHelper<MessageT, CallbackT>;

    // 确定给定的 CallbackT 是否为弃用的签名。Determine if the given CallbackT is a deprecated
    // signature or not.
    constexpr auto is_deprecated =
        rclcpp::function_traits::same_arguments<
            typename scbth::callback_type,
            std::function<void(std::shared_ptr<MessageT>)> >::value ||
        rclcpp::function_traits::same_arguments<
            typename scbth::callback_type,
            std::function<void(std::shared_ptr<MessageT>, const rclcpp::MessageInfo &)> >::value;

    // 使用发现的类型在分配变量时强制回调类型。Use the discovered type to force the type of callback
    // when assigning into the variant.
    if constexpr (is_deprecated) {
      // 如果已弃用，则调用已弃用的子例程。If deprecated, call sub-routine that is deprecated.
      set_deprecated(static_cast<typename scbth::callback_type>(callback));
    } else {
      // 否则只需分配它。Otherwise just assign it.
      callback_variant_ = static_cast<typename scbth::callback_type>(callback);
    }

    // 返回自身的副本以便于测试，通常会被编译器优化掉。Return copy of self for easier testing,
    // normally will be compiled out.
    return *this;
  }

  /// 设置 shared_ptr 到非 const MessageT 的函数，已弃用。
  /// Function for shared_ptr to non-const MessageT, which is deprecated.
  template <typename SetT>
#if !defined(RCLCPP_AVOID_DEPRECATIONS_FOR_UNIT_TESTS)
  // 在 `test_any_subscription_callback.cpp` 中抑制弃用警告
  // suppress deprecation warnings in `test_any_subscription_callback.cpp`
  [[deprecated("use 'void(std::shared_ptr<const MessageT>)' instead")]]
#endif
  void
  set_deprecated(
      std::function<void(std::shared_ptr<SetT>)> callback)  ///< [in] 回调函数（已弃用）
                                                            ///< [in] Callback function (deprecated)
  {
    callback_variant_ = callback;
  }

  /// 设置 shared_ptr 到非 const MessageT 和 MessageInfo 的函数，已弃用。
  /// Function for shared_ptr to non-const MessageT with MessageInfo, which is deprecated.
  template <typename SetT>
#if !defined(RCLCPP_AVOID_DEPRECATIONS_FOR_UNIT_TESTS)
  // 在 `test_any_subscription_callback.cpp` 中抑制弃用警告
  // suppress deprecation warnings in `test_any_subscription_callback.cpp`
  [[deprecated("use 'void(std::shared_ptr<const MessageT>, const rclcpp::MessageInfo &)' instead")]]
#endif
  void
  set_deprecated(std::function<void(std::shared_ptr<SetT>, const rclcpp::MessageInfo &)>
                     callback)  ///< [in] 带有 MessageInfo 的回调函数（已弃用）
                                ///< [in] Callback function with MessageInfo (deprecated)
  {
    callback_variant_ = callback;
  }

  /**
   * @brief 创建一个从 ROSMessageType 共享指针创建的独占指针 (Create a unique_ptr from a shared_ptr
   * of ROSMessageType)
   * @param message ROSMessageType 的共享指针 (Shared pointer of ROSMessageType)
   * @return 一个独占指针，其类型为 ROSMessageType (A unique_ptr of type ROSMessageType)
   */
  std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>
  create_ros_unique_ptr_from_ros_shared_ptr_message(
      const std::shared_ptr<const ROSMessageType> &message) {
    // 分配内存 (Allocate memory)
    auto ptr = ROSMessageTypeAllocatorTraits::allocate(ros_message_type_allocator_, 1);
    // 构造对象 (Construct object)
    ROSMessageTypeAllocatorTraits::construct(ros_message_type_allocator_, ptr, *message);
    // 返回独占指针 (Return unique_ptr)
    return std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>(ptr, ros_message_type_deleter_);
  }

  /**
   * @brief 创建一个从 rclcpp::SerializedMessage 共享指针创建的独占指针 (Create a unique_ptr from a
   * shared_ptr of rclcpp::SerializedMessage)
   * @param serialized_message rclcpp::SerializedMessage 的共享指针 (Shared pointer of
   * rclcpp::SerializedMessage)
   * @return 一个独占指针，其类型为 rclcpp::SerializedMessage (A unique_ptr of type
   * rclcpp::SerializedMessage)
   */
  std::unique_ptr<rclcpp::SerializedMessage, SerializedMessageDeleter>
  create_serialized_message_unique_ptr_from_shared_ptr(
      const std::shared_ptr<const rclcpp::SerializedMessage> &serialized_message) {
    // 分配内存 (Allocate memory)
    auto ptr = SerializedMessageAllocatorTraits::allocate(serialized_message_allocator_, 1);
    // 构造对象 (Construct object)
    SerializedMessageAllocatorTraits::construct(
        serialized_message_allocator_, ptr, *serialized_message);
    // 返回独占指针 (Return unique_ptr)
    return std::unique_ptr<rclcpp::SerializedMessage, SerializedMessageDeleter>(
        ptr, serialized_message_deleter_);
  }

  /**
   * @brief 创建一个从 SubscribedType 共享指针创建的独占指针 (Create a unique_ptr from a shared_ptr
   * of SubscribedType)
   * @param message SubscribedType 的共享指针 (Shared pointer of SubscribedType)
   * @return 一个独占指针，其类型为 SubscribedType (A unique_ptr of type SubscribedType)
   */
  std::unique_ptr<SubscribedType, SubscribedTypeDeleter>
  create_custom_unique_ptr_from_custom_shared_ptr_message(
      const std::shared_ptr<const SubscribedType> &message) {
    // 分配内存 (Allocate memory)
    auto ptr = SubscribedTypeAllocatorTraits::allocate(subscribed_type_allocator_, 1);
    // 构造对象 (Construct object)
    SubscribedTypeAllocatorTraits::construct(subscribed_type_allocator_, ptr, *message);
    // 返回独占指针 (Return unique_ptr)
    return std::unique_ptr<SubscribedType, SubscribedTypeDeleter>(ptr, subscribed_type_deleter_);
  }

  /**
   * @brief 将 ROS 消息转换为自定义类型的智能指针 (Convert a ROS message to a custom type
   * unique_ptr)
   *
   * @param msg 输入的 ROS 消息 (Input ROS message)
   * @return std::unique_ptr<SubscribedType, SubscribedTypeDeleter> 转换后的自定义类型智能指针
   * (Converted custom type unique_ptr)
   */
  std::unique_ptr<SubscribedType, SubscribedTypeDeleter>
  convert_ros_message_to_custom_type_unique_ptr(const ROSMessageType &msg) {
    // 判断 TypeAdapter 是否专门化 (Check if TypeAdapter is specialized)
    if constexpr (rclcpp::TypeAdapter<MessageT>::is_specialized::value) {
      // 为自定义类型分配内存 (Allocate memory for the custom type)
      auto ptr = SubscribedTypeAllocatorTraits::allocate(subscribed_type_allocator_, 1);
      // 在已分配的内存上构造自定义类型对象 (Construct the custom type object on the allocated
      // memory)
      SubscribedTypeAllocatorTraits::construct(subscribed_type_allocator_, ptr);
      // 将 ROS 消息转换为自定义类型 (Convert the ROS message to the custom type)
      rclcpp::TypeAdapter<MessageT>::convert_to_custom(msg, *ptr);
      // 返回自定义类型的智能指针 (Return the custom type unique_ptr)
      return std::unique_ptr<SubscribedType, SubscribedTypeDeleter>(ptr, subscribed_type_deleter_);
    } else {
      // 如果 TypeAdapter 未专门化，则抛出运行时错误 (Throw a runtime error if TypeAdapter is not
      // specialized)
      throw std::runtime_error(
          "convert_ros_message_to_custom_type_unique_ptr "
          "unexpectedly called without TypeAdapter");
    }
  }

  /**
   * @brief 将自定义类型转换为 ROS 消息的智能指针 (Convert a custom type to a ROS message
   * unique_ptr)
   *
   * @param msg 输入的自定义类型消息 (Input custom type message)
   * @return std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> 转换后的 ROS 消息类型智能指针
   * (Converted ROS message type unique_ptr)
   */
  std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>
  convert_custom_type_to_ros_message_unique_ptr(const SubscribedType &msg) {
    // 判断 TypeAdapter 是否专门化 (Check if TypeAdapter is specialized)
    if constexpr (rclcpp::TypeAdapter<MessageT>::is_specialized::value) {
      // 为 ROS 消息类型分配内存 (Allocate memory for the ROS message type)
      auto ptr = ROSMessageTypeAllocatorTraits::allocate(ros_message_type_allocator_, 1);
      // 在已分配的内存上构造 ROS 消息类型对象 (Construct the ROS message type object on the
      // allocated memory)
      ROSMessageTypeAllocatorTraits::construct(ros_message_type_allocator_, ptr);
      // 将自定义类型转换为 ROS 消息类型 (Convert the custom type to the ROS message type)
      rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(msg, *ptr);
      // 返回 ROS 消息类型的智能指针 (Return the ROS message type unique_ptr)
      return std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>(ptr, ros_message_type_deleter_);
    } else {
      // 如果 TypeAdapter 未专门化，则触发静态断言错误 (Trigger a static_assert error if TypeAdapter
      // is not specialized)
      static_assert(
          !sizeof(MessageT *),
          "convert_custom_type_to_ros_message_unique_ptr() "
          "unexpectedly called without specialized TypeAdapter");
    }
  }

  /**
   * @brief 分发回调函数，输入为 ROS 消息，输出可以是任何类型。
   * @brief Dispatch callback function, where input is a ROS message and output can be any type.
   *
   * @param message 接收到的 ROS 消息。
   * @param message_info 与接收到的消息相关的元信息。
   * @param message The received ROS message.
   * @param message_info The metadata related to the received message.
   */
  void dispatch(std::shared_ptr<ROSMessageType> message, const rclcpp::MessageInfo &message_info) {
    // 开始记录回调跟踪点
    // Start recording callback tracepoint
    TRACEPOINT(callback_start, static_cast<const void *>(this), false);

    // 检查回调变量是否为 "unset"，如果是则抛出异常。
    // Check if the callback variant is "unset", throw an exception if it is.
    if (callback_variant_.index() == 0) {
      if (std::get<0>(callback_variant_) == nullptr) {
        // 这可能是因为默认初始化或分配了 nullptr。
        // This can happen if it is default initialized or assigned nullptr.
        throw std::runtime_error("dispatch called on an unset AnySubscriptionCallback");
      }
    }

    // 分发回调。
    // Dispatch the callback.
    std::visit(
        [&message, &message_info, this](auto &&callback) {
          using T = std::decay_t<decltype(callback)>;
          static constexpr bool is_ta = rclcpp::TypeAdapter<MessageT>::is_specialized::value;

          // 输出为自定义消息的条件
          // Conditions for output being custom message
          if constexpr (is_ta && std::is_same_v<T, ConstRefCallback>) {
            // 考虑避免小消息的堆分配
            // TODO(wjwwood): consider avoiding heap allocation for small messages
            auto local_message = convert_ros_message_to_custom_type_unique_ptr(*message);
            callback(*local_message);
          } else if constexpr (is_ta && std::is_same_v<T, ConstRefWithInfoCallback>) {  // NOLINT
            auto local_message = convert_ros_message_to_custom_type_unique_ptr(*message);
            callback(*local_message, message_info);
          } else if constexpr (is_ta && std::is_same_v<T, UniquePtrCallback>) {
            callback(convert_ros_message_to_custom_type_unique_ptr(*message));
          } else if constexpr (is_ta && std::is_same_v<T, UniquePtrWithInfoCallback>) {
            callback(convert_ros_message_to_custom_type_unique_ptr(*message), message_info);
          } else if constexpr (  // NOLINT[readability/braces]
              is_ta && (std::is_same_v<T, SharedConstPtrCallback> ||
                        std::is_same_v<T, ConstRefSharedConstPtrCallback> ||
                        std::is_same_v<T, SharedPtrCallback>)) {
            callback(convert_ros_message_to_custom_type_unique_ptr(*message));
          } else if constexpr (  // NOLINT[readability/braces]
              is_ta && (std::is_same_v<T, SharedConstPtrWithInfoCallback> ||
                        std::is_same_v<T, ConstRefSharedConstPtrWithInfoCallback> ||
                        std::is_same_v<T, SharedPtrWithInfoCallback>)) {
            callback(convert_ros_message_to_custom_type_unique_ptr(*message), message_info);
          }
          // 输出为 ROS 消息的条件
          // Conditions for output being ROS message
          else if constexpr (std::is_same_v<T, ConstRefROSMessageCallback>) {  // NOLINT
            callback(*message);
          } else if constexpr (std::is_same_v<T, ConstRefWithInfoROSMessageCallback>) {
            callback(*message, message_info);
          } else if constexpr (std::is_same_v<T, UniquePtrROSMessageCallback>) {
            callback(create_ros_unique_ptr_from_ros_shared_ptr_message(message));
          } else if constexpr (std::is_same_v<T, UniquePtrWithInfoROSMessageCallback>) {
            callback(create_ros_unique_ptr_from_ros_shared_ptr_message(message), message_info);
          } else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, SharedConstPtrROSMessageCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrROSMessageCallback> ||
              std::is_same_v<T, SharedPtrROSMessageCallback>) {
            callback(message);
          } else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, SharedConstPtrWithInfoROSMessageCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrWithInfoROSMessageCallback> ||
              std::is_same_v<T, SharedPtrWithInfoROSMessageCallback>) {
            callback(message, message_info);
          }
          // 捕获 SerializedMessage 类型的条件
          // Condition to catch SerializedMessage types
          else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, ConstRefSerializedMessageCallback> ||
              std::is_same_v<T, ConstRefSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, UniquePtrSerializedMessageCallback> ||
              std::is_same_v<T, UniquePtrSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, SharedConstPtrSerializedMessageCallback> ||
              std::is_same_v<T, SharedConstPtrSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, SharedPtrSerializedMessageCallback> ||
              std::is_same_v<T, SharedPtrSerializedMessageWithInfoCallback>) {
            throw std::runtime_error(
                "Cannot dispatch std::shared_ptr<ROSMessageType> message "
                "to rclcpp::SerializedMessage");
          }
          // 捕获未处理的回调类型条件
          // Condition to catch unhandled callback types
          else {  // NOLINT[readability/braces]
            static_assert(always_false_v<T>, "unhandled callback type");
          }
        },
        callback_variant_);

    // 结束记录回调跟踪点
    // End recording callback tracepoint
    TRACEPOINT(callback_end, static_cast<const void *>(this));
  }

  /**
   * @brief 分发输入为序列化消息，输出可以是任何类型的回调函数 (Dispatch when input is a serialized
   * message and the output could be anything)
   *
   * @param[in] serialized_message 序列化消息的共享指针 (Shared pointer to the serialized message)
   * @param[in] message_info 消息相关信息 (Message related information)
   */
  void dispatch(
      std::shared_ptr<rclcpp::SerializedMessage> serialized_message,
      const rclcpp::MessageInfo &message_info) {
    // 开始回调跟踪点 (Begin callback tracepoint)
    TRACEPOINT(callback_start, static_cast<const void *>(this), false);

    // 如果变量未设置，则抛出异常 (Check if the variant is "unset", throw if it is)
    if (callback_variant_.index() == 0) {
      if (std::get<0>(callback_variant_) == nullptr) {
        // 默认初始化或分配nullptr时可能发生此情况 (This can happen if it is default initialized, or
        // if it is assigned nullptr)
        throw std::runtime_error("dispatch called on an unset AnySubscriptionCallback");
      }
    }

    // 分发处理 (Dispatch)
    std::visit(
        [&serialized_message, &message_info, this](auto &&callback) {
          using T = std::decay_t<decltype(callback)>;

          // 捕获 SerializedMessage 类型的条件 (Condition to catch SerializedMessage types)
          if constexpr (std::is_same_v<T, ConstRefSerializedMessageCallback>) {
            callback(*serialized_message);
          } else if constexpr (std::is_same_v<T, ConstRefSerializedMessageWithInfoCallback>) {
            callback(*serialized_message, message_info);
          } else if constexpr (std::is_same_v<T, UniquePtrSerializedMessageCallback>) {
            callback(create_serialized_message_unique_ptr_from_shared_ptr(serialized_message));
          } else if constexpr (std::is_same_v<T, UniquePtrSerializedMessageWithInfoCallback>) {
            callback(
                create_serialized_message_unique_ptr_from_shared_ptr(serialized_message),
                message_info);
          } else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, SharedConstPtrSerializedMessageCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageCallback> ||
              std::is_same_v<T, SharedPtrSerializedMessageCallback>) {
            callback(create_serialized_message_unique_ptr_from_shared_ptr(serialized_message));
          } else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, SharedConstPtrSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageWithInfoCallback> ||
              std_is_same_v<T, SharedPtrSerializedMessageWithInfoCallback>) {
            callback(
                create_serialized_message_unique_ptr_from_shared_ptr(serialized_message),
                message_info);
          }
          // 输出其他类型的条件 (Conditions for output anything else)
          else if constexpr (  // NOLINT[whitespace/newline]
              std::is_same_v<T, ConstRefCallback> ||
              std::is_same_v<T, ConstRefROSMessageCallback> ||
              std::is_same_v<T, ConstRefWithInfoCallback> ||
              std::is_same_v<T, ConstRefWithInfoROSMessageCallback> ||
              std::is_same_v<T, UniquePtrCallback> ||
              std::is_same_v<T, UniquePtrROSMessageCallback> ||
              std::is_same_v<T, UniquePtrWithInfoCallback> ||
              std::is_same_v<T, UniquePtrWithInfoROSMessageCallback> ||
              std::is_same_v<T, SharedConstPtrCallback> ||
              std::is_same_v<T, SharedConstPtrROSMessageCallback> ||
              std::is_same_v<T, SharedConstPtrWithInfoCallback> ||
              std::is_same_v<T, SharedConstPtrWithInfoROSMessageCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrROSMessageCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrWithInfoCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrWithInfoROSMessageCallback> ||
              std::is_same_v<T, SharedPtrCallback> ||
              std::is_same_v<T, SharedPtrROSMessageCallback> ||
              std::is_same_v<T, SharedPtrWithInfoCallback> ||
              std::is_same_v<T, SharedPtrWithInfoROSMessageCallback>) {
            throw std::runtime_error(
                "cannot dispatch rclcpp::SerializedMessage to "
                "non-rclcpp::SerializedMessage callbacks");
          }
          // 捕获未处理的回调类型条件 (Condition to catch unhandled callback types)
          else {  // NOLINT[readability/braces]
            static_assert(always_false_v<T>, "unhandled callback type");
          }
        },
        callback_variant_);

    // 结束回调跟踪点 (End callback tracepoint)
    TRACEPOINT(callback_end, static_cast<const void *>(this));
  }

  /**
   * @brief 分发内部进程消息 (Dispatch intra-process messages)
   *
   * @param message 共享指针，指向订阅的消息类型 (Shared pointer to the subscribed message type)
   * @param message_info 包含关于消息的元数据信息 (Contains metadata information about the message)
   */
  void dispatch_intra_process(
      std::shared_ptr<const SubscribedType> message, const rclcpp::MessageInfo &message_info) {
    // 开始回调跟踪点 (Start callback tracepoint)
    TRACEPOINT(callback_start, static_cast<const void *>(this), true);

    // 检查变体是否为 "unset"，如果是，则抛出异常 (Check if the variant is "unset", throw if it is)
    if (callback_variant_.index() == 0) {
      if (std::get<0>(callback_variant_) == nullptr) {
        // 如果默认初始化或分配 nullptr，则可能发生这种情况 (This can happen if it is default
        // initialized, or if it is assigned nullptr)
        throw std::runtime_error("dispatch called on an unset AnySubscriptionCallback");
      }
    }

    // 分发 (Dispatch)
    std::visit(
        [&message, &message_info, this](auto &&callback) {
          using T = std::decay_t<decltype(callback)>;
          static constexpr bool is_ta = rclcpp::TypeAdapter<MessageT>::is_specialized::value;

          // 自定义类型的条件 (Conditions for custom type)
          if constexpr (is_ta && std::is_same_v<T, ConstRefCallback>) {
            callback(*message);
          } else if constexpr (is_ta && std::is_same_v<T, ConstRefWithInfoCallback>) {  // NOLINT
            callback(*message, message_info);
          } else if constexpr (  // NOLINT[readability/braces]
              is_ta &&
              (std::is_same_v<T, UniquePtrCallback> || std::is_same_v<T, SharedPtrCallback>)) {
            callback(create_custom_unique_ptr_from_custom_shared_ptr_message(message));
          } else if constexpr (  // NOLINT[readability/braces]
              is_ta && (std::is_same_v<T, UniquePtrWithInfoCallback> ||
                        std::is_same_v<T, SharedPtrWithInfoCallback>)) {
            callback(
                create_custom_unique_ptr_from_custom_shared_ptr_message(message), message_info);
          } else if constexpr (  // NOLINT[readability/braces]
              is_ta && (std::is_same_v<T, SharedConstPtrCallback> ||
                        std::is_same_v<T, ConstRefSharedConstPtrCallback>)) {
            callback(message);
          } else if constexpr (  // NOLINT[readability/braces]
              is_ta && (std::is_same_v<T, SharedConstPtrWithInfoCallback> ||
                        std::is_same_v<T, ConstRefSharedConstPtrWithInfoCallback>)) {
            callback(message, message_info);
          }
          // ROS 消息类型的条件 (Conditions for ROS message type)
          else if constexpr (std::is_same_v<
                                 T, ConstRefROSMessageCallback>) {  // NOLINT[readability/braces]
            if constexpr (is_ta) {
              auto local = convert_custom_type_to_ros_message_unique_ptr(*message);
              callback(*local);
            } else {
              callback(*message);
            }
          } else if constexpr (
              std::is_same_v<
                  T,
                  ConstRefWithInfoROSMessageCallback>) {  // NOLINT[readability/braces]
            if constexpr (is_ta) {
              auto local = convert_custom_type_to_ros_message_unique_ptr(*message);
              callback(*local, message_info);
            } else {
              callback(*message, message_info);
            }
          } else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, UniquePtrROSMessageCallback> ||
              std::is_same_v<T, SharedPtrROSMessageCallback>) {
            if constexpr (is_ta) {
              callback(convert_custom_type_to_ros_message_unique_ptr(*message));
            } else {
              callback(create_ros_unique_ptr_from_ros_shared_ptr_message(message));
            }
          } else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, UniquePtrWithInfoROSMessageCallback> ||
              std::is_same_v<T, SharedPtrWithInfoROSMessageCallback>) {
            if constexpr (is_ta) {
              callback(convert_custom_type_to_ros_message_unique_ptr(*message), message_info);
            } else {
              callback(create_ros_unique_ptr_from_ros_shared_ptr_message(message), message_info);
            }
          } else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, SharedConstPtrROSMessageCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrROSMessageCallback>) {
            if constexpr (is_ta) {
              callback(convert_custom_type_to_ros_message_unique_ptr(*message));
            } else {
              callback(message);
            }
          } else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, SharedConstPtrWithInfoROSMessageCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrWithInfoROSMessageCallback>) {
            if constexpr (is_ta) {
              callback(convert_custom_type_to_ros_message_unique_ptr(*message), message_info);
            } else {
              callback(message, message_info);
            }
          }
          // 捕获 SerializedMessage 类型的条件 (Condition to catch SerializedMessage types)
          else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, ConstRefSerializedMessageCallback> ||
              std::is_same_v<T, ConstRefSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, UniquePtrSerializedMessageCallback> ||
              std::is_same_v<T, UniquePtrSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, SharedConstPtrSerializedMessageCallback> ||
              std::is_same_v<T, SharedConstPtrSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, SharedPtrSerializedMessageCallback> ||
              std::is_same_v<T, SharedPtrSerializedMessageWithInfoCallback>) {
            throw std::runtime_error(
                "Cannot dispatch std::shared_ptr<const ROSMessageType> message "
                "to rclcpp::SerializedMessage");
          }
          // 捕获未处理的回调类型的条件 (Condition to catch unhandled callback types)
          else {  // NOLINT[readability/braces]
            static_assert(always_false_v<T>, "unhandled callback type");
          }
        },
        callback_variant_);

    // 结束回调跟踪点 (End callback tracepoint)
    TRACEPOINT(callback_end, static_cast<const void *>(this));
  }

  /**
   * @brief 分发内部进程消息 (Dispatch intra-process messages)
   *
   * @param message 唯一指针类型的订阅消息 (Unique pointer to the subscribed message)
   * @param message_info 消息相关信息 (Message-related information)
   */
  void dispatch_intra_process(
      std::unique_ptr<SubscribedType, SubscribedTypeDeleter> message,
      const rclcpp::MessageInfo &message_info) {
    // 开始回调跟踪点 (Start callback tracepoint)
    TRACEPOINT(callback_start, static_cast<const void *>(this), true);

    // 检查变量是否为 "unset"，如果是，则抛出异常 (Check if the variant is "unset", throw if it is)
    if (callback_variant_.index() == 0) {
      if (std::get<0>(callback_variant_) == nullptr) {
        // 如果默认初始化或分配nullptr，会发生这种情况 (This can happen if it is default
        // initialized, or if it is assigned nullptr)
        throw std::runtime_error("dispatch called on an unset AnySubscriptionCallback");
      }
    }

    // 分发 (Dispatch)
    std::visit(
        [&message, &message_info, this](auto &&callback) {
          // 静默clang关于未使用的 'this' lambda捕获的警告 (Silence clang warning about unused
          // 'this' lambda capture)
          (void)this;

          using T = std::decay_t<decltype(callback)>;
          static constexpr bool is_ta = rclcpp::TypeAdapter<MessageT>::is_specialized::value;

          // 自定义类型条件 (Conditions for custom type)
          if constexpr (is_ta && std::is_same_v<T, ConstRefCallback>) {
            callback(*message);
          } else if constexpr (is_ta && std::is_same_v<T, ConstRefWithInfoCallback>) {  // NOLINT
            callback(*message, message_info);
          } else if constexpr (  // NOLINT[readability/braces]
              is_ta &&
              (std::is_same_v<T, UniquePtrCallback> || std::is_same_v<T, SharedPtrCallback>)) {
            callback(std::move(message));
          } else if constexpr (  // NOLINT[readability/braces]
              is_ta && (std::is_same_v<T, UniquePtrWithInfoCallback> ||
                        std::is_same_v<T, SharedPtrWithInfoCallback>)) {
            callback(std::move(message), message_info);
          } else if constexpr (  // NOLINT[readability/braces]
              is_ta && (std::is_same_v<T, SharedConstPtrCallback> ||
                        std::is_same_v<T, ConstRefSharedConstPtrCallback>)) {
            callback(std::move(message));
          } else if constexpr (  // NOLINT[readability/braces]
              is_ta && (std::is_same_v<T, SharedConstPtrWithInfoCallback> ||
                        std::is_same_v<T, ConstRefSharedConstPtrWithInfoCallback>)) {
            callback(std::move(message), message_info);
          }
          // ROS消息类型条件 (Conditions for ROS message type)
          else if constexpr (std::is_same_v<  // NOLINT[readability/braces]
                                 T, ConstRefROSMessageCallback>) {
            if constexpr (is_ta) {
              auto local = convert_custom_type_to_ros_message_unique_ptr(*message);
              callback(*local);
            } else {
              callback(*message);
            }
          } else if constexpr (std::is_same_v<  // NOLINT[readability/braces]
                                   T, ConstRefWithInfoROSMessageCallback>) {
            if constexpr (is_ta) {
              auto local = convert_custom_type_to_ros_message_unique_ptr(*message);
              callback(*local, message_info);
            } else {
              callback(*message, message_info);
            }
          } else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, UniquePtrROSMessageCallback> ||
              std::is_same_v<T, SharedPtrROSMessageCallback>) {
            if constexpr (is_ta) {
              callback(convert_custom_type_to_ros_message_unique_ptr(*message));
            } else {
              callback(std::move(message));
            }
          } else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, UniquePtrWithInfoROSMessageCallback> ||
              std::is_same_v<T, SharedPtrWithInfoROSMessageCallback>) {
            if constexpr (is_ta) {
              callback(convert_custom_type_to_ros_message_unique_ptr(*message), message_info);
            } else {
              callback(std::move(message), message_info);
            }
          } else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, SharedConstPtrROSMessageCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrROSMessageCallback>) {
            if constexpr (is_ta) {
              callback(convert_custom_type_to_ros_message_unique_ptr(*message));
            } else {
              callback(std::move(message));
            }
          } else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, SharedConstPtrWithInfoROSMessageCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrWithInfoROSMessageCallback>) {
            if constexpr (is_ta) {
              callback(convert_custom_type_to_ros_message_unique_ptr(*message), message_info);
            } else {
              callback(std::move(message), message_info);
            }
          }
          // 捕获SerializedMessage类型条件 (Condition to catch SerializedMessage types)
          else if constexpr (  // NOLINT[readability/braces]
              std::is_same_v<T, ConstRefSerializedMessageCallback> ||
              std::is_same_v<T, ConstRefSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, UniquePtrSerializedMessageCallback> ||
              std::is_same_v<T, UniquePtrSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, SharedConstPtrSerializedMessageCallback> ||
              std::is_same_v<T, SharedConstPtrSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageCallback> ||
              std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageWithInfoCallback> ||
              std::is_same_v<T, SharedPtrSerializedMessageCallback> ||
              std::is_same_v<T, SharedPtrSerializedMessageWithInfoCallback>) {
            throw std::runtime_error(
                "Cannot dispatch std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> message "
                "to rclcpp::SerializedMessage");
          }
          // 捕获未处理的回调类型条件 (Condition to catch unhandled callback types)
          else {  // NOLINT[readability/braces]
            static_assert(always_false_v<T>, "unhandled callback type");
          }
        },
        callback_variant_);

    // 结束回调跟踪点 (End callback tracepoint)
    TRACEPOINT(callback_end, static_cast<const void *>(this));
  }

  /**
   * @brief 判断是否使用 take_shared 方法 (Determine if the take_shared method is used)
   *
   * @return 如果使用 take_shared 方法则返回 true，否则返回 false (Returns true if the take_shared
   * method is used, otherwise returns false)
   */
  constexpr bool use_take_shared_method() const {
    // 检查 callback_variant_ 是否为以下类型之一 (Check if callback_variant_ is one of the following
    // types)
    return std::holds_alternative<SharedConstPtrCallback>(callback_variant_) ||
           std::holds_alternative<SharedConstPtrWithInfoCallback>(callback_variant_) ||
           std::holds_alternative<ConstRefSharedConstPtrCallback>(callback_variant_) ||
           std::holds_alternative<ConstRefSharedConstPtrWithInfoCallback>(callback_variant_);
  }

  /**
   * @brief 判断回调是否为序列化消息回调 (Determine if the callback is a serialized message
   * callback)
   *
   * @return 如果是序列化消息回调，则返回 true，否则返回 false (Returns true if it is a serialized
   * message callback, otherwise returns false)
   */
  constexpr bool is_serialized_message_callback() const {
    // 检查 callback_variant_ 是否为以下类型之一 (Check if callback_variant_ is one of the following
    // types)
    return
        // 此回调类型用于处理已序列化的消息，以便在不进行反序列化的情况下直接处理它们。
        std::holds_alternative<ConstRefSerializedMessageCallback>(callback_variant_) ||
        // 此回调类型允许用户在处理序列化消息时拥有消息的所有权，从而避免不必要的拷贝。
        std::holds_alternative<UniquePtrSerializedMessageCallback>(callback_variant_) ||
        //  此回调类型允许多个实例共享对序列化消息的只读访问，以减少内存占用和拷贝次数。
        std::holds_alternative<SharedConstPtrSerializedMessageCallback>(callback_variant_) ||
        // 此回调类型结合了共享指针和常量引用的优点，以便在多个实例之间共享序列化消息的只读访问，同时避免拷贝共享指针。
        std::holds_alternative<ConstRefSharedConstPtrSerializedMessageCallback>(
            callback_variant_) ||
        // 此回调类型允许多个实例共享对序列化消息的读写访问，以减少内存占用和拷贝次数。
        std::holds_alternative<SharedPtrSerializedMessageCallback>(callback_variant_) ||
        // 此回调类型用于处理已序列化的消息及其相关信息（如发布者等），以便在不进行反序列化的情况下直接处理它们。
        std::holds_alternative<ConstRefSerializedMessageWithInfoCallback>(callback_variant_) ||
        // 此回调类型允许用户在处理序列化消息及其相关信息时拥有所有权，从而避免不必要的拷贝。
        std::holds_alternative<UniquePtrSerializedMessageWithInfoCallback>(callback_variant_) ||
        // 此回调类型允许多个实例共享对序列化消息及其相关信息的只读访问，以减少内存占用和拷贝次数。
        std::holds_alternative<SharedConstPtrSerializedMessageWithInfoCallback>(
            callback_variant_) ||
        // 此回调类型结合了共享指针和常量引用的优点，以便在多个实例之间共享序列化消息及其相关信息的只读访问，同时避免拷贝共享指针。
        std::holds_alternative<ConstRefSharedConstPtrSerializedMessageWithInfoCallback>(
            callback_variant_) ||
        // 此回调类型允许多个实例共享对序列化消息及其相关信息的读写访问，以减少内存占用和拷贝次数。
        std::holds_alternative<SharedPtrSerializedMessageWithInfoCallback>(callback_variant_);
  }

  /*
  这段代码主要包含两个函数：register_callback_for_tracing() 和
  get_variant()。register_callback_for_tracing() 函数用于在未禁用 tracetools
  的情况下，记录回调注册的 tracepoint。get_variant()
  函数分为两个版本，一个用于获取变体引用，另一个用于获取常量变体引用
  */

  /**
   * @brief 注册回调以进行跟踪 (Register callback for tracing)
   */
  void register_callback_for_tracing() {
#ifndef TRACETOOLS_DISABLED  // 如果没有禁用 tracetools (If tracetools is not disabled)
    std::visit(
        [this](auto &&callback) {
          // 记录回调注册的 tracepoint (Record the tracepoint of callback registration)
          TRACEPOINT(
              rclcpp_callback_register, static_cast<const void *>(this),
              tracetools::get_symbol(callback));
        },
        callback_variant_);  // 对存储在 callback_variant_ 中的回调进行访问 (Visit the callback
                             // stored in callback_variant_)
#endif  // TRACETOOLS_DISABLED
  }

  /**
   * @brief 获取变体引用 (Get variant reference)
   * @return 变体引用 (Variant reference)
   */
  typename HelperT::variant_type &get_variant() { return callback_variant_; }

  /**
   * @brief 获取常量变体引用 (Get const variant reference)
   * @return 常量变体引用 (Const variant reference)
   */
  const typename HelperT::variant_type &get_variant() const { return callback_variant_; }

private:
  /**
   * @brief 一个基于 rclcpp 的代码段，用于处理回调函数和消息类型的内存分配与删除。
   * @brief A code snippet based on rclcpp for handling callback functions and memory allocation and
   * deletion of message types.
   */
  // TODO(wjwwood): 切换到继承自 std::variant (即 HelperT::variant_type) 一旦
  // 继承自 std::variant 成为现实 (可能是 C++23?)，参见:
  //   http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p2162r0.html
  // 目前，将 variant 作为私有属性组合到此类中。
  // TODO(wjwwood): switch to inheriting from std::variant (i.e. HelperT::variant_type) once
  // inheriting from std::variant is realistic (maybe C++23?), see:
  //   http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p2162r0.html
  // For now, compose the variant into this class as a private attribute.
  typename HelperT::variant_type callback_variant_;

  // 订阅类型分配器，用于分配订阅消息类型的内存
  // Subscribed type allocator, used for allocating memory for subscribed message types
  SubscribedTypeAllocator subscribed_type_allocator_;
  // 订阅类型删除器，用于删除订阅消息类型的内存
  // Subscribed type deleter, used for deleting memory for subscribed message types
  SubscribedTypeDeleter subscribed_type_deleter_;
  // ROS 消息类型分配器，用于分配 ROS 消息类型的内存
  // ROS message type allocator, used for allocating memory for ROS message types
  ROSMessageTypeAllocator ros_message_type_allocator_;
  // ROS 消息类型删除器，用于删除 ROS 消息类型的内存
  // ROS message type deleter, used for deleting memory for ROS message types
  ROSMessageTypeDeleter ros_message_type_deleter_;
  // 序列化消息分配器，用于分配序列化消息的内存
  // Serialized message allocator, used for allocating memory for serialized messages
  SerializedMessageAllocator serialized_message_allocator_;
  // 序列化消息删除器，用于删除序列化消息的内存
  // Serialized message deleter, used for deleting memory for serialized messages
  SerializedMessageDeleter serialized_message_deleter_;
};

}  // namespace rclcpp

#endif  // RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_

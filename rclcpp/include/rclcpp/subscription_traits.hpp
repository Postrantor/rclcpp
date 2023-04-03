// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__SUBSCRIPTION_TRAITS_HPP_
#define RCLCPP__SUBSCRIPTION_TRAITS_HPP_

#include <memory>

#include "rcl/types.h"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription_options.hpp"

namespace rclcpp {

class QoS;

namespace subscription_traits {

// 当前 uncrustify 版本在此处对于用于继承的 `:` 和用于初始化列表的 `:` 存在误解
// 当使用模板结构时，冒号必须紧挨着它而没有任何空格，
// 而当不使用模板时，冒号必须用一个空格分隔。
// 干杯！
// The current version of uncrustify has a misinterpretation here
// between `:` used for inheritance vs for initializer list
// The result is that whenever a templated struct is used,
// the colon has to be without any whitespace next to it whereas
// when no template is used, the colon has to be separated by a space.
// Cheers!
template <typename T>
struct is_serialized_subscription_argument : std::false_type {};

// 针对 SerializedMessage 类型的特化
// Specialization for SerializedMessage type
template <>
struct is_serialized_subscription_argument<SerializedMessage> : std::true_type {};

// 针对 std::shared_ptr<SerializedMessage> 类型的特化
// Specialization for std::shared_ptr<SerializedMessage> type
template <>
struct is_serialized_subscription_argument<std::shared_ptr<SerializedMessage>> : std::true_type {};

// 判断给定类型 T 是否为序列化订阅类型
// Check if given type T is a serialized subscription type
template <typename T>
struct is_serialized_subscription : is_serialized_subscription_argument<T> {};

// 判断给定回调类型 CallbackT 的第一个参数是否为序列化订阅类型
// Check if the first argument of the given callback type CallbackT is a serialized subscription
// type
template <typename CallbackT>
struct is_serialized_callback
    : is_serialized_subscription_argument<
          typename rclcpp::function_traits::function_traits<CallbackT>::template argument_type<0>> {
};

// 提取给定类型 MessageT 的消息类型
// Extract message type from given type MessageT
template <typename MessageT>
struct extract_message_type {
  using type = typename std::remove_cv_t<std::remove_reference_t<MessageT>>;
};

// 针对 std::shared_ptr<MessageT> 类型的特化
// Specialization for std::shared_ptr<MessageT> type
template <typename MessageT>
struct extract_message_type<std::shared_ptr<MessageT>> : extract_message_type<MessageT> {};

// 针对 std::unique_ptr<MessageT, Deleter> 类型的特化
// Specialization for std::unique_ptr<MessageT, Deleter> type
template <typename MessageT, typename Deleter>
struct extract_message_type<std::unique_ptr<MessageT, Deleter>> : extract_message_type<MessageT> {};

// 判断给定回调类型 CallbackT 是否具有消息类型
// Check if given callback type CallbackT has a message type
template <
    typename CallbackT,
    typename AllocatorT = std::allocator<void>,
    // 如果 CallbackT 是整数（误认为是深度），则不尝试
    // Do not attempt if CallbackT is an integer (mistaken for depth)
    typename = std::enable_if_t<
        !std::is_integral<std::remove_cv_t<std::remove_reference_t<CallbackT>>>::value>,
    // 如果 CallbackT 是 QoS（误认为是 qos），则不尝试
    // Do not attempt if CallbackT is a QoS (mistaken for qos)
    typename = std::enable_if_t<
        !std::is_base_of<rclcpp::QoS, std::remove_cv_t<std::remove_reference_t<CallbackT>>>::value>,
    // 如果 CallbackT 是 rmw_qos_profile_t（误认为是 qos profile），则不尝试
    // Do not attempt if CallbackT is a rmw_qos_profile_t (mistaken for qos profile)
    typename = std::enable_if_t<!std::is_same<
        rmw_qos_profile_t,
        std::remove_cv_t<std::remove_reference_t<CallbackT>>>::value>,
    // 如果 CallbackT 是 rclcpp::SubscriptionOptionsWithAllocator，则不尝试
    // Do not attempt if CallbackT is a rclcpp::SubscriptionOptionsWithAllocator
    typename = std::enable_if_t<!std::is_same<
        rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>,
        std::remove_cv_t<std::remove_reference_t<CallbackT>>>::value>>
struct has_message_type
    : extract_message_type<
          typename rclcpp::function_traits::function_traits<CallbackT>::template argument_type<0>> {
};

}  // namespace subscription_traits
}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_TRAITS_HPP_

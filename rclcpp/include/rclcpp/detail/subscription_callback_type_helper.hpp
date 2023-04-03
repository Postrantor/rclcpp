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

#ifndef RCLCPP__DETAIL__SUBSCRIPTION_CALLBACK_TYPE_HELPER_HPP_
#define RCLCPP__DETAIL__SUBSCRIPTION_CALLBACK_TYPE_HELPER_HPP_

#include <memory>
#include <type_traits>

#include "rclcpp/function_traits.hpp"
#include "rclcpp/message_info.hpp"

namespace rclcpp {
namespace detail {

/// 模板元编程辅助器，用于将回调参数解析为 std::function。
/// Template metaprogramming helper used to resolve the callback argument into a std::function.
/**
 * 有时 CallbackT 已经是一个 std::function，但它也可能是一个函数指针、lambda、bind 或这些的变体。
 * Sometimes the CallbackT is a std::function already, but it could also be a function pointer,
 * lambda, bind, or some variant of those.
 *
 * 在某些情况下，例如 lambda 中可以在彼此之间转换参数的情况，
 * e.g. std::function<void (shared_ptr<...>)> and std::function<void (unique_ptr<...>)>,
 * 需要通过使用 function traits 独立检查参数来消除歧义，而不是依赖于重载两个 std::function 类型。
 * you need to make that not ambiguous by checking the arguments independently using function traits
 * rather than rely on overloading the two std::function types.
 *
 * 可以使用以下最小程序演示此问题（与 lambda 的问题）：
 * This issue, with the lambda's, can be demonstrated with this minimal program:
 *
 * \code{.cpp}
 *   #include <functional>
 *   #include <memory>
 *
 *   void f(std::function<void (std::shared_ptr<int>)>) {}
 *   void f(std::function<void (std::unique_ptr<int>)>) {}
 *
 *   int main() {
 *     // 编译失败，报错 "ambiguous call"。
 *     // Fails to compile with an "ambiguous call" error.
 *     f([](std::shared_ptr<int>){});
 *
 *     // 正常工作。
 *     // Works.
 *     std::function<void (std::shared_ptr<int>)> cb = [](std::shared_ptr<int>){};
 *     f(cb);
 *   }
 * \endcode
 *
 * 如果这个程序在 C++ 的未来版本中开始正常工作，那么这个类可能会变得多余。
 * If this program ever starts working in a future version of C++, this class may become redundant.
 *
 * 通过使用 SFINAE 和 rclcpp::function_traits::same_arguments<> 来缩小给定 CallbackT 的确切
 * std::function<> 类型，实现此辅助功能。 This helper works by using SFINAE with
 * rclcpp::function_traits::same_arguments<> to narrow down the exact std::function<> type for the
 * given CallbackT.
 */
template <typename MessageT, typename CallbackT, typename Enable = void>
struct SubscriptionCallbackTypeHelper {
  // 使用 rclcpp::function_traits::as_std_function 将 CallbackT 转换为 std::function 类型。
  // Convert CallbackT to std::function type using rclcpp::function_traits::as_std_function.
  using callback_type = typename rclcpp::function_traits::as_std_function<CallbackT>::type;
};

// 针对回调类型为 std::function<void(std::shared_ptr<const MessageT>)> 的特化。
// Specialization for callback type std::function<void(std::shared_ptr<const MessageT>)>.
template <typename MessageT, typename CallbackT>
struct SubscriptionCallbackTypeHelper<
    MessageT,
    CallbackT,
    typename std::enable_if_t<rclcpp::function_traits::same_arguments<
        CallbackT,
        std::function<void(std::shared_ptr<const MessageT>)>>::value>> {
  using callback_type = std::function<void(std::shared_ptr<const MessageT>)>;
};

// 针对回调类型为 std::function<void(std::shared_ptr<const MessageT>, const rclcpp::MessageInfo &)>
// 的特化。 Specialization for callback type std::function<void(std::shared_ptr<const MessageT>,
// const rclcpp::MessageInfo &)>.
template <typename MessageT, typename CallbackT>
struct SubscriptionCallbackTypeHelper<
    MessageT,
    CallbackT,
    typename std::enable_if_t<rclcpp::function_traits::same_arguments<
        CallbackT,
        std::function<void(std::shared_ptr<const MessageT>, const rclcpp::MessageInfo &)>>::
                                  value>> {
  using callback_type =
      std::function<void(std::shared_ptr<const MessageT>, const rclcpp::MessageInfo &)>;
};

// 针对回调类型为 std::function<void(const std::shared_ptr<const MessageT> &)> 的特化。
// Specialization for callback type std::function<void(const std::shared_ptr<const MessageT> &)>.
template <typename MessageT, typename CallbackT>
struct SubscriptionCallbackTypeHelper<
    MessageT,
    CallbackT,
    typename std::enable_if_t<rclcpp::function_traits::same_arguments<
        CallbackT,
        std::function<void(const std::shared_ptr<const MessageT> &)>>::value>> {
  using callback_type = std::function<void(const std::shared_ptr<const MessageT> &)>;
};

// 针对回调类型为 std::function<void(const std::shared_ptr<const MessageT> &, const
// rclcpp::MessageInfo &)> 的特化。 Specialization for callback type std::function<void(const
// std::shared_ptr<const MessageT> &, const rclcpp::MessageInfo &)>.
template <typename MessageT, typename CallbackT>
struct SubscriptionCallbackTypeHelper<
    MessageT,
    CallbackT,
    typename std::enable_if_t<rclcpp::function_traits::same_arguments<
        CallbackT,
        std::function<void(const std::shared_ptr<const MessageT> &, const rclcpp::MessageInfo &)>>::
                                  value>> {
  using callback_type =
      std::function<void(const std::shared_ptr<const MessageT> &, const rclcpp::MessageInfo &)>;
};

// 针对回调类型为 std::function<void(std::shared_ptr<MessageT>)> 的特化。
// Specialization for callback type std::function<void(std::shared_ptr<MessageT>)>.
template <typename MessageT, typename CallbackT>
struct SubscriptionCallbackTypeHelper<
    MessageT,
    CallbackT,
    typename std::enable_if_t<rclcpp::function_traits::same_arguments<
        CallbackT,
        std::function<void(std::shared_ptr<MessageT>)>>::value>> {
  using callback_type = std::function<void(std::shared_ptr<MessageT>)>;
};

// 针对回调类型为 std::function<void(std::shared_ptr<MessageT>, const rclcpp::MessageInfo &)>
// 的特化。 Specialization for callback type std::function<void(std::shared_ptr<MessageT>, const
// rclcpp::MessageInfo &)>.
template <typename MessageT, typename CallbackT>
struct SubscriptionCallbackTypeHelper<
    MessageT,
    CallbackT,
    typename std::enable_if_t<rclcpp::function_traits::same_arguments<
        CallbackT,
        std::function<void(std::shared_ptr<MessageT>, const rclcpp::MessageInfo &)>>::value>> {
  using callback_type = std::function<void(std::shared_ptr<MessageT>, const rclcpp::MessageInfo &)>;
};

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__SUBSCRIPTION_CALLBACK_TYPE_HELPER_HPP_

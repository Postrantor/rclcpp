// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DETAIL__TEMPLATE_CONTAINS_HPP_
#define RCLCPP__DETAIL__TEMPLATE_CONTAINS_HPP_

#include <type_traits>

namespace rclcpp {
namespace detail {

/// \brief 模板元函数，用于检查给定的 T 是否包含在列表 Us 中。
/// Template meta-function that checks if a given T is contained in the list Us.
template <typename T, typename... Us>
struct template_contains;

/// \brief 内联常量表达式，用于获取模板元函数 template_contains 的值。
/// Inline constexpr to get the value of the template meta-function template_contains.
template <typename... Args>
inline constexpr bool template_contains_v = template_contains<Args...>::value;

/// \brief 模板偏特化，用于递归地检查 T 是否等于 NextT 或者包含在 Us 中。
/// Template specialization for recursively checking if T is equal to NextT or contained in Us.
template <typename T, typename NextT, typename... Us>
struct template_contains<T, NextT, Us...> {
  // 判断 T 是否等于 NextT 或者包含在 Us 中。
  // Check if T is equal to NextT or contained in Us.
  enum { value = (std::is_same_v<T, NextT> || template_contains_v<T, Us...>)};
};

/// \brief 模板偏特化，用于处理 T 不包含在任何类型中的情况。
/// Template specialization for handling the case when T is not contained in any type.
template <typename T>
struct template_contains<T> {
  // 当 T 不包含在任何类型中时，值为 false。
  // Value is false when T is not contained in any type.
  enum { value = false };
};

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__TEMPLATE_CONTAINS_HPP_

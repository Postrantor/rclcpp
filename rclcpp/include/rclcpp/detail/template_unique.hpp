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

#ifndef RCLCPP__DETAIL__TEMPLATE_UNIQUE_HPP_
#define RCLCPP__DETAIL__TEMPLATE_UNIQUE_HPP_

#include <type_traits>

#include "rclcpp/detail/template_contains.hpp"

namespace rclcpp {
namespace detail {

/*
这段代码实现了一个模板元函数 template_unique，用于检查类型列表 Ts
中的类型是否唯一。它首先定义一个内联常量表达式 template_unique_v，用于简化对 template_unique
值的访问。然后，递归地定义 template_unique
结构体以处理多个类型的情况。在这个递归定义中，计算当前类型 NextT 是否在剩余类型列表 Ts
中，并与剩余类型列表的唯一性进行逻辑与操作。最后，定义 template_unique
结构体的特化版本来处理单个类型的情况。对于单个类型，返回 true 表示唯一。
*/

/// \brief 模板元函数，用于检查给定的类型列表 Ts 是否包含唯一的类型。
/// Template meta-function that checks if a given list Ts contains unique types.
template <typename... Ts>
struct template_unique;

/// \brief 一个内联常量表达式，用于简化对 template_unique 的访问。
/// An inline constexpr for easy access to template_unique value.
template <typename... Args>
inline constexpr bool template_unique_v = template_unique<Args...>::value;

/// \brief 递归定义 template_unique 结构体，用于处理多个类型的情况。
/// Recursively defines the template_unique struct to handle multiple types.
template <typename NextT, typename... Ts>
struct template_unique<NextT, Ts...> {
  /// \brief 计算当前类型 NextT 是否在剩余类型列表 Ts 中，并与剩余类型列表的唯一性进行逻辑与操作。
  /// Calculate whether the current type NextT is in the remaining type list Ts and perform logical
  /// AND with the uniqueness of the remaining type list.
  enum { value = !template_contains_v<NextT, Ts...> && template_unique_v<Ts...> };
};

/// \brief 定义 template_unique 结构体的特化版本，用于处理单个类型的情况。
/// Defines a specialization of the template_unique struct to handle a single type case.
template <typename T>
struct template_unique<T> {
  /// \brief 对于单个类型，返回 true，表示唯一。
  /// For a single type, return true indicating uniqueness.
  enum { value = true };
};

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__TEMPLATE_UNIQUE_HPP_

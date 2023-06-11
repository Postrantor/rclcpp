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

#ifndef RCLCPP_LIFECYCLE__TYPE_TRAITS__IS_MANAGEABLE_NODE_HPP_
#define RCLCPP_LIFECYCLE__TYPE_TRAITS__IS_MANAGEABLE_NODE_HPP_

#include <type_traits>
#include <utility>

/**
 * @brief 模板结构体 has_on_activate，检查是否存在 on_activate 成员函数
 * Template struct has_on_activate, checks for the existence of an on_activate member function
 *
 * @tparam T 类型参数
 * @tparam typename 用于 SFINAE 的类型，默认为 void
 * @tparam T Type parameter
 * @tparam typename Type used for SFINAE, defaults to void
 */
template <class T, typename = void>
struct has_on_activate {
  // 默认值为 false
  // Default value is false
  static constexpr bool value = false;
};

// 特化版本，当 T 类型中存在 on_activate 成员函数时，value 为 true
// Specialized version, when there's an on_activate member function in type T, value is true
template <class T>
struct has_on_activate<
    T,
    typename std::enable_if<
        std::is_same<void, decltype(std::declval<T>().on_activate())>::value>::type> {
  static constexpr bool value = true;
};

/**
 * @brief 模板结构体 has_on_deactivate，检查是否存在 on_deactivate 成员函数
 * Template struct has_on_deactivate, checks for the existence of an on_deactivate member function
 *
 * @tparam T 类型参数
 * @tparam typename 用于 SFINAE 的类型，默认为 void
 * @tparam T Type parameter
 * @tparam typename Type used for SFINAE, defaults to void
 */
template <class T, typename = void>
struct has_on_deactivate {
  // 默认值为 false
  // Default value is false
  static constexpr bool value = false;
};

// 特化版本，当 T 类型中存在 on_deactivate 成员函数时，value 为 true
// Specialized version, when there's an on_deactivate member function in type T, value is true
template <class T>
struct has_on_deactivate<
    T,
    typename std::enable_if<
        std::is_same<void, decltype(std::declval<T>().on_deactivate())>::value>::type> {
  static constexpr bool value = true;
};

/**
 * @brief 模板结构体 is_manageable_node，检查节点是否可管理（具有 on_activate 和 on_deactivate
 * 成员函数）
 * Template struct is_manageable_node, checks if the node is manageable (has on_activate
 * and on_deactivate member functions)
 *
 * @tparam T 类型参数
 * @tparam typename 用于 SFINAE 的类型，默认为 void
 * @tparam T Type parameter
 * @tparam typename Type used for SFINAE, defaults to void
 */
template <class T, typename = void>
struct is_manageable_node : std::false_type {};

// 特化版本，当 T 类型中同时存在 on_activate 和 on_deactivate 成员函数时，继承自 std::true_type
// Specialized version, when both on_activate and on_deactivate member functions exist in type T,
// inherits from std::true_type
template <class T>
struct is_manageable_node<
    T,
    typename std::enable_if<has_on_activate<T>::value && has_on_deactivate<T>::value>::type>
    : std::true_type {};

#endif  // RCLCPP_LIFECYCLE__TYPE_TRAITS__IS_MANAGEABLE_NODE_HPP_

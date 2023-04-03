// Copyright 2020 Open Source Robotics Foundation, Inc.
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
@{
uppercase_interface_name = interface_name.upper()
interface_typename = ''.join([part.capitalize() for part in interface_name.split('_')])
}@
#ifndef RCLCPP__NODE_INTERFACES__ @(uppercase_interface_name) _TRAITS_HPP_
#define RCLCPP__NODE_INTERFACES__ @(uppercase_interface_name) _TRAITS_HPP_

#include <functional>
#include <type_traits>

#include "rclcpp/node_interfaces/@(interface_name).hpp"

namespace rclcpp
{
  namespace node_interfaces {

  // 模板结构体，用于检查类 T 是否具有特定接口函数
  // Template structure for checking if class T has a specific interface function
  template <class T, typename = void>
  struct has_ @(interface_name) : std::false_type{};

  // 特化模板结构体，当类 T 具有特定接口函数时，将其值设为 true_type
  // Specialized template structure, setting its value to true_type when class T has the specific
  // interface function
  template <class T>
  struct has_ @(interface_name)<
      T,
      // 使用 std::enable_if 和 std::is_same 来检查类 T 的 get_@(interface_name)()
      // 函数是否返回正确的类型 Use std::enable_if and std::is_same to check if the
      // get_@(interface_name)() function of class T returns the correct type
      typename std::enable_if<std::is_same<
          // 期望的返回类型：std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)>
          // Expected return type: std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)>
          std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)>,
          // 使用 decltype 和 std::declval 获取类 T 的 get_@(interface_name)() 函数的返回类型
          // Use decltype and std::declval to get the return type of the get_@(interface_name)()
          // function of class T
          decltype(std::declval<T>().get_ @(interface_name)())>::value>::type> : std::true_type{};

  }  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__@(uppercase_interface_name)_TRAITS_HPP_

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
}@
#ifndef RCLCPP__NODE_INTERFACES__GET_ @(uppercase_interface_name) _HPP_
#define RCLCPP__NODE_INTERFACES__GET_ @(uppercase_interface_name) _HPP_

#include <memory>
#include <type_traits>
#include <utility>

#include "rclcpp/node_interfaces/@(interface_name).hpp"
#include "rclcpp/node_interfaces/@(interface_name)_traits.hpp"
#include "rcpputils/pointer_traits.hpp"

@{
interface_typename = ''.join([part.capitalize() for part in interface_name.split('_')])
}@

/// 这个头文件提供了 get_@(interface_name)() 模板函数。
/// This header provides the get_@(interface_name)() template function.
/**
 * 这个函数对于从各种类似 Node 的类中获取 @(interface_typename) 指针非常有用。
 * This function is useful for getting the @(interface_typename) pointer from
 * various kinds of Node-like classes.
 *
 * 只要该类具有一个名为 ``get_@(interface_name)()`` 的方法并返回一个指针，它就能获取到一个 std::shared_ptr<@(interface_typename)>。
 * It's able to get a std::shared_ptr to a @(interface_typename) so long as the class
 * has a method called ``get_@(interface_name)()`` which returns one.
 */

namespace rclcpp
{
  namespace node_interfaces {
  namespace detail {

  // 如果 NodeType 有一个名为 get_@(interface_name)() 的方法，该方法返回一个共享指针。
  // If NodeType has a method called get_@(interface_name)() which returns a shared pointer.
  template <
      typename NodeType,
      typename std::enable_if<
          has_ @(interface_name) < typename rcpputils::remove_pointer<NodeType>::type>::value,
      int>
  ::type = 0 > std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)> get_
           @(interface_name) _from_pointer(NodeType node_pointer) {
    // 如果节点指针为空，则抛出异常
    // If the node_pointer is nullptr, throw an exception
    if (!node_pointer) {
      throw std::invalid_argument("node cannot be nullptr");
    }
    // 返回节点指针的接口
    // Return the interface of the node_pointer
    return node_pointer->get_ @(interface_name)();
  }

  }  // namespace detail

  // 从指向 "Node like" 对象的指针中获取 @(interface_typename) 作为共享指针。
  // Get the @(interface_typename) as a shared pointer from a pointer to a "Node like" object.
  template <
      typename NodeType,
      typename std::enable_if<rcpputils::is_pointer<NodeType>::value, int>::type = 0>
  inline std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)> get_
      @(interface_name)(NodeType&& node) {
    // 将指针直接传递给详细实现
    // Forward pointers to detail implementation directly.
    return detail::get_ @(interface_name) _from_pointer(node);
  }

  // 从 "Node like" 对象中获取 @(interface_typename) 作为共享指针。
  // Get the @(interface_typename) as a shared pointer from a "Node like" object.
  template <
      typename NodeType,
      typename std::enable_if<!rcpputils::is_pointer<NodeType>::value, int>::type = 0>
  inline std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)> get_
      @(interface_name)(NodeType&& node) {
    // 将引用作为指针传递给详细实现
    // Forward references to detail implementation as a pointer.
    return detail::get_ @(interface_name) _from_pointer(&node);
  }

  // 保持 @(interface_typename) 为共享指针。
  // Keep the @(interface_typename) a shared pointer.
  inline std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)> get_ @(interface_name)(
      std::shared_ptr<rclcpp::node_interfaces::@(interface_typename)>& node_interface) {
    return node_interface;
  }

  }  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__GET_@(uppercase_interface_name)_HPP_

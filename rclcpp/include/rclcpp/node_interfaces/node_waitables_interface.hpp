// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_WAITABLES_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_WAITABLES_INTERFACE_HPP_

#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/detail/node_interfaces_helpers.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp {
namespace node_interfaces {

/// 纯虚拟接口类，用于Node API的NodeWaitables部分。
/// Pure virtual interface class for the NodeWaitables part of the Node API.
class NodeWaitablesInterface {
public:
  // 使用智能指针别名，仅限RCLCPP。
  // Using smart pointer aliases, only for RCLCPP.
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeWaitablesInterface)

  // 公共虚拟析构函数，默认实现。
  // Public virtual destructor with default implementation.
  RCLCPP_PUBLIC
  virtual ~NodeWaitablesInterface() = default;

  /// \brief 添加等待对象
  /// \param waitable_ptr 要添加的等待对象的共享指针
  /// \param group 回调组的共享指针
  ///
  /// \brief Add a waitable object
  /// \param waitable_ptr Shared pointer of the waitable object to be added
  /// \param group Shared pointer of the callback group
  RCLCPP_PUBLIC
  virtual void add_waitable(
      rclcpp::Waitable::SharedPtr waitable_ptr, rclcpp::CallbackGroup::SharedPtr group) = 0;

  /// \note 该函数不应抛出异常，因为它可能在析构函数中被调用
  /// \brief 移除等待对象
  /// \param waitable_ptr 要移除的等待对象的共享指针
  /// \param group 回调组的共享指针
  ///
  /// \note This function should not throw because it may be called in destructors
  /// \brief Remove a waitable object
  /// \param waitable_ptr Shared pointer of the waitable object to be removed
  /// \param group Shared pointer of the callback group
  RCLCPP_PUBLIC
  virtual void remove_waitable(
      rclcpp::Waitable::SharedPtr waitable_ptr,
      rclcpp::CallbackGroup::SharedPtr group) noexcept = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeWaitablesInterface, waitables)

#endif  // RCLCPP__NODE_INTERFACES__NODE_WAITABLES_INTERFACE_HPP_

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

#ifndef RCLCPP__NODE_INTERFACES__NODE_WAITABLES_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_WAITABLES_HPP_

#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp {
namespace node_interfaces {

/// Implementation of the NodeWaitables part of the Node API.
/**
 * @class NodeWaitables
 * @brief 一个用于管理等待对象的类 (A class for managing waitable objects)
 *
 * @tparam NodeWaitablesInterface 基础接口类型 (Base interface type)
 */
class NodeWaitables : public NodeWaitablesInterface {
public:
  /// 使用智能指针别名，仅限于此类 (Use smart pointer aliases, only for this class)
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeWaitables)

  /**
   * @brief 构造函数 (Constructor)
   *
   * @param node_base 指向节点基础接口的指针 (Pointer to the node base interface)
   */
  RCLCPP_PUBLIC
  explicit NodeWaitables(rclcpp::node_interfaces::NodeBaseInterface* node_base);

  /**
   * @brief 析构函数 (Destructor)
   */
  RCLCPP_PUBLIC
  virtual ~NodeWaitables();

  /**
   * @brief 添加等待对象 (Add a waitable object)
   *
   * @param waitable_base_ptr 等待对象的智能指针 (Smart pointer of the waitable object)
   * @param group 回调组的智能指针 (Smart pointer of the callback group)
   */
  RCLCPP_PUBLIC
  void add_waitable(
      rclcpp::Waitable::SharedPtr waitable_base_ptr,
      rclcpp::CallbackGroup::SharedPtr group) override;

  /**
   * @brief 移除等待对象 (Remove a waitable object)
   *
   * @param waitable_ptr 等待对象的智能指针 (Smart pointer of the waitable object)
   * @param group 回调组的智能指针 (Smart pointer of the callback group)
   */
  RCLCPP_PUBLIC
  void remove_waitable(
      rclcpp::Waitable::SharedPtr waitable_ptr,
      rclcpp::CallbackGroup::SharedPtr group) noexcept override;

private:
  /// 禁用拷贝构造函数和赋值运算符 (Disable copy constructor and assignment operator)
  RCLCPP_DISABLE_COPY(NodeWaitables)

  /// 节点基础接口的指针 (Pointer to the node base interface)
  rclcpp::node_interfaces::NodeBaseInterface* node_base_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_WAITABLES_HPP_

// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_COMPONENTS__NODE_INSTANCE_WRAPPER_HPP__
#define RCLCPP_COMPONENTS__NODE_INSTANCE_WRAPPER_HPP__

#include <functional>
#include <memory>

#include "rclcpp/node_interfaces/node_base_interface.hpp"

namespace rclcpp_components {
/// \class NodeInstanceWrapper
/// \brief 封装节点实例的类 (Encapsulates the node instance.)
class NodeInstanceWrapper {
public:
  /// \typedef NodeBaseInterfaceGetter
  /// \brief 获取 NodeBaseInterface 的函数类型定义 (Function type definition for getting
  /// NodeBaseInterface) \param[in] shared_ptr<void> 共享指针，指向任意类型的节点 (Shared pointer
  /// pointing to any type of node) \return NodeBaseInterface::SharedPtr 返回 NodeBaseInterface
  /// 的共享指针 (Returns a shared pointer to NodeBaseInterface)
  using NodeBaseInterfaceGetter =
      std::function<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr(
          const std::shared_ptr<void> &)>;

  /**
   * @brief 构造一个空的节点实例包装器 (Constructs an empty NodeInstanceWrapper)
   */
  NodeInstanceWrapper() : node_instance_(nullptr) {}

  /**
   * @brief 构造一个节点实例包装器，包含节点实例和节点基本接口获取器 (Constructs a
   * NodeInstanceWrapper with a node instance and a node base interface getter)
   *
   * @param[in] node_instance 节点实例的共享指针 (Shared pointer to the node instance)
   * @param[in] node_base_interface_getter 节点基本接口获取器的函数对象 (Function object for getting
   * the node base interface)
   */
  NodeInstanceWrapper(
      std::shared_ptr<void> node_instance, NodeBaseInterfaceGetter node_base_interface_getter)
      : node_instance_(node_instance), node_base_interface_getter_(node_base_interface_getter) {}

  /**
   * @brief 获取原始节点实例的类型擦除指针 (Get a type-erased pointer to the original Node instance)
   *
   * 这仅用于调试和特殊情况。对于大多数情况，“get_node_base_interface”将足够使用。
   * (This is only for debugging and special cases. For most cases `get_node_base_interface` will be
   * sufficient.)
   *
   * @return 封装的节点实例的共享指针 (Shared pointer to the encapsulated Node instance)
   */
  const std::shared_ptr<void> get_node_instance() const { return node_instance_; }

  /**
   * @brief 获取封装的节点实例的 NodeBaseInterface 指针 (Get NodeBaseInterface pointer for the
   * encapsulated Node Instance)
   *
   * @return 封装的节点实例的 NodeBaseInterface 共享指针 (Shared NodeBaseInterface pointer of the
   * encapsulated Node instance)
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
    return node_base_interface_getter_(node_instance_);
  }

private:
  std::shared_ptr<void> node_instance_;
  NodeBaseInterfaceGetter node_base_interface_getter_;
};
}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__NODE_INSTANCE_WRAPPER_HPP__

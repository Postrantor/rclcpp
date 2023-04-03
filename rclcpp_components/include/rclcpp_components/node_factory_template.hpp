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

#ifndef RCLCPP_COMPONENTS__NODE_FACTORY_TEMPLATE_HPP__
#define RCLCPP_COMPONENTS__NODE_FACTORY_TEMPLATE_HPP__

#include <functional>
#include <memory>

#include "rclcpp_components/node_factory.hpp"

namespace rclcpp_components {

/// NodeFactoryTemplate 是一个用于实例化组件的便捷类。
/// NodeFactoryTemplate is a convenience class for instantiating components.
/**
 * NodeFactoryTemplate 类可用于为实现单参数构造函数和 `get_node_base_interface` 的组件提供
 * NodeFactory 接口。 The NodeFactoryTemplate class can be used to provide the NodeFactory interface
 * for components that implement a single-argument constructor and `get_node_base_interface`.
 */
template <typename NodeT>
class NodeFactoryTemplate : public NodeFactory {
public:
  // 默认构造函数
  // Default constructor
  NodeFactoryTemplate() = default;

  // 默认析构函数
  // Default destructor
  virtual ~NodeFactoryTemplate() = default;

  /// 创建一个组件实例
  /// Create an instance of a component
  /**
   * \param[in] options 在构建组件时使用的附加选项。
   * \param[in] options Additional options used in the construction of the component.
   */
  NodeInstanceWrapper create_node_instance(const rclcpp::NodeOptions& options) override {
    // 使用给定的选项创建 NodeT 类型的共享指针
    // Create a shared_ptr of type NodeT with given options
    auto node = std::make_shared<NodeT>(options);

    // 返回一个 NodeInstanceWrapper 对象，其中包含创建的节点及其基本接口
    // Return a NodeInstanceWrapper object containing the created node and its base interface
    return NodeInstanceWrapper(node, std::bind(&NodeT::get_node_base_interface, node));
  }
};
}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__NODE_FACTORY_TEMPLATE_HPP__

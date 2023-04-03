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

#ifndef RCLCPP_COMPONENTS__NODE_FACTORY_HPP__
#define RCLCPP_COMPONENTS__NODE_FACTORY_HPP__

#include "rclcpp_components/node_instance_wrapper.hpp"

namespace rclcpp_components {

/// NodeFactory 接口用于类加载器实例化组件。
/// The NodeFactory interface is used by the class loader to instantiate components.
/**
 * NodeFactory 接口有两个目的：
 *  * 它允许不是从 `rclcpp::Node` 派生的类被用作组件。
 *  * 它允许在加载组件时调用派生构造函数。
 *
 * The NodeFactory interface serves two purposes:
 *  * It allows for classes not derived from `rclcpp::Node` to be used as components.
 *  * It allows derived constructors to be called when components are loaded.
 */
class NodeFactory {
public:
  // 默认构造函数
  // Default constructor
  NodeFactory() = default;

  // 默认虚析构函数，用于允许子类析构时正确调用基类析构函数
  // Default virtual destructor, allowing correct invocation of base class destructor when a derived
  // class is destructed
  virtual ~NodeFactory() = default;

  /// 创建一个组件实例
  /// Create an instance of a component
  /**
   * \param[in] options 用于构建组件的附加选项。
   * \param[in] options Additional options used in the construction of the component.
   */
  virtual NodeInstanceWrapper create_node_instance(const rclcpp::NodeOptions& options) = 0;
};
}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__NODE_FACTORY_HPP__

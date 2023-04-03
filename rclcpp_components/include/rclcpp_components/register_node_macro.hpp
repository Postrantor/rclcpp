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

#ifndef RCLCPP_COMPONENTS__REGISTER_NODE_MACRO_HPP__
#define RCLCPP_COMPONENTS__REGISTER_NODE_MACRO_HPP__

#include "class_loader/class_loader.hpp"
#include "rclcpp_components/node_factory_template.hpp"

/// 注册一个可以在运行时动态加载的组件 (Register a component that can be dynamically loaded at
/// runtime)
/**
 * 注册宏应该每个组件库中出现一次。
 * 该宏应该出现在单个翻译单元中。
 *
 * 对于 NodeClass 的有效参数，需要满足以下条件：
 *  * 具有一个构造函数，接受一个 `rclcpp::NodeOptions` 实例作为唯一参数。
 *  * 具有一个如下签名的方法：
 *      `rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface`
 *
 * 注意：NodeClass 不需要继承自 `rclcpp::Node`，但这是最简单的方法。
 */
#define RCLCPP_COMPONENTS_REGISTER_NODE(NodeClass) \
  CLASS_LOADER_REGISTER_CLASS(                     \
      rclcpp_components::NodeFactoryTemplate<NodeClass>, rclcpp_components::NodeFactory)

#endif  // RCLCPP_COMPONENTS__REGISTER_NODE_MACRO_HPP__

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

#ifndef RCLCPP__NODE_INTERFACES__NODE_INTERFACES_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_INTERFACES_HPP_

#include <memory>

#include "rclcpp/detail/template_unique.hpp"
#include "rclcpp/node_interfaces/detail/node_interfaces_helpers.hpp"

#define ALL_RCLCPP_NODE_INTERFACES                                                                \
  rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeClockInterface,        \
      rclcpp::node_interfaces::NodeGraphInterface, rclcpp::node_interfaces::NodeLoggingInterface, \
      rclcpp::node_interfaces::NodeParametersInterface,                                           \
      rclcpp::node_interfaces::NodeServicesInterface,                                             \
      rclcpp::node_interfaces::NodeTimeSourceInterface,                                           \
      rclcpp::node_interfaces::NodeTimersInterface, rclcpp::node_interfaces::NodeTopicsInterface, \
      rclcpp::node_interfaces::NodeWaitablesInterface

namespace rclcpp {
namespace node_interfaces {

/// \brief 一个用于聚合节点接口的辅助类 (A helper class for aggregating node interfaces)
/// \tparam InterfaceTs 节点接口类型 (Node interface types)
template <typename... InterfaceTs>
class NodeInterfaces
    : public detail::
          // 检查提供的接口类型是否符合要求 (Check if the provided interface types meet the
          // requirements)
      NodeInterfacesSupportCheck<detail::NodeInterfacesStorage<InterfaceTs...>, InterfaceTs...>,
      public detail::
          // 提供对接口类型的支持 (Provide support for the interface types)
      NodeInterfacesSupports<detail::NodeInterfacesStorage<InterfaceTs...>, InterfaceTs...> {
  // 静态断言，确保至少提供一个接口类型作为模板参数 (Static assertion to ensure at least one
  // interface is provided as a template argument)
  static_assert(
      0 != sizeof...(InterfaceTs), "must provide at least one interface as a template argument");
  // 静态断言，确保提供的模板参数是唯一的 (Static assertion to ensure that the provided template
  // parameters are unique)
  static_assert(
      rclcpp::detail::template_unique_v<InterfaceTs...>, "must provide unique template parameters");

  // 定义 NodeInterfacesSupportsT 类型，用于支持提供的接口类型 (Define the NodeInterfacesSupportsT
  // type for supporting the provided interface types)
  using NodeInterfacesSupportsT =
      detail::NodeInterfacesSupports<detail::NodeInterfacesStorage<InterfaceTs...>, InterfaceTs...>;

public:
  /// 创建一个新的NodeInterfaces对象，使用给定的类似节点对象的接口。
  /// Create a new NodeInterfaces object using the given node-like object's interfaces.
  /**
   * 通过将它们作为模板参数传递来指定您需要哪些接口。
   * Specify which interfaces you need by passing them as template parameters.
   *
   * 这允许您将来自不同来源的接口聚合在一起，作为单个聚合对象传递给任何接受节点接口或类似节点的函数，而无需对该函数进行模板化。
   * This allows you to aggregate interfaces from different sources together to pass as a single
   * aggregate object to any functions that take node interfaces or node-likes, without needing to
   * templatize that function.
   *
   * 您还可以使用此构造函数创建包含另一个NodeInterfaces接口子集的NodeInterfaces。
   * You may also use this constructor to create a NodeInterfaces that contains a subset of
   * another NodeInterfaces' interfaces.
   *
   * 最后，这个类支持从类似节点对象的隐式转换，允许您直接将类似节点传递给接受NodeInterfaces对象的函数。
   * Finally, this class supports implicit conversion from node-like objects, allowing you to
   * directly pass a node-like to a function that takes a NodeInterfaces object.
   *
   * 使用示例：
   * Usage examples:
   *   ```cpp
   *   // 假设我们有一些函数：
   *   // Suppose we have some function:
   *   void fn(NodeInterfaces<NodeBaseInterface, NodeClockInterface> interfaces);
   *
   *   // 然后我们可以，显式地：
   *   // Then we can, explicitly:
   *   rclcpp::Node node("some_node");
   *   auto ni = NodeInterfaces<NodeBaseInterface, NodeClockInterface>(node);
   *   fn(ni);
   *
   *   // 但也可以：
   *   // But also:
   *   fn(node);
   *
   *   // 子集一个NodeInterfaces对象也可以！
   *   // Subsetting a NodeInterfaces object also works!
   *   auto ni_base = NodeInterfaces<NodeBaseInterface>(ni);
   *
   *   // 或者聚合它们（您可以从不同的类似节点中聚合接口）
   *   // Or aggregate them (you could aggregate interfaces from disparate node-likes)
   *   auto ni_aggregated = NodeInterfaces<NodeBaseInterface, NodeClockInterface>(
   *     node->get_node_base_interface(),
   *     node->get_node_clock_interface()
   *   )
   *
   *   // 然后访问接口：
   *   // And then to access the interfaces:
   *   // 使用get<>获取
   *   // Get with get<>
   *   auto base = ni.get<NodeBaseInterface>();
   *
   *   // 或者使用适当的getter
   *   // Or the appropriate getter
   *   auto clock = ni.get_clock_interface();
   *   ```
   *
   * 您可以使用rclcpp附带的任何标准节点接口：
   * You may use any of the standard node interfaces that come with rclcpp:
   *   - rclcpp::node_interfaces::NodeBaseInterface
   *   - rclcpp::node_interfaces::NodeClockInterface
   *   - rclcpp::node_interfaces::NodeGraphInterface
   *   - rclcpp::node_interfaces::NodeLoggingInterface
   *   - rclcpp::node_interfaces::NodeParametersInterface
   *   - rclcpp::node_interfaces::NodeServicesInterface
   *   - rclcpp::node_interfaces::NodeTimeSourceInterface
   *   - rclcpp::node_interfaces::NodeTimersInterface
   *   - rclcpp::node_interfaces::NodeTopicsInterface
   *   - rclcpp::node_interfaces::NodeWaitablesInterface
   *
   * 或者您可以使用自定义接口，只要您使用RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT宏为rclcpp::node_interfaces::detail::NodeInterfacesSupport结构创建一个模板特化。
   * Or you use custom interfaces as long as you make a template specialization
   * of the rclcpp::node_interfaces::detail::NodeInterfacesSupport struct using
   * the RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT macro.
   *
   * 使用示例：
   * Usage example:
   *   ```RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeBaseInterface, base)```
   *
   * 如果您选择不使用helper宏，那么您可以自己专门化模板，但是您必须：
   * If you choose not to use the helper macro, then you can specialize the
   * template yourself, but you must:
   *
   *   -
   * 提供一个get_from_node_like方法的模板特化，该方法从存储接口的任何类似节点中获取接口，使用类似节点的getter
   *   - Provide a template specialization of the get_from_node_like method that gets the interface
   *     from any node-like that stores the interface, using the node-like's getter
   *   - 使用using指令将is_supported类型指定为std::true_type
   *   - Designate the is_supported type as std::true_type using a using directive
   *   -
   * 提供任意数量的getter方法，用于在NodeInterface对象中获取接口，注意存储类的getter将应用于所有支持的接口。
   *   - Provide any number of getter methods to be used to obtain the interface with the
   *     NodeInterface object, noting that the getters of the storage class will apply to all
   *     supported interfaces.
   *     -
   * getter方法的名称不应与其他接口getter特化的名称冲突，如果这些其他接口要在同一个NodeInterfaces对象中进行聚合。
   *     - The getter method names should not clash in name with any other interface getter
   *       specializations if those other interfaces are meant to be aggregated in the same
   *       NodeInterfaces object.
   *
   * \param[in] node 从中获取节点接口的类似节点对象
   * \param[in] node Node-like object from which to get the node interfaces
   */
  template <typename NodeT>
  NodeInterfaces(NodeT& node)  // NOLINT(runtime/explicit)
      : NodeInterfacesSupportsT(node) {}

  explicit NodeInterfaces(std::shared_ptr<InterfaceTs>... args)
      : NodeInterfacesSupportsT(args...) {}
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_INTERFACES_HPP_

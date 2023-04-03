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

#ifndef RCLCPP__NODE_INTERFACES__DETAIL__NODE_INTERFACES_HELPERS_HPP_
#define RCLCPP__NODE_INTERFACES__DETAIL__NODE_INTERFACES_HELPERS_HPP_

#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {
namespace detail {

/**
 * @brief 初始化元组的辅助函数 (Helper function to initialize the tuple)
 *
 * @tparam NodeT 节点类型 (Node type)
 * @tparam Ts 接口类型参数包 (Interface types parameter pack)
 * @param n 节点引用 (Node reference)
 * @return std::tuple<std::shared_ptr<Ts>...> 初始化后的接口元组 (Initialized interface tuple)
 */
template <typename NodeT, typename... Ts>
std::tuple<std::shared_ptr<Ts>...> init_tuple(NodeT &n);

/// 存储接口的元组、提供构造函数和getter方法的结构体 (Structure that stores the interfaces in a
/// tuple, provides constructors, and getters.)
template <typename... InterfaceTs>
struct NodeInterfacesStorage {
  /**
   * @brief 构造函数，使用节点初始化接口元组 (Constructor using node to initialize the interface
   * tuple)
   *
   * @tparam NodeT 节点类型 (Node type)
   * @param node 节点引用 (Node reference)
   */
  template <typename NodeT>
  NodeInterfacesStorage(NodeT &node)  // NOLINT(runtime/explicit)
      : interfaces_(init_tuple<decltype(node), InterfaceTs...>(node)) {}

  /**
   * @brief 显式构造函数，使用共享指针初始化接口元组 (Explicit constructor using shared pointers to
   * initialize the interface tuple)
   *
   * @param args 接口共享指针 (Interface shared pointers)
   */
  explicit NodeInterfacesStorage(std::shared_ptr<InterfaceTs>... args) : interfaces_(args...) {}

  /**
   * @brief 获取单个节点接口的非常量 getter 方法 (Non-const getter for individual Node Interface)
   *
   * @tparam NodeInterfaceT 要获取的节点接口类型 (Type of the Node Interface to get)
   * @return std::shared_ptr<NodeInterfaceT> 节点接口的共享指针 (Shared pointer to the Node
   * Interface)
   */
  template <typename NodeInterfaceT>
  std::shared_ptr<NodeInterfaceT> get() {
    static_assert(
        (std::is_same_v<NodeInterfaceT, InterfaceTs> || ...),
        "NodeInterfaces class does not contain given NodeInterfaceT");
    return std::get<std::shared_ptr<NodeInterfaceT>>(interfaces_);
  }

  /**
   * @brief 获取单个节点接口的常量 getter 方法 (Const getter for individual Node Interface)
   *
   * @tparam NodeInterfaceT 要获取的节点接口类型 (Type of the Node Interface to get)
   * @return std::shared_ptr<const NodeInterfaceT> 节点接口的常量共享指针 (Const shared pointer to
   * the Node Interface)
   */
  template <typename NodeInterfaceT>
  std::shared_ptr<const NodeInterfaceT> get() const {
    static_assert(
        (std::is_same_v<NodeInterfaceT, InterfaceTs> || ...),
        "NodeInterfaces class does not contain given NodeInterfaceT");
    return std::get<std::shared_ptr<NodeInterfaceT>>(interfaces_);
  }

protected:
  std::tuple<std::shared_ptr<InterfaceTs>...> interfaces_;  ///< 接口元组 (Interface tuple)
};

/// 原型 NodeInterfacesSupports.
/**
 * 应将 NodeInterfacesSupports<..., T, ...> 读作 "NodeInterfaces 支持 T"，并且
 * 如果为 T 专门化了 NodeInterfacesSupport，则 is_supported 应设置为
 * std::true_type，但默认情况下，它是 std::false_type，这将
 * 导致在尝试使用 NodeInterfaces 的 T 时出现编译器错误。
 */
template <typename StorageClassT, typename... Ts>
struct NodeInterfacesSupports;

/// NodeInterfacesSupportCheck 模板元函数的原型.
/**
 * 此元函数检查给定的所有类型是否受支持，
 * 如果使用不受支持的类型，将抛出更易于人类阅读的错误。
 */
template <typename StorageClassT, typename... InterfaceTs>
struct NodeInterfacesSupportCheck;

/// 迭代特殊化，确保类受支持并被继承。
template <typename StorageClassT, typename NextInterfaceT, typename... RemainingInterfaceTs>
struct NodeInterfacesSupportCheck<StorageClassT, NextInterfaceT, RemainingInterfaceTs...>
    : public NodeInterfacesSupportCheck<StorageClassT, RemainingInterfaceTs...> {
  // 静态断言，检查 NodeInterfacesSupports 是否支持 NextInterfaceT
  static_assert(
      NodeInterfacesSupports<StorageClassT, NextInterfaceT>::is_supported::value,
      "given NodeInterfaceT is not supported by rclcpp::node_interfaces::NodeInterfaces");
};

/// 当没有更多的 "RemainingInterfaceTs" 时终止。
template <typename StorageClassT>
struct NodeInterfacesSupportCheck<StorageClassT> {};

/// 默认特化，需要为每个支持的接口进行特化。
template <typename StorageClassT, typename... RemainingInterfaceTs>
struct NodeInterfacesSupports {
  // 特化需要将其设置为 std::true_type，以便于其他接口使用。
  using is_supported = std::false_type;
};

/// Terminating specialization of NodeInterfacesSupports.
/**
 * @tparam StorageClassT 类型参数，用于指定存储类 (Type parameter, used to specify the storage
 * class)
 */
template <typename StorageClassT>
struct NodeInterfacesSupports<StorageClassT> : public StorageClassT {
  /// Perfect forwarding constructor to get arguments down to StorageClassT.
  /**
   * @tparam ArgsT 类型参数包，用于构造函数的完美转发 (Type parameter pack, used for perfect
   * forwarding in the constructor)
   * @param args 可变参数列表，用于传递给 StorageClassT 的构造函数 (Variadic argument list, used to
   * pass to the constructor of StorageClassT)
   */
  template <typename... ArgsT>
  explicit NodeInterfacesSupports(ArgsT &&...args) : StorageClassT(std::forward<ArgsT>(args)...) {}
};

// Helper functions to initialize the tuple in NodeInterfaces.

/**
 * @brief 初始化元素 (Initialize element)
 *
 * @tparam StorageClassT 存储类类型 (Storage class type)
 * @tparam ElementT 元素类型 (Element type)
 * @tparam TupleT 元组类型 (Tuple type)
 * @tparam NodeT 节点类型 (Node type)
 * @param t 元组引用 (Tuple reference)
 * @param n 节点引用 (Node reference)
 */
template <typename StorageClassT, typename ElementT, typename TupleT, typename NodeT>
void init_element(TupleT &t, NodeT &n) {
  std::get<std::shared_ptr<ElementT>>(t) =
      NodeInterfacesSupports<StorageClassT, ElementT>::get_from_node_like(n);
}

/**
 * @brief 初始化元组 (Initialize tuple)
 *
 * @tparam NodeT 节点类型 (Node type)
 * @tparam Ts 可变模板参数，元素类型 (Variadic template parameters, element types)
 * @param n 节点引用 (Node reference)
 * @return 初始化后的元组 (Initialized tuple)
 */
template <typename NodeT, typename... Ts>
std::tuple<std::shared_ptr<Ts>...> init_tuple(NodeT &n) {
  using StorageClassT = NodeInterfacesStorage<Ts...>;
  std::tuple<std::shared_ptr<Ts>...> t;
  (init_element<StorageClassT, Ts>(t, n), ...);
  return t;
}

/// Macro for creating specializations with less boilerplate.
/**
 * 使用此宏可以为接口类添加支持（在以下情况下）：
 * - 标准 getter 是 get_node_{NodeInterfaceName}_interface()，以及
 * - getter 返回一个非 const shared_ptr<{NodeInterfaceType}>
 *
 * 在 rclcpp 中，可以看到使用此功能的标准节点接口头文件示例，
 * 例如 rclcpp/node_interfaces/node_base_interface.hpp 包含：
 *
 *   RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeBaseInterface, base)
 *
 * 如果您的接口具有非标准 getter，或者希望对其进行检测或其他操作，
 * 则需要在不使用此宏的情况下创建自己的 NodeInterfacesSupports 结构体特化。
 *
 * You can use this macro to add support for your interface class if:
 * - The standard getter is get_node_{NodeInterfaceName}_interface(), and
 * - the getter returns a non-const shared_ptr<{NodeInterfaceType}>
 *
 * Examples of using this can be seen in the standard node interface headers
 * in rclcpp, e.g. rclcpp/node_interfaces/node_base_interface.hpp has:
 *
 *   RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeBaseInterface, base)
 *
 * If your interface has a non-standard getter, or you want to instrument it or
 * something like that, then you'll need to create your own specialization of
 * the NodeInterfacesSupports struct without this macro.
 */
#define RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(NodeInterfaceType, NodeInterfaceName)           \
  namespace rclcpp::node_interfaces::detail {                                                 \
  template <typename StorageClassT, typename... RemainingInterfaceTs>                         \
  struct NodeInterfacesSupports<StorageClassT, NodeInterfaceType, RemainingInterfaceTs...>    \
      : public NodeInterfacesSupports<StorageClassT, RemainingInterfaceTs...> {               \
    using is_supported = std::true_type;                                                      \
                                                                                              \
    template <typename NodeT>                                                                 \
    static std::shared_ptr<NodeInterfaceType> get_from_node_like(NodeT &node_like) {          \
      return node_like.get_node_##NodeInterfaceName##_interface();                            \
    }                                                                                         \
                                                                                              \
    /* Perfect forwarding constructor to get arguments down to StorageClassT (eventually). */ \
    template <typename... ArgsT>                                                              \
    explicit NodeInterfacesSupports(ArgsT &&...args)                                          \
        : NodeInterfacesSupports<StorageClassT, RemainingInterfaceTs...>(                     \
              std::forward<ArgsT>(args)...) {}                                                \
                                                                                              \
    std::shared_ptr<NodeInterfaceType> get_node_##NodeInterfaceName##_interface() {           \
      return StorageClassT::template get<NodeInterfaceType>();                                \
    }                                                                                         \
  };                                                                                          \
  }  // namespace rclcpp::node_interfaces::detail

}  // namespace detail
}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__DETAIL__NODE_INTERFACES_HELPERS_HPP_

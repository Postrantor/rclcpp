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

/** \mainpage rclcpp_components: 包含用于动态加载组件的工具的软件包。
 *  rclcpp_components: Package containing tools for dynamically loadable components.
 *
 * - ComponentManager: 管理组件的节点。它提供了加载、卸载和列出当前组件的服务。
 *   - ComponentManager: Node to manage components. It has the services to load, unload and list
 * current components.
 *   - (rclcpp_components/component_manager.hpp)
 * - Node factory: 类加载器用 NodeFactory 接口实例化组件。
 *   - Node factory: The NodeFactory interface is used by the class loader to instantiate
 * components.
 *   - (rclcpp_components/node_factory.hpp)
 *   - 它允许不是从 `rclcpp::Node` 派生的类作为组件使用。
 *   - It allows for classes not derived from `rclcpp::Node` to be used as components.
 *   - 当加载组件时，允许调用派生的构造函数。
 *   - It allows derived constructors to be called when components are loaded.
 *
 * 一些有用的抽象和实用程序：
 * - Some useful abstractions and utilities:
 * - [RCLCPP_COMPONENTS_REGISTER_NODE: 注册可在运行时动态加载的组件
 *   - [RCLCPP_COMPONENTS_REGISTER_NODE: Register a component that can be dynamically loaded at
 * runtime.
 *   - (include/rclcpp_components/register_node_macro.hpp)
 *
 * 一些有用的内部抽象和实用程序：
 * - Some useful internal abstractions and utilities:
 * - 控制库上符号可见性的宏
 *   - Macros for controlling symbol visibility on the library
 *   - (rclcpp_components/visibility_control.h)
 *
 * 包含用于注册组件的 CMake 工具的软件包：
 * - Package containing CMake tools for register components:
 * - `rclcpp_components_register_node` 使用 ament 资源索引注册 rclcpp 组件并创建可执行文件。
 *   - `rclcpp_components_register_node` Register an rclcpp component with the ament resource index
 * and create an executable.
 * - `rclcpp_components_register_nodes` 使用 ament 资源索引注册 rclcpp
 * 组件。传递的库可以包含每个通过宏注册的多个节点。
 *   - `rclcpp_components_register_nodes` Register an rclcpp component with the ament resource
 * index. The passed library can contain multiple nodes each registered via macro.
 */

#ifndef RCLCPP_COMPONENTS__COMPONENT_MANAGER_HPP__
#define RCLCPP_COMPONENTS__COMPONENT_MANAGER_HPP__

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "composition_interfaces/srv/list_nodes.hpp"
#include "composition_interfaces/srv/load_node.hpp"
#include "composition_interfaces/srv/unload_node.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "rclcpp_components/visibility_control.hpp"

namespace class_loader {
class ClassLoader;
}  // namespace class_loader

namespace rclcpp_components {

/// \class ComponentManagerException
/// \brief 当组件管理器类发生错误时抛出的异常。Thrown when an error occurs in the Component Manager
/// class.
class ComponentManagerException : public std::runtime_error {
public:
  /// \brief 构造函数，接受一个字符串作为错误描述。Constructor that takes a string as an error
  /// description. \param error_desc 错误描述。Error description.
  explicit ComponentManagerException(const std::string& error_desc)
      : std::runtime_error(error_desc) {
  }  // 使用传入的错误描述初始化基类 std::runtime_error。Initialize the base class
     // std::runtime_error with the passed error description.
};

/// ComponentManager 管理用于加载、卸载和获取已加载组件列表的服务。
/// (ComponentManager handles the services to load, unload, and get the list of loaded components.)
class ComponentManager : public rclcpp::Node {
public:
  // 使用 composition_interfaces 中的 LoadNode 作为加载节点的服务类型
  // (Use LoadNode from composition_interfaces as the service type for loading nodes)
  using LoadNode = composition_interfaces::srv::LoadNode;

  // 使用 composition_interfaces 中的 UnloadNode 作为卸载节点的服务类型
  // (Use UnloadNode from composition_interfaces as the service type for unloading nodes)
  using UnloadNode = composition_interfaces::srv::UnloadNode;

  // 使用 composition_interfaces 中的 ListNodes 作为列出节点的服务类型
  // (Use ListNodes from composition_interfaces as the service type for listing nodes)
  using ListNodes = composition_interfaces::srv::ListNodes;

  /// 表示一个组件资源。
  /// (Represents a component resource.)
  /**
   * 是类名（用于类加载器）和库路径（绝对路径）的一对值
   * (Is a pair of class name (for class loader) and library path (absolute))
   */
  using ComponentResource = std::pair<std::string, std::string>;

  /// 默认构造函数 (Default constructor)
  /**
   * 初始化组件管理器。它创建了以下服务：加载节点、卸载节点和列出节点。
   * (Initializes the component manager. It creates the services: load node, unload node and list
   * nodes.)
   *
   * \param executor 将会执行节点的执行器。(the executor which will spin the node)
   * \param node_name 数据来源节点的名称。(the name of the node that the data originates from)
   * \param node_options 控制节点创建的附加选项。(additional options to control creation of the
   * node)
   */
  RCLCPP_COMPONENTS_PUBLIC
  ComponentManager(
      std::weak_ptr<rclcpp::Executor> executor =
          std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
      std::string node_name = "ComponentManager",
      const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()
                                                    .start_parameter_services(false)
                                                    .start_parameter_event_publisher(false));

  // 虚拟析构函数 (Virtual destructor)
  RCLCPP_COMPONENTS_PUBLIC
  virtual ~ComponentManager();

  /// 返回给定包中有效可加载组件的列表。 (Return a list of valid loadable components in a given
  /// package)
  /**
   * \param package_name 包的名称 (name of the package)
   * \param resource_index 可执行文件的名称 (name of the executable)
   * \throws ComponentManagerException 如果资源未找到或资源条目无效 (if the resource was not found
   * or an invalid resource entry) \return 组件资源列表 (a list of component resources)
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual std::vector<ComponentResource> get_component_resources(
      const std::string& package_name,
      const std::string& resource_index = "rclcpp_components") const;

  /// 从动态库实例化一个组件。 (Instantiate a component from a dynamic library)
  /**
   * \param resource 组件资源（类名+库路径）(a component resource (class name + library path))
   * \return 一个 NodeFactory 接口 (a NodeFactory interface)
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual std::shared_ptr<rclcpp_components::NodeFactory> create_component_factory(
      const ComponentResource& resource);

  /// 成员函数，用于在组件中设置一个执行器 (Member function to set an executor in the component)
  /**
   * \param executor 要设置的执行器 (executor to be set)
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual void set_executor(const std::weak_ptr<rclcpp::Executor> executor);

protected:
  /// 创建加载组件的节点选项
  /// Create node options for loaded component
  /**
   * \param request 包含要加载节点的信息
   * \param request information with the node to load
   * \return 节点选项
   * \return node options
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual rclcpp::NodeOptions create_node_options(const std::shared_ptr<LoadNode::Request> request);

  /// 将组件节点添加到执行器模型中，它在 on_load_node() 中被调用
  /// Add component node to executor model, it's invoked in on_load_node()
  /**
   * \param node_id 在 node_wrappers_ 中已加载的组件节点的 node_id
   * \param node_id  node_id of loaded component node in node_wrappers_
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual void add_node_to_executor(uint64_t node_id);

  /// 从执行器模型中移除组件节点，它在 on_unload_node() 中被调用
  /// Remove component node from executor model, it's invoked in on_unload_node()
  /**
   * \param node_id 在 node_wrappers_ 中已加载的组件节点的 node_id
   * \param node_id  node_id of loaded component node in node_wrappers_
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual void remove_node_from_executor(uint64_t node_id);

  /// 加载组件中新节点的服务回调
  /// Service callback to load a new node in the component
  /**
   * 此函数允许添加参数、重新映射规则、特定节点、命名空间和/或附加参数。
   * This function allows to add parameters, remap rules, a specific node, name a namespace
   * and/or additional arguments.
   *
   * \param request_header 未使用
   * \param request_header unused
   * \param request 包含要加载节点的信息
   * \param request information with the node to load
   * \param response 响应
   * \param response
   * \throws std::overflow_error 如果 node_id 发生溢出。以 1 kHz
   * 的速度（非常乐观的速率）发生的可能性很小。需要 585 年。 \throws std::overflow_error if node_id
   * suffers an overflow. Very unlikely to happen at 1 kHz (very optimistic rate). it would take 585
   * years. \throws ComponentManagerException 在组件构造函数抛出异常的情况下，重新抛出到以下 catch
   * 块中。 \throws ComponentManagerException In the case that the component constructor throws an
   *   exception, rethrow into the following catch block.
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual void on_load_node(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<LoadNode::Request> request,
      std::shared_ptr<LoadNode::Response> response);

  /**
   * \deprecated 使用 on_load_node() 替代 (Use on_load_node() instead)
   */
  [[deprecated(
      "使用 on_load_node() 替代 (Use on_load_node() "
      "instead)")]] RCLCPP_COMPONENTS_PUBLIC virtual void
  OnLoadNode(
      const std::shared_ptr<rmw_request_id_t> request_header,  ///< 请求头 (Request header)
      const std::shared_ptr<LoadNode::Request> request,  ///< 载入节点的请求 (Load node request)
      std::shared_ptr<LoadNode::Response> response)  ///< 载入节点的响应 (Load node response)
  {
    // 调用 on_load_node 方法 (Call the on_load_node method)
    on_load_node(request_header, request, response);
  }

  /// 服务回调以在组件中卸载一个节点 (Service callback to unload a node in the component)
  /**
   * \param request_header 未使用 (unused)
   * \param request 从组件中删除的唯一标识符 (unique identifier to remove from the component)
   * \param response 如果节点卸载成功，则在 success 字段上为 true，否则为 false，并且 error_message
   * 字段包含错误 (true on the success field if the node unload was successful, otherwise false and
   * the error_message field contains the error)
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual void on_unload_node(
      const std::shared_ptr<rmw_request_id_t> request_header,  ///< 请求头 (Request header)
      const std::shared_ptr<UnloadNode::Request> request,  ///< 卸载节点的请求 (Unload node request)
      std::shared_ptr<UnloadNode::Response> response);  ///< 卸载节点的响应 (Unload node response)

  /**
   * \deprecated 使用 on_unload_node() 替代 (Use on_unload_node() instead)
   */
  [[deprecated(
      "使用 on_unload_node() 替代 (Use on_unload_node() "
      "instead)")]] RCLCPP_COMPONENTS_PUBLIC virtual void
  OnUnloadNode(
      const std::shared_ptr<rmw_request_id_t> request_header,  ///< 请求头 (Request header)
      const std::shared_ptr<UnloadNode::Request> request,  ///< 卸载节点的请求 (Unload node request)
      std::shared_ptr<UnloadNode::Response> response)  ///< 卸载节点的响应 (Unload node response)
  {
    // 调用 on_unload_node 方法 (Call the on_unload_node method)
    on_unload_node(request_header, request, response);
  }

  /// 服务回调，用于获取组件中的节点列表 (Service callback to get the list of nodes in the
  /// component)
  /**
   * 返回两个列表：一个包含唯一标识符，另一个包含节点的完整名称 (Return two lists: one with the
   * unique identifiers and other with full node names)
   *
   * \param request_header 未使用 (unused)
   * \param request 未使用 (unused)
   * \param response 包含唯一标识符和完整节点名称的列表 (list with the unique ids and full node
   * names)
   */
  RCLCPP_COMPONENTS_PUBLIC
  virtual void on_list_nodes(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<ListNodes::Request> request,
      std::shared_ptr<ListNodes::Response> response);

  /**
   * \deprecated 使用 on_list_nodes() 替代 (Use on_list_nodes() instead)
   */
  [[deprecated(
      "使用 on_list_nodes() 替代 (Use on_list_nodes() "
      "instead)")]] RCLCPP_COMPONENTS_PUBLIC virtual void
  OnListNodes(
      const std::shared_ptr<rmw_request_id_t>
          request_header,  // 请求头，未使用 (request header, unused)
      const std::shared_ptr<ListNodes::Request> request,  // 请求，未使用 (request, unused)
      std::shared_ptr<ListNodes::Response>
          response)  // 响应，包含唯一标识符和完整节点名称的列表 (response, list with the unique ids
                     // and full node names)
  {
    // 调用 on_list_nodes() 函数处理请求 (Call the on_list_nodes() function to handle the request)
    on_list_nodes(request_header, request, response);
  }

protected:
  // 定义一个弱指针，用于存储 rclcpp::Executor 对象 (Define a weak pointer to store the
  // rclcpp::Executor object)
  std::weak_ptr<rclcpp::Executor> executor_;
  // 定义一个唯一的 ID 变量，初始值为 1 (Define a unique ID variable, initialized to 1)
  uint64_t unique_id_{1};
  // 使用 std::map 存储一个字符串到类加载器（class_loader::ClassLoader）的映射 (Use std::map to
  // store a mapping from string to class loader (class_loader::ClassLoader))
  std::map<std::string, std::unique_ptr<class_loader::ClassLoader>> loaders_;
  // 使用 std::map 存储一个 uint64_t
  // 到节点实例包装器（rclcpp_components::NodeInstanceWrapper）的映射 (Use std::map to store a
  // mapping from uint64_t to node instance wrapper (rclcpp_components::NodeInstanceWrapper))
  std::map<uint64_t, rclcpp_components::NodeInstanceWrapper> node_wrappers_;
  // 定义一个共享指针，用于存储 LoadNode 服务对象 (Define a shared pointer to store the LoadNode
  // service object)
  rclcpp::Service<LoadNode>::SharedPtr loadNode_srv_;
  // 定义一个共享指针，用于存储 UnloadNode 服务对象 (Define a shared pointer to store the UnloadNode
  // service object)
  rclcpp::Service<UnloadNode>::SharedPtr unloadNode_srv_;
  // 定义一个共享指针，用于存储 ListNodes 服务对象 (Define a shared pointer to store the ListNodes
  // service object)
  rclcpp::Service<ListNodes>::SharedPtr listNodes_srv_;
};

}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__COMPONENT_MANAGER_HPP__

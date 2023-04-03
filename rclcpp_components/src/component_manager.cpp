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

#include "rclcpp_components/component_manager.hpp"

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_resource.hpp"
#include "class_loader/class_loader.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/split.hpp"

using namespace std::placeholders;

namespace rclcpp_components {

/**
 * @brief 构造函数，用于初始化 ComponentManager 类的实例 (Constructor for initializing an instance
 * of the ComponentManager class)
 *
 * @param[in] executor 弱指针，指向 rclcpp::Executor 类型的对象 (Weak pointer to an object of type
 * rclcpp::Executor)
 * @param[in] node_name 组件管理器节点的名称 (Name of the component manager node)
 * @param[in] node_options 与节点相关的选项 (Options related to the node)
 */
ComponentManager::ComponentManager(
    std::weak_ptr<rclcpp::Executor> executor,
    std::string node_name,
    const rclcpp::NodeOptions& node_options)
    : Node(std::move(node_name), node_options),
      executor_(executor)  // 初始化节点和执行器 (Initialize the node and executor)
{
  // 创建 load_node 服务，并绑定 on_load_node 回调函数 (Create the load_node service and bind the
  // on_load_node callback function)
  loadNode_srv_ = create_service<LoadNode>(
      "~/_container/load_node", std::bind(&ComponentManager::on_load_node, this, _1, _2, _3));

  // 创建 unload_node 服务，并绑定 on_unload_node 回调函数 (Create the unload_node service and bind
  // the on_unload_node callback function)
  unloadNode_srv_ = create_service<UnloadNode>(
      "~/_container/unload_node", std::bind(&ComponentManager::on_unload_node, this, _1, _2, _3));

  // 创建 list_nodes 服务，并绑定 on_list_nodes 回调函数 (Create the list_nodes service and bind the
  // on_list_nodes callback function)
  listNodes_srv_ = create_service<ListNodes>(
      "~/_container/list_nodes", std::bind(&ComponentManager::on_list_nodes, this, _1, _2, _3));

  {
    rcl_interfaces::msg::ParameterDescriptor
        desc{};                             // 创建参数描述符 (Create a parameter descriptor)
    desc.description = "Number of thread";  // 设置参数描述 (Set the parameter description)

    rcl_interfaces::msg::IntegerRange range{};  // 创建整数范围 (Create an integer range)
    range.from_value = 1;  // 设置范围的起始值 (Set the starting value of the range)
    range.to_value = std::thread::hardware_concurrency();  // 设置范围的结束值 (Set the ending value
                                                           // of the range)
    desc.integer_range.push_back(
        range);  // 将范围添加到参数描述符中 (Add the range to the parameter descriptor)

    desc.read_only = true;  // 设置参数为只读 (Set the parameter as read-only)

    // 声明参数并设置默认值和描述符 (Declare the parameter with the default value and descriptor)
    this->declare_parameter(
        "thread_num", static_cast<int64_t>(std::thread::hardware_concurrency()), desc);
  }
}

/**
 * @brief 析构函数，用于在销毁 ComponentManager 对象时清理资源 (Destructor, used for cleaning up
 * resources when destroying the ComponentManager object)
 */
ComponentManager::~ComponentManager() {
  // 判断 node_wrappers_ 的大小是否大于0 (Check if the size of node_wrappers_ is greater than 0)
  if (node_wrappers_.size()) {
    // 输出调试信息 (Output debug information)
    RCLCPP_DEBUG(get_logger(), "Removing components from executor");

    // 尝试获取一个弱引用的 executor 实例 (Try to get a weak reference to the executor instance)
    if (auto exec = executor_.lock()) {
      // 遍历 node_wrappers_ 中的所有元素 (Iterate through all elements in node_wrappers_)
      for (auto& wrapper : node_wrappers_) {
        // 从 executor 中移除节点 (Remove the node from the executor)
        exec->remove_node(wrapper.second.get_node_base_interface());
      }
    }
  }
}

/**
 * @brief 获取组件资源列表 (Get the list of component resources)
 *
 * @param package_name 要查找的包名 (The package name to search for)
 * @param resource_index 资源索引名称 (The resource index name)
 * @return std::vector<ComponentResource> 返回组件资源列表 (Return the list of component resources)
 */
std::vector<ComponentManager::ComponentResource> ComponentManager::get_component_resources(
    const std::string& package_name, const std::string& resource_index) const {
  std::string content;
  std::string base_path;

  // 尝试从 ament 索引中获取资源 (Try to get the resource from the ament index)
  if (!ament_index_cpp::get_resource(resource_index, package_name, content, &base_path)) {
    // 如果获取失败，抛出异常 (If the acquisition fails, throw an exception)
    throw ComponentManagerException("Could not find requested resource in ament index");
  }

  std::vector<ComponentResource> resources;
  // 将内容按换行符分割成字符串列表 (Split the content into a list of strings by newline character)
  std::vector<std::string> lines = rcpputils::split(content, '\n', true);

  // 遍历每一行 (Iterate through each line)
  for (const auto& line : lines) {
    // 将行内容按 ';' 分割成字符串列表 (Split the line content into a list of strings by ';')
    std::vector<std::string> parts = rcpputils::split(line, ';');

    // 如果分割后的部分数量不等于2，抛出异常 (If the number of parts after splitting is not equal to
    // 2, throw an exception)
    if (parts.size() != 2) {
      throw ComponentManagerException("Invalid resource entry");
    }

    // 获取库路径 (Get the library path)
    std::string library_path = parts[1];

    // 判断库路径是否为绝对路径 (Check if the library path is an absolute path)
    if (!rcpputils::fs::path(library_path).is_absolute()) {
      // 如果不是绝对路径，则拼接基础路径和库路径 (If it is not an absolute path, concatenate the
      // base path and the library path)
      library_path = base_path + "/" + library_path;
    }
    // 将组件资源添加到资源列表中 (Add the component resource to the resource list)
    resources.push_back({parts[0], library_path});
  }
  // 返回组件资源列表 (Return the list of component resources)
  return resources;
}

/**
 * @brief 创建组件工厂（Create a component factory）
 *
 * @param resource 包含类名和库路径的组件资源（Component resource containing class name and library
 * path）
 * @return std::shared_ptr<rclcpp_components::NodeFactory> 返回创建的节点工厂实例（Return created
 * node factory instance）
 */
std::shared_ptr<rclcpp_components::NodeFactory> ComponentManager::create_component_factory(
    const ComponentResource& resource) {
  // 获取库路径（Get the library path）
  std::string library_path = resource.second;
  // 获取类名（Get the class name）
  std::string class_name = resource.first;
  // 构造完全限定类名（Construct fully qualified class name）
  std::string fq_class_name = "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";

  // 定义类加载器指针（Define class loader pointer）
  class_loader::ClassLoader* loader;
  // 检查是否已经加载了库（Check if the library has already been loaded）
  if (loaders_.find(library_path) == loaders_.end()) {
    // 输出加载库的信息（Output information about loading the library）
    RCLCPP_INFO(get_logger(), "Load Library: %s", library_path.c_str());
    try {
      // 创建并存储类加载器实例（Create and store class loader instance）
      loaders_[library_path] = std::make_unique<class_loader::ClassLoader>(library_path);
    } catch (const std::exception& ex) {
      // 抛出加载库失败的异常（Throw exception for failed library loading）
      throw ComponentManagerException("Failed to load library: " + std::string(ex.what()));
    } catch (...) {
      // 抛出加载库失败的异常（Throw exception for failed library loading）
      throw ComponentManagerException("Failed to load library");
    }
  }
  // 获取类加载器实例（Get the class loader instance）
  loader = loaders_[library_path].get();

  // 获取可用的类列表（Get the list of available classes）
  auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
  // 遍历类列表（Iterate through the class list）
  for (const auto& clazz : classes) {
    // 输出找到的类信息（Output information about the found class）
    RCLCPP_INFO(get_logger(), "Found class: %s", clazz.c_str());
    // 检查类名是否匹配（Check if the class name matches）
    if (clazz == class_name || clazz == fq_class_name) {
      // 输出实例化类的信息（Output information about instantiating the class）
      RCLCPP_INFO(get_logger(), "Instantiate class: %s", clazz.c_str());
      // 创建并返回节点工厂实例（Create and return node factory instance）
      return loader->createInstance<rclcpp_components::NodeFactory>(clazz);
    }
  }
  // 如果没有找到匹配的类，返回空指针（If no matching class is found, return an empty pointer）
  return {};
}

/**
 * @brief 创建节点选项
 * @param request 一个指向 LoadNode::Request 类型的共享指针，包含创建节点所需的参数
 * @return 返回一个 rclcpp::NodeOptions 对象，包含从请求中解析出的参数
 *
 * @brief Create node options
 * @param request A shared pointer to a LoadNode::Request, which contains the parameters needed for
 * creating the node
 * @return Returns an rclcpp::NodeOptions object containing the parameters parsed from the request
 */
rclcpp::NodeOptions ComponentManager::create_node_options(
    const std::shared_ptr<LoadNode::Request> request) {
  // 声明一个 rclcpp::Parameter 类型的向量，用于存储解析后的参数
  // Declare a vector of type rclcpp::Parameter to store the parsed parameters
  std::vector<rclcpp::Parameter> parameters;

  // 遍历请求中的参数
  // Iterate through the parameters in the request
  for (const auto& p : request->parameters) {
    // 将每个参数从参数消息类型转换为 rclcpp::Parameter 类型，并添加到参数向量中
    // Convert each parameter from parameter message type to rclcpp::Parameter type and add it to
    // the parameter vector
    parameters.push_back(rclcpp::Parameter::from_parameter_msg(p));
  }

  // 使用解析后的参数创建一个新的 rclcpp::NodeOptions 对象并返回
  // Create a new rclcpp::NodeOptions object with the parsed parameters and return it
  return rclcpp::NodeOptions().parameters(parameters);

  /**
   * @brief 根据请求参数构建 remap_rules 和 NodeOptions。
   * @details 该代码段用于处理 ROS2 rclcpp 项目中节点名称、命名空间和其他参数的重新映射。
   * @param request 包含需要重新映射的规则、节点名称、节点命名空间和额外参数的请求对象。
   * @return options 返回一个根据请求参数配置的 rclcpp::NodeOptions 对象。
   *
   * @brief Build remap_rules and NodeOptions based on request parameters.
   * @details This code snippet is for handling remapping of node name, namespace, and other
   * arguments in ROS2 rclcpp project.
   * @param request The request object containing the remap rules, node name, node namespace, and
   * extra arguments to be processed.
   * @return options Returns an rclcpp::NodeOptions object configured according to the request
   * parameters.
   */
  std::vector<std::string> remap_rules;
  // 为 remap_rules 预留空间，以避免不必要的内存分配。
  // Reserve space for remap_rules to avoid unnecessary memory allocations.
  remap_rules.reserve(request->remap_rules.size() * 2 + 1);
  remap_rules.push_back("--ros-args");
  for (const std::string& rule : request->remap_rules) {
    remap_rules.push_back("-r");
    remap_rules.push_back(rule);
  }

  // 如果提供了节点名称，则添加到 remap_rules 中。
  // If a node name is provided, add it to remap_rules.
  if (!request->node_name.empty()) {
    remap_rules.push_back("-r");
    remap_rules.push_back("__node:=" + request->node_name);
  }

  // 如果提供了节点命名空间，则添加到 remap_rules 中。
  // If a node namespace is provided, add it to remap_rules.
  if (!request->node_namespace.empty()) {
    remap_rules.push_back("-r");
    remap_rules.push_back("__ns:=" + request->node_namespace);
  }

  // 使用 remap_rules 和其他参数创建 NodeOptions 对象。
  // Create a NodeOptions object using remap_rules and other parameters.
  auto options = rclcpp::NodeOptions()
                     .use_global_arguments(false)
                     .parameter_overrides(parameters)
                     .arguments(remap_rules);

  // 处理请求中的额外参数。
  // Process extra arguments in the request.
  for (const auto& a : request->extra_arguments) {
    const rclcpp::Parameter extra_argument = rclcpp::Parameter::from_parameter_msg(a);
    if (extra_argument.get_name() == "use_intra_process_comms") {
      if (extra_argument.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        throw ComponentManagerException(
            "Extra component argument 'use_intra_process_comms' must be a boolean");
      }
      options.use_intra_process_comms(extra_argument.get_value<bool>());
    } else if (extra_argument.get_name() == "forward_global_arguments") {
      if (extra_argument.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        throw ComponentManagerException(
            "Extra component argument 'forward_global_arguments' must be a boolean");
      }
      options.use_global_arguments(extra_argument.get_value<bool>());
      if (extra_argument.get_value<bool>()) {
        RCLCPP_WARN(
            get_logger(),
            "forward_global_arguments is true by default in nodes, but is not "
            "recommended in a component manager. If true, this will cause this node's behavior "
            "to be influenced by global arguments, not only those targeted at this node.");
      }
    }
  }

  return options;
}

/**
 * @brief 设置执行器 (Set the executor)
 *
 * @param executor 执行器的弱指针 (A weak pointer to the executor)
 */
void ComponentManager::set_executor(const std::weak_ptr<rclcpp::Executor> executor) {
  // 将输入参数的执行器赋值给成员变量 executor_ (Assign the input executor to the member variable
  // executor_)
  executor_ = executor;
}

/**
 * @brief 向执行器添加节点 (Add a node to the executor)
 *
 * @param node_id 要添加的节点的 ID (The ID of the node to add)
 */
void ComponentManager::add_node_to_executor(uint64_t node_id) {
  // 尝试从弱指针中获取执行器的共享指针 (Attempt to obtain the shared pointer of the executor from
  // the weak pointer)
  if (auto exec = executor_.lock()) {
    // 将节点添加到执行器中，并设置为自动启动 (Add the node to the executor and set it to
    // auto-start)
    exec->add_node(node_wrappers_[node_id].get_node_base_interface(), true);
  }
}

/**
 * @brief 从执行器中移除节点 (Remove a node from the executor)
 *
 * @param node_id 要移除的节点的 ID (The ID of the node to remove)
 */
void ComponentManager::remove_node_from_executor(uint64_t node_id) {
  // 尝试从弱指针中获取执行器的共享指针 (Attempt to obtain the shared pointer of the executor from
  // the weak pointer)
  if (auto exec = executor_.lock()) {
    // 从执行器中移除节点 (Remove the node from the executor)
    exec->remove_node(node_wrappers_[node_id].get_node_base_interface());
  }
}

/**
 * @brief 加载节点的函数，用于在组件管理器中加载组件节点 (Function for loading a node, used to load
 * component nodes in the Component Manager)
 *
 * @param request_header 请求头，用于标识特定请求 (Request header, used to identify specific
 * requests)
 * @param request 节点加载请求，包含需要加载的组件的信息 (Node load request, contains information
 * about the component to be loaded)
 * @param response 节点加载响应，用于返回加载结果及相关信息 (Node load response, used to return
 * loading results and related information)
 */
void ComponentManager::on_load_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<LoadNode::Request> request,
    std::shared_ptr<LoadNode::Response> response) {
  (void)request_header;  // 忽略请求头 (Ignore the request header)

  try {
    auto resources =
        get_component_resources(request->package_name);  // 获取组件资源 (Get component resources)

    for (const auto& resource : resources) {  // 遍历资源列表 (Iterate through the resource list)
      if (resource.first !=
          request->plugin_name) {  // 如果资源名与请求的插件名不匹配，则跳过当前资源 (If the
                                   // resource name does not match the requested plugin name, skip
                                   // the current resource)
        continue;
      }
      auto factory = create_component_factory(resource);  // 创建组件工厂 (Create component factory)

      if (factory == nullptr) {  // 如果工厂创建失败，则跳过当前资源 (If the factory creation fails,
                                 // skip the current resource)
        continue;
      }

      auto options = create_node_options(request);  // 创建节点选项 (Create node options)
      auto node_id = unique_id_++;  // 生成唯一节点ID (Generate unique node ID)

      if (0 == node_id) {
        // 此处的注释解释了为什么不需要处理溢出问题 (The comment here explains why overflow handling
        // is not needed)
        throw std::overflow_error("exhausted the unique ids for components in this process");
      }

      try {
        node_wrappers_[node_id] = factory->create_node_instance(
            options);  // 使用工厂创建节点实例并将其存储在 node_wrappers_ 中 (Create node instance
                       // using the factory and store it in node_wrappers_)
      } catch (const std::exception& ex) {
        // 如果组件构造函数抛出异常，则重新抛出到下面的 catch 块中 (If the component constructor
        // throws an exception, rethrow it into the following catch block)
        throw ComponentManagerException(
            "Component constructor threw an exception: " + std::string(ex.what()));
      } catch (...) {
        // 如果组件构造函数抛出异常，则重新抛出到下面的 catch 块中 (If the component constructor
        // throws an exception, rethrow it into the following catch block)
        throw ComponentManagerException("Component constructor threw an exception");
      }

      add_node_to_executor(node_id);  // 将节点添加到执行器中 (Add the node to the executor)

      auto node = node_wrappers_[node_id]
                      .get_node_base_interface();  // 获取节点基础接口 (Get the node base interface)
      response->full_node_name =
          node->get_fully_qualified_name();  // 设置完全限定节点名 (Set fully qualified node name)
      response->unique_id = node_id;         // 设置唯一节点ID (Set unique node ID)
      response->success = true;
      return;
    }
    RCLCPP_ERROR(
        get_logger(),
        "Failed to find class with the requested plugin name '%s' in "
        "the loaded library",
        request->plugin_name.c_str());
    response->error_message = "Failed to find class with the requested plugin name.";
    response->success = false;
  } catch (const ComponentManagerException& ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    response->error_message = ex.what();
    response->success = false;
  }
}

/**
 * @brief 卸载节点 (Unload a node)
 *
 * @param[in] request_header 请求头指针（不使用）(Pointer to the request header (not used))
 * @param[in] request 指向 UnloadNode::Request 的共享指针 (Shared pointer to UnloadNode::Request)
 * @param[out] response 指向 UnloadNode::Response 的共享指针 (Shared pointer to
 * UnloadNode::Response)
 */
void ComponentManager::on_unload_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<UnloadNode::Request> request,
    std::shared_ptr<UnloadNode::Response> response) {
  (void)request_header;  // 忽略请求头 (Ignore the request_header)

  // 查找具有给定唯一 ID 的节点包装器 (Find the node wrapper with the given unique ID)
  auto wrapper = node_wrappers_.find(request->unique_id);

  // 如果未找到节点包装器 (If the node wrapper is not found)
  if (wrapper == node_wrappers_.end()) {
    response->success = false;
    std::stringstream ss;
    // 设置错误消息 (Set the error message)
    ss << "No node found with unique_id: " << request->unique_id;
    response->error_message = ss.str();
    // 输出警告日志 (Output warning log)
    RCLCPP_WARN(get_logger(), "%s", ss.str().c_str());
  } else {
    // 从执行器中删除节点 (Remove the node from the executor)
    remove_node_from_executor(request->unique_id);
    // 从节点包装器映射中删除节点 (Delete the node from the node_wrappers_ map)
    node_wrappers_.erase(wrapper);
    response->success = true;
  }
}

/**
 * @brief 列出节点 (List nodes)
 *
 * @param[in] request_header 请求头指针（不使用）(Pointer to the request header (not used))
 * @param[in] request 指向 ListNodes::Request 的共享指针 (Shared pointer to ListNodes::Request)
 * @param[out] response 指向 ListNodes::Response 的共享指针 (Shared pointer to ListNodes::Response)
 */
void ComponentManager::on_list_nodes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListNodes::Request> request,
    std::shared_ptr<ListNodes::Response> response) {
  (void)request_header;  // 忽略请求头 (Ignore the request_header)
  (void)request;         // 忽略请求 (Ignore the request)

  // 遍历节点包装器映射 (Iterate through the node_wrappers_ map)
  for (auto& wrapper : node_wrappers_) {
    // 将唯一 ID 添加到响应中 (Add the unique ID to the response)
    response->unique_ids.push_back(wrapper.first);
    // 将完整节点名称添加到响应中 (Add the full node name to the response)
    response->full_node_names.push_back(
        wrapper.second.get_node_base_interface()->get_fully_qualified_name());
  }
}

}  // namespace rclcpp_components

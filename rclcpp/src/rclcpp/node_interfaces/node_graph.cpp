// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/node_interfaces/node_graph.hpp"

#include <algorithm>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rcl/graph.h"
#include "rcl/remap.h"
#include "rclcpp/event.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rcpputils/scope_exit.hpp"

using rclcpp::exceptions::throw_from_rcl_error;
using rclcpp::graph_listener::GraphListener;
using rclcpp::node_interfaces::NodeGraph;

/**
 * @brief 构造函数，用于创建 NodeGraph 对象 (Constructor for creating a NodeGraph object)
 *
 * @param node_base 是一个指向节点基础接口的指针 (A pointer to the node base interface)
 */
NodeGraph::NodeGraph(rclcpp::node_interfaces::NodeBaseInterface* node_base)
    : node_base_(node_base),
      graph_listener_(
          node_base->get_context()->get_sub_context<GraphListener>(node_base->get_context())),
      should_add_to_graph_listener_(true),
      graph_users_count_(0) {
  // 初始化 node_base_, graph_listener_, should_add_to_graph_listener_ 和 graph_users_count_
  // Initialize node_base_, graph_listener_, should_add_to_graph_listener_ and graph_users_count_
}

/**
 * @brief 析构函数，用于销毁 NodeGraph 对象 (Destructor for destroying a NodeGraph object)
 */
NodeGraph::~NodeGraph() {
  // 从图形监听器中移除自身 (Remove self from graph listener)
  // 使用 false 进行交换，以防止其他人在检查后尝试将此节点添加到图形监听器中 (Exchange with false to
  // prevent others from trying to add this node to the graph listener after checking that it was
  // not here)
  if (!should_add_to_graph_listener_.exchange(false)) {
    // 如果已经是 false，则现在需要删除它 (If it was already false, then it needs to now be removed)
    graph_listener_->remove_node(this);
  }
}

/**
 * @brief 获取主题名称和类型 (Get topic names and types)
 *
 * @param no_demangle 如果为 true，则不对主题名称进行解扰 (If true, do not demangle topic names)
 * @return 返回一个包含主题名称和类型的映射 (Returns a map containing topic names and types)
 */
std::map<std::string, std::vector<std::string>> NodeGraph::get_topic_names_and_types(
    bool no_demangle) const {
  // 初始化 topic_names_and_types 结构体 (Initialize the topic_names_and_types structure)
  rcl_names_and_types_t topic_names_and_types = rcl_get_zero_initialized_names_and_types();

  // 获取默认分配器 (Get the default allocator)
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // 获取节点主题名称和类型 (Get the node's topic names and types)
  auto ret = rcl_get_topic_names_and_types(
      node_base_->get_rcl_node_handle(), &allocator, no_demangle, &topic_names_and_types);
  if (ret != RCL_RET_OK) {
    // 处理错误情况 (Handle error cases)
    auto error_msg =
        std::string("failed to get topic names and types: ") + rcl_get_error_string().str;
    rcl_reset_error();
    if (rcl_names_and_types_fini(&topic_names_and_types) != RCL_RET_OK) {
      error_msg += std::string(", failed also to cleanup topic names and types, leaking memory: ") +
                   rcl_get_error_string().str;
      rcl_reset_error();
    }
    throw std::runtime_error(error_msg);
  }

  // 将获取到的主题名称和类型存储到 topics_and_types 映射中 (Store the retrieved topic names and
  // types in the topics_and_types map)
  std::map<std::string, std::vector<std::string>> topics_and_types;
  for (size_t i = 0; i < topic_names_and_types.names.size; ++i) {
    std::string topic_name = topic_names_and_types.names.data[i];
    for (size_t j = 0; j < topic_names_and_types.types[i].size; ++j) {
      topics_and_types[topic_name].emplace_back(topic_names_and_types.types[i].data[j]);
    }
  }

  // 清理 topic_names_and_types 结构体 (Clean up the topic_names_and_types structure)
  ret = rcl_names_and_types_fini(&topic_names_and_types);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error(
        std::string("could not destroy topic names and types: ") + rcl_get_error_string().str);
  }

  // 返回包含主题名称和类型的映射 (Return the map containing topic names and types)
  return topics_and_types;
}

/**
 * @brief 获取服务名称和类型列表 (Get the list of service names and types)
 *
 * @return std::map<std::string, std::vector<std::string>> 服务名称和类型的映射 (Mapping of service
 * names and types)
 */
std::map<std::string, std::vector<std::string>> NodeGraph::get_service_names_and_types() const {
  // 初始化一个空的服务名称和类型结构体 (Initialize an empty service names and types structure)
  rcl_names_and_types_t service_names_and_types = rcl_get_zero_initialized_names_and_types();

  // 获取默认分配器 (Get the default allocator)
  rcl_allocator_t allocator = rcl_get_default_allocator();

  // 获取节点中的服务名称和类型 (Get the service names and types in the node)
  auto ret = rcl_get_service_names_and_types(
      node_base_->get_rcl_node_handle(), &allocator, &service_names_and_types);

  // 检查是否成功获取服务名称和类型 (Check if the service names and types are successfully obtained)
  if (ret != RCL_RET_OK) {
    auto error_msg =
        std::string("failed to get service names and types: ") + rcl_get_error_string().str;
    rcl_reset_error();

    // 清理失败时，处理异常 (Handle the exception when cleanup fails)
    if (rcl_names_and_types_fini(&service_names_and_types) != RCL_RET_OK) {
      error_msg +=
          std::string(", failed also to cleanup service names and types, leaking memory: ") +
          rcl_get_error_string().str;
      rcl_reset_error();
    }

    // 抛出运行时异常 (Throw a runtime exception)
    throw std::runtime_error(error_msg);
  }

  // 创建一个映射，用于存储服务名称和类型 (Create a mapping to store service names and types)
  std::map<std::string, std::vector<std::string>> services_and_types;

  // 遍历获取到的服务名称和类型 (Iterate through the obtained service names and types)
  for (size_t i = 0; i < service_names_and_types.names.size; ++i) {
    std::string service_name = service_names_and_types.names.data[i];

    // 将获取到的类型添加到映射中 (Add the obtained types to the mapping)
    for (size_t j = 0; j < service_names_and_types.types[i].size; ++j) {
      services_and_types[service_name].emplace_back(service_names_and_types.types[i].data[j]);
    }
  }

  // 清理服务名称和类型结构体 (Clean up the service names and types structure)
  ret = rcl_names_and_types_fini(&service_names_and_types);

  // 检查是否成功销毁服务名称和类型 (Check if the service names and types are successfully
  // destroyed)
  if (ret != RCL_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
        std::string("could not destroy service names and types: ") + rcl_get_error_string().str);
    // *INDENT-ON*
  }

  // 返回服务名称和类型的映射 (Return the mapping of service names and types)
  return services_and_types;
}

/**
 * @brief 获取指定节点上的服务名称和类型（Get the service names and types on the specified node）
 *
 * @param node_name 节点名称（Node name）
 * @param namespace_ 命名空间（Namespace）
 * @return std::map<std::string, std::vector<std::string>> 服务名称和类型的映射（Mapping of service
 * names and types）
 */
std::map<std::string, std::vector<std::string>> NodeGraph::get_service_names_and_types_by_node(
    const std::string& node_name, const std::string& namespace_) const {
  // 初始化服务名称和类型结构体（Initialize the service names and types structure）
  rcl_names_and_types_t service_names_and_types = rcl_get_zero_initialized_names_and_types();

  // 获取默认分配器（Get the default allocator）
  rcl_allocator_t allocator = rcl_get_default_allocator();

  // 根据节点获取服务名称和类型（Get service names and types by node）
  rcl_ret_t ret = rcl_get_service_names_and_types_by_node(
      node_base_->get_rcl_node_handle(), &allocator, node_name.c_str(), namespace_.c_str(),
      &service_names_and_types);

  // 检查返回值是否为 RCL_RET_OK（Check if the return value is RCL_RET_OK）
  if (ret != RCL_RET_OK) {
    auto error_msg =
        std::string("failed to get service names and types by node: ") + rcl_get_error_string().str;
    rcl_reset_error();
    if (rcl_names_and_types_fini(&service_names_and_types) != RCL_RET_OK) {
      error_msg +=
          std::string(", failed also to cleanup service names and types, leaking memory: ") +
          rcl_get_error_string().str;
      rcl_reset_error();
    }
    // 抛出运行时错误（Throw a runtime error）
    throw std::runtime_error(error_msg);
  }

  // 创建一个服务名称和类型的映射（Create a mapping of service names and types）
  std::map<std::string, std::vector<std::string>> services_and_types;

  // 遍历获取到的服务名称和类型结构体（Iterate through the obtained service names and types
  // structure）
  for (size_t i = 0; i < service_names_and_types.names.size; ++i) {
    std::string service_name = service_names_and_types.names.data[i];

    // 将服务类型添加到映射中（Add the service types to the mapping）
    for (size_t j = 0; j < service_names_and_types.types[i].size; ++j) {
      services_and_types[service_name].emplace_back(service_names_and_types.types[i].data[j]);
    }
  }

  // 清理服务名称和类型结构体（Clean up the service names and types structure）
  ret = rcl_names_and_types_fini(&service_names_and_types);

  // 检查返回值是否为 RCL_RET_OK（Check if the return value is RCL_RET_OK）
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "could not destroy service names and types");
  }

  // 返回服务名称和类型的映射（Return the mapping of service names and types）
  return services_and_types;
}

/**
 * @brief 获取指定节点上的客户端名称和类型 (Get client names and types by node)
 *
 * @param node_name 节点名称 (Node name)
 * @param namespace_ 命名空间 (Namespace)
 * @return std::map<std::string, std::vector<std::string>> 客户端名称和类型映射 (Client names and
 * types mapping)
 */
std::map<std::string, std::vector<std::string>> NodeGraph::get_client_names_and_types_by_node(
    const std::string& node_name, const std::string& namespace_) const {
  // 初始化服务名称和类型结构体 (Initialize the service names and types structure)
  rcl_names_and_types_t service_names_and_types = rcl_get_zero_initialized_names_and_types();

  // 创建一个作用域退出对象，以确保在函数返回时正确释放资源 (Create a scope exit object to ensure
  // proper resource release when the function returns)
  auto service_names_and_types_finalizer = rcpputils::make_scope_exit([&service_names_and_types]() {
    if (rcl_names_and_types_fini(&service_names_and_types) != RCL_RET_OK) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "could not destroy service names and types");
    }
  });

  // 获取默认分配器 (Get the default allocator)
  rcl_allocator_t allocator = rcl_get_default_allocator();

  // 调用 rcl 函数获取节点上的客户端名称和类型 (Call the rcl function to get the client names and
  // types on the node)
  rcl_ret_t ret = rcl_get_client_names_and_types_by_node(
      node_base_->get_rcl_node_handle(), &allocator, node_name.c_str(), namespace_.c_str(),
      &service_names_and_types);

  // 检查返回值，如果不是 RCL_RET_OK，则抛出异常 (Check the return value, if it's not RCL_RET_OK,
  // throw an exception)
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "failed to get service names and types by node");
  }

  // 定义一个映射存储服务名称和类型 (Define a mapping to store service names and types)
  std::map<std::string, std::vector<std::string>> services_and_types;

  // 遍历服务名称和类型结构体，将其添加到映射中 (Iterate through the service names and types
  // structure and add them to the mapping)
  for (size_t i = 0; i < service_names_and_types.names.size; ++i) {
    std::string service_name = service_names_and_types.names.data[i];
    for (size_t j = 0; j < service_names_and_types.types[i].size; ++j) {
      services_and_types[service_name].emplace_back(service_names_and_types.types[i].data[j]);
    }
  }

  // 返回客户端名称和类型映射 (Return the client names and types mapping)
  return services_and_types;
}

/**
 * @brief 获取指定节点的发布者主题名称和类型
 * @brief Get publisher topic names and types for a specified node
 *
 * @param node_name 节点名称
 * @param node_name The node name
 * @param namespace_ 命名空间
 * @param namespace_ The namespace
 * @param no_demangle 是否取消改编（用于解决跨语言问题）
 * @param no_demangle Whether to demangle or not (for cross-language compatibility)
 * @return 返回一个包含主题名称和类型的映射表
 * @return A map containing topic names and types
 */
std::map<std::string, std::vector<std::string>> NodeGraph::get_publisher_names_and_types_by_node(
    const std::string& node_name, const std::string& namespace_, bool no_demangle) const {
  // 初始化 topic 名称和类型的结构体
  // Initialize the structure for topic names and types
  rcl_names_and_types_t topic_names_and_types = rcl_get_zero_initialized_names_and_types();

  // 创建一个作用域退出时自动销毁 topic_names_and_types 的对象
  // Create an object that automatically destroys topic_names_and_types when going out of scope
  auto topic_names_and_types_finalizer = rcpputils::make_scope_exit([&topic_names_and_types]() {
    if (rcl_names_and_types_fini(&topic_names_and_types) != RCL_RET_OK) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "could not destroy topic names and types");
    }
  });

  // 获取默认分配器
  // Get the default allocator
  rcl_allocator_t allocator = rcl_get_default_allocator();

  // 调用 rcl 函数获取指定节点的发布者主题名称和类型
  // Call the rcl function to get publisher topic names and types for the specified node
  rcl_ret_t ret = rcl_get_publisher_names_and_types_by_node(
      node_base_->get_rcl_node_handle(), &allocator, no_demangle, node_name.c_str(),
      namespace_.c_str(), &topic_names_and_types);

  // 如果调用失败，抛出异常
  // If the call fails, throw an exception
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "failed to get topic names and types by node");
  }

  // 创建一个映射表用于存储主题名称和类型
  // Create a map to store topic names and types
  std::map<std::string, std::vector<std::string>> topics_and_types;

  // 遍历获取到的主题名称和类型，并将它们添加到映射表中
  // Iterate through the obtained topic names and types, and add them to the map
  for (size_t i = 0; i < topic_names_and_types.names.size; ++i) {
    std::string topic_name = topic_names_and_types.names.data[i];
    for (size_t j = 0; j < topic_names_and_types.types[i].size; ++j) {
      topics_and_types[topic_name].emplace_back(topic_names_and_types.types[i].data[j]);
    }
  }

  // 返回包含主题名称和类型的映射表
  // Return the map containing topic names and types
  return topics_and_types;
}

/**
 * @brief 获取节点订阅者的名称和类型 (Get the names and types of subscribers for a node)
 *
 * @param node_name 要查询的节点名称 (The name of the node to query)
 * @param namespace_ 节点所在的命名空间 (The namespace of the node)
 * @param no_demangle 是否对话题名称进行反混淆处理 (Whether to demangle topic names or not)
 * @return std::map<std::string, std::vector<std::string>> 包含订阅者名称和类型的映射表 (A map
 * containing the subscriber names and types)
 */
std::map<std::string, std::vector<std::string>> NodeGraph::get_subscriber_names_and_types_by_node(
    const std::string& node_name, const std::string& namespace_, bool no_demangle) const {
  // 初始化一个空的 rcl_names_and_types_t 结构体 (Initialize an empty rcl_names_and_types_t
  // structure)
  rcl_names_and_types_t topic_names_and_types = rcl_get_zero_initialized_names_and_types();

  // 创建一个作用域退出时自动执行的清理函数 (Create a cleanup function that runs automatically when
  // leaving the scope)
  auto topic_names_and_types_finalizer = rcpputils::make_scope_exit([&topic_names_and_types]() {
    if (rcl_names_and_types_fini(&topic_names_and_types) != RCL_RET_OK) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "could not destroy topic names and types");
    }
  });

  // 获取默认分配器 (Get the default allocator)
  rcl_allocator_t allocator = rcl_get_default_allocator();

  // 调用 rcl 函数获取节点订阅者的名称和类型 (Call the rcl function to get the names and types of
  // subscribers for the node)
  rcl_ret_t ret = rcl_get_subscriber_names_and_types_by_node(
      node_base_->get_rcl_node_handle(), &allocator, no_demangle, node_name.c_str(),
      namespace_.c_str(), &topic_names_and_types);

  // 检查返回值，如果不是 RCL_RET_OK，则抛出异常 (Check the return value, if it is not RCL_RET_OK,
  // throw an exception)
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "failed to get topic names and types by node");
  }

  // 创建一个用于存储订阅者名称和类型的映射表 (Create a map to store subscriber names and types)
  std::map<std::string, std::vector<std::string>> topics_and_types;

  // 遍历 topic_names_and_types 结构体中的名称和类型数据 (Iterate through the names and types data
  // in the topic_names_and_types structure)
  for (size_t i = 0; i < topic_names_and_types.names.size; ++i) {
    // 获取话题名称 (Get the topic name)
    std::string topic_name = topic_names_and_types.names.data[i];

    // 遍历该话题对应的类型列表 (Iterate through the list of types for this topic)
    for (size_t j = 0; j < topic_names_and_types.types[i].size; ++j) {
      // 将话题名称和类型添加到映射表中 (Add the topic name and type to the map)
      topics_and_types[topic_name].emplace_back(topic_names_and_types.types[i].data[j]);
    }
  }

  // 返回包含订阅者名称和类型的映射表 (Return the map containing subscriber names and types)
  return topics_and_types;
}

/**
 * @brief 获取节点名称列表 (Get the list of node names)
 *
 * @return std::vector<std::string> 节点名称列表 (List of node names)
 */
std::vector<std::string> NodeGraph::get_node_names() const {
  // 创建一个字符串向量用于存储节点名称 (Create a vector of strings to store the node names)
  std::vector<std::string> nodes;

  // 获取节点名称和命名空间的对应关系 (Get the mapping of node names and namespaces)
  auto names_and_namespaces = get_node_names_and_namespaces();

  // 对节点名称和命名空间进行转换，将它们合并为完全限定的节点名称 (Transform the node names and
  // namespaces, combining them into fully-qualified node names)
  std::transform(
      names_and_namespaces.begin(), names_and_namespaces.end(), std::back_inserter(nodes),
      [](std::pair<std::string, std::string> nns) {
        // 创建一个返回字符串变量 (Create a return string variable)
        std::string return_string;

        // 判断命名空间是否以斜杠结尾 (Check if the namespace ends with a slash)
        if (nns.second.back() == '/') {
          // 将命名空间和节点名称连接起来 (Concatenate the namespace and node name)
          return_string = nns.second + nns.first;
        } else {
          // 在命名空间和节点名称之间添加斜杠并连接 (Add a slash between the namespace and node name
          // and concatenate)
          return_string = nns.second + '/' + nns.first;
        }

        // 快速检查确保我们以斜杠开头，因为完全限定的字符串需要以斜杠开头 (Quick check to make sure
        // that we start with a slash, since fully-qualified strings need to start with a slash)
        if (return_string.front() != '/') {
          // 在返回字符串前添加斜杠 (Add a slash in front of the return string)
          return_string = "/" + return_string;
        }

        // 返回完全限定的节点名称 (Return the fully-qualified node name)
        return return_string;
      });

  // 返回节点名称列表 (Return the list of node names)
  return nodes;
}

/**
 * @brief 获取包含 enclave 的节点名称列表 (Get a list of node names with enclaves)
 *
 * @return std::vector<std::tuple<std::string, std::string, std::string>> 包含节点名称、命名空间和
 * enclave 的元组列表 (A list of tuples containing node name, namespace, and enclave)
 */
std::vector<std::tuple<std::string, std::string, std::string>>
NodeGraph::get_node_names_with_enclaves() const {
  // 初始化节点名称、命名空间和 enclave 的字符串数组 (Initialize string arrays for node names,
  // namespaces, and enclaves)
  rcutils_string_array_t node_names_c = rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t node_namespaces_c = rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t node_enclaves_c = rcutils_get_zero_initialized_string_array();

  // 获取默认分配器 (Get the default allocator)
  auto allocator = rcl_get_default_allocator();
  // 使用 rcl 库函数获取包含 enclave 的节点名称列表 (Use rcl library function to get the list of
  // node names with enclaves)
  auto ret = rcl_get_node_names_with_enclaves(
      node_base_->get_rcl_node_handle(), allocator, &node_names_c, &node_namespaces_c,
      &node_enclaves_c);
  // 检查返回值是否为 RCL_RET_OK，如果不是，则处理错误 (Check if the return value is RCL_RET_OK, if
  // not, handle the error)
  if (ret != RCL_RET_OK) {
    auto error_msg =
        std::string("failed to get node names with enclaves: ") + rcl_get_error_string().str;
    rcl_reset_error();
    if (rcutils_string_array_fini(&node_names_c) != RCUTILS_RET_OK) {
      error_msg += std::string(", failed also to cleanup node names, leaking memory: ") +
                   rcl_get_error_string().str;
      rcl_reset_error();
    }
    if (rcutils_string_array_fini(&node_namespaces_c) != RCUTILS_RET_OK) {
      error_msg += std::string(", failed also to cleanup node namespaces, leaking memory: ") +
                   rcl_get_error_string().str;
      rcl_reset_error();
    }
    if (rcutils_string_array_fini(&node_enclaves_c) != RCUTILS_RET_OK) {
      error_msg += std::string(", failed also to cleanup node enclaves, leaking memory: ") +
                   rcl_get_error_string().str;
      rcl_reset_error();
    }
    throw std::runtime_error(error_msg);
  }

  // 创建一个包含节点名称、命名空间和 enclave 的元组列表 (Create a list of tuples containing node
  // name, namespace, and enclave)
  std::vector<std::tuple<std::string, std::string, std::string>> node_tuples;
  for (size_t i = 0; i < node_names_c.size; ++i) {
    if (node_names_c.data[i] && node_namespaces_c.data[i] && node_enclaves_c.data[i]) {
      node_tuples.emplace_back(std::make_tuple(
          node_names_c.data[i], node_namespaces_c.data[i], node_enclaves_c.data[i]));
    }
  }

  // 初始化错误字符串 (Initialize error string)
  std::string error("failed to finalize array");
  // 清理节点名称字符串数组并检查返回值 (Clean up the node names string array and check the return
  // value)
  rcl_ret_t ret_names = rcutils_string_array_fini(&node_names_c);
  if (ret_names != RCUTILS_RET_OK) {
    error += std::string(", could not destroy node names, leaking memory: ") +
             rcl_get_error_string().str;
    rcl_reset_error();
  }
  // 清理节点命名空间字符串数组并检查返回值 (Clean up the node namespaces string array and check the
  // return value)
  rcl_ret_t ret_ns = rcutils_string_array_fini(&node_namespaces_c);
  if (ret_ns != RCUTILS_RET_OK) {
    error += std::string(", could not destroy node namespaces, leaking memory: ") +
             rcl_get_error_string().str;
    rcl_reset_error();
  }

  // 清理节点 enclave 字符串数组并检查返回值 (Clean up the node enclaves string array and check the
  // return value)
  rcl_ret_t ret_ecv = rcutils_string_array_fini(&node_enclaves_c);
  if (ret_ecv != RCUTILS_RET_OK) {
    error += std::string(", could not destroy node enclaves, leaking memory: ") +
             rcl_get_error_string().str;
    rcl_reset_error();
  }

  // 如果任何清理操作失败，抛出运行时错误 (If any cleanup operation fails, throw a runtime error)
  if (ret_names != RCUTILS_RET_OK || ret_ns != RCUTILS_RET_OK || ret_ecv != RCUTILS_RET_OK) {
    throw std::runtime_error(error);
  }

  // 返回包含节点名称、命名空间和 enclave 的元组列表 (Return the list of tuples containing node
  // name, namespace, and enclave)
  return node_tuples;
}

/**
 * @brief 获取节点名称和命名空间列表 (Get the list of node names and namespaces)
 *
 * @return std::vector<std::pair<std::string, std::string>> 节点名称和命名空间的键值对列表 (List of
 * key-value pairs of node names and namespaces)
 */
std::vector<std::pair<std::string, std::string>> NodeGraph::get_node_names_and_namespaces() const {
  // 初始化节点名称字符串数组 (Initialize the node name string array)
  rcutils_string_array_t node_names_c = rcutils_get_zero_initialized_string_array();
  // 初始化节点命名空间字符串数组 (Initialize the node namespace string array)
  rcutils_string_array_t node_namespaces_c = rcutils_get_zero_initialized_string_array();

  // 获取默认分配器 (Get the default allocator)
  auto allocator = rcl_get_default_allocator();
  // 使用RCL（ROS客户端库）获取节点名称和命名空间 (Get node names and namespaces using RCL (ROS
  // Client Library))
  auto ret = rcl_get_node_names(
      node_base_->get_rcl_node_handle(), allocator, &node_names_c, &node_namespaces_c);
  // 如果获取失败，处理错误并清理资源 (If the acquisition fails, handle the error and clean up the
  // resources)
  if (ret != RCL_RET_OK) {
    auto error_msg = std::string("failed to get node names: ") + rcl_get_error_string().str;
    rcl_reset_error();
    if (rcutils_string_array_fini(&node_names_c) != RCUTILS_RET_OK) {
      error_msg += std::string(", failed also to cleanup node names, leaking memory: ") +
                   rcl_get_error_string().str;
      rcl_reset_error();
    }
    if (rcutils_string_array_fini(&node_namespaces_c) != RCUTILS_RET_OK) {
      error_msg += std::string(", failed also to cleanup node namespaces, leaking memory: ") +
                   rcl_get_error_string().str;
      rcl_reset_error();
    }
    // TODO(karsten1987): Append rcutils_error_message once it's in master
    throw std::runtime_error(error_msg);
  }

  // 创建节点名称和命名空间的键值对向量 (Create a key-value pair vector for node names and
  // namespaces)
  std::vector<std::pair<std::string, std::string>> node_names;
  node_names.reserve(node_names_c.size);
  // 遍历字符串数组，将有效的节点名称和命名空间添加到向量中 (Iterate through the string array,
  // adding valid node names and namespaces to the vector)
  for (size_t i = 0; i < node_names_c.size; ++i) {
    if (node_names_c.data[i] && node_namespaces_c.data[i]) {
      node_names.emplace_back(node_names_c.data[i], node_namespaces_c.data[i]);
    }
  }

  // 清理节点名称和命名空间字符串数组资源，并处理错误 (Clean up node name and namespace string array
  // resources and handle errors)
  std::string error;
  rcl_ret_t ret_names = rcutils_string_array_fini(&node_names_c);
  if (ret_names != RCUTILS_RET_OK) {
    // *INDENT-OFF*
    // TODO(karsten1987): Append rcutils_error_message once it's in master
    error = "could not destroy node names";
    // *INDENT-ON*
  }
  rcl_ret_t ret_ns = rcutils_string_array_fini(&node_namespaces_c);
  if (ret_ns != RCUTILS_RET_OK) {
    // *INDENT-OFF*
    // TODO(karsten1987): Append rcutils_error_message once it's in master
    error += ", could not destroy node namespaces";
    // *INDENT-ON*
  }

  // 如果清理失败，抛出运行时错误 (If the cleanup fails, throw a runtime error)
  if (ret_names != RCUTILS_RET_OK || ret_ns != RCUTILS_RET_OK) {
    throw std::runtime_error(error);
  }

  // 返回节点名称和命名空间的键值对列表 (Return the list of key-value pairs of node names and
  // namespaces)
  return node_names;
}

/*!
 * \brief 计算发布者的数量 (Count the number of publishers)
 * \param topic_name 要计算发布者数量的主题名称 (The name of the topic to count publishers for)
 * \return 发布者数量 (The number of publishers)
 */
size_t NodeGraph::count_publishers(const std::string& topic_name) const {
  // 获取 rcl_node_handle (Get the rcl_node_handle)
  auto rcl_node_handle = node_base_->get_rcl_node_handle();

  // 对主题名称进行扩展，以获取完全限定的主题名称 (Expand the topic name to get the fully qualified
  // topic name)
  auto fqdn = rclcpp::expand_topic_or_service_name(
      topic_name, rcl_node_get_name(rcl_node_handle), rcl_node_get_namespace(rcl_node_handle),
      false);  // false = not a service

  size_t count;
  // 调用 rcl_count_publishers 函数计算发布者数量 (Call the rcl_count_publishers function to count
  // the publishers)
  auto ret = rcl_count_publishers(rcl_node_handle, fqdn.c_str(), &count);
  if (ret != RMW_RET_OK) {
    // 如果返回值不为 RMW_RET_OK，则抛出异常 (If the return value is not RMW_RET_OK, throw an
    // exception)
    throw std::runtime_error(
        std::string("could not count publishers: ") + rmw_get_error_string().str);
  }
  return count;
}

/*!
 * \brief 计算订阅者的数量 (Count the number of subscribers)
 * \param topic_name 要计算订阅者数量的主题名称 (The name of the topic to count subscribers for)
 * \return 订阅者数量 (The number of subscribers)
 */
size_t NodeGraph::count_subscribers(const std::string& topic_name) const {
  // 获取 rcl_node_handle (Get the rcl_node_handle)
  auto rcl_node_handle = node_base_->get_rcl_node_handle();

  // 对主题名称进行扩展，以获取完全限定的主题名称 (Expand the topic name to get the fully qualified
  // topic name)
  auto fqdn = rclcpp::expand_topic_or_service_name(
      topic_name, rcl_node_get_name(rcl_node_handle), rcl_node_get_namespace(rcl_node_handle),
      false);  // false = not a service

  size_t count;
  // 调用 rcl_count_subscribers 函数计算订阅者数量 (Call the rcl_count_subscribers function to count
  // the subscribers)
  auto ret = rcl_count_subscribers(rcl_node_handle, fqdn.c_str(), &count);
  if (ret != RMW_RET_OK) {
    // 如果返回值不为 RMW_RET_OK，则抛出异常 (If the return value is not RMW_RET_OK, throw an
    // exception)
    throw std::runtime_error(
        std::string("could not count subscribers: ") + rmw_get_error_string().str);
  }
  return count;
}

/*!
 * \brief 获取 graph_guard_condition (Get the graph_guard_condition)
 * \return 指向 rcl_guard_condition_t 的指针 (A pointer to the rcl_guard_condition_t)
 */
const rcl_guard_condition_t* NodeGraph::get_graph_guard_condition() const {
  // 调用 rcl_node_get_graph_guard_condition 函数获取 graph_guard_condition (Call the
  // rcl_node_get_graph_guard_condition function to get the graph_guard_condition)
  return rcl_node_get_graph_guard_condition(node_base_->get_rcl_node_handle());
}

/**
 * @brief 通知图形变更 (Notify graph change)
 *
 * 该函数用于在图形发生变化时通知相关事件。 (This function is used to notify related events when the
 * graph changes.)
 */
void NodeGraph::notify_graph_change() {
  {
    // 使用 lock_guard 对象对图形互斥锁进行加锁，以保护图形数据的访问 (Use lock_guard object to lock
    // the graph mutex, to protect the access of graph data)
    std::lock_guard<std::mutex> graph_changed_lock(graph_mutex_);

    // 标记是否遇到无效指针 (Flag for whether an invalid pointer is encountered)
    bool bad_ptr_encountered = false;

    // 遍历图形事件列表 (Iterate through the graph event list)
    for (auto& event_wptr : graph_events_) {
      // 尝试从弱指针获取共享指针 (Try to get a shared pointer from the weak pointer)
      auto event_ptr = event_wptr.lock();

      // 如果共享指针有效，则设置事件 (If the shared pointer is valid, set the event)
      if (event_ptr) {
        event_ptr->set();
      } else {
        // 否则，标记遇到了无效指针 (Otherwise, flag that an invalid pointer has been encountered)
        bad_ptr_encountered = true;
      }
    }

    // 如果遇到无效指针 (If an invalid pointer is encountered)
    if (bad_ptr_encountered) {
      // 使用擦除-删除惯用法移除无效指针 (Remove invalid pointers using the erase-remove idiom)
      graph_events_.erase(
          std::remove_if(
              graph_events_.begin(), graph_events_.end(),
              [](const rclcpp::Event::WeakPtr& wptr) { return wptr.expired(); }),
          graph_events_.end());

      // 更新图形用户计数 (Update the graph users count)
      graph_users_count_.store(graph_events_.size());
    }
  }

  // 通知所有等待的线程 (Notify all waiting threads)
  graph_cv_.notify_all();

  // 获取节点的通知保护条件引用 (Get the reference to the node's notify guard condition)
  auto& node_gc = node_base_->get_notify_guard_condition();

  try {
    // 触发节点的通知保护条件 (Trigger the node's notify guard condition)
    node_gc.trigger();
  } catch (const rclcpp::exceptions::RCLError& ex) {
    // 如果触发失败，抛出运行时错误 (If triggering fails, throw a runtime error)
    throw std::runtime_error(
        std::string("failed to notify wait set on graph change: ") + ex.what());
  }
}

/**
 * @brief 通知节点图关闭 (Notify the node graph to shut down)
 */
void NodeGraph::notify_shutdown() {
  // 通知这里的任何内容，它不会被 ctrl-c 或 rclcpp::shutdown() 唤醒
  // (Notify anything here that will not be woken up by ctrl-c or rclcpp::shutdown().)
  graph_cv_.notify_all();
}

/**
 * @brief 获取节点图事件 (Get the node graph event)
 * @return 返回共享指针类型的事件 (Returns a shared pointer of the event)
 */
rclcpp::Event::SharedPtr NodeGraph::get_graph_event() {
  auto event = rclcpp::Event::make_shared();
  {
    std::lock_guard<std::mutex> graph_changed_lock(graph_mutex_);
    graph_events_.push_back(event);
    graph_users_count_++;
  }
  // 第一次调用时，将节点添加到 graph_listener_
  // (On the first call, add the node to graph_listener_)
  if (should_add_to_graph_listener_.exchange(false)) {
    graph_listener_->add_node(this);
    graph_listener_->start_if_not_started();
  }
  return event;
}

/**
 * @brief 等待节点图改变 (Wait for the node graph change)
 * @param event 共享指针类型的事件 (Shared pointer of the event)
 * @param timeout 超时时间，以纳秒为单位 (Timeout duration in nanoseconds)
 */
void NodeGraph::wait_for_graph_change(
    rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout) {
  using rclcpp::exceptions::EventNotRegisteredError;
  using rclcpp::exceptions::InvalidEventError;
  // 如果事件为空，则抛出 InvalidEventError 异常
  // (If the event is null, throw an InvalidEventError exception)
  if (!event) {
    throw InvalidEventError();
  }
  {
    std::lock_guard<std::mutex> graph_changed_lock(graph_mutex_);
    bool event_in_graph_events = false;
    // 遍历 graph_events_，检查事件是否在其中
    // (Iterate through graph_events_ and check if the event is present)
    for (const auto& event_wptr : graph_events_) {
      if (event == event_wptr.lock()) {
        event_in_graph_events = true;
        break;
      }
    }
    // 如果事件不在 graph_events_ 中，则抛出 EventNotRegisteredError 异常
    // (If the event is not in graph_events_, throw an EventNotRegisteredError exception)
    if (!event_in_graph_events) {
      throw EventNotRegisteredError();
    }
  }
  // 定义一个谓词函数，用于检查事件是否已触发或上下文是否仍然有效
  // (Define a predicate function to check if the event has been triggered or the context is still
  // valid)
  auto pred = [&event, context = node_base_->get_context()]() {
    return event->check() || !rclcpp::ok(context);
  };
  std::unique_lock<std::mutex> graph_lock(graph_mutex_);
  // 如果谓词为 false，则等待超时或谓词变为 true
  // (Wait for the timeout or the predicate to become true if the predicate is false)
  if (!pred()) {
    graph_cv_.wait_for(graph_lock, timeout, pred);
  }
}

/**
 * @brief 计算图中用户的数量 (Count the number of users in the graph)
 *
 * @return size_t 返回图中用户的数量 (Return the number of users in the graph)
 */
size_t NodeGraph::count_graph_users() const {
  // 加载并返回图中用户的数量 (Load and return the number of users in the graph)
  return graph_users_count_.load();
}

/**
 * @brief 将 rcl_topic_endpoint_info_array_t 转换为 TopicEndpointInfo 列表 (Convert
 * rcl_topic_endpoint_info_array_t to a list of TopicEndpointInfo)
 *
 * @param info_array 输入的 rcl_topic_endpoint_info_array_t 结构体 (Input
 * rcl_topic_endpoint_info_array_t structure)
 * @return std::vector<rclcpp::TopicEndpointInfo> 返回转换后的 TopicEndpointInfo 列表 (Return the
 * converted list of TopicEndpointInfo)
 */
static std::vector<rclcpp::TopicEndpointInfo> convert_to_topic_info_list(
    const rcl_topic_endpoint_info_array_t& info_array) {
  // 创建一个空的 TopicEndpointInfo 列表 (Create an empty TopicEndpointInfo list)
  std::vector<rclcpp::TopicEndpointInfo> topic_info_list;

  // 遍历输入的结构体数组，将每个元素转换为 TopicEndpointInfo 并添加到列表中 (Iterate through the
  // input structure array, convert each element to TopicEndpointInfo and add it to the list)
  for (size_t i = 0; i < info_array.size; ++i) {
    topic_info_list.push_back(rclcpp::TopicEndpointInfo(info_array.info_array[i]));
  }

  // 返回转换后的列表 (Return the converted list)
  return topic_info_list;
}

/**
 * @brief 获取指定主题的信息 (Get information about the specified topic)
 *
 * @tparam EndpointType 终端类型字符串 (Endpoint type string)
 * @tparam FunctionT 可调用对象类型，用于获取主题信息的函数 (Callable object type, function for
 * getting topic information)
 * @param node_base 节点基本接口指针 (Pointer to the node base interface)
 * @param topic_name 主题名称 (Topic name)
 * @param no_mangle 是否禁用主题名称修饰 (Whether to disable topic name decoration)
 * @param rcl_get_info_by_topic 用于获取主题信息的函数 (Function for getting topic information)
 * @return std::vector<rclcpp::TopicEndpointInfo> 返回指定主题的信息列表 (Return a list of
 * information about the specified topic)
 */
template <const char* EndpointType, typename FunctionT>
static std::vector<rclcpp::TopicEndpointInfo> get_info_by_topic(
    rclcpp::node_interfaces::NodeBaseInterface* node_base,
    const std::string& topic_name,
    bool no_mangle,
    FunctionT rcl_get_info_by_topic) {
  // 定义一个完全限定的主题名 (Define a fully qualified topic name)
  std::string fqdn;

  // 获取节点句柄 (Get the node handle)
  auto rcl_node_handle = node_base->get_rcl_node_handle();

  // 根据 no_mangle 参数决定是否修饰主题名称 (Decide whether to decorate the topic name based on the
  // no_mangle parameter)
  if (no_mangle) {
    fqdn = topic_name;
  } else {
    // 扩展主题名称 (Expand the topic name)
    fqdn = rclcpp::expand_topic_or_service_name(
        topic_name, rcl_node_get_name(rcl_node_handle), rcl_node_get_namespace(rcl_node_handle),
        false);  // false = not a service

    // 获取节点选项 (Get the node options)
    const rcl_node_options_t* node_options = rcl_node_get_options(rcl_node_handle);
    if (nullptr == node_options) {
      throw std::runtime_error("Need valid node options in get_info_by_topic()");
    }
    const rcl_arguments_t* global_args = nullptr;
    if (node_options->use_global_arguments) {
      global_args = &(rcl_node_handle->context->global_arguments);
    }

    // 重新映射主题名称 (Remap the topic name)
    char* remapped_topic_name = nullptr;
    rcl_ret_t ret = rcl_remap_topic_name(
        &(node_options->arguments), global_args, fqdn.c_str(), rcl_node_get_name(rcl_node_handle),
        rcl_node_get_namespace(rcl_node_handle), node_options->allocator, &remapped_topic_name);
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, std::string("Failed to remap topic name ") + fqdn);
    } else if (nullptr != remapped_topic_name) {
      fqdn = remapped_topic_name;
      node_options->allocator.deallocate(remapped_topic_name, node_options->allocator.state);
    }
  }

  // 初始化分配器和信息数组 (Initialize allocator and info array)
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_topic_endpoint_info_array_t info_array = rcl_get_zero_initialized_topic_endpoint_info_array();

  // 调用传入的函数获取主题信息 (Call the passed function to get topic information)
  rcl_ret_t ret =
      rcl_get_info_by_topic(rcl_node_handle, &allocator, fqdn.c_str(), no_mangle, &info_array);
  if (RCL_RET_OK != ret) {
    auto error_msg =
        std::string("Failed to get information by topic for ") + EndpointType + std::string(":");
    if (RCL_RET_UNSUPPORTED == ret) {
      error_msg += std::string("function not supported by RMW_IMPLEMENTATION");
    } else {
      error_msg += rcl_get_error_string().str;
    }
    rcl_reset_error();
    if (RCL_RET_OK != rcl_topic_endpoint_info_array_fini(&info_array, &allocator)) {
      error_msg += std::string(", failed also to cleanup topic info array, leaking memory: ") +
                   rcl_get_error_string().str;
      rcl_reset_error();
    }
    throw_from_rcl_error(ret, error_msg);
  }

  // 将信息数组转换为 TopicEndpointInfo 列表 (Convert the info array to a list of TopicEndpointInfo)
  std::vector<rclcpp::TopicEndpointInfo> topic_info_list = convert_to_topic_info_list(info_array);

  // 清理信息数组 (Clean up the info array)
  ret = rcl_topic_endpoint_info_array_fini(&info_array, &allocator);
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "rcl_topic_info_array_fini failed.");
  }

  // 返回主题信息列表 (Return the topic information list)
  return topic_info_list;
}

/*!
 * \brief 定义发布者端点类型名称常量
 * \brief Define publisher endpoint type name constant
 */
static constexpr char kPublisherEndpointTypeName[] = "publishers";

/*!
 * \brief 获取给定主题的发布者信息
 * \brief Get publisher information for a given topic
 * \param topic_name 主题名称
 * \param no_mangle 是否不混淆名称
 * \return 返回一个包含发布者信息的向量
 * \param topic_name Topic name
 * \param no_mangle Whether to not mangle the names
 * \return A vector containing publisher information
 */
std::vector<rclcpp::TopicEndpointInfo> NodeGraph::get_publishers_info_by_topic(
    const std::string& topic_name, bool no_mangle) const {
  return get_info_by_topic<kPublisherEndpointTypeName>(
      node_base_, topic_name, no_mangle, rcl_get_publishers_info_by_topic);
}

/*!
 * \brief 定义订阅者端点类型名称常量
 * \brief Define subscription endpoint type name constant
 */
static constexpr char kSubscriptionEndpointTypeName[] = "subscriptions";

/*!
 * \brief 获取给定主题的订阅者信息
 * \brief Get subscriber information for a given topic
 * \param topic_name 主题名称
 * \param no_mangle 是否不混淆名称
 * \return 返回一个包含订阅者信息的向量
 * \param topic_name Topic name
 * \param no_mangle Whether to not mangle the names
 * \return A vector containing subscriber information
 */
std::vector<rclcpp::TopicEndpointInfo> NodeGraph::get_subscriptions_info_by_topic(
    const std::string& topic_name, bool no_mangle) const {
  return get_info_by_topic<kSubscriptionEndpointTypeName>(
      node_base_, topic_name, no_mangle, rcl_get_subscriptions_info_by_topic);
}

/*!
 * \brief 获取节点名称
 * \brief Get node name
 * \return 返回节点名称的引用
 * \return A reference to the node name
 */
std::string& rclcpp::TopicEndpointInfo::node_name() { return node_name_; }

/*!
 * \brief 获取节点名称（常量）
 * \brief Get node name (constant)
 * \return 返回节点名称的常量引用
 * \return A constant reference to the node name
 */
const std::string& rclcpp::TopicEndpointInfo::node_name() const { return node_name_; }

/*!
 * \brief 获取节点命名空间
 * \brief Get node namespace
 * \return 返回节点命名空间的引用
 * \return A reference to the node namespace
 */
std::string& rclcpp::TopicEndpointInfo::node_namespace() { return node_namespace_; }

/*!
 * \brief 获取节点命名空间（常量）
 * \brief Get node namespace (constant)
 * \return 返回节点命名空间的常量引用
 * \return A constant reference to the node namespace
 */
const std::string& rclcpp::TopicEndpointInfo::node_namespace() const { return node_namespace_; }

/*!
 * \brief 获取主题类型
 * \brief Get topic type
 * \return 返回主题类型的引用
 * \return A reference to the topic type
 */
std::string& rclcpp::TopicEndpointInfo::topic_type() { return topic_type_; }

/*!
 * \brief 获取主题类型（常量）
 * \brief Get topic type (constant)
 * \return 返回主题类型的常量引用
 * \return A constant reference to the topic type
 */
const std::string& rclcpp::TopicEndpointInfo::topic_type() const { return topic_type_; }

/*!
 * \brief 获取端点类型
 * \brief Get endpoint type
 * \return 返回端点类型的引用
 * \return A reference to the endpoint type
 */
rclcpp::EndpointType& rclcpp::TopicEndpointInfo::endpoint_type() { return endpoint_type_; }

/*!
 * \brief 获取端点类型（常量）
 * \brief Get endpoint type (constant)
 * \return 返回端点类型的常量引用
 * \return A constant reference to the endpoint type
 */
const rclcpp::EndpointType& rclcpp::TopicEndpointInfo::endpoint_type() const {
  return endpoint_type_;
}

/*!
 * \brief 获取端点 GID
 * \brief Get endpoint GID
 * \return 返回端点 GID 的引用
 * \return A reference to the endpoint GID
 */
std::array<uint8_t, RMW_GID_STORAGE_SIZE>& rclcpp::TopicEndpointInfo::endpoint_gid() {
  return endpoint_gid_;
}

/*!
 * \brief 获取端点 GID（常量）
 * \brief Get endpoint GID (constant)
 * \return 返回端点 GID 的常量引用
 * \return A constant reference to the endpoint GID
 */
const std::array<uint8_t, RMW_GID_STORAGE_SIZE>& rclcpp::TopicEndpointInfo::endpoint_gid() const {
  return endpoint_gid_;
}

/*!
 * \brief 获取 QoS 配置文件
 * \brief Get QoS profile
 * \return 返回 QoS 配置文件的引用
 * \return A reference to the QoS profile
 */
rclcpp::QoS& rclcpp::TopicEndpointInfo::qos_profile() { return qos_profile_; }

/*!
 * \brief 获取 QoS 配置文件（常量）
 * \brief Get QoS profile (constant)
 * \return 返回 QoS 配置文件的常量引用
 * \return A constant reference to the QoS profile
 */
const rclcpp::QoS& rclcpp::TopicEndpointInfo::qos_profile() const { return qos_profile_; }

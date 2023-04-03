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

#ifndef RCLCPP__CREATE_CLIENT_HPP_
#define RCLCPP__CREATE_CLIENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/rmw.h"

namespace rclcpp {
/// 创建一个具有给定类型的服务客户端。 (Create a service client with a given type.)
/**
 * \param[in] node_base 用于创建客户端的节点上的 NodeBaseInterface 实现。 (NodeBaseInterface
 * implementation of the node on which to create the client.) \param[in] node_graph
 * 用于创建客户端的节点上的 NodeGraphInterface 实现。 (NodeGraphInterface implementation of the node
 * on which to create the client.) \param[in] node_services 用于创建客户端的节点上的
 * NodeServicesInterface 实现。 (NodeServicesInterface implementation of the node on which to create
 * the client.) \param[in] service_name 可访问服务的名称。 (The name on which the service is
 * accessible.) \param[in] qos 客户端的服务质量配置文件。 (Quality of service profile for client.)
 * \param[in] group 处理对服务调用的回复的回调组。 (Callback group to handle the reply to service
 * calls.) \return 创建的客户端的共享指针。 (Shared pointer to the created client.)
 */
template <typename ServiceT>
typename rclcpp::Client<ServiceT>::SharedPtr create_client(
    std::shared_ptr<node_interfaces::NodeBaseInterface>
        node_base,  // 节点基础接口的共享指针。 (Shared pointer to the NodeBaseInterface.)
    std::shared_ptr<node_interfaces::NodeGraphInterface>
        node_graph,  // 节点图形接口的共享指针。 (Shared pointer to the NodeGraphInterface.)
    std::shared_ptr<node_interfaces::NodeServicesInterface>
        node_services,  // 节点服务接口的共享指针。 (Shared pointer to the NodeServicesInterface.)
    const std::string& service_name,  // 服务名称。 (Service name.)
    const rclcpp::QoS& qos =
        rclcpp::ServicesQoS(),  // 客户端的服务质量配置文件，默认为 ServicesQoS。 (Quality of
                                // service profile for client, default is ServicesQoS.)
    rclcpp::CallbackGroup::SharedPtr group =
        nullptr)  // 处理对服务调用的回复的回调组，默认为空。 (Callback group to handle the reply to
                  // service calls, default is nullptr.)
{
  return create_client<ServiceT>(  // 使用给定参数创建客户端。 (Create a client with the given
                                   // parameters.)
      node_base, node_graph, node_services, service_name,
      qos.get_rmw_qos_profile(),  // 获取 RMW 服务质量配置文件。 (Get the RMW quality of service
                                  // profile.)
      group);
}

/// 创建一个具有给定类型的服务客户端。
/// Create a service client with a given type.
/// \internal
template <typename ServiceT>
// 定义一个共享指针类型别名，用于存储 rclcpp::Client<ServiceT> 类型的对象。
// Define a shared pointer alias for storing rclcpp::Client<ServiceT> type objects.
typename rclcpp::Client<ServiceT>::SharedPtr create_client(
    // 传入一个 NodeBaseInterface 的共享指针。
    // Pass in a shared pointer of NodeBaseInterface.
    std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
    // 传入一个 NodeGraphInterface 的共享指针。
    // Pass in a shared pointer of NodeGraphInterface.
    std::shared_ptr<node_interfaces::NodeGraphInterface> node_graph,
    // 传入一个 NodeServicesInterface 的共享指针。
    // Pass in a shared pointer of NodeServicesInterface.
    std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
    // 传入一个表示服务名称的字符串。
    // Pass in a string representing the service name.
    const std::string& service_name,
    // 传入一个 rmw_qos_profile_t 类型的引用，表示服务质量配置文件。
    // Pass in a reference to an rmw_qos_profile_t type, which represents the quality of service
    // profile.
    const rmw_qos_profile_t& qos_profile,
    // 传入一个 CallbackGroup 的共享指针。
    // Pass in a shared pointer of CallbackGroup.
    rclcpp::CallbackGroup::SharedPtr group) {
  // 获取默认的 rcl_client_options_t 结构体。
  // Get the default rcl_client_options_t structure.
  rcl_client_options_t options = rcl_client_get_default_options();
  // 设置 options 的服务质量配置文件。
  // Set the quality of service profile for options.
  options.qos = qos_profile;

  // 使用传入的参数创建一个 rclcpp::Client<ServiceT> 类型的共享指针对象。
  // Create a shared pointer object of type rclcpp::Client<ServiceT> using the passed parameters.
  auto cli =
      rclcpp::Client<ServiceT>::make_shared(node_base.get(), node_graph, service_name, options);

  // 将创建的 rclcpp::Client<ServiceT> 共享指针对象转换为 rclcpp::ClientBase 类型的共享指针。
  // Convert the created rclcpp::Client<ServiceT> shared pointer object to a shared pointer of type
  // rclcpp::ClientBase.
  auto cli_base_ptr = std::dynamic_pointer_cast<rclcpp::ClientBase>(cli);
  // 将转换后的共享指针添加到 node_services 中，并关联到传入的回调组。
  // Add the converted shared pointer to node_services and associate it with the passed callback
  // group.
  node_services->add_client(cli_base_ptr, group);
  // 返回创建的 rclcpp::Client<ServiceT> 共享指针对象。
  // Return the created shared pointer object of type rclcpp::Client<ServiceT>.
  return cli;
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_CLIENT_HPP_

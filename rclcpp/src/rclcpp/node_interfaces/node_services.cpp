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

#include "rclcpp/node_interfaces/node_services.hpp"

#include <string>

using rclcpp::node_interfaces::NodeServices;

/**
 * @brief 构造函数，初始化 NodeServices 类的实例 (Constructor, initializes an instance of the
 * NodeServices class)
 *
 * @param node_base 一个指向 NodeBaseInterface 的指针 (A pointer to a NodeBaseInterface)
 */
NodeServices::NodeServices(rclcpp::node_interfaces::NodeBaseInterface* node_base)
    : node_base_(
          node_base)  // 初始化 node_base_ 成员变量 (Initialize the node_base_ member variable)
{}

// 析构函数 (Destructor)
NodeServices::~NodeServices() {}

/**
 * @brief 添加一个服务 (Add a service)
 *
 * @param service_base_ptr 一个指向 ServiceBase 的共享指针 (A shared pointer to a ServiceBase)
 * @param group 一个指向 CallbackGroup 的共享指针 (A shared pointer to a CallbackGroup)
 */
void NodeServices::add_service(
    rclcpp::ServiceBase::SharedPtr service_base_ptr, rclcpp::CallbackGroup::SharedPtr group) {
  if (group) {  // 如果提供了回调组 (If a callback group is provided)
    if (!node_base_->callback_group_in_node(group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create service, group not in node.");
    }
  } else {  // 如果未提供回调组，则获取默认回调组 (If no callback group is provided, get the default
            // callback group)
    group = node_base_->get_default_callback_group();
  }

  group->add_service(
      service_base_ptr);  // 将服务添加到回调组中 (Add the service to the callback group)

  // 使用父节点通知执行器创建了新服务 (Notify the executor that a new service was created using the
  // parent Node)
  auto& node_gc = node_base_->get_notify_guard_condition();
  try {
    node_gc.trigger();
    group->trigger_notify_guard_condition();
  } catch (const rclcpp::exceptions::RCLError& ex) {
    throw std::runtime_error(
        std::string("failed to notify wait set on service creation: ") + ex.what());
  }
}

/**
 * @brief 添加一个客户端 (Add a client)
 *
 * @param client_base_ptr 一个指向 ClientBase 的共享指针 (A shared pointer to a ClientBase)
 * @param group 一个指向 CallbackGroup 的共享指针 (A shared pointer to a CallbackGroup)
 */
void NodeServices::add_client(
    rclcpp::ClientBase::SharedPtr client_base_ptr, rclcpp::CallbackGroup::SharedPtr group) {
  if (group) {  // 如果提供了回调组 (If a callback group is provided)
    if (!node_base_->callback_group_in_node(group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create client, group not in node.");
    }
  } else {  // 如果未提供回调组，则获取默认回调组 (If no callback group is provided, get the default
            // callback group)
    group = node_base_->get_default_callback_group();
  }

  group->add_client(
      client_base_ptr);  // 将客户端添加到回调组中 (Add the client to the callback group)

  // 使用父节点通知执行器创建了新客户端 (Notify the executor that a new client was created using the
  // parent Node)
  auto& node_gc = node_base_->get_notify_guard_condition();
  try {
    node_gc.trigger();
    group->trigger_notify_guard_condition();
  } catch (const rclcpp::exceptions::RCLError& ex) {
    throw std::runtime_error(
        std::string("failed to notify wait set on client creation: ") + ex.what());
  }
}

/**
 * @brief 解析服务名称 (Resolve the service name)
 *
 * @param name 服务名称 (The service name)
 * @param only_expand 是否仅展开名称 (Whether to only expand the name)
 * @return std::string 解析后的服务名称 (The resolved service name)
 */
std::string NodeServices::resolve_service_name(const std::string& name, bool only_expand) const {
  return node_base_->resolve_topic_or_service_name(name, true, only_expand);
}

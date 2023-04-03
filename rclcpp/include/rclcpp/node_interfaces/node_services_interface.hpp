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

#ifndef RCLCPP__NODE_INTERFACES__NODE_SERVICES_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_SERVICES_INTERFACE_HPP_

#include <string>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/detail/node_interfaces_helpers.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/// 纯虚接口类，用于Node API的NodeServices部分。
/// Pure virtual interface class for the NodeServices part of the Node API.
class NodeServicesInterface {
public:
  // 智能指针别名定义，仅限于NodeServicesInterface
  // Smart pointer aliases definition, only for NodeServicesInterface
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeServicesInterface)

  // 公共析构函数，为虚函数且默认实现
  // Public destructor, virtual and with default implementation
  RCLCPP_PUBLIC
  virtual ~NodeServicesInterface() = default;

  /**
   * @brief 添加客户端
   * @param client_base_ptr 客户端基类的智能指针
   * @param group 回调组的智能指针
   */
  // 添加客户端，接受一个ClientBase智能指针和一个CallbackGroup智能指针，返回类型为void，纯虚函数
  // Add a client, taking a ClientBase smart pointer and a CallbackGroup smart pointer, return type
  // is void, pure virtual function
  RCLCPP_PUBLIC
  virtual void add_client(
      rclcpp::ClientBase::SharedPtr client_base_ptr, rclcpp::CallbackGroup::SharedPtr group) = 0;

  /**
   * @brief 添加服务
   * @param service_base_ptr 服务基类的智能指针
   * @param group 回调组的智能指针
   */
  // 添加服务，接受一个ServiceBase智能指针和一个CallbackGroup智能指针，返回类型为void，纯虚函数
  // Add a service, taking a ServiceBase smart pointer and a CallbackGroup smart pointer, return
  // type is void, pure virtual function
  RCLCPP_PUBLIC
  virtual void add_service(
      rclcpp::ServiceBase::SharedPtr service_base_ptr, rclcpp::CallbackGroup::SharedPtr group) = 0;

  /**
   * @brief 解析服务名称
   * @param name 输入的服务名称
   * @param only_expand 默认值为false，如果为true，则仅扩展名称，不进行重映射
   * @return 返回解析后的服务名称
   */
  /// 获取给定输入名称的重映射和扩展服务名称。
  /// Get the remapped and expanded service name given an input name.
  RCLCPP_PUBLIC
  virtual std::string resolve_service_name(
      const std::string& name, bool only_expand = false) const = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeServicesInterface, services)

#endif  // RCLCPP__NODE_INTERFACES__NODE_SERVICES_INTERFACE_HPP_

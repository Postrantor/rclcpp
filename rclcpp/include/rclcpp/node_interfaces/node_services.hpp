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

#ifndef RCLCPP__NODE_INTERFACES__NODE_SERVICES_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_SERVICES_HPP_

#include <string>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/// \brief 实现节点 API 的 NodeServices 部分 (Implementation of the NodeServices part of the Node
/// API)
class NodeServices : public NodeServicesInterface {
public:
  /// \brief 只使用智能指针别名 (Use smart pointer aliases only)
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeServices)

  /// \brief 公共构造函数 (Public constructor)
  /// \param node_base 指向 NodeBaseInterface 类型的指针 (Pointer to a NodeBaseInterface type)
  RCLCPP_PUBLIC
  explicit NodeServices(rclcpp::node_interfaces::NodeBaseInterface* node_base);

  /// \brief 虚拟析构函数 (Virtual destructor)
  RCLCPP_PUBLIC
  virtual ~NodeServices();

  /// \brief 添加客户端 (Add client)
  /// \param client_base_ptr 指向 ClientBase 类型的智能指针 (Shared pointer to a ClientBase type)
  /// \param group 指向 CallbackGroup 类型的智能指针 (Shared pointer to a CallbackGroup type)
  RCLCPP_PUBLIC
  void add_client(
      rclcpp::ClientBase::SharedPtr client_base_ptr,
      rclcpp::CallbackGroup::SharedPtr group) override;

  /// \brief 添加服务 (Add service)
  /// \param service_base_ptr 指向 ServiceBase 类型的智能指针 (Shared pointer to a ServiceBase type)
  /// \param group 指向 CallbackGroup 类型的智能指针 (Shared pointer to a CallbackGroup type)
  RCLCPP_PUBLIC
  void add_service(
      rclcpp::ServiceBase::SharedPtr service_base_ptr,
      rclcpp::CallbackGroup::SharedPtr group) override;

  /// \brief 解析服务名称 (Resolve service name)
  /// \param name 服务名称字符串 (Service name string)
  /// \param only_expand 是否仅展开，默认为 false (Whether to only expand, default is false)
  /// \return 返回解析后的服务名称字符串 (Returns the resolved service name string)
  RCLCPP_PUBLIC
  std::string resolve_service_name(
      const std::string& name, bool only_expand = false) const override;

private:
  /// \brief 禁用复制构造函数和赋值运算符 (Disable copy constructor and assignment operator)
  RCLCPP_DISABLE_COPY(NodeServices)

  /// \brief 指向 NodeBaseInterface 类型的私有成员指针 (Private member pointer to a
  /// NodeBaseInterface type)
  rclcpp::node_interfaces::NodeBaseInterface* node_base_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_SERVICES_HPP_

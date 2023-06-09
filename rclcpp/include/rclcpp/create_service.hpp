// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CREATE_SERVICE_HPP_
#define RCLCPP__CREATE_SERVICE_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"

namespace rclcpp {
/// 创建具有给定类型的服务。
/**
 * \param[in] node_base 用于创建服务的节点的 NodeBaseInterface 实现。
 * \param[in] node_services 用于在其上创建服务的节点的 NodeServicesInterface 实现。
 * \param[in] service_name 服务可访问的名称。
 * \param[in] callback 当服务收到请求时调用的回调。
 * \param[in] qos 服务的服务质量配置文件。
 * \param[in] group 处理对服务调用的回复的回调组。
 * \return 创建的服务的共享指针。
 */
template <typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr create_service(
    std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
    std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
    const std::string& service_name,
    CallbackT&& callback,
    const rclcpp::QoS& qos,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 调用另一个重载版本的 create_service 函数
  return create_service<ServiceT, CallbackT>(
      node_base, node_services, service_name, std::forward<CallbackT>(callback),
      qos.get_rmw_qos_profile(), group);
}

/// 创建具有给定类型的服务。
/// \internal
template <typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr create_service(
    std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
    std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
    const std::string& service_name,
    CallbackT&& callback,
    const rmw_qos_profile_t& qos_profile,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 创建一个 AnyServiceCallback 对象
  rclcpp::AnyServiceCallback<ServiceT> any_service_callback;
  // 设置回调函数
  any_service_callback.set(std::forward<CallbackT>(callback));

  // 获取默认的服务选项
  rcl_service_options_t service_options = rcl_service_get_default_options();
  // 设置服务选项的 QoS 配置文件
  service_options.qos = qos_profile;

  // 创建一个共享指针的 Service 对象
  auto serv = Service<ServiceT>::make_shared(
      node_base->get_shared_rcl_node_handle(), service_name, any_service_callback, service_options);
  // 将 Service 对象转换为 ServiceBase 类型的共享指针
  auto serv_base_ptr = std::dynamic_pointer_cast<ServiceBase>(serv);
  // 将服务添加到节点服务中，并设置回调组
  node_services->add_service(serv_base_ptr, group);
  // 返回创建的服务共享指针
  return serv;
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_SERVICE_HPP_

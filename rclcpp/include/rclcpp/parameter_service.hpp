// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__PARAMETER_SERVICE_HPP_
#define RCLCPP__PARAMETER_SERVICE_HPP_

#include <memory>
#include <string>

#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/srv/get_parameter_types.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"

namespace rclcpp {

/**
 * @class ParameterService
 * @brief 参数服务类，用于处理节点参数的获取、设置等操作 (A class for handling parameter operations,
 * such as getting and setting node parameters)
 */
class ParameterService {
public:
  // 定义智能指针类型 (Define smart pointer types)
  RCLCPP_SMART_PTR_DEFINITIONS(ParameterService)

  /**
   * @brief 已弃用的构造函数，请使用 rclcpp::QoS 而不是 rmw_qos_profile_t (Deprecated constructor,
   * use rclcpp::QoS instead of rmw_qos_profile_t)
   * @param[in] node_base 节点基础接口的共享指针 (Shared pointer to the node base interface)
   * @param[in] node_services 节点服务接口的共享指针 (Shared pointer to the node services interface)
   * @param[in] node_params 节点参数接口指针 (Pointer to the node parameters interface)
   * @param[in] qos_profile QoS配置 (QoS configuration)
   */
  [[deprecated("use rclcpp::QoS instead of rmw_qos_profile_t")]] RCLCPP_PUBLIC ParameterService(
      const std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
      const std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
      rclcpp::node_interfaces::NodeParametersInterface* node_params,
      const rmw_qos_profile_t& qos_profile)
      : ParameterService(
            node_base,
            node_services,
            node_params,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile))) {}

  /**
   * @brief 构造函数 (Constructor)
   * @param[in] node_base 节点基础接口的共享指针 (Shared pointer to the node base interface)
   * @param[in] node_services 节点服务接口的共享指针 (Shared pointer to the node services interface)
   * @param[in] node_params 节点参数接口指针 (Pointer to the node parameters interface)
   * @param[in] qos_profile QoS配置，默认为 rclcpp::ParametersQoS() (QoS configuration, default is
   * rclcpp::ParametersQoS())
   */
  RCLCPP_PUBLIC
  ParameterService(
      const std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
      const std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
      rclcpp::node_interfaces::NodeParametersInterface* node_params,
      const rclcpp::QoS& qos_profile = rclcpp::ParametersQoS());

private:
  // 获取参数服务的共享指针 (Shared pointer to the get parameters service)
  rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_service_;
  // 获取参数类型服务的共享指针 (Shared pointer to the get parameter types service)
  rclcpp::Service<rcl_interfaces::srv::GetParameterTypes>::SharedPtr get_parameter_types_service_;
  // 设置参数服务的共享指针 (Shared pointer to the set parameters service)
  rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_service_;
  // 原子设置参数服务的共享指针 (Shared pointer to the set parameters atomically service)
  rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr
      set_parameters_atomically_service_;
  // 描述参数服务的共享指针 (Shared pointer to the describe parameters service)
  rclcpp::Service<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_service_;
  // 列出参数服务的共享指针 (Shared pointer to the list parameters service)
  rclcpp::Service<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_service_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_SERVICE_HPP_

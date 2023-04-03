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

#include "rclcpp/parameter_service.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "./parameter_service_names.hpp"
#include "rclcpp/logging.hpp"

using rclcpp::ParameterService;

/**
 * @brief 构造函数，用于创建 ParameterService 对象 (Constructor for creating a ParameterService
 * object)
 *
 * @param node_base 节点基础接口的共享指针 (Shared pointer to the node base interface)
 * @param node_services 节点服务接口的共享指针 (Shared pointer to the node services interface)
 * @param node_params 节点参数接口的指针 (Pointer to the node parameters interface)
 * @param qos_profile 服务的质量配置 (Quality of Service profile for the services)
 */
ParameterService::ParameterService(
    const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base,
    const std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services,
    rclcpp::node_interfaces::NodeParametersInterface* node_params,
    const rclcpp::QoS& qos_profile) {
  // 获取节点名称 (Get the node name)
  const std::string node_name = node_base->get_name();

  // 创建 get_parameters 服务 (Create the get_parameters service)
  get_parameters_service_ = create_service<rcl_interfaces::srv::GetParameters>(
      node_base, node_services, node_name + "/" + parameter_service_names::get_parameters,
      [node_params](
          const std::shared_ptr<rmw_request_id_t>,
          const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request,
          std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response) {
        try {
          // 获取请求中的参数 (Get the parameters from the request)
          auto parameters = node_params->get_parameters(request->names);
          // 将参数值添加到响应中 (Add the parameter values to the response)
          for (const auto& param : parameters) {
            response->values.push_back(param.get_value_message());
          }
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException& ex) {
          // 处理参数未声明的异常情况 (Handle the exception case when a parameter is not declared)
          RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Failed to get parameters: %s", ex.what());
        } catch (const rclcpp::exceptions::ParameterUninitializedException& ex) {
          // 处理参数未初始化的异常情况 (Handle the exception case when a parameter is
          // uninitialized)
          RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Failed to get parameters: %s", ex.what());
        }
      },
      qos_profile, nullptr);

  // 创建 get_parameter_types 服务 (Create the get_parameter_types service)
  get_parameter_types_service_ = create_service<rcl_interfaces::srv::GetParameterTypes>(
      node_base, node_services, node_name + "/" + parameter_service_names::get_parameter_types,
      [node_params](
          const std::shared_ptr<rmw_request_id_t>,
          const std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Request> request,
          std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Response> response) {
        try {
          // 获取请求中的参数类型 (Get the parameter types from the request)
          auto types = node_params->get_parameter_types(request->names);
          // 将参数类型添加到响应中 (Add the parameter types to the response)
          std::transform(
              types.cbegin(), types.cend(), std::back_inserter(response->types),
              [](const uint8_t& type) { return static_cast<rclcpp::ParameterType>(type); });
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException& ex) {
          // 处理参数未声明的异常情况 (Handle the exception case when a parameter is not declared)
          RCLCPP_DEBUG(
              rclcpp::get_logger("rclcpp"), "Failed to get parameter types: %s", ex.what());
        }
      },
      qos_profile, nullptr);

  // 创建 set_parameters 服务 (Create the set_parameters service)
  set_parameters_service_ = create_service<rcl_interfaces::srv::SetParameters>(
      node_base, node_services, node_name + "/" + parameter_service_names::set_parameters,
      [node_params](
          const std::shared_ptr<rmw_request_id_t>,
          const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
          std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response) {
        // 逐个设置参数，因为如果 set_parameters() 失败，则无法返回部分结果
        // (Set parameters one-by-one, since there's no way to return a partial result if
        // set_parameters() fails.)
        auto result = rcl_interfaces::msg::SetParametersResult();
        for (auto& p : request->parameters) {
          try {
            // 尝试原子地设置参数 (Try to set the parameter atomically)
            result =
                node_params->set_parameters_atomically({rclcpp::Parameter::from_parameter_msg(p)});
          } catch (const rclcpp::exceptions::ParameterNotDeclaredException& ex) {
            // 处理参数未声明的异常情况 (Handle the exception case when a parameter is not declared)
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Failed to set parameter: %s", ex.what());
            result.successful = false;
            result.reason = ex.what();
          }
          // 将结果添加到响应中 (Add the result to the response)
          response->results.push_back(result);
        }
      },
      qos_profile, nullptr);

  // 创建 set_parameters_atomically 服务 (Create the set_parameters_atomically service)
  set_parameters_atomically_service_ = create_service<rcl_interfaces::srv::SetParametersAtomically>(
      node_base, node_services,
      node_name + "/" + parameter_service_names::set_parameters_atomically,
      [node_params](
          const std::shared_ptr<rmw_request_id_t>,
          const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> request,
          std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response) {
        // 转换请求中的参数 (Transform the parameters from the request)
        std::vector<rclcpp::Parameter> pvariants;
        std::transform(
            request->parameters.cbegin(), request->parameters.cend(), std::back_inserter(pvariants),
            [](const rcl_interfaces::msg::Parameter& p) {
              return rclcpp::Parameter::from_parameter_msg(p);
            });
        try {
          // 尝试原子地设置参数 (Try to set the parameters atomically)
          auto result = node_params->set_parameters_atomically(pvariants);
          response->result = result;
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException& ex) {
          // 处理参数未声明的异常情况 (Handle the exception case when a parameter is not declared)
          RCLCPP_DEBUG(
              rclcpp::get_logger("rclcpp"), "Failed to set parameters atomically: %s", ex.what());
          response->result.successful = false;
          response->result.reason = "One or more parameters were not declared before setting";
        }
      },
      qos_profile, nullptr);

  // 创建 describe_parameters 服务 (Create the describe_parameters service)
  describe_parameters_service_ = create_service<rcl_interfaces::srv::DescribeParameters>(
      node_base, node_services, node_name + "/" + parameter_service_names::describe_parameters,
      [node_params](
          const std::shared_ptr<rmw_request_id_t>,
          const std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Request> request,
          std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Response> response) {
        try {
          // 获取请求中的参数描述符 (Get the parameter descriptors from the request)
          auto descriptors = node_params->describe_parameters(request->names);
          response->descriptors = descriptors;
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException& ex) {
          // 处理参数未声明的异常情况 (Handle the exception case when a parameter is not declared)
          RCLCPP_DEBUG(
              rclcpp::get_logger("rclcpp"), "Failed to describe parameters: %s", ex.what());
        }
      },
      qos_profile, nullptr);

  // 创建 list_parameters 服务 (Create the list_parameters service)
  list_parameters_service_ = create_service<rcl_interfaces::srv::ListParameters>(
      node_base, node_services, node_name + "/" + parameter_service_names::list_parameters,
      [node_params](
          const std::shared_ptr<rmw_request_id_t>,
          const std::shared_ptr<rcl_interfaces::srv::ListParameters::Request> request,
          std::shared_ptr<rcl_interfaces::srv::ListParameters::Response> response) {
        // 获取请求中的参数列表 (Get the parameter list from the request)
        auto result = node_params->list_parameters(request->prefixes, request->depth);
        response->result = result;
      },
      qos_profile, nullptr);
}

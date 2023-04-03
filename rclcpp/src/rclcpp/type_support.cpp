// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/srv/get_parameter_types.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

/**
 * @brief 获取 ParameterEvent 消息类型支持的函数 (Get the type support for ParameterEvent message)
 *
 * @return 返回 rosidl_message_type_support_t 类型的指针 (Return a pointer of type
 * rosidl_message_type_support_t)
 */
const rosidl_message_type_support_t* rclcpp::type_support::get_parameter_event_msg_type_support() {
  // 获取 ParameterEvent 消息类型支持句柄 (Get the message type support handle for ParameterEvent)
  return rosidl_typesupport_cpp::get_message_type_support_handle<
      rcl_interfaces::msg::ParameterEvent>();
}

/**
 * @brief 获取 SetParametersResult 消息类型支持的函数 (Get the type support for SetParametersResult
 * message)
 *
 * @return 返回 rosidl_message_type_support_t 类型的指针 (Return a pointer of type
 * rosidl_message_type_support_t)
 */
const rosidl_message_type_support_t*
rclcpp::type_support::get_set_parameters_result_msg_type_support() {
  // 获取 SetParametersResult 消息类型支持句柄 (Get the message type support handle for
  // SetParametersResult)
  return rosidl_typesupport_cpp::get_message_type_support_handle<
      rcl_interfaces::msg::SetParametersResult>();
}

/**
 * @brief 获取 ParameterDescriptor 消息类型支持的函数 (Get the type support for ParameterDescriptor
 * message)
 *
 * @return 返回 rosidl_message_type_support_t 类型的指针 (Return a pointer of type
 * rosidl_message_type_support_t)
 */
const rosidl_message_type_support_t*
rclcpp::type_support::get_parameter_descriptor_msg_type_support() {
  // 获取 ParameterDescriptor 消息类型支持句柄 (Get the message type support handle for
  // ParameterDescriptor)
  return rosidl_typesupport_cpp::get_message_type_support_handle<
      rcl_interfaces::msg::ParameterDescriptor>();
}

/**
 * @brief 获取 ListParametersResult 消息类型支持的函数 (Get the type support for
 * ListParametersResult message)
 *
 * @return 返回 rosidl_message_type_support_t 类型的指针 (Return a pointer of type
 * rosidl_message_type_support_t)
 */
const rosidl_message_type_support_t*
rclcpp::type_support::get_list_parameters_result_msg_type_support() {
  // 获取 ListParametersResult 消息类型支持句柄 (Get the message type support handle for
  // ListParametersResult)
  return rosidl_typesupport_cpp::get_message_type_support_handle<
      rcl_interfaces::msg::ListParametersResult>();
}

/**
 * @brief 获取 GetParameters 服务类型支持的函数 (Get the type support for GetParameters service)
 *
 * @return 返回 rosidl_service_type_support_t 类型的指针 (Return a pointer of type
 * rosidl_service_type_support_t)
 */
const rosidl_service_type_support_t* rclcpp::type_support::get_get_parameters_srv_type_support() {
  // 获取 GetParameters 服务类型支持句柄 (Get the service type support handle for GetParameters)
  return rosidl_typesupport_cpp::get_service_type_support_handle<
      rcl_interfaces::srv::GetParameters>();
}

/**
 * @brief 获取 GetParameterTypes 服务类型支持的函数 (Get the type support for GetParameterTypes
 * service)
 *
 * @return 返回 rosidl_service_type_support_t 类型的指针 (Return a pointer of type
 * rosidl_service_type_support_t)
 */
const rosidl_service_type_support_t*
rclcpp::type_support::get_get_parameter_types_srv_type_support() {
  // 获取 GetParameterTypes 服务类型支持句柄 (Get the service type support handle for
  // GetParameterTypes)
  return rosidl_typesupport_cpp::get_service_type_support_handle<
      rcl_interfaces::srv::GetParameterTypes>();
}

/**
 * @brief 获取 SetParameters 服务类型支持的函数 (Get the type support for SetParameters service)
 *
 * @return 返回 rosidl_service_type_support_t 类型的指针 (Return a pointer of type
 * rosidl_service_type_support_t)
 */
const rosidl_service_type_support_t* rclcpp::type_support::get_set_parameters_srv_type_support() {
  // 获取 SetParameters 服务类型支持句柄 (Get the service type support handle for SetParameters)
  return rosidl_typesupport_cpp::get_service_type_support_handle<
      rcl_interfaces::srv::SetParameters>();
}

/**
 * @brief 获取 ListParameters 服务类型支持的函数 (Get the type support for ListParameters service)
 *
 * @return 返回 rosidl_service_type_support_t 类型的指针 (Return a pointer of type
 * rosidl_service_type_support_t)
 */
const rosidl_service_type_support_t* rclcpp::type_support::get_list_parameters_srv_type_support() {
  // 获取 ListParameters 服务类型支持句柄 (Get the service type support handle for ListParameters)
  return rosidl_typesupport_cpp::get_service_type_support_handle<
      rcl_interfaces::srv::ListParameters>();
}

/**
 * @brief 获取 DescribeParameters 服务类型支持的函数 (Get the type support for DescribeParameters
 * service)
 *
 * @return 返回 rosidl_service_type_support_t 类型的指针 (Return a pointer of type
 * rosidl_service_type_support_t)
 */
const rosidl_service_type_support_t*
rclcpp::type_support::get_describe_parameters_srv_type_support() {
  // 获取 DescribeParameters 服务类型支持句柄 (Get the service type support handle for
  // DescribeParameters)
  return rosidl_typesupport_cpp::get_service_type_support_handle<
      rcl_interfaces::srv::DescribeParameters>();
}

/**
 * @brief 获取 SetParametersAtomically 服务类型支持的函数 (Get the type support for
 * SetParametersAtomically service)
 *
 * @return 返回 rosidl_service_type_support_t 类型的指针 (Return a pointer of type
 * rosidl_service_type_support_t)
 */
const rosidl_service_type_support_t*
rclcpp::type_support::get_set_parameters_atomically_srv_type_support() {
  // 获取 SetParametersAtomically 服务类型支持句柄 (Get the service type support handle for
  // SetParametersAtomically)
  return rosidl_typesupport_cpp::get_service_type_support_handle<
      rcl_interfaces::srv::SetParametersAtomically>();
}

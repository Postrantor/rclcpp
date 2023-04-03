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

#ifndef RCLCPP__TYPE_SUPPORT_DECL_HPP_
#define RCLCPP__TYPE_SUPPORT_DECL_HPP_

#include "rclcpp/visibility_control.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"
#include "rosidl_runtime_cpp/service_type_support_decl.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace rclcpp {
namespace type_support {

/**
 * @brief 获取 intra_process_message 消息的类型支持
 * @return 返回 rosidl_message_type_support_t 类型的指针
 *
 * @brief Get the type support for intra_process_message messages
 * @return Returns a pointer of type rosidl_message_type_support_t
 */
RCLCPP_PUBLIC
const rosidl_message_type_support_t* get_intra_process_message_msg_type_support();

/**
 * @brief 获取 parameter_event 消息的类型支持
 * @return 返回 rosidl_message_type_support_t 类型的指针
 *
 * @brief Get the type support for parameter_event messages
 * @return Returns a pointer of type rosidl_message_type_support_t
 */
RCLCPP_PUBLIC
const rosidl_message_type_support_t* get_parameter_event_msg_type_support();

/**
 * @brief 获取 set_parameters_result 消息的类型支持
 * @return 返回 rosidl_message_type_support_t 类型的指针
 *
 * @brief Get the type support for set_parameters_result messages
 * @return Returns a pointer of type rosidl_message_type_support_t
 */
RCLCPP_PUBLIC
const rosidl_message_type_support_t* get_set_parameters_result_msg_type_support();

/**
 * @brief 获取 parameter_descriptor 消息的类型支持
 * @return 返回 rosidl_message_type_support_t 类型的指针
 *
 * @brief Get the type support for parameter_descriptor messages
 * @return Returns a pointer of type rosidl_message_type_support_t
 */
RCLCPP_PUBLIC
const rosidl_message_type_support_t* get_parameter_descriptor_msg_type_support();

/**
 * @brief 获取 list_parameters_result 消息的类型支持
 * @return 返回 rosidl_message_type_support_t 类型的指针
 *
 * @brief Get the type support for list_parameters_result messages
 * @return Returns a pointer of type rosidl_message_type_support_t
 */
RCLCPP_PUBLIC
const rosidl_message_type_support_t* get_list_parameters_result_msg_type_support();

/**
 * @brief 获取 get_parameters 服务的类型支持
 * @return 返回 rosidl_service_type_support_t 类型的指针
 *
 * @brief Get the type support for get_parameters service
 * @return Returns a pointer of type rosidl_service_type_support_t
 */
RCLCPP_PUBLIC
const rosidl_service_type_support_t* get_get_parameters_srv_type_support();

/**
 * @brief 获取 get_parameter_types 服务的类型支持
 * @return 返回 rosidl_service_type_support_t 类型的指针
 *
 * @brief Get the type support for get_parameter_types service
 * @return Returns a pointer of type rosidl_service_type_support_t
 */
RCLCPP_PUBLIC
const rosidl_service_type_support_t* get_get_parameter_types_srv_type_support();

/**
 * @brief 获取 set_parameters 服务的类型支持
 * @return 返回 rosidl_service_type_support_t 类型的指针
 *
 * @brief Get the type support for set_parameters service
 * @return Returns a pointer of type rosidl_service_type_support_t
 */
RCLCPP_PUBLIC
const rosidl_service_type_support_t* get_set_parameters_srv_type_support();

/**
 * @brief 获取 list_parameters 服务的类型支持
 * @return 返回 rosidl_service_type_support_t 类型的指针
 *
 * @brief Get the type support for list_parameters service
 * @return Returns a pointer of type rosidl_service_type_support_t
 */
RCLCPP_PUBLIC
const rosidl_service_type_support_t* get_list_parameters_srv_type_support();

/**
 * @brief 获取 describe_parameters 服务的类型支持
 * @return 返回 rosidl_service_type_support_t 类型的指针
 *
 * @brief Get the type support for describe_parameters service
 * @return Returns a pointer of type rosidl_service_type_support_t
 */
RCLCPP_PUBLIC
const rosidl_service_type_support_t* get_describe_parameters_srv_type_support();

/**
 * @brief 获取 set_parameters_atomically 服务的类型支持
 * @return 返回 rosidl_service_type_support_t 类型的指针
 *
 * @brief Get the type support for set_parameters_atomically service
 * @return Returns a pointer of type rosidl_service_type_support_t
 */
RCLCPP_PUBLIC
const rosidl_service_type_support_t* get_set_parameters_atomically_srv_type_support();

}  // namespace type_support
}  // namespace rclcpp

#endif  // RCLCPP__TYPE_SUPPORT_DECL_HPP_

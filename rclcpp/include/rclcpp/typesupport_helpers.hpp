// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2021, Apex.AI Inc.
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

#ifndef RCLCPP__TYPESUPPORT_HELPERS_HPP_
#define RCLCPP__TYPESUPPORT_HELPERS_HPP_

#include <memory>
#include <string>
#include <tuple>

#include "rclcpp/visibility_control.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

namespace rclcpp {
/// 加载给定类型的类型支持库。 (Load the type support library for the given type.)
/**
 * \param[in] type 主题类型，例如 "std_msgs/msg/String" (The topic type, e.g. "std_msgs/msg/String")
 * \param[in] typesupport_identifier 类型支持标识符，通常为 "rosidl_typesupport_cpp" (Type support
 * identifier, typically "rosidl_typesupport_cpp") \return 一个共享库 (A shared library)
 */
RCLCPP_PUBLIC
std::shared_ptr<rcpputils::SharedLibrary> get_typesupport_library(
    const std::string& type, const std::string& typesupport_identifier);

/// 从库中提取类型支持句柄。 (Extract the type support handle from the library.)
/**
 * 库需要匹配主题类型。共享库必须在结果的生命周期内保持加载。 (The library needs to match the topic
 * type. The shared library must stay loaded for the lifetime of the result.) \param[in] type
 * 主题类型，例如 "std_msgs/msg/String" (The topic type, e.g. "std_msgs/msg/String") \param[in]
 * typesupport_identifier 类型支持标识符，通常为 "rosidl_typesupport_cpp" (Type support identifier,
 * typically "rosidl_typesupport_cpp") \param[in] library 共享类型支持库 (The shared type support
 * library) \return 类型支持句柄 (A type support handle)
 */
RCLCPP_PUBLIC
const rosidl_message_type_support_t* get_typesupport_handle(
    const std::string& type,
    const std::string& typesupport_identifier,
    rcpputils::SharedLibrary& library);

}  // namespace rclcpp

#endif  // RCLCPP__TYPESUPPORT_HELPERS_HPP_

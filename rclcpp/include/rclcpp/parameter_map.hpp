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

#ifndef RCLCPP__PARAMETER_MAP_HPP_
#define RCLCPP__PARAMETER_MAP_HPP_

#include <rcl_yaml_param_parser/parser.h>
#include <rcl_yaml_param_parser/types.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/// 一个映射类型，将完全限定节点名称映射到参数列表
/// A map of fully qualified node names to a list of parameters
using ParameterMap = std::unordered_map<std::string, std::vector<Parameter>>;

/// 将 rcl_yaml_param_parser 中的参数转换为 C++ 类实例。
/// Convert parameters from rcl_yaml_param_parser into C++ class instances.
/// \param[in] c_params 包含多个节点参数的 C 结构体。
/// \param[in] c_params C structures containing parameters for multiple nodes.
/// \param[in] node_fqn 节点的完全限定名称，默认值为 nullptr。
/// \param[in] node_fqn a Fully Qualified Name of node, default value is nullptr.
///   如果不是 nullptr，则返回属于此 node_fqn 的相关节点参数。
///   If it's not nullptr, return the relative node parameters belonging to this node_fqn.
/// \returns 一个映射，其中键是完全限定节点名称，值是参数列表。
/// \returns a map where the keys are fully qualified node names and values a list of parameters.
/// \throws InvalidParametersException 如果 `rcl_params_t` 不一致或无效。
/// \throws InvalidParametersException if the `rcl_params_t` is inconsistent or invalid.
RCLCPP_PUBLIC
ParameterMap parameter_map_from(const rcl_params_t* const c_params, const char* node_fqn = nullptr);

/// 将 rcl_yaml_param_parser 中的参数值转换为 C++ 类实例。
/// Convert parameter value from rcl_yaml_param_parser into a C++ class instance.
/// \param[in] c_value 包含参数值的 C 结构体。
/// \param[in] c_value C structure containing a value of a parameter.
/// \returns 参数值的实例
/// \returns an instance of a parameter value
/// \throws InvalidParameterValueException 如果 `rcl_variant_t` 不一致或无效。
/// \throws InvalidParameterValueException if the `rcl_variant_t` is inconsistent or invalid.
RCLCPP_PUBLIC
ParameterValue parameter_value_from(const rcl_variant_t* const c_value);

/// 从 yaml 文件获取 ParameterMap。
/// Get the ParameterMap from a yaml file.
/// \param[in] yaml_filename yaml 文件的完整名称。
/// \param[in] yaml_filename full name of the yaml file.
/// \param[in] node_fqn 节点的完全限定名称，默认值为 nullptr。
/// \param[in] node_fqn a Fully Qualified Name of node, default value is nullptr.
/// \returns 参数映射的实例
/// \returns an instance of a parameter map
/// \throws 来自 rcl_parse_yaml_file() 的 rcl 错误
/// \throws from rcl error of rcl_parse_yaml_file()
RCLCPP_PUBLIC
ParameterMap parameter_map_from_yaml_file(
    const std::string& yaml_filename, const char* node_fqn = nullptr);

/// 从 ParameterMap 获取 Parameters。
/// Get the Parameters from ParameterMap.
/// \param[in] parameter_map 参数映射。
/// \param[in] parameter_map a parameter map.
/// \param[in] node_fqn 节点的完全限定名称，默认值为 nullptr。
/// \param[in] node_fqn a Fully Qualified Name of node, default value is nullptr.
/// \returns 参数列表
/// \returns a list of a parameter
RCLCPP_PUBLIC
std::vector<Parameter> parameters_from_map(
    const ParameterMap& parameter_map, const char* node_fqn = nullptr);

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_MAP_HPP_

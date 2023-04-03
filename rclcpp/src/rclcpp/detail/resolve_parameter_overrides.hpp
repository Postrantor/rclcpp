// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DETAIL__RESOLVE_PARAMETER_OVERRIDES_HPP_
#define RCLCPP__DETAIL__RESOLVE_PARAMETER_OVERRIDES_HPP_

#include <map>
#include <string>
#include <vector>

#include "rcl/arguments.h"

#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace detail
{
/// \internal
/// \brief 获取参数覆盖值 (Get the parameter overrides from the arguments)
/// \param[in] node_name 节点名称 (Node name)
/// \param[in] parameter_overrides 参数覆盖集合 (Collection of parameter overrides)
/// \param[in] local_args 本地参数 (Local arguments)
/// \param[in] global_args 全局参数 (Global arguments)
/// \return 返回一个映射，包含解析出的参数覆盖值 (Returns a map containing the resolved parameter overrides)
RCLCPP_LOCAL
std::map<std::string, rclcpp::ParameterValue> resolve_parameter_overrides(
  const std::string & node_name, ///< 节点名称 (Node name)
  const std::vector<rclcpp::Parameter> &
    parameter_overrides,                ///< 参数覆盖集合 (Collection of parameter overrides)
  const rcl_arguments_t * local_args,   ///< 本地参数 (Local arguments)
  const rcl_arguments_t * global_args); ///< 全局参数 (Global arguments)

} // namespace detail
} // namespace rclcpp

#endif // RCLCPP__DETAIL__RESOLVE_PARAMETER_OVERRIDES_HPP_

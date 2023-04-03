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

#include "resolve_parameter_overrides.hpp"

#include <map>
#include <string>
#include <vector>

#include "rcl_yaml_param_parser/parser.h"
#include "rclcpp/parameter_map.hpp"
#include "rcpputils/scope_exit.hpp"

/**
 * @brief 解析参数覆盖值 (Resolve parameter overrides)
 *
 * @param node_fqn 完全限定的节点名称 (Fully qualified node name)
 * @param parameter_overrides 构造函数传递的参数覆盖列表 (List of parameter overrides passed to the
 * constructor)
 * @param local_args 与节点相关的局部参数 (Local arguments related to the node)
 * @param global_args 全局参数 (Global arguments)
 * @return std::map<std::string, rclcpp::ParameterValue> 返回解析后的参数覆盖值映射 (Returns the
 * resolved parameter override map)
 */
std::map<std::string, rclcpp::ParameterValue> rclcpp::detail::resolve_parameter_overrides(
    const std::string &node_fqn,
    const std::vector<rclcpp::Parameter> &parameter_overrides,
    const rcl_arguments_t *local_args,
    const rcl_arguments_t *global_args) {
  // 初始化结果映射 (Initialize the result map)
  std::map<std::string, rclcpp::ParameterValue> result;

  // 先处理全局参数，再处理局部参数，这样局部参数可以覆盖全局参数 (Process global before local so
  // that local overwrites global)
  std::array<const rcl_arguments_t *, 2> argument_sources = {global_args, local_args};

  // 获取重新映射后的完全限定节点名称，以便在 yaml 文件中找到节点的参数 (Get fully qualified node
  // name post-remapping to use to find node's params in yaml files)

  // 遍历参数源 (Iterate through the argument sources)
  for (const rcl_arguments_t *source : argument_sources) {
    if (!source) {
      continue;
    }
    rcl_params_t *params = NULL;
    rcl_ret_t ret = rcl_arguments_get_param_overrides(source, &params);
    if (RCL_RET_OK != ret) {
      // 从 RCL 错误中抛出异常 (Throw exception from RCL error)
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
    if (params) {
      // 清理参数资源 (Clean up parameter resources)
      auto cleanup_params =
          rcpputils::make_scope_exit([params]() { rcl_yaml_node_struct_fini(params); });
      // 从参数源获取参数映射 (Get parameter map from the parameter source)
      rclcpp::ParameterMap initial_map = rclcpp::parameter_map_from(params, node_fqn.c_str());

      // 如果参数映射包含节点名称 (If the parameter map contains the node name)
      if (initial_map.count(node_fqn) > 0) {
        // 合并参数 yaml 文件，覆盖旧值 (Combine parameter yaml files, overwriting values in older
        // ones)
        for (const rclcpp::Parameter &param : initial_map.at(node_fqn)) {
          result[param.get_name()] = rclcpp::ParameterValue(param.get_value_message());
        }
      }
    }
  }

  // 构造函数传递的参数覆盖将覆盖来自 yaml 文件源的覆盖 (Parameter overrides passed to constructor
  // will overwrite overrides from yaml file sources)
  for (auto &param : parameter_overrides) {
    result[param.get_name()] = rclcpp::ParameterValue(param.get_value_message());
  }
  return result;
}

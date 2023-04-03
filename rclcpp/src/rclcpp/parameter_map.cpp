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

#include "rclcpp/parameter_map.hpp"

#include <regex>
#include <string>
#include <vector>

#include "rcpputils/find_and_replace.hpp"
#include "rcpputils/scope_exit.hpp"

using rclcpp::ParameterMap;
using rclcpp::ParameterValue;
using rclcpp::exceptions::InvalidParametersException;
using rclcpp::exceptions::InvalidParameterValueException;

/**
 * @brief 判断节点名称是否匹配 (Check if the node name matches)
 *
 * @param node_name 节点名称 (Node name)
 * @param node_fqn 完全限定节点名称 (Fully qualified node name)
 * @return 如果节点名称匹配，则返回 true，否则返回 false (Return true if the node names match,
 * otherwise return false)
 */
static bool is_node_name_matched(const std::string& node_name, const char* node_fqn) {
  // 更新正则表达式 ["/*" -> "(/\w+)" and "/**" -> "(/\w+)*"] (Update the regular expression)
  std::string regex = rcpputils::find_and_replace(node_name, "/*", "(/\w+)");
  return std::regex_match(node_fqn, std::regex(regex));
}

/**
 * @brief 将 rcl_params_t 结构体转换为参数映射 (Convert rcl_params_t structure to a parameter map)
 *
 * @param c_params 指向 rcl_params_t 结构体的指针 (Pointer to the rcl_params_t structure)
 * @param node_fqn 完全限定节点名称 (Fully qualified node name)
 * @return 参数映射 (Parameter map)
 */
ParameterMap rclcpp::parameter_map_from(const rcl_params_t* const c_params, const char* node_fqn) {
  // 检查输入参数是否为空 (Check if input parameters are NULL)
  if (NULL == c_params) {
    throw InvalidParametersException("parameters struct is NULL");
  } else if (NULL == c_params->node_names) {
    throw InvalidParametersException("node names array is NULL");
  } else if (NULL == c_params->params) {
    throw InvalidParametersException("node params array is NULL");
  }

  // 将 C 结构体转换为参数列表 (Convert C structs into a list of parameters to set)
  ParameterMap parameters;
  for (size_t n = 0; n < c_params->num_nodes; ++n) {
    const char* c_node_name = c_params->node_names[n];
    if (NULL == c_node_name) {
      throw InvalidParametersException("Node name at index " + std::to_string(n) + " is NULL");
    }

    /// 确保完全限定节点名称前面有一个斜杠 (Make sure there is a leading slash on the fully
    /// qualified node name)
    std::string node_name("/");
    if ('/' != c_node_name[0]) {
      node_name += c_node_name;
    } else {
      node_name = c_node_name;
    }

    // 如果提供了 node_fqn，则检查节点名称是否匹配 (If node_fqn is provided, check if the node name
    // matches)
    if (node_fqn) {
      if (!is_node_name_matched(node_name, node_fqn)) {
        // 用户只关心 node_fqn，无需解析项目 (No need to parse the items because the user just cares
        // about node_fqn)
        continue;
      }

      node_name = node_fqn;
    }

    const rcl_node_params_t* const c_params_node = &(c_params->params[n]);

    std::vector<Parameter>& params_node = parameters[node_name];
    params_node.reserve(c_params_node->num_params);

    // 遍历参数并将其添加到参数映射中 (Iterate through the parameters and add them to the parameter
    // map)
    for (size_t p = 0; p < c_params_node->num_params; ++p) {
      const char* const c_param_name = c_params_node->parameter_names[p];
      if (NULL == c_param_name) {
        std::string message(
            "At node " + std::to_string(n) + " parameter " + std::to_string(p) + " name is NULL");
        throw InvalidParametersException(message);
      }
      const rcl_variant_t* const c_param_value = &(c_params_node->parameter_values[p]);
      params_node.emplace_back(c_param_name, parameter_value_from(c_param_value));
    }
  }

  return parameters;
}

/**
 * @brief 将 rcl_variant_t 类型转换为 ParameterValue 类型 (Converts an rcl_variant_t type to a
 * ParameterValue type)
 *
 * @param c_param_value 指向 rcl_variant_t 类型的指针 (Pointer to an rcl_variant_t type)
 * @return ParameterValue 返回转换后的 ParameterValue 类型 (Returns the converted ParameterValue
 * type)
 * @throws InvalidParameterValueException 当传入参数为 NULL 或没有设置参数值时抛出异常 (Throws an
 * exception when the passed argument is NULL or no parameter value is set)
 */
ParameterValue rclcpp::parameter_value_from(const rcl_variant_t* const c_param_value) {
  // 判断传入的参数是否为 NULL (Check if the passed argument is NULL)
  // 如果为 NULL，抛出 InvalidParameterValueException 异常 (If it's NULL, throw an
  // InvalidParameterValueException)
  if (NULL == c_param_value) {
    throw InvalidParameterValueException("Passed argument is NULL");
  }

  // 判断参数类型并进行相应的转换 (Determine the parameter type and perform the corresponding
  // conversion)
  if (c_param_value->bool_value) {            // 布尔值 (Boolean value)
    return ParameterValue(*(c_param_value->bool_value));
  } else if (c_param_value->integer_value) {  // 整数值 (Integer value)
    return ParameterValue(*(c_param_value->integer_value));
  } else if (c_param_value->double_value) {  // 双精度浮点值 (Double-precision floating-point value)
    return ParameterValue(*(c_param_value->double_value));
  } else if (c_param_value->string_value) {      // 字符串值 (String value)
    return ParameterValue(std::string(c_param_value->string_value));
  } else if (c_param_value->byte_array_value) {  // 字节数组 (Byte array)
    const rcl_byte_array_t* const byte_array = c_param_value->byte_array_value;
    std::vector<uint8_t> bytes;
    bytes.reserve(byte_array->size);
    for (size_t v = 0; v < byte_array->size; ++v) {
      bytes.push_back(byte_array->values[v]);
    }
    return ParameterValue(bytes);
  } else if (c_param_value->bool_array_value) {  // 布尔值数组 (Boolean value array)
    const rcl_bool_array_t* const bool_array = c_param_value->bool_array_value;
    std::vector<bool> bools;
    bools.reserve(bool_array->size);
    for (size_t v = 0; v < bool_array->size; ++v) {
      bools.push_back(bool_array->values[v]);
    }
    return ParameterValue(bools);
  } else if (c_param_value->integer_array_value) {  // 整数数组 (Integer array)
    const rcl_int64_array_t* const int_array = c_param_value->integer_array_value;
    std::vector<int64_t> integers;
    integers.reserve(int_array->size);
    for (size_t v = 0; v < int_array->size; ++v) {
      integers.push_back(int_array->values[v]);
    }
    return ParameterValue(integers);
  } else if (c_param_value
                 ->double_array_value) {  // 双精度浮点数组 (Double-precision floating-point array)
    const rcl_double_array_t* const double_array = c_param_value->double_array_value;
    std::vector<double> doubles;
    doubles.reserve(double_array->size);
    for (size_t v = 0; v < double_array->size; ++v) {
      doubles.push_back(double_array->values[v]);
    }
    return ParameterValue(doubles);
  } else if (c_param_value->string_array_value) {  // 字符串数组 (String array)
    const rcutils_string_array_t* const string_array = c_param_value->string_array_value;
    std::vector<std::string> strings;
    strings.reserve(string_array->size);
    for (size_t v = 0; v < string_array->size; ++v) {
      strings.emplace_back(string_array->data[v]);
    }
    return ParameterValue(strings);
  }

  // 如果没有设置参数值，抛出 InvalidParameterValueException 异常 (If no parameter value is set,
  // throw an InvalidParameterValueException)
  throw InvalidParameterValueException("No parameter value set");
}

/**
 * @brief 从 YAML 文件中获取参数映射（ParameterMap）
 * @param yaml_filename YAML 文件名
 * @param node_fqn 节点的完全限定名称（Fully Qualified Name）
 * @return 参数映射（ParameterMap）
 *
 * @brief Obtains ParameterMap from a YAML file
 * @param yaml_filename The YAML filename
 * @param node_fqn The fully qualified name of the node
 * @return The ParameterMap
 */
ParameterMap rclcpp::parameter_map_from_yaml_file(
    const std::string& yaml_filename, const char* node_fqn) {
  // 获取默认分配器
  // Get the default allocator
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  // 初始化 rcl 参数结构体
  // Initialize the rcl parameters structure
  rcl_params_t* rcl_parameters = rcl_yaml_node_struct_init(allocator);

  // 在作用域退出时，清理 rcl 参数结构体
  // Clean up the rcl parameters structure on scope exit
  RCPPUTILS_SCOPE_EXIT(rcl_yaml_node_struct_fini(rcl_parameters););

  // 将字符串转换为 C 风格字符串
  // Convert the string to a C-style string
  const char* path = yaml_filename.c_str();

  // 解析 YAML 文件并填充 rcl 参数结构体
  // Parse the YAML file and populate the rcl parameters structure
  if (!rcl_parse_yaml_file(path, rcl_parameters)) {
    rclcpp::exceptions::throw_from_rcl_error(RCL_RET_ERROR);
  }

  // 从 rcl 参数结构体创建参数映射
  // Create a parameter map from the rcl parameters structure
  return rclcpp::parameter_map_from(rcl_parameters, node_fqn);
}

/**
 * @brief 从参数映射（ParameterMap）中获取指定节点的参数
 * @param parameter_map 参数映射
 * @param node_fqn 节点的完全限定名称（Fully Qualified Name）
 * @return 包含指定节点参数的向量
 *
 * @brief Get parameters of the specified node from a ParameterMap
 * @param parameter_map The ParameterMap
 * @param node_fqn The fully qualified name of the node
 * @return A vector containing the parameters of the specified node
 */
std::vector<rclcpp::Parameter> rclcpp::parameters_from_map(
    const ParameterMap& parameter_map, const char* node_fqn) {
  // 创建一个参数向量用于存储结果
  // Create a vector of parameters to store the result
  std::vector<rclcpp::Parameter> parameters;

  // 保存上一个节点名称
  // Save the previous node name
  std::string node_name_old;

  // 遍历参数映射
  // Iterate through the parameter map
  for (auto& [node_name, node_parameters] : parameter_map) {
    // 如果提供了节点的完全限定名称，并且与当前节点名称不匹配，则跳过该节点
    // If a fully qualified node name is provided and does not match the current node name, skip the
    // node
    if (node_fqn && !is_node_name_matched(node_name, node_fqn)) {
      // 用户只关心 node_fqn，因此无需解析项
      // No need to parse the items because the user just cares about node_fqn
      continue;
    }
    // 将当前节点的参数插入到结果参数向量中
    // Insert the parameters of the current node into the result parameter vector
    parameters.insert(parameters.end(), node_parameters.begin(), node_parameters.end());
  }

  // 返回包含指定节点参数的向量
  // Return the vector containing the parameters of the specified node
  return parameters;
}

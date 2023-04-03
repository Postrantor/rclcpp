// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/expand_topic_or_service_name.hpp"

#include <string>

#include "rcl/expand_topic_name.h"
#include "rcl/validate_topic_name.h"
#include "rclcpp/exceptions.hpp"
#include "rcutils/logging_macros.h"
#include "rcutils/types/string_map.h"
#include "rmw/error_handling.h"
#include "rmw/validate_full_topic_name.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

using rclcpp::exceptions::throw_from_rcl_error;

/*!
 * \brief 展开给定的主题或服务名称。
 *        Expand the given topic or service name.
 *
 * \param[in] name 要展开的主题或服务名称。The topic or service name to be expanded.
 * \param[in] node_name 用于替换占位符的节点名称。Node name used for substitution of placeholders.
 * \param[in] namespace_ 用于替换占位符的命名空间。Namespace used for substitution of placeholders.
 * \param[in] is_service 是否是服务名称。True if it's a service name, false if it's a topic name.
 * \return std::string 返回展开后的主题或服务名称。Expanded topic or service name as a string.
 */
std::string rclcpp::expand_topic_or_service_name(
    const std::string& name,
    const std::string& node_name,
    const std::string& namespace_,
    bool is_service) {
  // 定义一个 char 指针，用于存储展开后的主题名称。Define a char pointer to store the expanded topic
  // name.
  char* expanded_topic = nullptr;
  // 获取默认的内存分配器。Get the default memory allocator.
  rcl_allocator_t allocator = rcl_get_default_allocator();
  // 获取默认的 rcutils 内存分配器。Get the default rcutils memory allocator.
  rcutils_allocator_t rcutils_allocator = rcutils_get_default_allocator();
  // 创建一个字符串映射表，用于存储主题名称的替换规则。Create a string map to store the topic name
  // substitution rules.
  rcutils_string_map_t substitutions_map = rcutils_get_zero_initialized_string_map();

  // 初始化字符串映射表。Initialize the string map.
  rcutils_ret_t rcutils_ret = rcutils_string_map_init(&substitutions_map, 0, rcutils_allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    if (rcutils_ret == RCUTILS_RET_BAD_ALLOC) {
      throw_from_rcl_error(RCL_RET_BAD_ALLOC, "", rcutils_get_error_state(), rcutils_reset_error);
    } else {
      throw_from_rcl_error(RCL_RET_ERROR, "", rcutils_get_error_state(), rcutils_reset_error);
    }
  }
  // 获取默认的主题名称替换规则。Get the default topic name substitution rules.
  rcl_ret_t ret = rcl_get_default_topic_name_substitutions(&substitutions_map);
  if (ret != RCL_RET_OK) {
    const rcutils_error_state_t* error_state = rcl_get_error_state();
    // 在抛出异常之前，完成字符串映射表的清理工作。Finalize the string map before throwing an
    // exception.
    rcutils_ret = rcutils_string_map_fini(&substitutions_map);
    if (rcutils_ret != RCUTILS_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
          "rclcpp", "failed to fini string_map (%d) during error handling: %s", rcutils_ret,
          rcutils_get_error_string().str);
      rcutils_reset_error();
    }
    throw_from_rcl_error(ret, "", error_state);
  }

  // 展开主题名称。Expand the topic name.
  ret = rcl_expand_topic_name(
      name.c_str(), node_name.c_str(), namespace_.c_str(), &substitutions_map, allocator,
      &expanded_topic);

  // 创建一个字符串用于存储结果。Create a string to store the result.
  std::string result;
  if (ret == RCL_RET_OK) {
    result = expanded_topic;
    allocator.deallocate(expanded_topic, allocator.state);
  }

  // 清理字符串映射表。Clean up the string map.
  rcutils_ret = rcutils_string_map_fini(&substitutions_map);
  if (rcutils_ret != RCUTILS_RET_OK) {
    throw_from_rcl_error(RCL_RET_ERROR, "", rcutils_get_error_state(), rcutils_reset_error);
  }

  // 如果展开失败。If expansion failed.
  if (ret != RCL_RET_OK) {
    // 如果主题名称无效或未知替换。If invalid topic name or unknown substitution.
    if (ret == RCL_RET_TOPIC_NAME_INVALID || ret == RCL_RET_UNKNOWN_SUBSTITUTION) {
      rcl_reset_error();  // 显式丢弃 rcl_expand_topic_name() 的错误。Explicitly discard error from
                          // rcl_expand_topic_name().
      int validation_result;
      size_t invalid_index;
      rcl_ret_t ret = rcl_validate_topic_name(name.c_str(), &validation_result, &invalid_index);
      if (ret != RCL_RET_OK) {
        throw_from_rcl_error(ret);
      }

      // 对 ROS2 项目中 rclcpp 相关的代码进行参数列表说明和详细注释
      if (validation_result != RCL_TOPIC_NAME_VALID) {
        // 获取验证结果对应的字符串消息
        // Get the string message corresponding to the validation result
        const char* validation_message = rcl_topic_name_validation_result_string(validation_result);

        // 判断是否为服务，分别处理服务名称和主题名称的错误
        // Determine whether it is a service, and handle errors of service names and topic names
        // separately
        if (is_service) {
          // 使用 rclcpp::exceptions::InvalidServiceNameError 异常类
          // Use the rclcpp::exceptions::InvalidServiceNameError exception class
          using rclcpp::exceptions::InvalidServiceNameError;

          // 抛出无效服务名称异常，包括服务名称、验证消息和无效字符索引
          // Throw an invalid service name exception, including service name, validation message,
          // and invalid character index
          throw InvalidServiceNameError(name.c_str(), validation_message, invalid_index);
        } else {
          // 使用 rclcpp::exceptions::InvalidTopicNameError 异常类
          // Use the rclcpp::exceptions::InvalidTopicNameError exception class
          using rclcpp::exceptions::InvalidTopicNameError;

          // 抛出无效主题名称异常，包括主题名称、验证消息和无效字符索引
          // Throw an invalid topic name exception, including topic name, validation message, and
          // invalid character index
          throw InvalidTopicNameError(name.c_str(), validation_message, invalid_index);
        }
      } else {
        // 当主题名称意外有效时，抛出运行时错误
        // Throw a runtime error when the topic name is unexpectedly valid
        throw std::runtime_error("topic name unexpectedly valid");
      }
      // 如果节点名称无效。If invalid node name.
    } else if (ret == RCL_RET_NODE_INVALID_NAME) {
      rcl_reset_error();  // 显式丢弃 rcl_expand_topic_name() 的错误。Explicitly discard error from
                          // rcl_expand_topic_name().
      int validation_result;
      size_t invalid_index;
      rmw_ret_t rmw_ret =
          rmw_validate_node_name(node_name.c_str(), &validation_result, &invalid_index);
      if (rmw_ret != RMW_RET_OK) {
        if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
          throw_from_rcl_error(
              RCL_RET_INVALID_ARGUMENT, "failed to validate node name", rmw_get_error_state(),
              rmw_reset_error);
        }
        throw_from_rcl_error(
            RCL_RET_ERROR, "failed to validate node name", rmw_get_error_state(), rmw_reset_error);
      }

      if (validation_result != RMW_NODE_NAME_VALID) {
        throw rclcpp::exceptions::InvalidNodeNameError(
            node_name.c_str(), rmw_node_name_validation_result_string(validation_result),
            invalid_index);
      } else {
        throw std::runtime_error("invalid rcl node name but valid rmw node name");
      }

      // 如果命名空间无效。If invalid namespace.
    } else if (ret == RCL_RET_NODE_INVALID_NAMESPACE) {
      rcl_reset_error();  // 显式丢弃 rcl_expand_topic_name() 的错误。Explicitly discard error from
                          // rcl_expand_topic_name().
      int validation_result;
      size_t invalid_index;
      rmw_ret_t rmw_ret =
          rmw_validate_namespace(namespace_.c_str(), &validation_result, &invalid_index);
      if (rmw_ret != RMW_RET_OK) {
        if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
          throw_from_rcl_error(
              RCL_RET_INVALID_ARGUMENT, "failed to validate namespace", rmw_get_error_state(),
              rmw_reset_error);
        }
        throw_from_rcl_error(
            RCL_RET_ERROR, "failed to validate namespace", rmw_get_error_state(), rmw_reset_error);
      }

      if (validation_result != RMW_NAMESPACE_VALID) {
        throw rclcpp::exceptions::InvalidNamespaceError(
            namespace_.c_str(), rmw_namespace_validation_result_string(validation_result),
            invalid_index);
      } else {
        throw std::runtime_error("invalid rcl namespace but valid rmw namespace");
      }
      // 发生其他问题。Something else happened.
    } else {
      throw_from_rcl_error(ret);
    }
  }

  // 展开成功，但完整名称验证可能仍然失败。Expansion succeeded, but full name validation may still
  // fail.
  int validation_result;
  size_t invalid_index;
  rmw_ret_t rmw_ret =
      rmw_validate_full_topic_name(result.c_str(), &validation_result, &invalid_index);
  if (rmw_ret != RMW_RET_OK) {
    if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
      throw_from_rcl_error(
          RCL_RET_INVALID_ARGUMENT, "failed to validate full topic name", rmw_get_error_state(),
          rmw_reset_error);
    }
    throw_from_rcl_error(
        RCL_RET_ERROR, "failed to validate full topic name", rmw_get_error_state(),
        rmw_reset_error);
  }

  if (validation_result != RMW_TOPIC_VALID) {
    if (is_service) {
      throw rclcpp::exceptions::InvalidServiceNameError(
          result.c_str(), rmw_full_topic_name_validation_result_string(validation_result),
          invalid_index);
    } else {
      throw rclcpp::exceptions::InvalidTopicNameError(
          result.c_str(), rmw_full_topic_name_validation_result_string(validation_result),
          invalid_index);
    }
  }

  // 返回展开后的主题或服务名称。Return the expanded topic or service name.
  return result;
}

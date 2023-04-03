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

#ifndef RCLCPP__EXPAND_TOPIC_OR_SERVICE_NAME_HPP_
#define RCLCPP__EXPAND_TOPIC_OR_SERVICE_NAME_HPP_

#include <string>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/// 扩展一个主题或服务名称并在无效时抛出异常。
/// Expand a topic or service name and throw if it is not valid.
/**
 * 该函数也可以用于“仅”验证主题或服务名称，
 * 因为扩展主题名称是完全验证名称所必需的。
 * This function can be used to "just" validate a topic or service name too,
 * since expanding the topic name is required to fully validate a name.
 *
 * 如果名称无效，则抛出 InvalidTopicNameError 异常，如果 is_service 为 true，则抛出
 * InvalidServiceNameError 异常。 If the name is invalid, then InvalidTopicNameError is thrown or
 * InvalidServiceNameError if is_service is true.
 *
 * 该函数可以采用任何形式的主题或服务名称，即它不必是一个完全限定的名称。
 * 节点名称和命名空间用于在验证时进行扩展（如果需要）。
 * This function can take any form of a topic or service name, i.e. it does not
 * have to be a fully qualified name.
 * The node name and namespace are used to expand it if necessary while
 * validating it.
 *
 * 使用 rcl_expand_topic_name 进行扩展。
 * 使用 rcl_validate_topic_name 和 rmw_validate_full_topic_name
 * 进行验证，因此可以在这些函数的文档中找到有关失败的详细信息。 Expansion is done with
 * rcl_expand_topic_name. The validation is done with rcl_validate_topic_name and
 * rmw_validate_full_topic_name, so details about failures can be found in the
 * documentation for those functions.
 *
 * \param name 要验证的主题或服务名称
 * \param name the topic or service name to be validated
 * \param node_name 与名称关联的节点名称
 * \param node_name the name of the node associated with the name
 * \param namespace_ 与名称关联的节点的命名空间
 * \param namespace_ the namespace of the node associated with the name
 * \param is_service 如果为 true，则抛出 InvalidServiceNameError 而不是 InvalidTopicNameError
 * \param is_service if true InvalidServiceNameError is thrown instead
 * \returns 扩展（和验证）的主题名称
 * \returns expanded (and validated) topic name
 * \throws InvalidTopicNameError 如果名称无效且 is_service 为 false
 * \throws InvalidTopicNameError if name is invalid and is_service is false
 * \throws InvalidServiceNameError 如果名称无效且 is_service 为 true
 * \throws InvalidServiceNameError if name is invalid and is_service is true
 * \throws std::bad_alloc 如果无法分配内存
 * \throws std::bad_alloc if memory cannot be allocated
 * \throws RCLError 如果发生意外错误
 * \throws RCLError if an unexpected error occurs
 * \throws std::runtime_error 如果主题名称意外有效，或者 rcl 名称无效，或者 rcl 命名空间无效
 * \throws std::runtime_error if the topic name is unexpectedly valid or,
 *    if the rcl name is invalid or if the rcl namespace is invalid
 */
RCLCPP_PUBLIC
std::string expand_topic_or_service_name(
    const std::string& name,
    const std::string& node_name,
    const std::string& namespace_,
    bool is_service = false);

}  // namespace rclcpp

#endif  // RCLCPP__EXPAND_TOPIC_OR_SERVICE_NAME_HPP_

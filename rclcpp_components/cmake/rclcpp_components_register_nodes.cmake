# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

#
# Register an rclcpp component with the ament resource index.
#
# The passed library can contain multiple nodes each registered via macro.
#
# :param target: the shared library target :type target: string :param ARGN: the
# unique plugin names being exported using class_loader :type ARGN: list of
# strings :param RESOURCE_INDEX: the ament resource index to register the
# components :type RESOURCE_INDEX: string
#
# @brief 宏 rclcpp_components_register_nodes 用于注册节点插件 (Macro
# rclcpp_components_register_nodes for registering node plugins) @param target
# 要注册的目标 (The target to register)
macro(rclcpp_components_register_nodes target)

  # 检查目标是否存在 (Check if the target exists)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "rclcpp_components_register_nodes() first argument "
                        "'${target}' is not a target")
  endif()

  # 解析参数 (Parse arguments)
  cmake_parse_arguments(ARGS "" "RESOURCE_INDEX" "" ${ARGN})

  # 如果未指定，默认为 rclcpp_components (Default to rclcpp_components if not specified
  # otherwise)
  set(resource_index "rclcpp_components")
  if(NOT "${ARGS_RESOURCE_INDEX}" STREQUAL "")
    set(resource_index ${ARGS_RESOURCE_INDEX})
    message(
      STATUS
        "Setting component resource index to non-default value ${resource_index}"
    )
  endif()

  # 检查目标类型是否为共享库 (Check if the target type is a shared library)
  get_target_property(_target_type ${target} TYPE)
  if(NOT _target_type STREQUAL "SHARED_LIBRARY")
    message(FATAL_ERROR "rclcpp_components_register_nodes() first argument "
                        "'${target}' is not a shared library target")
  endif()

  # 如果有额外的参数 (If there are additional arguments)
  if(${ARGC} GREATER 0)
    _rclcpp_components_register_package_hook()
    set(_unique_names)
    foreach(_arg ${ARGS_UNPARSED_ARGUMENTS})

      # 检查插件名称是否唯一 (Check if the plugin names are unique)
      if(_arg IN_LIST _unique_names)
        message(
          FATAL_ERROR "rclcpp_components_register_nodes() the plugin names "
                      "must be unique (multiple '${_arg}')")
      endif()
      list(APPEND _unique_names "${_arg}")

      # 设置插件路径 (Set the plugin path)
      if(WIN32)
        set(_path "bin")
      else()
        set(_path "lib")
      endif()

      # 将插件添加到资源列表中 (Add the plugin to the resource list)
      set(_RCLCPP_COMPONENTS_${resource_index}__NODES
          "${_RCLCPP_COMPONENTS_${resource_index}__NODES}${_arg};${_path}/$<TARGET_FILE_NAME:${target}>\n"
      )

      # 将资源索引添加到资源索引列表中 (Add the resource index to the resource indices list)
      list(APPEND _RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES ${resource_index})
    endforeach()
  endif()
endmacro()

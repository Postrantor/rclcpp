# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Register an rclcpp component with the ament
# resource index and create an executable.
#
# usage: rclcpp_components_register_node(
#        <target> PLUGIN <component> EXECUTABLE <node>)
#
# :param target: the shared library target
# :type target: string
# :param PLUGIN: the plugin name
# :type PLUGIN: string
# :param EXECUTABLE: the node's executable name
# :type EXECUTABLE: string
# :param RESOURCE_INDEX: the ament resource index to register the components
# :type RESOURCE_INDEX: string

## @brief Macro to register an rclcpp component node.
## @param target The target library name
## @param PLUGIN The component plugin name
## @param EXECUTABLE The executable file name
## @param EXECUTOR The executor type, default is "SingleThreadedExecutor"
## @param RESOURCE_INDEX The resource index, default is "rclcpp_components"
macro(rclcpp_components_register_node target)
  # Parse arguments
  cmake_parse_arguments(ARGS "" "PLUGIN;EXECUTABLE;EXECUTOR;RESOURCE_INDEX" "" ${ARGN})

  # Check for unused arguments
  if(ARGS_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "rclcpp_components_register_node() called with unused "
      "arguments: ${ARGS_UNPARSED_ARGUMENTS}")
  endif()

  # Check if PLUGIN argument exists
  if("${ARGS_PLUGIN}" STREQUAL "")
    message(FATAL_ERROR "rclcpp_components_register_node macro requires a PLUGIN argument for target ${target}")
  endif()

  # Check if EXECUTABLE argument exists
  if("${ARGS_EXECUTABLE}" STREQUAL "")
    message(FATAL_ERROR "rclcpp_components_register_node macro requires a EXECUTABLE argument for target ${target}")
  endif()

  # Set the resource index to rclcpp_components by default if not specified otherwise
  set(resource_index "rclcpp_components")
  if(NOT "${ARGS_RESOURCE_INDEX}" STREQUAL "")
    set(resource_index ${ARGS_RESOURCE_INDEX})
    message(STATUS "Setting component resource index to non-default value ${resource_index}")
  endif()

  # Set the executor to SingleThreadedExecutor by default if not specified otherwise
  set(executor "SingleThreadedExecutor")
  if(NOT "${ARGS_EXECUTOR}" STREQUAL "")
    set(executor ${ARGS_EXECUTOR})
    message(STATUS "Setting executor non-default value ${executor}")
  endif()

  # Set the component and node variables
  set(component ${ARGS_PLUGIN})
  set(node ${ARGS_EXECUTABLE})

  # Register package hook
  _rclcpp_components_register_package_hook()

  # Set library path
  set(_path "lib")
  set(library_name "$<TARGET_FILE_NAME:${target}>")
  if(WIN32)
    set(_path "bin")
  endif()

  # Add component node to resource index list
  set(_RCLCPP_COMPONENTS_${resource_index}__NODES
    "${_RCLCPP_COMPONENTS_${resource_index}__NODES}${component};${_path}/$<TARGET_FILE_NAME:${target}>\n")
  list(APPEND _RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES ${resource_index})

  # Configure file
  configure_file(${rclcpp_components_NODE_TEMPLATE}
    ${PROJECT_BINARY_DIR}/rclcpp_components/node_main_configured_${node}.cpp.in)

  # Generate source file
  file(GENERATE OUTPUT ${PROJECT_BINARY_DIR}/rclcpp_components/node_main_${node}.cpp
    INPUT ${PROJECT_BINARY_DIR}/rclcpp_components/node_main_configured_${node}.cpp.in)

  # Add executable
  add_executable(${node} ${PROJECT_BINARY_DIR}/rclcpp_components/node_main_${node}.cpp)

  # Add dependencies
  ament_target_dependencies(${node}
    "rclcpp"
    "class_loader"
    "rclcpp_components")

  # Install target
  install(TARGETS
    ${node}
    DESTINATION lib/${PROJECT_NAME})
endmacro()

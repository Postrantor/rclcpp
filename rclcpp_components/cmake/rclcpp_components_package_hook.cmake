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

#/**
# * @brief 注册节点插件 (Register node plugins)
# *
# * @param[in] _RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES 节点插件的资源索引列表，其中重复项将被删除 (List of resource indices for node plugins, where duplicates will be removed)
# */

#// 删除 _RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES 中的重复项
#// Remove duplicates from _RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES
list(REMOVE_DUPLICATES _RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES)

#// 遍历 _RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES 中的每个资源索引
#// Iterate through each resource index in _RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES
foreach(resource_index ${_RCLCPP_COMPONENTS_PACKAGE_RESOURCE_INDICES})

  #/**
  # * @brief 使用 ament_index_register_resource 函数注册节点插件资源
  # *        (Register node plugin resources using the ament_index_register_resource function)
  # *
  # * @param[in] resource_index 当前遍历到的资源索引 (The currently iterated resource index)
  # * @param[in] CONTENT 节点插件内容，存储在 _RCLCPP_COMPONENTS_${resource_index}__NODES 变量中
  # *                    (Node plugin content, stored in the _RCLCPP_COMPONENTS_${resource_index}__NODES variable)
  # */
  ament_index_register_resource(
    ${resource_index} CONTENT "${_RCLCPP_COMPONENTS_${resource_index}__NODES}")

#// 结束 foreach 循环
#// End the foreach loop
endforeach()

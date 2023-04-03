// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/detail/utilities.hpp"

#include <cassert>
#include <string>
#include <utility>
#include <vector>

#include "rcl/allocator.h"
#include "rcl/arguments.h"
#include "rclcpp/exceptions.hpp"

namespace rclcpp {
namespace detail {

/**
 * @brief 获取未解析的ROS参数 (Get the unparsed ROS arguments)
 *
 * @param argc 参数数量 (Number of arguments)
 * @param argv 参数值 (Argument values)
 * @param arguments 已经解析的参数结构体 (Parsed argument structure)
 * @param allocator 内存分配器 (Memory allocator)
 * @return std::vector<std::string> 未解析的ROS参数列表 (List of unparsed ROS arguments)
 */
std::vector<std::string> get_unparsed_ros_arguments(
    int argc, char const* const* argv, rcl_arguments_t* arguments, rcl_allocator_t allocator) {
  // 忽略argc，避免编译警告 (Ignore argc to avoid compiler warnings)
  (void)argc;

  // 存储未解析的ROS参数 (Store unparsed ROS arguments)
  std::vector<std::string> unparsed_ros_arguments;

  // 获取未解析的ROS参数数量 (Get the count of unparsed ROS arguments)
  int unparsed_ros_args_count = rcl_arguments_get_count_unparsed_ros(arguments);

  // 如果有未解析的ROS参数 (If there are any unparsed ROS arguments)
  if (unparsed_ros_args_count > 0) {
    // 用于存储未解析参数索引的指针 (Pointer to store indices of unparsed arguments)
    int* unparsed_ros_args_indices = nullptr;

    // 获取未解析的ROS参数索引 (Get the indices of unparsed ROS arguments)
    rcl_ret_t ret =
        rcl_arguments_get_unparsed_ros(arguments, allocator, &unparsed_ros_args_indices);

    // 如果获取未解析参数索引失败，则抛出异常 (If getting the indices of unparsed arguments fails,
    // throw an exception)
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to get unparsed ROS arguments");
    }

    // 尝试将未解析的参数添加到结果向量中 (Try adding the unparsed arguments to the result vector)
    try {
      for (int i = 0; i < unparsed_ros_args_count; ++i) {
        // 检查索引是否在有效范围内 (Check if the index is within valid range)
        assert(unparsed_ros_args_indices[i] >= 0);
        assert(unparsed_ros_args_indices[i] < argc);

        // 添加未解析参数到向量中 (Add the unparsed argument to the vector)
        unparsed_ros_arguments.push_back(argv[unparsed_ros_args_indices[i]]);
      }

      // 释放未解析参数索引内存 (Free the memory of the unparsed argument indices)
      allocator.deallocate(unparsed_ros_args_indices, allocator.state);
    } catch (...) {
      // 如果出现异常，释放内存并重新抛出异常 (If an exception occurs, free the memory and rethrow
      // the exception)
      allocator.deallocate(unparsed_ros_args_indices, allocator.state);
      throw;
    }
  }

  // 返回未解析的ROS参数列表 (Return the list of unparsed ROS arguments)
  return unparsed_ros_arguments;
}

}  // namespace detail
}  // namespace rclcpp

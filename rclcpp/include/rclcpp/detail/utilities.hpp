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

#ifndef RCLCPP__DETAIL__UTILITIES_HPP_
#define RCLCPP__DETAIL__UTILITIES_HPP_

#include <string>
#include <vector>

#include "rcl/allocator.h"
#include "rcl/arguments.h"
#include "rclcpp/detail/utilities.hpp"

namespace rclcpp {
namespace detail {

/**
 * \brief 获取未解析的 ROS 参数 (Get unparsed ROS arguments)
 * \param argc 参数个数 (Number of arguments)
 * \param argv 参数字符串数组 (Argument string array)
 * \param arguments 指向 rcl_arguments_t 结构体的指针，用于存储解析后的参数信息 (Pointer to
 * rcl_arguments_t structure for storing parsed argument information) \param allocator
 * 用于分配内存的 rcl_allocator_t 结构体 (rcl_allocator_t structure for memory allocation) \return
 * 返回一个包含未解析参数的 std::vector<std::string> (Returns a std::vector<std::string> containing
 * the unparsed arguments)
 */
std::vector<std::string> get_unparsed_ros_arguments(
    int argc, char const* const* argv, rcl_arguments_t* arguments, rcl_allocator_t allocator);

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__UTILITIES_HPP_

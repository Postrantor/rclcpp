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

#include "rclcpp_action/types.hpp"

#include <sstream>
#include <string>

namespace rclcpp_action {
/**
 * @brief 将 GoalUUID 转换为 std::string 类型 (Converts a GoalUUID to a std::string)
 * @param goal_id 一个 GoalUUID 对象 (A GoalUUID object)
 * @return 返回转换后的 std::string 类型字符串 (Returns the converted std::string)
 */
std::string to_string(const GoalUUID& goal_id) {
  // 创建一个 std::stringstream 对象 (Create a std::stringstream object)
  std::stringstream stream;

  // 设置流格式为十六进制 (Set the stream format to hexadecimal)
  stream << std::hex;

  // 遍历 GoalUUID 中的每个元素 (Iterate through each element in the GoalUUID)
  for (const auto& element : goal_id) {
    // 将元素值转换为整数并添加到流中 (Convert the element value to an integer and add it to the
    // stream)
    stream << static_cast<int>(element);
  }

  // 返回转换后的十六进制字符串 (Return the converted hexadecimal string)
  return stream.str();
}

/**
 * @brief 将 GoalUUID 转换为 rcl_action_goal_info_t 类型 (Converts a GoalUUID to a
 * rcl_action_goal_info_t type)
 * @param goal_id 一个 GoalUUID 对象 (A GoalUUID object)
 * @param info 一个指向 rcl_action_goal_info_t 的指针 (A pointer to an rcl_action_goal_info_t)
 */
void convert(const GoalUUID& goal_id, rcl_action_goal_info_t* info) {
  // 遍历 GoalUUID 中的每个元素 (Iterate through each element in the GoalUUID)
  for (size_t i = 0; i < 16; ++i) {
    // 将 GoalUUID 中的元素值赋给 rcl_action_goal_info_t 的 uuid 数组 (Assign the element value from
    // the GoalUUID to the uuid array in the rcl_action_goal_info_t)
    info->goal_id.uuid[i] = goal_id[i];
  }
}

/**
 * @brief 将 rcl_action_goal_info_t 转换为 GoalUUID 类型 (Converts a rcl_action_goal_info_t to a
 * GoalUUID type)
 * @param info 一个 rcl_action_goal_info_t 对象 (An rcl_action_goal_info_t object)
 * @param goal_id 一个指向 GoalUUID 的指针 (A pointer to a GoalUUID)
 */
void convert(const rcl_action_goal_info_t& info, GoalUUID* goal_id) {
  // 遍历 rcl_action_goal_info_t 中的每个元素 (Iterate through each element in the
  // rcl_action_goal_info_t)
  for (size_t i = 0; i < 16; ++i) {
    // 将 rcl_action_goal_info_t 中的 uuid 数组元素值赋给 GoalUUID (Assign the element value from
    // the uuid array in the rcl_action_goal_info_t to the GoalUUID)
    (*goal_id)[i] = info.goal_id.uuid[i];
  }
}
}  // namespace rclcpp_action

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

#ifndef RCLCPP_ACTION__TYPES_HPP_
#define RCLCPP_ACTION__TYPES_HPP_

#include <array>
#include <climits>
#include <functional>
#include <string>

#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "rcl_action/types.h"
#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action {

// 使用 std::array<uint8_t, UUID_SIZE> 类型表示目标的 UUID（唯一标识符）
using GoalUUID = std::array<uint8_t, UUID_SIZE>;
// 使用 action_msgs::msg::GoalStatus 类型表示目标状态
using GoalStatus = action_msgs::msg::GoalStatus;
// 使用 action_msgs::msg::GoalInfo 类型表示目标信息
using GoalInfo = action_msgs::msg::GoalInfo;

/**
 * @brief 将目标ID转换为易读的字符串 (Convert a goal id to a human readable string)
 *
 * @param[in] goal_id GoalUUID类型的目标ID (GoalUUID type of goal id)
 * @return std::string 转换后的字符串 (The converted string)
 */
RCLCPP_ACTION_PUBLIC
std::string to_string(const GoalUUID& goal_id);

/**
 * @brief 将C++ GoalID转换为rcl_action_goal_info_t (Convert C++ GoalID to rcl_action_goal_info_t)
 *
 * @param[in] goal_id GoalUUID类型的目标ID (GoalUUID type of goal id)
 * @param[out] info 输出的rcl_action_goal_info_t类型指针 (Output pointer of type
 * rcl_action_goal_info_t)
 */
RCLCPP_ACTION_PUBLIC
void convert(const GoalUUID& goal_id, rcl_action_goal_info_t* info);

/**
 * @brief 将rcl_action_goal_info_t转换为C++ GoalID (Convert rcl_action_goal_info_t to C++ GoalID)
 *
 * @param[in] info 输入的rcl_action_goal_info_t类型引用 (Input reference of type
 * rcl_action_goal_info_t)
 * @param[out] goal_id 输出的GoalUUID类型指针 (Output pointer of type GoalUUID)
 */
RCLCPP_ACTION_PUBLIC
void convert(const rcl_action_goal_info_t& info, GoalUUID* goal_id);
}  // namespace rclcpp_action

namespace std {
/**
 * @brief 一个比较器，用于比较两个 rclcpp_action::GoalUUID 对象。
 *        A comparator for comparing two rclcpp_action::GoalUUID objects.
 *
 * @tparam rclcpp_action::GoalUUID 目标 UUID 类型。
 *         The Goal UUID type.
 */
template <>
struct less<rclcpp_action::GoalUUID> {
  /**
   * @brief 比较运算符，用于比较两个 rclcpp_action::GoalUUID 对象的大小。
   *        Comparison operator for comparing the order of two rclcpp_action::GoalUUID objects.
   *
   * @param lhs 左侧操作数（第一个比较对象）。
   *            Left-hand side operand (the first object to compare).
   * @param rhs 右侧操作数（第二个比较对象）。
   *            Right-hand side operand (the second object to compare).
   * @return 如果 lhs 小于 rhs，则返回 true；否则返回 false。
   *         Returns true if lhs is less than rhs, otherwise returns false.
   */
  bool operator()(const rclcpp_action::GoalUUID& lhs, const rclcpp_action::GoalUUID& rhs) const {
    // 使用内置的小于运算符比较两个 GoalUUID 对象，并返回结果。
    // Compare the two GoalUUID objects using the built-in less-than operator and return the result.
    return lhs < rhs;
  }
};

/// \brief Hash a goal id so it can be used as a key in std::unordered_map
/// \brief 对目标 ID 进行哈希处理，以便将其用作 std::unordered_map 中的键
template <>
struct hash<rclcpp_action::GoalUUID> {
  /// \brief The function call operator that computes the hash value for a given GoalUUID.
  /// \brief 计算给定 GoalUUID 的哈希值的函数调用运算符。
  size_t operator()(const rclcpp_action::GoalUUID& uuid) const noexcept {
    // TODO(sloretz) Use someone else's hash function and cite it
    // TODO（sloretz）使用其他人的哈希函数并引用它

    // Initialize the result variable to store the computed hash value
    // 初始化结果变量，以存储计算出的哈希值
    size_t result = 0;

    // Iterate through each element of the GoalUUID
    // 遍历 GoalUUID 的每个元素
    for (size_t i = 0; i < uuid.size(); ++i) {
      // Iterate through each byte of the size_t data type
      // 遍历 size_t 数据类型的每个字节
      for (size_t b = 0; b < sizeof(size_t); ++b) {
        // Get the value of the current element in the GoalUUID, and store it in 'part'
        // 获取 GoalUUID 中当前元素的值，并将其存储在 'part' 中
        size_t part = uuid[i];

        // Shift the bits of 'part' left by CHAR_BIT times the current byte index
        // 将 'part' 的位左移 CHAR_BIT 乘以当前字节索引
        part <<= CHAR_BIT * b;

        // XOR the result with the shifted value of 'part'
        // 将结果与 'part' 的移位值进行异或
        result ^= part;
      }
    }

    // Return the computed hash value
    // 返回计算出的哈希值
    return result;
  }
};
}  // namespace std
#endif  // RCLCPP_ACTION__TYPES_HPP_

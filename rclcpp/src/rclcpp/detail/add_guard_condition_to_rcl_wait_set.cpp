// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"

#include "rclcpp/exceptions.hpp"

namespace rclcpp {
namespace detail {

/**
 * @brief 添加一个守护条件到 rcl_wait_set 中 (Add a guard condition to the rcl_wait_set)
 *
 * @param wait_set rcl_wait_set_t 类型的引用，用于存储等待集合 (Reference to an rcl_wait_set_t,
 * which stores the wait set)
 * @param guard_condition rclcpp::GuardCondition 类型的常量引用，表示要添加的守护条件 (Constant
 * reference to an rclcpp::GuardCondition, representing the guard condition to be added)
 */
void add_guard_condition_to_rcl_wait_set(
    rcl_wait_set_t& wait_set, const rclcpp::GuardCondition& guard_condition) {
  // 获取守护条件的 rcl_guard_condition_t 结构体 (Get the rcl_guard_condition_t structure of the
  // guard condition)
  const auto& gc = guard_condition.get_rcl_guard_condition();

  // 将守护条件添加到 wait_set 中，并检查返回值 (Add the guard condition to the wait_set and check
  // the return value)
  rcl_ret_t ret = rcl_wait_set_add_guard_condition(&wait_set, &gc, NULL);

  // 如果返回值不是 RCL_RET_OK，则抛出异常 (If the return value is not RCL_RET_OK, throw an
  // exception)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to add guard condition to wait set");
  }
}

}  // namespace detail
}  // namespace rclcpp

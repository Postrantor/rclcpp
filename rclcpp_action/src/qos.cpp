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

#include <rcl_action/default_qos.h>
#include <rclcpp_action/qos.hpp>

namespace rclcpp_action {

/**
 * @brief 构造函数，初始化 DefaultActionStatusQoS 对象。 (Constructor, initializes the
 * DefaultActionStatusQoS object.)
 *
 * @param 无参数 (No parameters)
 */
DefaultActionStatusQoS::DefaultActionStatusQoS()
    // 使用 rcl_action_qos_profile_status_default 初始化 QoS 设置。
    // Initialize QoS settings using rcl_action_qos_profile_status_default.
    : rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rcl_action_qos_profile_status_default)) {
  // 将当前对象的 rmw_qos_profile 属性设置为 rcl_action_qos_profile_status_default。
  // Set the rmw_qos_profile attribute of the current object to
  // rcl_action_qos_profile_status_default.
  this->get_rmw_qos_profile() = rcl_action_qos_profile_status_default;
}

}  // namespace rclcpp_action

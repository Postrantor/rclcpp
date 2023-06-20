// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include "rclcpp_lifecycle/managed_entity.hpp"

namespace rclcpp_lifecycle {

/**
 * @brief 激活 SimpleManagedEntity 实例 (Activate the SimpleManagedEntity instance)
 */
void SimpleManagedEntity::on_activate() {
  // 将 activated_ 原子变量设置为 true，表示实例已激活
  // (Set the atomic variable activated_ to true, indicating that the instance is activated)
  activated_.store(true);
}

/**
 * @brief 取消激活 SimpleManagedEntity 实例 (Deactivate the SimpleManagedEntity instance)
 */
void SimpleManagedEntity::on_deactivate() {
  // 将 activated_ 原子变量设置为 false，表示实例已取消激活
  // (Set the atomic variable activated_ to false, indicating that the instance is deactivated)
  activated_.store(false);
}

/**
 * @brief 查询 SimpleManagedEntity 实例是否已激活
 * (Check if the SimpleManagedEntity instance is activated)
 * @return 返回 true 表示已激活，false 表示未激活
 * (Return true if activated, false if not activated)
 */
bool SimpleManagedEntity::is_activated() const {
  // 从 activated_ 原子变量加载值并返回，用于判断实例是否处于激活状态
  // (Load the value from the atomic variable activated_ and return it, used to determine whether
  // the instance is in an activated state)
  return activated_.load();
}

}  // namespace rclcpp_lifecycle

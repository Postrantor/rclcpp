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

#ifndef RCLCPP_LIFECYCLE__MANAGED_ENTITY_HPP_
#define RCLCPP_LIFECYCLE__MANAGED_ENTITY_HPP_

#include <atomic>

#include "rclcpp_lifecycle/visibility_control.h"

namespace rclcpp_lifecycle {

/// 生命周期实体的基类，如 `LifecyclePublisher`。
/// Base class for lifecycle entities, like `LifecyclePublisher`.
class ManagedEntityInterface {
public:
  // RCLCPP_LIFECYCLE_PUBLIC 是 ROS2 的宏，用于定义符号可见性。
  // RCLCPP_LIFECYCLE_PUBLIC is a ROS2 macro used to define symbol visibility.
  RCLCPP_LIFECYCLE_PUBLIC
  // 虚拟析构函数，允许子类正确地销毁。
  // Virtual destructor, allowing subclasses to be destroyed correctly.
  virtual ~ManagedEntityInterface() {}

  // 激活生命周期实体时调用的虚拟函数。
  // Virtual function called when activating the lifecycle entity.
  RCLCPP_LIFECYCLE_PUBLIC
  virtual void on_activate() = 0;

  // 停用生命周期实体时调用的虚拟函数。
  // Virtual function called when deactivating the lifecycle entity.
  RCLCPP_LIFECYCLE_PUBLIC
  virtual void on_deactivate() = 0;
};

/// 实现 `ManagedEntityInterface` 的简单类，切换标志。
/// A simple implementation of `ManagedEntityInterface`, which toggles a flag.
class SimpleManagedEntity : public ManagedEntityInterface {
public:
  // 使用默认的析构函数。
  // Use the default destructor.
  RCLCPP_LIFECYCLE_PUBLIC
  ~SimpleManagedEntity() override = default;

  // 重写 on_activate 函数，激活实体。
  // Override the on_activate function, to activate the entity.
  RCLCPP_LIFECYCLE_PUBLIC
  void on_activate() override;

  // 重写 on_deactivate 函数，停用实体。
  // Override the on_deactivate function, to deactivate the entity.
  RCLCPP_LIFECYCLE_PUBLIC
  void on_deactivate() override;

  // 返回激活状态的布尔值。
  // Returns a boolean indicating the activation status.
  RCLCPP_LIFECYCLE_PUBLIC
  bool is_activated() const;

private:
  // 使用 std::atomic<bool> 存储激活状态，确保线程安全。
  // Store the activation status using std::atomic<bool> for thread safety.
  std::atomic<bool> activated_ = false;
};

}  // namespace rclcpp_lifecycle
#endif  // RCLCPP_LIFECYCLE__MANAGED_ENTITY_HPP_

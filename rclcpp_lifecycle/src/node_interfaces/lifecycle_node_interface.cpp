// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "rclcpp_lifecycle/state.hpp"

namespace rclcpp_lifecycle {
namespace node_interfaces {

/**
 * @brief 在配置状态时调用的回调函数 (Callback function called during the configure state)
 *
 * @param[in] state 当前节点的状态 (The current state of the node)
 * @return LifecycleNodeInterface::CallbackReturn 成功返回SUCCESS (Returns SUCCESS if successful)
 */
LifecycleNodeInterface::CallbackReturn LifecycleNodeInterface::on_configure(const State &) {
  // 返回成功状态 (Return success status)
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief 在清理状态时调用的回调函数 (Callback function called during the cleanup state)
 *
 * @param[in] state 当前节点的状态 (The current state of the node)
 * @return LifecycleNodeInterface::CallbackReturn 成功返回SUCCESS (Returns SUCCESS if successful)
 */
LifecycleNodeInterface::CallbackReturn LifecycleNodeInterface::on_cleanup(const State &) {
  // 返回成功状态 (Return success status)
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief 在关闭状态时调用的回调函数 (Callback function called during the shutdown state)
 *
 * @param[in] state 当前节点的状态 (The current state of the node)
 * @return LifecycleNodeInterface::CallbackReturn 成功返回SUCCESS (Returns SUCCESS if successful)
 */
LifecycleNodeInterface::CallbackReturn LifecycleNodeInterface::on_shutdown(const State &) {
  // 返回成功状态 (Return success status)
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief 在激活状态时调用的回调函数 (Callback function called during the activate state)
 *
 * @param[in] state 当前节点的状态 (The current state of the node)
 * @return LifecycleNodeInterface::CallbackReturn 成功返回SUCCESS (Returns SUCCESS if successful)
 */
LifecycleNodeInterface::CallbackReturn LifecycleNodeInterface::on_activate(const State &) {
  // 返回成功状态 (Return success status)
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief 在停用状态时调用的回调函数 (Callback function called during the deactivate state)
 *
 * @param[in] state 当前节点的状态 (The current state of the node)
 * @return LifecycleNodeInterface::CallbackReturn 成功返回SUCCESS (Returns SUCCESS if successful)
 */
LifecycleNodeInterface::CallbackReturn LifecycleNodeInterface::on_deactivate(const State &) {
  // 返回成功状态 (Return success status)
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief 在错误状态时调用的回调函数 (Callback function called during the error state)
 *
 * @param[in] state 当前节点的状态 (The current state of the node)
 * @return LifecycleNodeInterface::CallbackReturn 成功返回SUCCESS (Returns SUCCESS if successful)
 */
LifecycleNodeInterface::CallbackReturn LifecycleNodeInterface::on_error(const State &) {
  // 返回成功状态 (Return success status)
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace node_interfaces
}  // namespace rclcpp_lifecycle

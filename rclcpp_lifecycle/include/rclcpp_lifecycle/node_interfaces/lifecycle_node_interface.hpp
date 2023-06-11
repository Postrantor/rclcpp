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

#ifndef RCLCPP_LIFECYCLE__NODE_INTERFACES__LIFECYCLE_NODE_INTERFACE_HPP_
#define RCLCPP_LIFECYCLE__NODE_INTERFACES__LIFECYCLE_NODE_INTERFACE_HPP_

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/node_interfaces/detail/node_interfaces_helpers.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/visibility_control.h"

namespace rclcpp_lifecycle {
namespace node_interfaces {
/// 生命周期节点接口类 (Interface class for a managed node).
/** 虚拟函数定义如下
 * http://design.ros2.org/articles/node_lifecycle.html
 *
 * 如果回调函数执行成功，则完成指定的转换
 * 如果回调函数失败或抛出未捕获的异常，则调用on_error函数
 * 默认情况下，所有函数都可以选择覆写，并返回true。
 * 除了on_error函数，该函数返回false，从而进入关闭/最终状态
 *
 * (If the callback function returns successfully),
 * (the specified transition is completed).
 * (If the callback function fails or throws an),
 * (uncaught exception, the on_error function is called).
 * (By default, all functions remain optional to overwrite)
 * (and return true. Except the on_error function, which)
 * (returns false and thus goes to shutdown/finalize state).
 */
class LifecycleNodeInterface {
protected:
  // RCLCPP_LIFECYCLE_PUBLIC宏用于声明符号可见性
  RCLCPP_LIFECYCLE_PUBLIC
  // 生命周期节点接口构造函数
  LifecycleNodeInterface() {}

public:
  // 定义回调返回值的枚举类 (Define an enumeration class for callback return values)
  enum class CallbackReturn : uint8_t {
    // 成功状态，对应于 TRANSITION_CALLBACK_SUCCESS
    SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS,
    // 失败状态，对应于 TRANSITION_CALLBACK_FAILURE
    FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE,
    // 错误状态，对应于 TRANSITION_CALLBACK_ERROR
    ERROR = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR
  };

  /// 配置过渡的回调函数 (Callback function for configure transition)
  /**
   * \param previous_state 上一个状态 (The previous state)
   * \return 默认返回 SUCCESS (Returns SUCCESS by default)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  virtual CallbackReturn on_configure(const State& previous_state);

  /// 清理过渡的回调函数 (Callback function for cleanup transition)
  /**
   * \param previous_state 上一个状态 (The previous state)
   * \return 默认返回 SUCCESS (Returns SUCCESS by default)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  virtual CallbackReturn on_cleanup(const State& previous_state);

  /// 关闭过渡的回调函数 (Callback function for shutdown transition)
  /**
   * \param previous_state 上一个状态 (The previous state)
   * \return 默认返回 SUCCESS (Returns SUCCESS by default)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  virtual CallbackReturn on_shutdown(const State& previous_state);

  /// Callback function for activate transition
  /*
   * \return SUCCESS by default
   */
  RCLCPP_LIFECYCLE_PUBLIC
  virtual CallbackReturn on_activate(const State& previous_state);

  /// Callback function for deactivate transition
  /*
   * \return SUCCESS by default
   */
  RCLCPP_LIFECYCLE_PUBLIC
  virtual CallbackReturn on_deactivate(const State& previous_state);

  /// Callback function for errorneous transition
  /*
   * \return SUCCESS by default
   */
  RCLCPP_LIFECYCLE_PUBLIC
  virtual CallbackReturn on_error(const State& previous_state);

  RCLCPP_LIFECYCLE_PUBLIC
  virtual ~LifecycleNodeInterface() {}
};

}  // namespace node_interfaces
}  // namespace rclcpp_lifecycle

RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface, lifecycle_node)

#endif  // RCLCPP_LIFECYCLE__NODE_INTERFACES__LIFECYCLE_NODE_INTERFACE_HPP_

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

#ifndef RCLCPP_LIFECYCLE__STATE_HPP_
#define RCLCPP_LIFECYCLE__STATE_HPP_

#include <mutex>
#include <string>

#include "rcl_lifecycle/data_types.h"
#include "rclcpp_lifecycle/visibility_control.h"
#include "rcutils/allocator.h"

namespace rclcpp_lifecycle {

/// 生命周期状态的抽象类（Abstract class for the Lifecycle's states）
/**
 * 主要有4个状态：未配置（Unconfigured）、非活动（Inactive）、活动（Active）和完成（Finalized）。
 * There are 4 primary states: Unconfigured, Inactive, Active and Finalized.
 */
class State {
public:
  // 公共生命周期函数（Public lifecycle function）
  RCLCPP_LIFECYCLE_PUBLIC
  // 使用默认分配器构造 State 对象（Construct a State object with the default allocator）
  explicit State(rcutils_allocator_t allocator = rcutils_get_default_allocator());

  /// 状态构造函数（State constructor）
  /**
   * \param[in] id 状态的 ID（ID of the state）
   * \param[in] label 状态的标签（Label of the state）
   * \param[in] allocator 用于初始化状态的有效分配器（A valid allocator used to initialize the
   * state）
   */
  RCLCPP_LIFECYCLE_PUBLIC
  State(
      uint8_t id,
      const std::string& label,
      rcutils_allocator_t allocator = rcutils_get_default_allocator());

  /// 状态构造函数（State constructor）
  /**
   * \param[in] rcl_lifecycle_state_handle 包含状态详细信息的结构体（Structure with the state
   * details） \param[in] allocator 用于初始化状态的有效分配器（A valid allocator used to initialize
   * the state）
   */
  RCLCPP_LIFECYCLE_PUBLIC
  explicit State(
      const rcl_lifecycle_state_t* rcl_lifecycle_state_handle,
      rcutils_allocator_t allocator = rcutils_get_default_allocator());

  // 拷贝构造函数（Copy constructor）
  RCLCPP_LIFECYCLE_PUBLIC
  State(const State& rhs);

  // 虚拟析构函数（Virtual destructor）
  RCLCPP_LIFECYCLE_PUBLIC
  virtual ~State();

  // 赋值运算符（Assignment operator）
  RCLCPP_LIFECYCLE_PUBLIC
  State& operator=(const State& rhs);

  /// 返回状态 ID（Return the state ID）
  /**
   * \return 状态的 ID（ID of the state）
   */
  RCLCPP_LIFECYCLE_PUBLIC
  uint8_t id() const;

  /// 返回状态标签（Return the state label）
  /**
   * \return 状态的标签（Label of the state）
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::string label() const;

protected:
  // 重置状态（Reset the state）
  RCLCPP_LIFECYCLE_PUBLIC
  void reset() noexcept;

  // 分配器（Allocator）
  rcutils_allocator_t allocator_;

  // 是否拥有 rcl_state_handle_（Whether it owns rcl_state_handle_）
  bool owns_rcl_state_handle_;

  // 用于保护 state_handle_ 的递归互斥锁（Recursive mutex to protect state_handle_）
  mutable std::recursive_mutex state_handle_mutex_;
  // 生命周期状态句柄（Lifecycle state handle）
  rcl_lifecycle_state_t* state_handle_;
};

}  // namespace rclcpp_lifecycle
#endif  // RCLCPP_LIFECYCLE__STATE_HPP_

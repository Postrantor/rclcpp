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

#ifndef RCLCPP_LIFECYCLE__TRANSITION_HPP_
#define RCLCPP_LIFECYCLE__TRANSITION_HPP_

#include <string>

#include "rcl_lifecycle/data_types.h"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/visibility_control.h"
#include "rcutils/allocator.h"

namespace rclcpp_lifecycle {

/// Transition 类抽象了生命周期的状态。
/// The Transition class abstracts the Lifecycle's states.
/**
 * 共有 7 个转换暴露给监督进程，它们分别是：create（创建），configure（配置），
 * cleanup（清理），activate（激活），deactivate（停用），shutdown（关闭）和 destroy（销毁）。
 * There are 7 transitions exposed to a supervisory process, they are: create, configure,
 * cleanup, activate, deactivate, shutdown, and destroy.
 */
class Transition {
public:
  // 使用 RCLCPP_LIFECYCLE_PUBLIC 宏来设置类的可见性
  // Using the RCLCPP_LIFECYCLE_PUBLIC macro to set the visibility of the class
  RCLCPP_LIFECYCLE_PUBLIC

  // 删除默认构造函数，禁止创建无参数的 Transition 对象
  // Delete the default constructor, preventing the creation of Transition objects without
  // parameters
  Transition() = delete;

  /// \brief 过渡构造函数 (Transition constructor)
  ///
  /// \param[in] id 过渡的ID (id of the transition)
  /// \param[in] label 过渡的标签 (label of the transition)
  /// \param[in] allocator 用于初始化状态的有效分配器 (a valid allocator used to initialize the
  /// state)
  ///
  RCLCPP_LIFECYCLE_PUBLIC
  explicit Transition(
      uint8_t id,
      const std::string& label = "",
      rcutils_allocator_t allocator = rcutils_get_default_allocator());

  /// \brief 过渡构造函数 (Transition constructor)
  ///
  /// \param[in] id 过渡的ID (id of the transition)
  /// \param[in] label 过渡的标签 (label of the transition)
  /// \param[in] start 过渡的起始状态 (start state of the transition)
  /// \param[in] goal 过渡的目标状态 (goal state of the transition)
  /// \param[in] allocator 用于初始化状态的有效分配器 (a valid allocator used to initialize the
  /// state)
  ///
  RCLCPP_LIFECYCLE_PUBLIC
  Transition(
      uint8_t id,
      const std::string& label,
      State&& start,
      State&& goal,
      rcutils_allocator_t allocator = rcutils_get_default_allocator());

  /// \brief 过渡构造函数 (Transition constructor)
  ///
  /// \param[in] rcl_lifecycle_transition_handle 包含过渡详细信息的结构 (structure with the
  /// transition details) \param[in] allocator 用于初始化状态的有效分配器 (a valid allocator used to
  /// initialize the state)
  ///
  RCLCPP_LIFECYCLE_PUBLIC
  explicit Transition(
      const rcl_lifecycle_transition_t* rcl_lifecycle_transition_handle,
      rcutils_allocator_t allocator = rcutils_get_default_allocator());

  /// \brief 拷贝构造函数 (Copy constructor)
  ///
  /// \param[in] rhs 要复制的Transition对象 (The Transition object to copy)
  ///
  RCLCPP_LIFECYCLE_PUBLIC
  Transition(const Transition& rhs);

  /// \brief 析构函数 (Destructor)
  ///
  RCLCPP_LIFECYCLE_PUBLIC
  virtual ~Transition();

  /// \brief 赋值运算符重载 (Assignment operator overload)
  ///
  /// \param[in] rhs 要赋值的Transition对象 (The Transition object to assign)
  /// \return 返回*this以便链式赋值 (Returns *this for chained assignment)
  ///
  RCLCPP_LIFECYCLE_PUBLIC
  Transition& operator=(const Transition& rhs);

  /// 返回 id（Return the id）.
  /**
   * \return 状态的 id（id of the state）
   */
  RCLCPP_LIFECYCLE_PUBLIC
  uint8_t id() const;

  /// 返回标签（Return the label）.
  /**
   * \return 转换的标签（label of the transition）
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::string label() const;

  /// 返回转换的起始状态（Return the start state of the transition）.
  /**
   * \return 转换的起始状态（start state of the transition）
   */
  RCLCPP_LIFECYCLE_PUBLIC
  State start_state() const;

  /// 返回转换的目标状态（Return the goal state of the transition）.
  /**
   * \return 转换的目标状态（goal state of the transition）
   */
  RCLCPP_LIFECYCLE_PUBLIC
  State goal_state() const;

protected:
  // 公共生命周期函数（Public lifecycle function）
  RCLCPP_LIFECYCLE_PUBLIC
  // 重置函数，不抛出异常（Reset function, noexcept indicates it does not throw exceptions）
  void reset() noexcept;

  // 分配器变量（Allocator variable）
  rcutils_allocator_t allocator_;

  // 布尔值，表示是否拥有 rcl_transition_handle_ （Boolean value indicating if
  // owns_rcl_transition_handle_）
  bool owns_rcl_transition_handle_;

  // 指向 rcl_lifecycle_transition_t 类型的指针（Pointer to rcl_lifecycle_transition_t type）
  rcl_lifecycle_transition_t* transition_handle_;
};

}  // namespace rclcpp_lifecycle
#endif  // RCLCPP_LIFECYCLE__TRANSITION_HPP_

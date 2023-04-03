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

#include "rclcpp_lifecycle/transition.hpp"

#include <stdexcept>
#include <string>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/allocator.h"

namespace rclcpp_lifecycle {

/**
 * @brief 构造函数，用于创建一个生命周期转换对象 (Constructor for creating a lifecycle transition
 * object)
 *
 * @param id 转换的唯一标识符 (Unique identifier for the transition)
 * @param label 转换的标签，用于描述转换 (Label for the transition, used to describe the transition)
 * @param allocator 分配器，用于分配内存 (Allocator for allocating memory)
 */
Transition::Transition(uint8_t id, const std::string &label, rcutils_allocator_t allocator)
    : allocator_(allocator), owns_rcl_transition_handle_(true), transition_handle_(nullptr) {
  // 为 rcl_lifecycle_transition_t 分配内存 (Allocate memory for rcl_lifecycle_transition_t)
  transition_handle_ = static_cast<rcl_lifecycle_transition_t *>(
      allocator_.allocate(sizeof(rcl_lifecycle_transition_t), allocator_.state));

  // 检查是否成功分配内存 (Check if memory allocation was successful)
  if (!transition_handle_) {
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_transition_t");
  }

  // 将 transition_handle_ 的各个成员初始化为零或空指针 (Zero initialize members of
  // transition_handle_)
  transition_handle_->id = 0;
  transition_handle_->label = nullptr;
  transition_handle_->start = nullptr;
  transition_handle_->goal = nullptr;

  // 使用提供的参数初始化 rcl_lifecycle_transition_t 结构体 (Initialize the
  // rcl_lifecycle_transition_t struct with provided parameters)
  auto ret = rcl_lifecycle_transition_init(
      transition_handle_, id, label.c_str(), nullptr, nullptr, &allocator_);

  // 检查初始化是否成功，如果不成功，则重置并抛出异常 (Check if initialization was successful, if
  // not, reset and throw exception)
  if (ret != RCL_RET_OK) {
    reset();
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

/**
 * @brief 构造函数，用于创建一个生命周期转换对象 (Constructor for creating a lifecycle transition
 * object)
 *
 * @param id 转换的唯一标识符 (Unique identifier for the transition)
 * @param label 转换的标签 (Label for the transition)
 * @param start 起始状态 (Starting state)
 * @param goal 目标状态 (Goal state)
 * @param allocator 内存分配器 (Memory allocator)
 */
Transition::Transition(
    uint8_t id,
    const std::string &label,
    State &&start,
    State &&goal,
    rcutils_allocator_t allocator)
    : Transition(id, label, allocator) {
  // 为起始状态分配内存 (Allocate memory for the starting state)
  transition_handle_->start = static_cast<rcl_lifecycle_state_t *>(
      allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  // 检查内存是否分配成功 (Check if memory allocation was successful)
  if (!transition_handle_->start) {
    reset();
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
  }
  // 初始化起始状态 (Initialize starting state)
  transition_handle_->start->id = 0;
  transition_handle_->start->label = nullptr;

  // 使用给定的起始状态初始化 rcl_lifecycle_state_t 结构 (Initialize the rcl_lifecycle_state_t
  // structure with the given starting state)
  auto ret = rcl_lifecycle_state_init(
      transition_handle_->start, start.id(), start.label().c_str(), &allocator_);
  // 检查初始化是否成功 (Check if initialization was successful)
  if (ret != RCL_RET_OK) {
    reset();
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // 为目标状态分配内存 (Allocate memory for the goal state)
  transition_handle_->goal = static_cast<rcl_lifecycle_state_t *>(
      allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  // 检查内存是否分配成功 (Check if memory allocation was successful)
  if (!transition_handle_->goal) {
    reset();
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
  }
  // 初始化目标状态 (Initialize goal state)
  transition_handle_->goal->id = 0;
  transition_handle_->goal->label = nullptr;

  // 使用给定的目标状态初始化 rcl_lifecycle_state_t 结构 (Initialize the rcl_lifecycle_state_t
  // structure with the given goal state)
  ret = rcl_lifecycle_state_init(
      transition_handle_->goal, goal.id(), goal.label().c_str(), &allocator_);
  // 检查初始化是否成功 (Check if initialization was successful)
  if (ret != RCL_RET_OK) {
    reset();
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

/**
 * @brief 构造函数，用于从 rcl_lifecycle_transition_t 对象创建 Transition 对象。
 * @param rcl_lifecycle_transition_handle 指向 rcl_lifecycle_transition_t 的指针。
 * @param allocator 用于分配内存的 rcutils_allocator_t 对象。
 *
 * Constructor for creating a Transition object from an rcl_lifecycle_transition_t object.
 * @param rcl_lifecycle_transition_handle A pointer to an rcl_lifecycle_transition_t object.
 * @param allocator An rcutils_allocator_t object for memory allocation.
 */
Transition::Transition(
    const rcl_lifecycle_transition_t *rcl_lifecycle_transition_handle,
    rcutils_allocator_t allocator)
    : allocator_(allocator), owns_rcl_transition_handle_(false), transition_handle_(nullptr) {
  // 如果 rcl_lifecycle_transition_handle 为空，则抛出异常。
  // Throw an exception if rcl_lifecycle_transition_handle is null.
  if (!rcl_lifecycle_transition_handle) {
    throw std::runtime_error("rcl_lifecycle_transition_handle is null");
  }
  // 将传入的 rcl_lifecycle_transition_handle 赋值给 transition_handle_。
  // Assign the given rcl_lifecycle_transition_handle to transition_handle_.
  transition_handle_ = const_cast<rcl_lifecycle_transition_t *>(rcl_lifecycle_transition_handle);
}

/**
 * @brief 拷贝构造函数，用于复制一个已存在的 Transition 对象。
 * @param rhs 要复制的 Transition 对象。
 *
 * Copy constructor for copying an existing Transition object.
 * @param rhs The Transition object to be copied.
 */
Transition::Transition(const Transition &rhs)
    : allocator_(rhs.allocator_), owns_rcl_transition_handle_(false), transition_handle_(nullptr) {
  // 使用赋值运算符复制 rhs。
  // Copy rhs using the assignment operator.
  *this = rhs;
}

/**
 * @brief 析构函数，用于销毁 Transition 对象并释放资源。
 *
 * Destructor for destroying a Transition object and releasing resources.
 */
Transition::~Transition() { reset(); }

/**
 * @brief 重载赋值运算符，实现 Transition 类的深拷贝
 * @param rhs 右侧操作数，即要复制的源对象
 * @return 返回当前对象的引用
 *
 * @brief Overload assignment operator to implement deep copy for Transition class
 * @param rhs The right-hand operand, the source object to be copied
 * @return Returns a reference to the current object
 */
Transition &Transition::operator=(const Transition &rhs) {
  // 如果左右两个操作数是同一个对象，则直接返回当前对象的引用
  // If the left and right operands are the same object, return the reference of the current object
  // directly
  if (this == &rhs) {
    return *this;
  }

  // 重置所有当前使用的资源
  // Reset all currently used resources
  reset();

  allocator_ = rhs.allocator_;
  owns_rcl_transition_handle_ = rhs.owns_rcl_transition_handle_;

  // 如果我们不拥有句柄，则可以直接返回
  // If we don't own the handle, we can return straight ahead
  if (!owns_rcl_transition_handle_) {
    transition_handle_ = rhs.transition_handle_;
    return *this;
  }

  // 如果我们拥有句柄，则需要对 rhs 对象进行深拷贝
  // If we own the handle, we need to deep-copy the rhs object
  transition_handle_ = static_cast<rcl_lifecycle_transition_t *>(
      allocator_.allocate(sizeof(rcl_lifecycle_transition_t), allocator_.state));
  if (!transition_handle_) {
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_transition_t");
  }
  // 零初始化
  // Zero initialization
  transition_handle_->id = 0;
  transition_handle_->label = nullptr;
  transition_handle_->start = nullptr;
  transition_handle_->goal = nullptr;

  auto ret = rcl_lifecycle_transition_init(
      transition_handle_, rhs.id(), rhs.label().c_str(), nullptr, nullptr, &allocator_);
  if (ret != RCL_RET_OK) {
    reset();
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // 当开始状态可用时，才进行拷贝
  // Copy the start state only when it is available
  if (rhs.transition_handle_->start) {
    transition_handle_->start = static_cast<rcl_lifecycle_state_t *>(
        allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
    if (!transition_handle_->start) {
      reset();
      throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
    }
    // 零初始化
    // Zero initialization
    transition_handle_->start->id = 0;
    transition_handle_->start->label = nullptr;

    ret = rcl_lifecycle_state_init(
        transition_handle_->start, rhs.start_state().id(), rhs.start_state().label().c_str(),
        &allocator_);
    if (ret != RCL_RET_OK) {
      reset();
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
  }

  // 当目标状态可用时，才进行拷贝
  // Copy the goal state only when it is available
  if (rhs.transition_handle_->goal) {
    transition_handle_->goal = static_cast<rcl_lifecycle_state_t *>(
        allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
    if (!transition_handle_->goal) {
      reset();
      throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
    }
    // 零初始化
    // Zero initialization
    transition_handle_->goal->id = 0;
    transition_handle_->goal->label = nullptr;

    ret = rcl_lifecycle_state_init(
        transition_handle_->goal, rhs.goal_state().id(), rhs.goal_state().label().c_str(),
        &allocator_);
    if (ret != RCL_RET_OK) {
      reset();
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
  }
  return *this;
}

/**
 * @brief 获取转换的 ID (Get the ID of the transition)
 *
 * @return uint8_t 转换的 ID (The ID of the transition)
 * @throws std::runtime_error 如果内部 transition_handle 为空 (If the internal transition_handle is
 * null)
 */
uint8_t Transition::id() const {
  // 检查 transition_handle 是否为空，如果为空则抛出异常
  // Check if transition_handle is null, throw an exception if it is
  if (!transition_handle_) {
    throw std::runtime_error("internal transition_handle is null");
  }

  // 返回转换的 ID
  // Return the ID of the transition
  return static_cast<uint8_t>(transition_handle_->id);
}

/**
 * @brief 获取转换的标签 (Get the label of the transition)
 *
 * @return std::string 转换的标签 (The label of the transition)
 * @throws std::runtime_error 如果内部 transition_handle 为空 (If the internal transition_handle is
 * null)
 */
std::string Transition::label() const {
  // 检查 transition_handle 是否为空，如果为空则抛出异常
  // Check if transition_handle is null, throw an exception if it is
  if (!transition_handle_) {
    throw std::runtime_error("internal transition_handle is null");
  }

  // 返回转换的标签
  // Return the label of the transition
  return transition_handle_->label;
}

/**
 * @brief 获取起始状态 (Get the start state)
 *
 * @return State 起始状态 (The start state)
 * @throws std::runtime_error 如果内部 transition_handle 为空 (If the internal transition_handle is
 * null)
 */
State Transition::start_state() const {
  // 检查 transition_handle 是否为空，如果为空则抛出异常
  // Check if transition_handle is null, throw an exception if it is
  if (!transition_handle_) {
    throw std::runtime_error("internal transition_handle is null");
  }

  // 使用 start 指针创建 State 对象，如果指针为空则构造函数会抛出异常
  // Create a State object using the start pointer, the constructor throws an exception if the
  // pointer is null
  return State(transition_handle_->start, allocator_);
}

/**
 * @brief 获取目标状态 (Get the goal state)
 *
 * @return State 目标状态 (The goal state)
 * @throws std::runtime_error 如果内部 transition_handle 为空 (If the internal transition_handle is
 * null)
 */
State Transition::goal_state() const {
  // 检查 transition_handle 是否为空，如果为空则抛出异常
  // Check if transition_handle is null, throw an exception if it is
  if (!transition_handle_) {
    throw std::runtime_error("internal transition_handle is null");
  }

  // 使用 goal 指针创建 State 对象，如果指针为空则构造函数会抛出异常
  // Create a State object using the goal pointer, the constructor throws an exception if the
  // pointer is null
  return State(transition_handle_->goal, allocator_);
}

/**
 * @brief 重置 Transition 对象 (Reset the Transition object)
 *
 * 这个函数会释放 Transition 对象所持有的资源，并将其状态重置为初始状态。
 * (This function releases the resources held by the Transition object and resets its state to the
 * initial state.)
 */
void Transition::reset() noexcept {
  // 如果不拥有 rcl_transition_handle_，则无法释放任何资源
  // (Can't free anything which is not owned)
  if (!owns_rcl_transition_handle_) {
    transition_handle_ = nullptr;
  }

  // 如果 transition_handle_ 为空，则直接返回
  // (If transition_handle_ is null, return directly)
  if (!transition_handle_) {
    return;
  }

  // 调用 rcl_lifecycle_transition_fini 函数来完成资源释放
  // (Call rcl_lifecycle_transition_fini function to complete resource release)
  auto ret = rcl_lifecycle_transition_fini(transition_handle_, &allocator_);

  // 使用分配器释放 transition_handle_ 的内存
  // (Use allocator to release memory of transition_handle_)
  allocator_.deallocate(transition_handle_, allocator_.state);

  // 将 transition_handle_ 设置为 nullptr
  // (Set transition_handle_ to nullptr)
  transition_handle_ = nullptr;

  // 检查 rcl_lifecycle_transition_fini 是否成功执行
  // (Check if rcl_lifecycle_transition_fini executed successfully)
  if (ret != RCL_RET_OK) {
    RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp_lifecycle"),
        "rcl_lifecycle_transition_fini did not complete successfully, leaking memory");
  }
}
}  // namespace rclcpp_lifecycle

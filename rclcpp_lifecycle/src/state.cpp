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

#include "rclcpp_lifecycle/state.hpp"

#include <stdexcept>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/allocator.h"

namespace rclcpp_lifecycle {

/**
 * @brief 构造一个新的 State 对象 (Construct a new State object)
 *
 * @param allocator 分配器 (Allocator)
 */
State::State(rcutils_allocator_t allocator)
    : State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "unknown", allocator) {}

/**
 * @brief 构造一个新的 State 对象 (Construct a new State object)
 *
 * @param id 状态 ID (State ID)
 * @param label 状态标签 (State label)
 * @param allocator 分配器 (Allocator)
 */
State::State(uint8_t id, const std::string &label, rcutils_allocator_t allocator)
    : allocator_(allocator), owns_rcl_state_handle_(true), state_handle_(nullptr) {
  // 检查状态标签是否为空，如果为空则抛出异常 (Check if the state label is empty, and throw an
  // exception if it is)
  if (label.empty()) {
    throw std::runtime_error("Lifecycle State cannot have an empty label.");
  }

  // 为 rcl_lifecycle_state_t 分配内存 (Allocate memory for rcl_lifecycle_state_t)
  state_handle_ = static_cast<rcl_lifecycle_state_t *>(
      allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  // 如果分配失败，则抛出异常 (Throw an exception if allocation fails)
  if (!state_handle_) {
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
  }
  // 将 state_handle_ 的 id 和 label 初始化为零 (Initialize the id and label of state_handle_ to
  // zero)
  state_handle_->id = 0;
  state_handle_->label = nullptr;

  // 使用给定的参数初始化 rcl_lifecycle_state_t (Initialize the rcl_lifecycle_state_t with the given
  // parameters)
  auto ret = rcl_lifecycle_state_init(state_handle_, id, label.c_str(), &allocator_);
  // 如果初始化失败，则重置 state_handle_ 并抛出异常 (If initialization fails, reset state_handle_
  // and throw an exception)
  if (ret != RCL_RET_OK) {
    reset();
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

/**
 * @brief 构造一个 State 对象，使用给定的 rcl_lifecycle_state_t 句柄和分配器。
 *        Construct a State object using the given rcl_lifecycle_state_t handle and allocator.
 *
 * @param[in] rcl_lifecycle_state_handle 指向 rcl_lifecycle_state_t 结构体的指针。
 *                                       Pointer to an rcl_lifecycle_state_t structure.
 * @param[in] allocator 分配器，用于管理内存。Allocator for managing memory.
 */
State::State(const rcl_lifecycle_state_t *rcl_lifecycle_state_handle, rcutils_allocator_t allocator)
    : allocator_(allocator), owns_rcl_state_handle_(false), state_handle_(nullptr) {
  // 判断传入的 rcl_lifecycle_state_handle 是否为空，为空则抛出异常。
  // Check if the provided rcl_lifecycle_state_handle is null, and throw an exception if it is.
  if (!rcl_lifecycle_state_handle) {
    throw std::runtime_error("rcl_lifecycle_state_handle is null");
  }

  // 使用 const_cast 移除 const 属性，并将 rcl_lifecycle_state_handle 赋值给 state_handle_。
  // Remove the const attribute using const_cast and assign rcl_lifecycle_state_handle to
  // state_handle_.
  state_handle_ = const_cast<rcl_lifecycle_state_t *>(rcl_lifecycle_state_handle);
}

/**
 * @brief 拷贝构造函数。Copy constructor.
 *
 * @param[in] rhs 需要拷贝的 State 对象。The State object to be copied.
 */
State::State(const State &rhs)
    : allocator_(rhs.allocator_), owns_rcl_state_handle_(false), state_handle_(nullptr) {
  // 使用赋值运算符重载函数来完成拷贝构造。
  // Use the overloaded assignment operator function to complete the copy construction.
  *this = rhs;
}

/*!
 * \brief 析构函数，用于释放 State 对象占用的资源。
 * \details 该析构函数会调用 reset() 方法来释放资源。
 * Destructor for releasing the resources occupied by the State object.
 * This destructor calls the reset() method to release resources.
 */
State::~State() { reset(); }

/*!
 * \brief 赋值运算符重载。
 * \param rhs 右侧操作数，一个 State 类型的常量引用。
 * \return 返回当前对象的引用。
 * \details 该方法实现了深拷贝，以确保在赋值过程中不会出现潜在问题。
 * Overloaded assignment operator.
 * \param rhs The right-hand operand, a constant reference of type State.
 * \return Returns a reference to the current object.
 * \details This method implements deep copying to ensure that no potential problems occur during
 * assignment.
 */
State &State::operator=(const State &rhs) {
  // 判断自赋值情况
  // Check for self-assignment
  if (this == &rhs) {
    return *this;
  }

  // 保持锁定，直到 state_handle_ 重建
  // Hold the lock until state_handle_ is reconstructed
  std::lock_guard<std::recursive_mutex> lock(state_handle_mutex_);
  // 重置所有当前使用的资源
  // Reset all currently used resources
  reset();

  allocator_ = rhs.allocator_;
  owns_rcl_state_handle_ = rhs.owns_rcl_state_handle_;

  // 如果我们不拥有句柄，则可以直接返回
  // If we don't own the handle, we can return straight away
  if (!owns_rcl_state_handle_) {
    state_handle_ = rhs.state_handle_;
    return *this;
  }

  // 如果我们拥有句柄，那么我们必须深度复制 rhs 对象
  // If we own the handle, we have to deep-copy the rhs object
  state_handle_ = static_cast<rcl_lifecycle_state_t *>(
      allocator_.allocate(sizeof(rcl_lifecycle_state_t), allocator_.state));
  if (!state_handle_) {
    throw std::runtime_error("failed to allocate memory for rcl_lifecycle_state_t");
  }
  // 零初始化
  // Zero initialization
  state_handle_->id = 0;
  state_handle_->label = nullptr;

  auto ret = rcl_lifecycle_state_init(state_handle_, rhs.id(), rhs.label().c_str(), &allocator_);
  if (ret != RCL_RET_OK) {
    reset();
    throw std::runtime_error("failed to duplicate label for rcl_lifecycle_state_t");
  }

  return *this;
}

/*!
 * \brief 获取状态的 ID。
 * \return 返回一个 uint8_t 类型的状态 ID。
 * \details 在获取状态 ID 之前，会先对 state_handle_ 进行合法性检查。
 * Get the ID of the state.
 * \return Returns a uint8_t type status ID.
 * \details Before getting the state ID, it will check the validity of state_handle_ first.
 */
uint8_t State::id() const {
  std::lock_guard<std::recursive_mutex> lock(state_handle_mutex_);
  if (!state_handle_) {
    throw std::runtime_error("Error in state! Internal state_handle is NULL.");
  }
  return static_cast<uint8_t>(state_handle_->id);
}

/**
 * @brief 获取状态标签（Get the state label）
 *
 * @return std::string 状态标签字符串（State label string）
 */
std::string State::label() const {
  // 创建一个递归互斥锁，保护 state_handle_ 的访问（Create a recursive mutex lock to protect access
  // to state_handle_）
  std::lock_guard<std::recursive_mutex> lock(state_handle_mutex_);

  // 检查 state_handle_ 是否为空，如果为空则抛出异常（Check if state_handle_ is NULL, and throw an
  // exception if it is）
  if (!state_handle_) {
    throw std::runtime_error("Error in state! Internal state_handle is NULL.");
  }

  // 返回 state_handle_ 的标签（Return the label of state_handle_）
  return state_handle_->label;
}

/**
 * @brief 重置状态（Reset the state）
 */
void State::reset() noexcept {
  // 创建一个递归互斥锁，保护 state_handle_ 的访问（Create a recursive mutex lock to protect access
  // to state_handle_）
  std::lock_guard<std::recursive_mutex> lock(state_handle_mutex_);

  // 如果不拥有 rcl_state_handle_，将 state_handle_ 设置为 nullptr（If not owning rcl_state_handle_,
  // set state_handle_ to nullptr）
  if (!owns_rcl_state_handle_) {
    state_handle_ = nullptr;
  }

  // 如果 state_handle_ 仍然为空，直接返回（If state_handle_ is still NULL, return directly）
  if (!state_handle_) {
    return;
  }

  // 调用 rcl_lifecycle_state_fini 函数释放 state_handle_ 资源（Call rcl_lifecycle_state_fini to
  // release resources of state_handle_）
  auto ret = rcl_lifecycle_state_fini(state_handle_, &allocator_);

  // 使用分配器释放 state_handle_ 内存（Use allocator to deallocate memory of state_handle_）
  allocator_.deallocate(state_handle_, allocator_.state);

  // 将 state_handle_ 设置为 nullptr（Set state_handle_ to nullptr）
  state_handle_ = nullptr;

  // 检查 rcl_lifecycle_state_fini 是否成功完成，如果没有则输出错误日志（Check if
  // rcl_lifecycle_state_fini completed successfully, and output an error log if not）
  if (ret != RCL_RET_OK) {
    RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp_lifecycle"),
        "rcl_lifecycle_transition_fini did not complete successfully, leaking memory");
  }
}

}  // namespace rclcpp_lifecycle

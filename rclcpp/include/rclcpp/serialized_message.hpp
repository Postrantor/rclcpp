// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__SERIALIZED_MESSAGE_HPP_
#define RCLCPP__SERIALIZED_MESSAGE_HPP_

#include "rcl/allocator.h"
#include "rcl/types.h"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/// 面向对象版本的 rcl_serialized_message_t，带有析构函数以避免内存泄漏
/// Object oriented version of rcl_serialized_message_t with destructor to avoid memory leaks
class RCLCPP_PUBLIC_TYPE SerializedMessage {
public:
  /// SerializedMessage 的默认构造函数
  /// Default constructor for a SerializedMessage
  /**
   * 默认构造一个序列化消息，并使用初始容量为 0 进行初始化。
   * 分配器默认为 `rcl_get_default_allocator()`。
   * Default constructs a serialized message and initializes it
   * with initial capacity of 0.
   * The allocator defaults to `rcl_get_default_allocator()`.
   *
   * \param[in] allocator 用于初始化的分配器。
   * \param[in] allocator The allocator to be used for the initialization.
   */
  explicit SerializedMessage(const rcl_allocator_t& allocator = rcl_get_default_allocator());

  /// SerializedMessage 的默认构造函数
  /// Default constructor for a SerializedMessage
  /**
   * 默认构造一个序列化消息，并使用提供的容量进行初始化。
   * 分配器默认为 `rcl_get_default_allocator()`。
   * Default constructs a serialized message and initializes it
   * with the provided capacity.
   * The allocator defaults to `rcl_get_default_allocator()`.
   *
   * \param[in] initial_capacity 要分配的内存量。
   * \param[in] allocator 用于初始化的分配器。
   * \param[in] initial_capacity The amount of memory to be allocated.
   * \param[in] allocator The allocator to be used for the initialization.
   */
  explicit SerializedMessage(
      size_t initial_capacity, const rcl_allocator_t& allocator = rcl_get_default_allocator());

  /// SerializedMessage 的拷贝构造函数
  /// Copy Constructor for a SerializedMessage
  SerializedMessage(const SerializedMessage& other);

  /// 从 rcl_serialized_message_t 构造一个 SerializedMessage
  /// Constructor for a SerializedMessage from a rcl_serialized_message_t
  explicit SerializedMessage(const rcl_serialized_message_t& other);

  /// SerializedMessage 的移动构造函数
  /// Move Constructor for a SerializedMessage
  SerializedMessage(SerializedMessage&& other);

  /// 从移动的 rcl_serialized_message_t 构造一个 SerializedMessage
  /// Constructor for a SerializedMessage from a moved rcl_serialized_message_t
  explicit SerializedMessage(rcl_serialized_message_t&& other);

  /// 拷贝赋值运算符
  /// Copy assignment operator
  SerializedMessage& operator=(const SerializedMessage& other);

  /// 从 rcl_serialized_message_t 的拷贝赋值运算符
  /// Copy assignment operator from a rcl_serialized_message_t
  SerializedMessage& operator=(const rcl_serialized_message_t& other);

  /// 移动赋值运算符
  /// Move assignment operator
  SerializedMessage& operator=(SerializedMessage&& other);

  /// 从 rcl_serialized_message_t 的移动赋值运算符
  /// Move assignment operator from a rcl_serialized_message_t
  SerializedMessage& operator=(rcl_serialized_message_t&& other);

  /// SerializedMessage 的析构函数
  /// Destructor for a SerializedMessage
  virtual ~SerializedMessage();

  /// 获取底层 rcl_serialized_t 句柄
  /// Get the underlying rcl_serialized_t handle
  rcl_serialized_message_t& get_rcl_serialized_message();

  // 获取底层 rcl_serialized_message_t 的 const 句柄
  // Get a const handle to the underlying rcl_serialized_message_t
  const rcl_serialized_message_t& get_rcl_serialized_message() const;

  /// 获取序列化数据缓冲区的大小
  /// Get the size of the serialized data buffer
  /**
   * 注意，这与实际分配的内存量不同。
   * 可以通过调用 `capacity` 获得这个值。
   * Note, this is different from the actual amount of allocated memory.
   * This can be obtained via a call to `capacity`.
   */
  size_t size() const;

  /// 获取数据缓冲区的分配内存大小
  /// Get the size of allocated memory for the data buffer
  /**
   * 注意，这与缓冲区中的内容量不同。
   * 可以通过调用 `size` 获得这个值。
   * Note, this is different from the amount of content in the buffer.
   * This can be obtained via a call to `size`.
   */
  size_t capacity() const;

  /// 在数据缓冲区中分配内存
  /// Allocate memory in the data buffer
  /**
   * 底层 rcl_serialized_message_t 的数据缓冲区将被调整大小。
   * 这可能会改变数据布局并使指向数据的所有指针失效。
   * The data buffer of the underlying rcl_serialized_message_t will be resized.
   * This might change the data layout and invalidates all pointers to the data.
   */
  void reserve(size_t capacity);

  /// 释放底层 rcl_serialized_message_t
  /// Release the underlying rcl_serialized_message_t
  /**
   * 序列化消息的内存（即数据缓冲区）将不再由此实例管理，并且在销毁时不会释放内存。
   * The memory (i.e. the data buffer) of the serialized message will no longer
   * be managed by this instance and the memory won't be deallocated on destruction.
   */
  rcl_serialized_message_t release_rcl_serialized_message();

private:
  rcl_serialized_message_t serialized_message_;
};

}  // namespace rclcpp

#endif  // RCLCPP__SERIALIZED_MESSAGE_HPP_

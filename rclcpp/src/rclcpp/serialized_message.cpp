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

#include "rclcpp/serialized_message.hpp"

#include <cstring>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rmw/types.h"

namespace rclcpp {

/**
 * @brief 复制 rcl 序列化消息 (Copy rcl serialized message)
 *
 * @param from 源 rcl_serialized_message_t 消息 (Source rcl_serialized_message_t message)
 * @param to 目标 rcl_serialized_message_t 消息 (Destination rcl_serialized_message_t message)
 */
inline void copy_rcl_message(const rcl_serialized_message_t& from, rcl_serialized_message_t& to) {
  // 初始化目标序列化消息，分配内存 (Initialize the destination serialized message and allocate
  // memory)
  const auto ret = rmw_serialized_message_init(&to, from.buffer_capacity, &from.allocator);

  // 如果初始化失败，抛出异常 (If initialization fails, throw an exception)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // 如果源消息和目标消息的缓冲区不同，执行 memcpy (Perform memcpy if the source and destination
  // message buffers are different)
  if (to.buffer != from.buffer) {
    std::memcpy(to.buffer, from.buffer, from.buffer_length);
  }

  // 设置目标消息的 buffer_length (Set the buffer_length of the destination message)
  to.buffer_length = from.buffer_length;
}

/// 面向对象版本的 rcl_serialized_message_t，具有析构函数以避免内存泄漏 (Object oriented version of
/// rcl_serialized_message_t with destructor to avoid memory leaks)
class SerializedMessage {
public:
  /**
   * @brief 构造函数，使用指定的分配器 (Constructor with specified allocator)
   *
   * @param allocator 分配器 (Allocator)
   */
  SerializedMessage(const rcl_allocator_t& allocator) : SerializedMessage(0u, allocator) {}

  /**
   * @brief 构造函数，使用指定的初始容量和分配器 (Constructor with specified initial capacity and
   * allocator)
   *
   * @param initial_capacity 初始容量 (Initial capacity)
   * @param allocator 分配器 (Allocator)
   */
  SerializedMessage(size_t initial_capacity, const rcl_allocator_t& allocator)
      : serialized_message_(rmw_get_zero_initialized_serialized_message()) {
    // 初始化序列化消息 (Initialize the serialized message)
    const auto ret =
        rmw_serialized_message_init(&serialized_message_, initial_capacity, &allocator);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
  }

  /**
   * @brief 拷贝构造函数 (Copy constructor)
   *
   * @param other 另一个 SerializedMessage 对象 (Another SerializedMessage object)
   */
  SerializedMessage(const SerializedMessage& other)
      : SerializedMessage(other.serialized_message_) {}

  /**
   * @brief 拷贝构造函数，从 rcl_serialized_message_t 创建 (Copy constructor, create from
   * rcl_serialized_message_t)
   *
   * @param other 另一个 rcl_serialized_message_t 对象 (Another rcl_serialized_message_t object)
   */
  SerializedMessage(const rcl_serialized_message_t& other)
      : serialized_message_(rmw_get_zero_initialized_serialized_message()) {
    copy_rcl_message(other, serialized_message_);
  }

  /**
   * @brief 移动构造函数 (Move constructor)
   *
   * @param other 另一个 SerializedMessage 对象 (Another SerializedMessage object)
   */
  SerializedMessage(SerializedMessage&& other)
      : serialized_message_(std::exchange(
            other.serialized_message_, rmw_get_zero_initialized_serialized_message())) {}

  /**
   * @brief 移动构造函数，从 rcl_serialized_message_t 创建 (Move constructor, create from
   * rcl_serialized_message_t)
   *
   * @param other 另一个 rcl_serialized_message_t 对象 (Another rcl_serialized_message_t object)
   */
  SerializedMessage(rcl_serialized_message_t&& other)
      : serialized_message_(std::exchange(other, rmw_get_zero_initialized_serialized_message())) {}
};

/// \brief SerializedMessage 类的赋值运算符重载
/// \param other 另一个 SerializedMessage 对象的引用
/// \return 返回 *this 的引用
/// \brief Assignment operator overload for the SerializedMessage class
/// \param other A reference to another SerializedMessage object
/// \return Returns a reference to *this
SerializedMessage& SerializedMessage::operator=(const SerializedMessage& other) {
  // 检查是否是自赋值 (Check for self-assignment)
  if (this != &other) {
    // 初始化序列化消息为零值 (Initialize serialized message to zero value)
    serialized_message_ = rmw_get_zero_initialized_serialized_message();
    // 复制 RCL 消息 (Copy RCL message)
    copy_rcl_message(other.serialized_message_, serialized_message_);
  }

  return *this;
}

/// \brief SerializedMessage 类的赋值运算符重载
/// \param other 另一个 rcl_serialized_message_t 对象的引用
/// \return 返回 *this 的引用
/// \brief Assignment operator overload for the SerializedMessage class
/// \param other A reference to another rcl_serialized_message_t object
/// \return Returns a reference to *this
SerializedMessage& SerializedMessage::operator=(const rcl_serialized_message_t& other) {
  // 检查是否是自赋值 (Check for self-assignment)
  if (&serialized_message_ != &other) {
    // 初始化序列化消息为零值 (Initialize serialized message to zero value)
    serialized_message_ = rmw_get_zero_initialized_serialized_message();
    // 复制 RCL 消息 (Copy RCL message)
    copy_rcl_message(other, serialized_message_);
  }

  return *this;
}

/// \brief SerializedMessage 类的移动赋值运算符重载
/// \param other 另一个 SerializedMessage 对象的右值引用
/// \return 返回 *this 的引用
/// \brief Move assignment operator overload for the SerializedMessage class
/// \param other An rvalue reference to another SerializedMessage object
/// \return Returns a reference to *this
SerializedMessage& SerializedMessage::operator=(SerializedMessage&& other) {
  // 检查是否是自赋值 (Check for self-assignment)
  if (this != &other) {
    // 使用 std::exchange 交换 serialized_message_ 和 other.serialized_message_ 的值
    // (Exchange the values of serialized_message_ and other.serialized_message_ using
    // std::exchange)
    serialized_message_ =
        std::exchange(other.serialized_message_, rmw_get_zero_initialized_serialized_message());
  }

  return *this;
}

/// \brief SerializedMessage 类的移动赋值运算符重载
/// \param other 另一个 rcl_serialized_message_t 对象的右值引用
/// \return 返回 *this 的引用
/// \brief Move assignment operator overload for the SerializedMessage class
/// \param other An rvalue reference to another rcl_serialized_message_t object
/// \return Returns a reference to *this
SerializedMessage& SerializedMessage::operator=(rcl_serialized_message_t&& other) {
  // 检查是否是自赋值 (Check for self-assignment)
  if (&serialized_message_ != &other) {
    // 使用 std::exchange 交换 serialized_message_ 和 other 的值
    // (Exchange the values of serialized_message_ and other using std::exchange)
    serialized_message_ = std::exchange(other, rmw_get_zero_initialized_serialized_message());
  }
  return *this;
}

/// \brief SerializedMessage 类的析构函数
/// \brief Destructor for the SerializedMessage class
SerializedMessage::~SerializedMessage() {
  // 检查序列化消息缓冲区是否为空 (Check if serialized message buffer is not null)
  if (nullptr != serialized_message_.buffer) {
    // 销毁序列化消息 (Destroy serialized message)
    const auto fini_ret = rmw_serialized_message_fini(&serialized_message_);
    // 检查销毁操作是否成功 (Check if the destroy operation was successful)
    if (RCL_RET_OK != fini_ret) {
      RCLCPP_ERROR(
          get_logger("rclcpp"), "Failed to destroy serialized message: %s",
          rcl_get_error_string().str);
    }
  }
}

/// \brief 获取 rcl_serialized_message_t 的引用
/// \return 返回 rcl_serialized_message_t 对象的引用
/// \brief Get a reference to the rcl_serialized_message_t object
/// \return Returns a reference to the rcl_serialized_message_t object
rcl_serialized_message_t& SerializedMessage::get_rcl_serialized_message() {
  return serialized_message_;
}

/// \brief 获取 rcl_serialized_message_t 的常量引用
/// \return 返回 rcl_serialized_message_t 对象的常量引用
/// \brief Get a const reference to the rcl_serialized_message_t object
/// \return Returns a const reference to the rcl_serialized_message_t object
const rcl_serialized_message_t& SerializedMessage::get_rcl_serialized_message() const {
  return serialized_message_;
}

/// \brief 获取序列化消息的大小
/// \return 返回序列化消息的 buffer_length
/// \brief Get the size of the serialized message
/// \return Returns the buffer_length of the serialized message
size_t SerializedMessage::size() const { return serialized_message_.buffer_length; }

/// \brief 获取序列化消息的容量
/// \return 返回序列化消息的 buffer_capacity
/// \brief Get the capacity of the serialized message
/// \return Returns the buffer_capacity of the serialized message
size_t SerializedMessage::capacity() const { return serialized_message_.buffer_capacity; }

/// \brief 为序列化消息预留指定容量
/// \param capacity 需要预留的容量大小
/// \brief Reserve specified capacity for the serialized message
/// \param capacity The capacity size to reserve
void SerializedMessage::reserve(size_t capacity) {
  // 调整序列化消息的大小 (Resize the serialized message)
  auto ret = rmw_serialized_message_resize(&serialized_message_, capacity);
  // 检查调整操作是否成功 (Check if the resize operation was successful)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

/// \brief 释放 rcl_serialized_message_t 对象
/// \return 返回 rcl_serialized_message_t 对象
/// \brief Release the rcl_serialized_message_t object
/// \return Returns the rcl_serialized_message_t object
rcl_serialized_message_t SerializedMessage::release_rcl_serialized_message() {
  // 保存当前的 serialized_message_ 值 (Store the current value of serialized_message_)
  auto ret = serialized_message_;
  // 将 serialized_message_ 初始化为零值 (Initialize serialized_message_ to zero value)
  serialized_message_ = rmw_get_zero_initialized_serialized_message();

  return ret;
}
}  // namespace rclcpp

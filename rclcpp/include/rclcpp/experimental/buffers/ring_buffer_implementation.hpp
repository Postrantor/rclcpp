// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXPERIMENTAL__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_
#define RCLCPP__EXPERIMENTAL__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_

#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rclcpp/experimental/buffers/buffer_implementation_base.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace experimental {
namespace buffers {

/// 存储元素在固定大小的FIFO缓冲区中
/// Store elements in a fixed-size, FIFO buffer
/**
 * 所有公共成员函数都是线程安全的。
 * All public member functions are thread-safe.
 */
template <typename BufferT>
class RingBufferImplementation : public BufferImplementationBase<BufferT> {
public:
  /// 构造函数，设置缓冲区容量
  /// Constructor, setting the buffer capacity
  explicit RingBufferImplementation(size_t capacity)
      : capacity_(capacity),
        ring_buffer_(capacity),
        write_index_(capacity_ - 1),
        read_index_(0),
        size_(0) {
    // 容量必须为正且非零
    // Capacity must be positive and non-zero
    if (capacity == 0) {
      throw std::invalid_argument("capacity must be a positive, non-zero value");
    }
  }

  /// 虚析构函数
  /// Virtual destructor
  virtual ~RingBufferImplementation() {}

  /// 向环形缓冲区添加一个新元素
  /// Add a new element to store in the ring buffer
  /**
   * 此成员函数是线程安全的。
   * This member function is thread-safe.
   *
   * \param request 将要存储在环形缓冲区中的元素
   * \param request the element to be stored in the ring buffer
   */
  void enqueue(BufferT request) {
    // 锁定互斥体以确保线程安全
    // Lock the mutex to ensure thread safety
    std::lock_guard<std::mutex> lock(mutex_);

    // 更新写索引
    // Update the write index
    write_index_ = next_(write_index_);
    // 将新元素移动到环形缓冲区中
    // Move the new element into the ring buffer
    ring_buffer_[write_index_] = std::move(request);

    // 如果缓冲区已满，则更新读索引，否则增加大小
    // If the buffer is full, update the read index, otherwise increment the size
    if (is_full_()) {
      read_index_ = next_(read_index_);
    } else {
      size_++;
    }
  }

  /// 从环形缓冲区中删除最旧的元素
  /// Remove the oldest element from ring buffer
  /**
   * 此成员函数是线程安全的。
   * This member function is thread-safe.
   *
   * \return 被移除的环形缓冲区元素
   * \return the element that is being removed from the ring buffer
   */
  BufferT dequeue() {
    // 锁定互斥体以确保线程安全
    // Lock the mutex to ensure thread safety
    std::lock_guard<std::mutex> lock(mutex_);

    // 如果没有数据，则返回默认构造的元素
    // Return a default constructed element if there is no data
    if (!has_data_()) {
      return BufferT();
    }

    // 移动最旧的元素，并更新读索引
    // Move the oldest element and update the read index
    auto request = std::move(ring_buffer_[read_index_]);
    read_index_ = next_(read_index_);

    // 减小缓冲区大小
    // Decrement the buffer size
    size_--;

    return request;
  }

  /// 获取环形缓冲区的下一个索引值
  /// Get the next index value for the ring buffer
  /**
   * 此成员函数是线程安全的。
   * This member function is thread-safe.
   *
   * \param val 当前索引值
   * \param val the current index value
   * \return 下一个索引值
   * \return the next index value
   */
  inline size_t next(size_t val) {
    // 锁定互斥体以确保线程安全
    // Lock the mutex to ensure thread safety
    std::lock_guard<std::mutex> lock(mutex_);
    return next_(val);
  }

  /// 判断环形缓冲区是否至少存储了一个元素
  /// Get if the ring buffer has at least one element stored
  /**
   * 此成员函数是线程安全的。
   * This member function is thread-safe.
   *
   * \return 如果有数据则返回`true`，否则返回`false`
   * \return `true` if there is data and `false` otherwise
   */
  inline bool has_data() const {
    // 锁定互斥体以确保线程安全
    // Lock the mutex to ensure thread safety
    std::lock_guard<std::mutex> lock(mutex_);
    return has_data_();
  }

  /// 判断缓冲区大小是否等于其容量
  /// Get if the size of the buffer is equal to its capacity
  /**
   * 此成员函数是线程安全的。
   * This member function is thread-safe.
   *
   * \return 如果缓冲区大小等于容量则返回`true`，否则返回`false`
   * \return `true` if the size of the buffer is equal is capacity
   * and `false` otherwise
   */
  inline bool is_full() const {
    // 锁定互斥体以确保线程安全
    // Lock the mutex to ensure thread safety
    std::lock_guard<std::mutex> lock(mutex_);
    return is_full_();
  }

  /// 清空缓冲区
  /// Clear the buffer
  void clear() {}

private:
  /// 获取环形缓冲区的下一个索引值（非线程安全）
  /// Get the next index value for the ring buffer (not thread-safe)
  /**
   * 此成员函数不是线程安全的。
   * This member function is not thread-safe.
   *
   * \param val 当前索引值
   * \param val the current index value
   * \return 下一个索引值
   * \return the next index value
   */
  inline size_t next_(size_t val) { return (val + 1) % capacity_; }

  /// 判断环形缓冲区是否至少存储了一个元素（非线程安全）
  /// Get if the ring buffer has at least one element stored (not thread-safe)
  /**
   * 此成员函数不是线程安全的。
   * This member function is not thread-safe.
   *
   * \return 如果有数据则返回`true`，否则返回`false`
   * \return `true` if there is data and `false` otherwise
   */
  inline bool has_data_() const { return size_ != 0; }

  /// 判断缓冲区大小是否等于其容量（非线程安全）
  /// Get if the size of the buffer is equal to its capacity (not thread-safe)
  /**
   * 此成员函数不是线程安全的。
   * This member function is not thread-safe.
   *
   * \return 如果缓冲区大小等于容量则返回`true`，否则返回`false`
   * \return `true` if the size of the buffer is equal is capacity
   * and `false` otherwise
   */
  inline bool is_full_() const { return size_ == capacity_; }

  size_t capacity_;

  std::vector<BufferT> ring_buffer_;

  size_t write_index_;
  size_t read_index_;
  size_t size_;

  mutable std::mutex mutex_;
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_

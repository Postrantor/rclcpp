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

#include "rclcpp/wait_set_policies/detail/write_preferring_read_write_lock.hpp"

namespace rclcpp {
namespace wait_set_policies {
namespace detail {

/**
 * @brief 构造函数，初始化读写锁对象 (Constructor, initializes the read-write lock object)
 *
 * @param enter_waiting_function 等待函数，在等待时执行 (Waiting function to be executed while
 * waiting)
 */
WritePreferringReadWriteLock::WritePreferringReadWriteLock(
    std::function<void()> enter_waiting_function)
    : read_mutex_(*this), write_mutex_(*this), enter_waiting_function_(enter_waiting_function) {
  // 读互斥体和写互斥体初始化 (Initialize read and write mutexes)
}

/**
 * @brief 获取读互斥体的引用 (Get a reference to the read mutex)
 *
 * @return ReadMutex& 读互斥体引用 (Read mutex reference)
 */
WritePreferringReadWriteLock::ReadMutex& WritePreferringReadWriteLock::get_read_mutex() {
  return read_mutex_;
}

/**
 * @brief 获取写互斥体的引用 (Get a reference to the write mutex)
 *
 * @return WriteMutex& 写互斥体引用 (Write mutex reference)
 */
WritePreferringReadWriteLock::WriteMutex& WritePreferringReadWriteLock::get_write_mutex() {
  return write_mutex_;
}

/**
 * @brief 读互斥体构造函数 (Read mutex constructor)
 *
 * @param parent_lock 父级锁引用 (Parent lock reference)
 */
WritePreferringReadWriteLock::ReadMutex::ReadMutex(WritePreferringReadWriteLock& parent_lock)
    : parent_lock_(parent_lock) {}

/**
 * @brief 加锁读互斥体 (Lock the read mutex)
 */
void WritePreferringReadWriteLock::ReadMutex::lock() {
  std::unique_lock<std::mutex> lock(parent_lock_.mutex_);
  // 当有写者等待或活动时，等待 (Wait while there are writers waiting or active)
  while (parent_lock_.number_of_writers_waiting_ > 0 || parent_lock_.writer_active_ ||
         parent_lock_.reader_active_) {
    parent_lock_.condition_variable_.wait(lock);
  }
  parent_lock_.reader_active_ = true;
  // 隐式解锁 parent_lock_.mutex_ (Implicit unlock of parent_lock_.mutex_)
}

/**
 * @brief 解锁读互斥体 (Unlock the read mutex)
 */
void WritePreferringReadWriteLock::ReadMutex::unlock() {
  std::unique_lock<std::mutex> lock(parent_lock_.mutex_);
  parent_lock_.reader_active_ = false;
  parent_lock_.condition_variable_.notify_all();
  // 隐式解锁 parent_lock_.mutex_ (Implicit unlock of parent_lock_.mutex_)
}

/**
 * @brief 写互斥体构造函数 (Write mutex constructor)
 *
 * @param parent_lock 父级锁引用 (Parent lock reference)
 */
WritePreferringReadWriteLock::WriteMutex::WriteMutex(WritePreferringReadWriteLock& parent_lock)
    : parent_lock_(parent_lock) {}

/**
 * @brief 加锁写互斥体 (Lock the write mutex)
 */
void WritePreferringReadWriteLock::WriteMutex::lock() {
  std::unique_lock<std::mutex> lock(parent_lock_.mutex_);
  parent_lock_.number_of_writers_waiting_ += 1;
  // 调用等待函数 (Call the waiting function)
  if (nullptr != parent_lock_.enter_waiting_function_) {
    parent_lock_.enter_waiting_function_();
  }
  // 当读者或写者活动时，等待 (Wait while there are readers or a writer active)
  while (parent_lock_.reader_active_ || parent_lock_.writer_active_) {
    parent_lock_.condition_variable_.wait(lock);
  }
  parent_lock_.number_of_writers_waiting_ -= 1;
  parent_lock_.writer_active_ = true;
  // 隐式解锁 parent_lock_.mutex_ (Implicit unlock of parent_lock_.mutex_)
}

/**
 * @brief 解锁写互斥体 (Unlock the write mutex)
 */
void WritePreferringReadWriteLock::WriteMutex::unlock() {
  std::unique_lock<std::mutex> lock(parent_lock_.mutex_);
  parent_lock_.writer_active_ = false;
  parent_lock_.condition_variable_.notify_all();
  // 隐式解锁 parent_lock_.mutex_ (Implicit unlock of parent_lock_.mutex_)
}

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

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

#include "./logging_mutex.hpp"

#include <memory>
#include <mutex>

#include "rcutils/macros.h"

/**
 * @brief 获取全局日志互斥锁 (Get the global logging mutex)
 *
 * @return std::shared_ptr<std::recursive_mutex> 全局日志互斥锁的智能指针 (A smart pointer to the
 * global logging mutex)
 */
std::shared_ptr<std::recursive_mutex> get_global_logging_mutex() {
  // 使用 std::make_shared 创建一个 std::recursive_mutex 的智能指针并将其赋值给静态变量 mutex
  // (Create a smart pointer of std::recursive_mutex using std::make_shared and assign it to the
  // static variable mutex)
  static auto mutex = std::make_shared<std::recursive_mutex>();

  // 判断 mutex 是否为空，如果为空，则抛出运行时错误
  // (Check if mutex is nullptr, if so, throw a runtime error)
  if (RCUTILS_UNLIKELY(!mutex)) {
    // 抛出运行时错误，并附加错误信息
    // (Throw a runtime error with an error message)
    throw std::runtime_error("rclcpp global logging mutex is a nullptr");
  }

  // 返回全局日志互斥锁的智能指针
  // (Return the smart pointer to the global logging mutex)
  return mutex;
}

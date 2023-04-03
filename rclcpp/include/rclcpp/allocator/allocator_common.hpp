// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__ALLOCATOR__ALLOCATOR_COMMON_HPP_
#define RCLCPP__ALLOCATOR__ALLOCATOR_COMMON_HPP_

#include <cstring>
#include <memory>

#include "rcl/allocator.h"
#include "rclcpp/allocator/allocator_deleter.hpp"

namespace rclcpp {
namespace allocator {

// 为 T 类型的对象使用 AllocRebind 定义别名，用于重新绑定分配器特性
// Define an alias for AllocRebind for objects of type T, used to rebind allocator traits
template <typename T, typename Alloc>
using AllocRebind = typename std::allocator_traits<Alloc>::template rebind_traits<T>;

// 使用给定的分配器类型为指定大小的内存分配空间
// Allocate memory of specified size using the given allocator type
template <typename Alloc>
void *retyped_allocate(size_t size, void *untyped_allocator) {
  // 将 untyped_allocator 转换为 Alloc 类型的指针
  // Cast untyped_allocator to a pointer of type Alloc
  auto typed_allocator = static_cast<Alloc *>(untyped_allocator);
  if (!typed_allocator) {
    throw std::runtime_error("Received incorrect allocator type");
  }
  // 使用分配器为指定大小的内存分配空间
  // Allocate memory of specified size using the allocator
  return std::allocator_traits<Alloc>::allocate(*typed_allocator, size);
}

// 使用给定的分配器类型为指定数量和元素大小的内存分配并初始化为空值（0）的空间
// Allocate and initialize memory with zero values for specified number of elements and element size
// using the given allocator type
template <typename Alloc>
void *retyped_zero_allocate(size_t number_of_elem, size_t size_of_elem, void *untyped_allocator) {
  // 将 untyped_allocator 转换为 Alloc 类型的指针
  // Cast untyped_allocator to a pointer of type Alloc
  auto typed_allocator = static_cast<Alloc *>(untyped_allocator);
  if (!typed_allocator) {
    throw std::runtime_error("Received incorrect allocator type");
  }
  // 计算需要分配的内存大小
  // Calculate the size of memory to be allocated
  size_t size = number_of_elem * size_of_elem;
  // 使用分配器为指定大小的内存分配空间
  // Allocate memory of specified size using the allocator
  void *allocated_memory = std::allocator_traits<Alloc>::allocate(*typed_allocator, size);
  if (allocated_memory) {
    // 将分配的内存初始化为空值（0）
    // Initialize the allocated memory with zero values
    std::memset(allocated_memory, 0, size);
  }
  return allocated_memory;
}

// 释放给定的分配器类型分配的内存
// Deallocate memory allocated by the given allocator type
template <typename T, typename Alloc>
void retyped_deallocate(void *untyped_pointer, void *untyped_allocator) {
  // 将 untyped_allocator 转换为 Alloc 类型的指针
  // Cast untyped_allocator to a pointer of type Alloc
  auto typed_allocator = static_cast<Alloc *>(untyped_allocator);
  if (!typed_allocator) {
    throw std::runtime_error("Received incorrect allocator type");
  }
  // 将 untyped_pointer 转换为 T 类型的指针
  // Cast untyped_pointer to a pointer of type T
  auto typed_ptr = static_cast<T *>(untyped_pointer);
  // 使用分配器释放内存
  // Deallocate memory using the allocator
  std::allocator_traits<Alloc>::deallocate(*typed_allocator, typed_ptr, 1);
}

// 使用给定的分配器类型重新分配指定大小的内存
// Reallocate memory of specified size using the given allocator type
template <typename T, typename Alloc>
void *retyped_reallocate(void *untyped_pointer, size_t size, void *untyped_allocator) {
  // 将 untyped_allocator 转换为 Alloc 类型的指针
  // Cast untyped_allocator to a pointer of type Alloc
  auto typed_allocator = static_cast<Alloc *>(untyped_allocator);
  if (!typed_allocator) {
    throw std::runtime_error("Received incorrect allocator type");
  }
  // 将 untyped_pointer 转换为 T 类型的指针
  // Cast untyped_pointer to a pointer of type T
  auto typed_ptr = static_cast<T *>(untyped_pointer);
  // 使用分配器释放内存
  // Deallocate memory using the allocator
  std::allocator_traits<Alloc>::deallocate(*typed_allocator, typed_ptr, 1);
  // 使用分配器为指定大小的内存分配空间
  // Allocate memory of specified size using the allocator
  return std::allocator_traits<Alloc>::allocate(*typed_allocator, size);
}

// 将 std::allocator_traits 格式的分配器转换为 rcl 分配器
// Convert a std::allocator_traits-formatted Allocator into an rcl allocator
template <
    typename T,
    typename Alloc,
    typename std::enable_if<!std::is_same<Alloc, std::allocator<void>>::value>::type * = nullptr>
rcl_allocator_t get_rcl_allocator(Alloc &allocator) {
  rcl_allocator_t rcl_allocator = rcl_get_default_allocator();
#ifndef _WIN32
  rcl_allocator.allocate = &retyped_allocate<Alloc>;
  rcl_allocator.zero_allocate = &retyped_zero_allocate<Alloc>;
  rcl_allocator.deallocate = &retyped_deallocate<T, Alloc>;
  rcl_allocator.reallocate = &retyped_reallocate<T, Alloc>;
  rcl_allocator.state = &allocator;
#else
  (void)allocator;  // Remove warning
#endif
  return rcl_allocator;
}

// 对于不完整的 std::allocator<void> 实现的解决方法
// Workaround for an incomplete implementation of std::allocator<void>
template <
    typename T,
    typename Alloc,
    typename std::enable_if<std::is_same<Alloc, std::allocator<void>>::value>::type * = nullptr>
rcl_allocator_t get_rcl_allocator(Alloc &allocator) {
  (void)allocator;
  return rcl_get_default_allocator();
}

}  // namespace allocator
}  // namespace rclcpp

#endif  // RCLCPP__ALLOCATOR__ALLOCATOR_COMMON_HPP_

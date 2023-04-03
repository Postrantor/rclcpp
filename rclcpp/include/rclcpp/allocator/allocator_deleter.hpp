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

#ifndef RCLCPP__ALLOCATOR__ALLOCATOR_DELETER_HPP_
#define RCLCPP__ALLOCATOR__ALLOCATOR_DELETER_HPP_

#include <memory>
#include <stdexcept>

namespace rclcpp {
namespace allocator {

/**
 * @brief 分配器删除器模板类 (Allocator Deleter template class)
 *
 * @tparam Allocator 分配器类型 (Allocator type)
 */
template <typename Allocator>
class AllocatorDeleter {
  // 使用 AllocRebind 作为 T 类型的分配器重绑定 (Use AllocRebind as the allocator rebind for type T)
  template <typename T>
  using AllocRebind = typename std::allocator_traits<Allocator>::template rebind_alloc<T>;

public:
  /**
   * @brief 默认构造函数 (Default constructor)
   */
  AllocatorDeleter() : allocator_(nullptr) {}

  /**
   * @brief 显式构造函数 (Explicit constructor)
   *
   * @param a 分配器指针 (Allocator pointer)
   */
  explicit AllocatorDeleter(Allocator* a) : allocator_(a) {}

  /**
   * @brief 拷贝构造函数 (Copy constructor)
   *
   * @tparam T 类型参数 (Type parameter)
   * @param a 要拷贝的 AllocatorDeleter 对象 (The AllocatorDeleter object to be copied)
   */
  template <typename T>
  explicit AllocatorDeleter(const AllocatorDeleter<T>& a) {
    allocator_ = a.get_allocator();
  }

  /**
   * @brief 函数调用操作符重载 (Function call operator overload)
   *
   * @tparam T 类型参数 (Type parameter)
   * @param ptr 指向要销毁和释放的对象的指针 (Pointer to the object to be destroyed and deallocated)
   */
  template <typename T>
  void operator()(T* ptr) {
    std::allocator_traits<AllocRebind<T>>::destroy(*allocator_, ptr);
    std::allocator_traits<AllocRebind<T>>::deallocate(*allocator_, ptr, 1);
    ptr = nullptr;
  }

  /**
   * @brief 获取分配器指针 (Get the allocator pointer)
   *
   * @return Allocator* 分配器指针 (Allocator pointer)
   */
  Allocator* get_allocator() const { return allocator_; }

  /**
   * @brief 设置分配器指针 (Set the allocator pointer)
   *
   * @param alloc 分配器指针 (Allocator pointer)
   */
  void set_allocator(Allocator* alloc) { allocator_ = alloc; }

private:
  Allocator* allocator_;  // 分配器指针 (Allocator pointer)
};

/**
 * @brief 为删除器设置分配器 (Set allocator for deleter)
 *
 * @tparam Alloc 分配器类型 (Allocator type)
 * @tparam T 类型参数 (Type parameter)
 * @tparam D 删除器类型 (Deleter type)
 * @param deleter 删除器指针 (Deleter pointer)
 * @param alloc 分配器指针 (Allocator pointer)
 */
template <typename Alloc, typename T, typename D>
void set_allocator_for_deleter(D* deleter, Alloc* alloc) {
  (void)alloc;
  (void)deleter;
  throw std::runtime_error("Reached unexpected template specialization");
}

template <typename T, typename U>
void set_allocator_for_deleter(std::default_delete<T>* deleter, std::allocator<U>* alloc) {
  (void)deleter;
  (void)alloc;
}

template <typename Alloc, typename T>
void set_allocator_for_deleter(AllocatorDeleter<T>* deleter, Alloc* alloc) {
  if (!deleter || !alloc) {
    throw std::invalid_argument("Argument was NULL to set_allocator_for_deleter");
  }
  deleter->set_allocator(alloc);
}

/**
 * @brief Deleter 类型别名模板 (Deleter type alias template)
 *
 * @tparam Alloc 分配器类型 (Allocator type)
 * @tparam T 类型参数 (Type parameter)
 */
template <typename Alloc, typename T>
using Deleter = typename std::conditional<
    std::is_same<
        typename std::allocator_traits<Alloc>::template rebind_alloc<T>,
        std::allocator<T>>::value,
    std::default_delete<T>,
    AllocatorDeleter<Alloc>>::type;
}  // namespace allocator
}  // namespace rclcpp

#endif  // RCLCPP__ALLOCATOR__ALLOCATOR_DELETER_HPP_

// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DETAIL__CPP_CALLBACK_TRAMPOLINE_HPP_
#define RCLCPP__DETAIL__CPP_CALLBACK_TRAMPOLINE_HPP_

#include <functional>

namespace rclcpp {

namespace detail {

/// Trampoline pattern for wrapping std::function into C-style callbacks.
/**
 * 一个常见的C语言模式是函数接受一个函数指针和一个void指针作为 "用户数据"，
 * 当从C中调用时，将这些传递给函数指针。
 * (A common pattern in C is for a function to take a function pointer and a
 * void pointer for "user data" which is passed to the function pointer when it
 * is called from within C.)
 *
 * 它通过使用用户数据指针来存储指向std::function实例的指针来工作。
 * 因此，当从C中调用时，该函数将用户数据转换为正确的std::function类型并调用它。
 * (It works by using the user data pointer to store a pointer to a
 * std::function instance.
 * So when called from C, this function will cast the user data to the right
 * std::function type and call it.)
 *
 * 这应该允许您使用自由函数、带有和不带捕获的lambda以及各种类型的std::bind实例。
 * (This should allow you to use free functions, lambdas with and without
 * captures, and various kinds of std::bind instances.)
 *
 * 此函数的内部可能在C运行时中执行，因此此时不应抛出异常，
 * 并且这样做会导致未定义的行为。
 * (The interior of this function is likely to be executed within a C runtime,
 * so no exceptions should be thrown at this point, and doing so results in
 * undefined behavior.)
 *
 * \tparam UserDataRealT 传递函数的声明类型
 * \tparam UserDataT 基于传递给用户数据的内容推导出的类型，
 *   通常这个类型是 `void *` 或者 `const void *`。
 * \tparam Args 传递给回调的参数
 * \tparam ReturnT 此函数和回调的返回类型，默认为 void
 * \param user_data 函数指针，可能是类型擦除的
 * \param args 要转发给回调的参数
 * \returns 回调返回的内容（如果有）
 */
template <typename UserDataRealT, typename UserDataT, typename... Args, typename ReturnT = void>
ReturnT cpp_callback_trampoline(UserDataT user_data, Args... args) noexcept {
  // 获取实际回调函数引用，并将用户数据从 UserDataT 类型转换为 UserDataRealT 类型
  auto &actual_callback = *static_cast<const UserDataRealT *>(user_data);
  // 调用实际回调函数并传递参数
  return actual_callback(args...);
}

}  // namespace detail

}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__CPP_CALLBACK_TRAMPOLINE_HPP_

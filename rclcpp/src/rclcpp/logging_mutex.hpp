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

#ifndef RCLCPP__LOGGING_MUTEX_HPP_
#define RCLCPP__LOGGING_MUTEX_HPP_

#include <memory>
#include <mutex>

#include "rclcpp/visibility_control.hpp"

/// 全局日志互斥锁 (Global logging mutex)
/**
 * 在以下情况下会锁定此互斥锁：(This mutex is locked in the following situations:)
 *   - 在上下文的初始化/销毁期间。(In initialization/destruction of contexts.)
 *   - 在节点的初始化/销毁期间。(In initialization/destruction of nodes.)
 *   - 在 rclcpp 安装的 rcl 日志输出处理程序中，(In the rcl logging output handler installed by rclcpp,)
 *     也就是说：在所有对记录器宏的调用中，包括 RCUTILS_* 之类的。(i.e.: in all calls to the logger macros, including RCUTILS_* ones.)
 */
// 实现细节：(Implementation detail:)
// 使用指向互斥锁的共享指针，以便需要在销毁时使用它的对象可以保持其生存。(A shared pointer to the mutex is used, so that objects that need to use)
// 通过这种方式，避免了静态对象之间的销毁顺序问题。(it at destruction time can hold it alive. In that way, a destruction ordering problem between static objects is avoided.)
RCLCPP_LOCAL
std::shared_ptr<std::recursive_mutex> get_global_logging_mutex();

#endif // RCLCPP__LOGGING_MUTEX_HPP_

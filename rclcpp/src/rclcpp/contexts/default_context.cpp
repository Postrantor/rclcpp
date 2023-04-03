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

#include "rclcpp/contexts/default_context.hpp"

using rclcpp::contexts::DefaultContext;

/**
 * @brief 默认的构造函数，创建一个 DefaultContext 对象（Default constructor for creating a
 * DefaultContext object）
 */
DefaultContext::DefaultContext() {}

/**
 * @brief 获取全局默认上下文的函数（Function to get the global default context）
 *
 * @return 返回一个指向 DefaultContext 的共享指针（Returns a shared pointer to the DefaultContext）
 */
DefaultContext::SharedPtr rclcpp::contexts::get_global_default_context() {
  // 定义一个静态共享指针 default_context，并使用 make_shared 初始化（Define a static shared pointer
  // named default_context and initialize it using make_shared）
  static DefaultContext::SharedPtr default_context = DefaultContext::make_shared();

  // 返回 default_context（Return the default_context）
  return default_context;
}

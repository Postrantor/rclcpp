// Copyright 2014-2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTOR_OPTIONS_HPP_
#define RCLCPP__EXECUTOR_OPTIONS_HPP_

#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/// Options to be passed to the executor constructor.
// ExecutorOptions 结构体定义（Define the ExecutorOptions struct）
struct ExecutorOptions {
  // 默认构造函数（Default constructor）
  ExecutorOptions()
      // 初始化 memory_strategy 为默认内存策略（Initialize memory_strategy with the default memory
      // strategy）
      : memory_strategy(rclcpp::memory_strategies::create_default_strategy()),
        // 初始化 context 为全局默认上下文（Initialize context with the global default context）
        context(rclcpp::contexts::get_global_default_context()),
        // 初始化 max_conditions 为 0（Initialize max_conditions to 0）
        max_conditions(0) {}

  // 内存策略共享指针（Shared pointer for memory strategy）
  // 用于管理内存资源分配和回收（Used for managing memory resource allocation and deallocation）
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy;

  // 上下文共享指针（Shared pointer for context）
  // 用于存储 ROS2 节点的相关信息（Used for storing information related to the ROS2 node）
  rclcpp::Context::SharedPtr context;

  // 最大条件变量数量（Maximum number of conditions）
  // 用于限制执行器中可处理的事件数量（Used to limit the number of events that can be handled in the
  // executor）
  size_t max_conditions;
};

}  // namespace rclcpp

#endif  // RCLCPP__EXECUTOR_OPTIONS_HPP_

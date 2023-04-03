// Copyright 2014 Open Source Robotics Foundation, Inc.
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

/** \mainpage rclcpp: ROS Client Library for C++
 *
 * `rclcpp` 提供了与 ROS 交互的规范 C++ API。
 * 它包括以下主要组件：
 *
 * - Node（节点）
 *   - rclcpp::Node
 *   - rclcpp/node.hpp
 * - Publisher（发布器）
 *   - rclcpp::Node::create_publisher()
 *   - rclcpp::Publisher
 *   - rclcpp::Publisher::publish()
 *   - rclcpp/publisher.hpp
 * - Subscription（订阅器）
 *   - rclcpp::Node::create_subscription()
 *   - rclcpp::Subscription
 *   - rclcpp/subscription.hpp
 * - Service Client（服务客户端）
 *   - rclcpp::Node::create_client()
 *   - rclcpp::Client
 *   - rclcpp/client.hpp
 * - Service Server（服务服务器）
 *   - rclcpp::Node::create_service()
 *   - rclcpp::Service
 *   - rclcpp/service.hpp
 * - Timer（定时器）
 *   - rclcpp::Node::create_wall_timer()
 *   - rclcpp::WallTimer
 *   - rclcpp::TimerBase
 *   - rclcpp/timer.hpp
 * - Parameters（参数）:
 *   - rclcpp::Node::set_parameters()
 *   - rclcpp::Node::get_parameters()
 *   - rclcpp::Node::get_parameter()
 *   - rclcpp::Node::describe_parameters()
 *   - rclcpp::Node::list_parameters()
 *   - rclcpp::Node::add_on_set_parameters_callback()
 *   - rclcpp::Node::remove_on_set_parameters_callback()
 *   - rclcpp::Parameter
 *   - rclcpp::ParameterValue
 *   - rclcpp::AsyncParametersClient
 *   - rclcpp::SyncParametersClient
 *   - rclcpp/parameter.hpp
 *   - rclcpp/parameter_value.hpp
 *   - rclcpp/parameter_client.hpp
 *   - rclcpp/parameter_service.hpp
 * - Rate（速率）:
 *   - rclcpp::Rate
 *   - rclcpp::WallRate
 *   - rclcpp/rate.hpp
 *
 * 还有一些组件可以帮助控制回调的执行：
 *
 * - Executors（通过阻塞 spin 负责回调的执行）:
 *   - rclcpp::spin()
 *   - rclcpp::spin_some()
 *   - rclcpp::spin_until_future_complete()
 *   - rclcpp::executors::SingleThreadedExecutor
 *   - rclcpp::executors::SingleThreadedExecutor::add_node()
 *   - rclcpp::executors::SingleThreadedExecutor::spin()
 *   - rclcpp::executors::MultiThreadedExecutor
 *   - rclcpp::executors::MultiThreadedExecutor::add_node()
 *   - rclcpp::executors::MultiThreadedExecutor::spin()
 *   - rclcpp/executor.hpp
 *   - rclcpp/executors.hpp
 *   - rclcpp/executors/single_threaded_executor.hpp
 *   - rclcpp/executors/multi_threaded_executor.hpp
 * - CallbackGroups（用于为回调强制实施并发规则的机制）:
 *   - rclcpp::Node::create_callback_group()
 *   - rclcpp::CallbackGroup
 *   - rclcpp/callback_group.hpp
 *
 * 另外，还有一些用于自省 ROS 图的方法：
 *
 * - Graph Events（在图形发生变化时唤醒的可等待事件对象）:
 *   - rclcpp::Node::get_graph_event()
 *   - rclcpp::Node::wait_for_graph_change()
 *   - rclcpp::Event
 * - 列出主题名称和类型：
 *   - rclcpp::Node::get_topic_names_and_types()
 * - 获取主题上发布者或订阅者的数量：
 *   - rclcpp::Node::count_publishers()
 *   - rclcpp::Node::count_subscribers()
 *
 * 以及与日志相关的组件：
 *
 * - 日志宏：
 *   - 一些示例（不详尽）：
 *     - RCLCPP_DEBUG()
 *     - RCLCPP_INFO()
 *     - RCLCPP_WARN_ONCE()
 *     - RCLCPP_ERROR_SKIPFIRST()
 *   - rclcpp/logging.hpp
 * - Logger（记录器）：
 *   - rclcpp::Logger
 *   - rclcpp/logger.hpp
 *   - rclcpp::Node::get_logger()
 *
 * 最后，还有许多内部 API 和实用程序：
 *
 * - 异常：
 *   - rclcpp/exceptions.hpp
 * - 与分配器相关的项目：
 *   - rclcpp/allocator/allocator_common.hpp
 *   - rclcpp/allocator/allocator_deleter.hpp
 * - 通用发布器
 *   - rclcpp::Node::create_generic_publisher()
 *   - rclcpp::GenericPublisher
 *   - rclcpp::GenericPublisher::publish()
 *   - rclcpp/generic_publisher.hpp
 * - 通用订阅器
 *   - rclcpp::Node::create_generic_subscription()
 *   - rclcpp::GenericSubscription
 *   - rclcpp/generic_subscription.hpp
 * - 内存管理工具：
 *   - rclcpp/memory_strategies.hpp
 *   - rclcpp/memory_strategy.hpp
 *   - rclcpp/message_memory_strategy.hpp
 *   - rclcpp/strategies/allocator_memory_strategy.hpp
 *   - rclcpp/strategies/message_pool_memory_strategy.hpp
 * - 在多个节点之间共享的上下文对象：
 *   - rclcpp::Context
 *   - rclcpp/context.hpp
 *   - rclcpp/contexts/default_context.hpp
 * - 各种实用程序：
 *   - rclcpp/duration.hpp
 *   - rclcpp/function_traits.hpp
 *   - rclcpp/macros.hpp
 *   - rclcpp/time.hpp
 *   - rclcpp/utilities.hpp
 *   - rclcpp/typesupport_helpers.hpp
 *   - rclcpp/visibility_control.hpp
 */

#ifndef RCLCPP__RCLCPP_HPP_
#define RCLCPP__RCLCPP_HPP_

#include <csignal>
#include <memory>

#include "rclcpp/executors.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_event_handler.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_set.hpp"
#include "rclcpp/waitable.hpp"

#endif  // RCLCPP__RCLCPP_HPP_

// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/node_interfaces/node_clock.hpp"

#include <memory>
#include <string>

using rclcpp::node_interfaces::NodeClock;

/**
 * @brief 构造一个 NodeClock 对象 (Constructs a NodeClock object)
 *
 * @param node_base 节点基本接口的共享指针 (Shared pointer to the node base interface)
 * @param node_topics 节点主题接口的共享指针 (Shared pointer to the node topics interface)
 * @param node_graph 节点图形接口的共享指针 (Shared pointer to the node graph interface)
 * @param node_services 节点服务接口的共享指针 (Shared pointer to the node services interface)
 * @param node_logging 节点日志接口的共享指针 (Shared pointer to the node logging interface)
 * @param clock_type 时钟类型 (Clock type)
 */
NodeClock::NodeClock(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rcl_clock_type_t clock_type)
    : node_base_(node_base),
      node_topics_(node_topics),
      node_graph_(node_graph),
      node_services_(node_services),
      node_logging_(node_logging),
      clock_(std::make_shared<rclcpp::Clock>(
          clock_type))  // 创建一个新的 rclcpp::Clock 对象并设置时钟类型 (Create a new rclcpp::Clock
                        // object and set the clock type)
{}

// 析构函数 (Destructor)
NodeClock::~NodeClock() {}

/**
 * @brief 获取时钟的共享指针 (Get the shared pointer to the clock)
 *
 * @return rclcpp::Clock::SharedPtr 时钟的共享指针 (Shared pointer to the clock)
 */
rclcpp::Clock::SharedPtr NodeClock::get_clock() { return clock_; }

/**
 * @brief 获取时钟的常量共享指针 (Get the constant shared pointer to the clock)
 *
 * @return rclcpp::Clock::ConstSharedPtr 时钟的常量共享指针 (Constant shared pointer to the clock)
 */
rclcpp::Clock::ConstSharedPtr NodeClock::get_clock() const { return clock_; }

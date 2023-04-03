// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/node_interfaces/node_time_source.hpp"

#include <memory>
#include <string>

using rclcpp::node_interfaces::NodeTimeSource;

/**
 * @brief 构造函数，用于创建 NodeTimeSource 对象。
 * Constructor for creating a NodeTimeSource object.
 *
 * @param node_base 共享指针，指向 NodeBaseInterface 对象。
 *                  Shared pointer to a NodeBaseInterface object.
 * @param node_topics 共享指针，指向 NodeTopicsInterface 对象。
 *                    Shared pointer to a NodeTopicsInterface object.
 * @param node_graph 共享指针，指向 NodeGraphInterface 对象。
 *                   Shared pointer to a NodeGraphInterface object.
 * @param node_services 共享指针，指向 NodeServicesInterface 对象。
 *                      Shared pointer to a NodeServicesInterface object.
 * @param node_logging 共享指针，指向 NodeLoggingInterface 对象。
 *                     Shared pointer to a NodeLoggingInterface object.
 * @param node_clock 共享指针，指向 NodeClockInterface 对象。
 *                   Shared pointer to a NodeClockInterface object.
 * @param node_parameters 共享指针，指向 NodeParametersInterface 对象。
 *                        Shared pointer to a NodeParametersInterface object.
 * @param qos rclcpp::QoS 对象，用于设置 Quality of Service 参数。
 *            rclcpp::QoS object for setting Quality of Service parameters.
 * @param use_clock_thread 布尔值，表示是否使用单独的线程来处理时钟更新。
 *                         Boolean, indicating whether to use a separate thread for clock updates.
 */
NodeTimeSource::NodeTimeSource(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    const rclcpp::QoS& qos,
    bool use_clock_thread)
    : node_base_(node_base),
      node_topics_(node_topics),
      node_graph_(node_graph),
      node_services_(node_services),
      node_logging_(node_logging),
      node_clock_(node_clock),
      node_parameters_(node_parameters),
      time_source_(qos, use_clock_thread)  // 初始化成员变量
{
  // 将节点接口附加到 time_source_ 对象。
  // Attach node interfaces to the time_source_ object.
  time_source_.attachNode(
      node_base_, node_topics_, node_graph_, node_services_, node_logging_, node_clock_,
      node_parameters_);

  // 将时钟附加到 time_source_ 对象。
  // Attach clock to the time_source_ object.
  time_source_.attachClock(node_clock_->get_clock());
}

/**
 * @brief 析构函数，用于销毁 NodeTimeSource 对象。
 * Destructor for destroying a NodeTimeSource object.
 */
NodeTimeSource::~NodeTimeSource() {}

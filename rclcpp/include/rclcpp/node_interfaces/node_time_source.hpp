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

#ifndef RCLCPP__NODE_INTERFACES__NODE_TIME_SOURCE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_TIME_SOURCE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_time_source_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/// 实现 Node API 中的 NodeTimeSource 部分。 (Implementation of the NodeTimeSource part of the Node
/// API)
class NodeTimeSource : public NodeTimeSourceInterface {
public:
  // 只使用智能指针别名 (Use smart pointer aliases only)
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTimeSource)

  // 公共方法声明 (Public method declaration)
  RCLCPP_PUBLIC
  // 显示构造函数，接收多个节点接口类型的智能指针作为参数 (Explicit constructor, taking several node
  // interface type smart pointers as parameters)
  explicit NodeTimeSource(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
          node_base,    // 节点基础接口的智能指针 (Smart pointer to node base interface)
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
          node_topics,  // 节点主题接口的智能指针 (Smart pointer to node topics interface)
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
          node_graph,   // 节点图形接口的智能指针 (Smart pointer to node graph interface)
      rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
          node_services,  // 节点服务接口的智能指针 (Smart pointer to node services interface)
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
          node_logging,  // 节点日志接口的智能指针 (Smart pointer to node logging interface)
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr
          node_clock,  // 节点时钟接口的智能指针 (Smart pointer to node clock interface)
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
          node_parameters,  // 节点参数接口的智能指针 (Smart pointer to node parameters interface)
      const rclcpp::QoS& qos =
          rclcpp::ClockQoS(),  // 指定节点的服务质量 (Specify the Quality of Service for the node)
      bool use_clock_thread = true);  // 是否使用时钟线程 (Whether to use clock thread)

  // 公共方法声明 (Public method declaration)
  RCLCPP_PUBLIC
  // 虚拟析构函数 (Virtual destructor)
  virtual ~NodeTimeSource();

private:
  // 禁用拷贝构造函数和赋值运算符 (Disable copy constructor and assignment operator)
  RCLCPP_DISABLE_COPY(NodeTimeSource)

  // 声明多个节点接口类型的智能指针成员变量 (Declare several node interface type smart pointer
  // member variables)
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;

  // 声明一个时间源对象 (Declare a TimeSource object)
  rclcpp::TimeSource time_source_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_TIME_SOURCE_HPP_

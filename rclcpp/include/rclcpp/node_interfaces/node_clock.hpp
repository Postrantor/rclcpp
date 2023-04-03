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

#ifndef RCLCPP__NODE_INTERFACES__NODE_CLOCK_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_CLOCK_HPP_

#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/// \brief 实现 Node API 中的 NodeClock 部分（Implementation of the NodeClock part of the Node API）
class NodeClock : public NodeClockInterface {
public:
  // 仅使用智能指针别名（Use smart pointer aliases only）
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeClock)

  /// \brief 构造函数（Constructor）
  /// \param node_base 共享指针，指向 NodeBaseInterface 对象（Shared pointer to a NodeBaseInterface
  /// object） \param node_topics 共享指针，指向 NodeTopicsInterface 对象（Shared pointer to a
  /// NodeTopicsInterface object） \param node_graph 共享指针，指向 NodeGraphInterface 对象（Shared
  /// pointer to a NodeGraphInterface object） \param node_services 共享指针，指向
  /// NodeServicesInterface 对象（Shared pointer to a NodeServicesInterface object） \param
  /// node_logging 共享指针，指向 NodeLoggingInterface 对象（Shared pointer to a
  /// NodeLoggingInterface object） \param clock_type 表示时钟类型的枚举值（Enumeration value
  /// representing the clock type）
  RCLCPP_PUBLIC
  explicit NodeClock(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
      rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
      rcl_clock_type_t clock_type);

  /// \brief 析构函数（Destructor）
  RCLCPP_PUBLIC
  virtual ~NodeClock();

  /// \brief 获取一个由节点保持更新的时钟（Get a clock which will be kept up to date by the node）
  /// \return 返回一个共享指针，指向 Clock 对象（Returns a shared pointer to a Clock object）
  RCLCPP_PUBLIC
  rclcpp::Clock::SharedPtr get_clock() override;

  /// \brief 获取一个由节点保持更新的时钟（Get a clock which will be kept up to date by the node）
  /// \return 返回一个常量共享指针，指向 Clock 对象（Returns a const shared pointer to a Clock
  /// object）
  RCLCPP_PUBLIC
  rclcpp::Clock::ConstSharedPtr get_clock() const override;

private:
  // 禁用拷贝构造函数和赋值运算符（Disable copy constructor and assignment operator）
  RCLCPP_DISABLE_COPY(NodeClock)

  // 定义节点接口的共享指针（Shared pointers for node interfaces）
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

  // 定义时钟的共享指针（Shared pointer for the clock）
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_CLOCK_HPP_

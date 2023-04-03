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

#ifndef RCLCPP__ANY_EXECUTABLE_HPP_
#define RCLCPP__ANY_EXECUTABLE_HPP_

#include <memory>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp {

/**
 * @struct AnyExecutable
 * @brief 一个通用的可执行对象，可以是订阅、定时器、服务、客户端或等待对象。
 *        A generic executable object, which can be a subscription, timer, service, client, or
 * waitable.
 */
struct AnyExecutable {
  /// 构造函数。Constructor.
  RCLCPP_PUBLIC
  AnyExecutable();

  /// 虚拟析构函数。Virtual destructor.
  RCLCPP_PUBLIC
  virtual ~AnyExecutable();

  // 只有下面指针列表中的一个会被设置。Only one of the following pointers will be set.
  rclcpp::SubscriptionBase::SharedPtr subscription;  ///< 订阅指针。Subscription pointer.
  rclcpp::TimerBase::SharedPtr timer;                ///< 定时器指针。Timer pointer.
  rclcpp::ServiceBase::SharedPtr service;            ///< 服务指针。Service pointer.
  rclcpp::ClientBase::SharedPtr client;              ///< 客户端指针。Client pointer.
  rclcpp::Waitable::SharedPtr waitable;              ///< 等待对象指针。Waitable pointer.

  // 下面的变量用于保持包含项目的作用域。These are used to keep the scope on the containing items.
  rclcpp::CallbackGroup::SharedPtr callback_group;  ///< 回调组指针。Callback group pointer.
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
      node_base;               ///< 节点基础接口指针。Node base interface pointer.
  std::shared_ptr<void> data;  ///< 数据指针。Data pointer.
};

}  // namespace rclcpp

#endif  // RCLCPP__ANY_EXECUTABLE_HPP_

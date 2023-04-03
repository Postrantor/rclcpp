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

#ifndef RCLCPP__SUBSCRIPTION_WAIT_SET_MASK_HPP_
#define RCLCPP__SUBSCRIPTION_WAIT_SET_MASK_HPP_

#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/// \brief 用于确定订阅的哪些部分被添加到等待集或从中删除的选项（Options used to determine what
/// parts of a subscription get added to or removed from a wait set.）
class RCLCPP_PUBLIC SubscriptionWaitSetMask {
public:
  /// \brief 如果为真，则包含实际订阅。（If true, include the actual subscription.）
  bool include_subscription = true;

  /// \brief 如果为真，则包括附加到订阅的任何事件。（If true, include any events attached to the
  /// subscription.）
  bool include_events = true;

  /// \brief 如果为真，则包括用于处理进程内通信的可等待对象。（If true, include the waitable used to
  /// handle intra process communication.）
  bool include_intra_process_waitable = true;
};

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_WAIT_SET_MASK_HPP_

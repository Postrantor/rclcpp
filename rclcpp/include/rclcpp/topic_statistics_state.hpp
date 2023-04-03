// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__TOPIC_STATISTICS_STATE_HPP_
#define RCLCPP__TOPIC_STATISTICS_STATE_HPP_

namespace rclcpp {

/// \enum TopicStatisticsState
/// \brief 代表主题统计收集器的状态。用作 create_subscriber 的参数。(Represents the state of topic
/// statistics collector. Used as argument in create_subscriber.)
enum class TopicStatisticsState {
  /// \brief 在订阅级别显式启用主题统计。(Explicitly enable topic statistics at subscription level.)
  Enable,
  /// \brief 在订阅级别显式禁用主题统计。(Explicitly disable topic statistics at subscription
  /// level.)
  Disable,
  /// \brief 从节点获取主题统计状态。(Take topic statistics state from the node.)
  NodeDefault
};

}  // namespace rclcpp

#endif  // RCLCPP__TOPIC_STATISTICS_STATE_HPP_

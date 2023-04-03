// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef RCLCPP__DETAIL__RESOLVE_ENABLE_TOPIC_STATISTICS_HPP_
#define RCLCPP__DETAIL__RESOLVE_ENABLE_TOPIC_STATISTICS_HPP_

#include <stdexcept>

#include "rclcpp/topic_statistics_state.hpp"

namespace rclcpp {
namespace detail {

/**
 * @brief 解决是否启用主题统计功能，如果需要，解析 "NodeDefault"
 * @brief Resolve whether or not topic statistics is enabled, resolving "NodeDefault" if needed.
 *
 * @tparam OptionsT 选项类型
 * @tparam OptionsT Option type
 * @tparam NodeBaseT 节点基类类型
 * @tparam NodeBaseT Node base class type
 * @param options 主题统计选项
 * @param options Topic statistic options
 * @param node_base 节点基类引用
 * @param node_base Reference to the node base class
 * @return 是否启用主题统计
 * @return Whether topic statistics is enabled or not
 */
template <typename OptionsT, typename NodeBaseT>
bool resolve_enable_topic_statistics(const OptionsT& options, const NodeBaseT& node_base) {
  // 声明一个布尔变量，表示主题统计是否启用
  // Declare a boolean variable indicating whether topic statistics is enabled
  bool topic_stats_enabled;

  // 根据主题统计选项的状态来设置 topic_stats_enabled 的值
  // Set the value of topic_stats_enabled based on the state of topic statistics options
  switch (options.topic_stats_options.state) {
    // 如果主题统计状态为启用，则将 topic_stats_enabled 设置为 true
    // If the topic statistics state is Enable, set topic_stats_enabled to true
    case TopicStatisticsState::Enable:
      topic_stats_enabled = true;
      break;

    // 如果主题统计状态为禁用，则将 topic_stats_enabled 设置为 false
    // If the topic statistics state is Disable, set topic_stats_enabled to false
    case TopicStatisticsState::Disable:
      topic_stats_enabled = false;
      break;

    // 如果主题统计状态为节点默认值，则从节点基类获取默认值
    // If the topic statistics state is NodeDefault, get the default value from the node base class
    case TopicStatisticsState::NodeDefault:
      topic_stats_enabled = node_base.get_enable_topic_statistics_default();
      break;

    // 如果遇到无法识别的 EnableTopicStatistics 值，则抛出运行时错误
    // Throw a runtime error if an unrecognized EnableTopicStatistics value is encountered
    default:
      throw std::runtime_error("Unrecognized EnableTopicStatistics value");
  }

  // 返回主题统计是否启用的布尔值
  // Return the boolean value indicating whether topic statistics is enabled or not
  return topic_stats_enabled;
}

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__RESOLVE_ENABLE_TOPIC_STATISTICS_HPP_

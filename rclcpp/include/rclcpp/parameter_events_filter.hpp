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

#ifndef RCLCPP__PARAMETER_EVENTS_FILTER_HPP_
#define RCLCPP__PARAMETER_EVENTS_FILTER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"

namespace rclcpp {

/**
 * @brief ParameterEventsFilter 类用于过滤参数事件
 *        The ParameterEventsFilter class is used to filter parameter events.
 */
class ParameterEventsFilter {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ParameterEventsFilter)

  /**
   * @brief EventType 枚举类型表示事件类型 (新建、删除、修改)
   *        An enum for the type of event (NEW, DELETED, CHANGED).
   */
  enum class EventType { NEW, DELETED, CHANGED };

  /// 用于列出结果的 EventPair 类型定义
  /// EventPair type definition used for listing results
  using EventPair = std::pair<EventType, const rcl_interfaces::msg::Parameter *>;

  /**
   * @brief 构造一个参数事件的过滤视图。
   *        Construct a filtered view of a parameter event.
   *
   * @param[in] event 要过滤的参数事件消息。
   *                  The parameter event message to filter.
   * @param[in] names 感兴趣的参数名称列表。
   *                  A list of parameter names of interest.
   * @param[in] types 感兴趣的参数事件类型列表。
   *                  A list of the types of parameter events of interest.
   *                  EventType NEW, DELETED, or CHANGED
   *
   * 示例用法 (Example Usage)：
   *
   * 如果你收到了一个参数事件，并且只对参数 foo 和 bar 的添加或更改感兴趣，但不关心删除。
   * If you have received a parameter event and are only interested in parameters foo and
   * bar being added or changed but don't care about deletion.
   *
   * ```cpp
   * auto res = rclcpp::ParameterEventsFilter(
   *   event_shared_ptr,
   *   {"foo", "bar"},
   *   {rclcpp::ParameterEventsFilter::EventType::NEW,
   * rclcpp::ParameterEventsFilter::EventType::CHANGED});
   * ```
   */
  RCLCPP_PUBLIC
  ParameterEventsFilter(
      std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event,
      const std::vector<std::string> &names,
      const std::vector<EventType> &types);

  /**
   * @brief 获取过滤结果
   *        Get the result of the filter
   *
   * @return 匹配此事件中所有参数更改的 std::vector<EventPair>
   *         A std::vector<EventPair> of all matching parameter changes in this event.
   */
  RCLCPP_PUBLIC
  const std::vector<EventPair> &get_events() const;

private:
  // 仅允许通过 const 访问器访问。
  // Access only allowed via const accessor.
  std::vector<EventPair> result_;  ///< 存储结果向量 Storage of the resultant vector
  std::shared_ptr<const rcl_interfaces::msg::ParameterEvent>
      event_;                      ///< 保持事件在作用域内 Keep event in scope
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_EVENTS_FILTER_HPP_

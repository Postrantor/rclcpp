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

#include "rclcpp/parameter_events_filter.hpp"

#include <memory>
#include <string>
#include <vector>

using rclcpp::ParameterEventsFilter;
using EventType = rclcpp::ParameterEventsFilter::EventType;
using EventPair = rclcpp::ParameterEventsFilter::EventPair;

/**
 * @brief 构造函数，用于过滤参数事件 (Constructor for filtering parameter events)
 *
 * @param event 一个指向参数事件消息的共享指针 (A shared pointer to a parameter event message)
 * @param names 要过滤的参数名称列表 (A list of parameter names to filter)
 * @param types 要过滤的事件类型列表 (A list of event types to filter)
 */
ParameterEventsFilter::ParameterEventsFilter(
    std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event,
    const std::vector<std::string>& names,
    const std::vector<EventType>& types)
    : event_(event)  // 初始化 event_ 成员变量 (Initialize the event_ member variable)
{
  // 如果类型列表中包含 EventType::NEW (If the types list contains EventType::NEW)
  if (std::find(types.begin(), types.end(), EventType::NEW) != types.end()) {
    // 遍历 event 中的新参数 (Iterate through new parameters in the event)
    for (auto& new_parameter : event->new_parameters) {
      // 如果名称列表中包含新参数的名称 (If the names list contains the name of the new parameter)
      if (std::find(names.begin(), names.end(), new_parameter.name) != names.end()) {
        // 将新参数添加到结果列表中 (Add the new parameter to the result list)
        result_.push_back(EventPair(EventType::NEW, &new_parameter));
      }
    }
  }

  // 如果类型列表中包含 EventType::CHANGED (If the types list contains EventType::CHANGED)
  if (std::find(types.begin(), types.end(), EventType::CHANGED) != types.end()) {
    // 遍历 event 中的已更改参数 (Iterate through changed parameters in the event)
    for (auto& changed_parameter : event->changed_parameters) {
      // 如果名称列表中包含已更改参数的名称 (If the names list contains the name of the changed
      // parameter)
      if (std::find(names.begin(), names.end(), changed_parameter.name) != names.end()) {
        // 将已更改参数添加到结果列表中 (Add the changed parameter to the result list)
        result_.push_back(EventPair(EventType::CHANGED, &changed_parameter));
      }
    }
  }

  // 如果类型列表中包含 EventType::DELETED (If the types list contains EventType::DELETED)
  if (std::find(types.begin(), types.end(), EventType::DELETED) != types.end()) {
    // 遍历 event 中的已删除参数 (Iterate through deleted parameters in the event)
    for (auto& deleted_parameter : event->deleted_parameters) {
      // 如果名称列表中包含已删除参数的名称 (If the names list contains the name of the deleted
      // parameter)
      if (std::find(names.begin(), names.end(), deleted_parameter.name) != names.end()) {
        // 将已删除参数添加到结果列表中 (Add the deleted parameter to the result list)
        result_.push_back(EventPair(EventType::DELETED, &deleted_parameter));
      }
    }
  }
}

/**
 * @brief 获取过滤后的事件对列表 (Get the filtered list of event pairs)
 *
 * @return 过滤后的事件对列表的引用 (A reference to the filtered list of event pairs)
 */
const std::vector<EventPair>& ParameterEventsFilter::get_events() const { return result_; }

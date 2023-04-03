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

#include "rclcpp/experimental/subscription_intra_process_base.hpp"

#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"

using rclcpp::experimental::SubscriptionIntraProcessBase;

/**
 * @brief 将订阅器内部进程添加到等待集中 (Add the subscription intra-process to the wait set)
 *
 * @param wait_set 指向rcl_wait_set_t结构体的指针，用于存储等待集信息 (Pointer to rcl_wait_set_t
 * structure, used for storing wait set information)
 */
void SubscriptionIntraProcessBase::add_to_wait_set(rcl_wait_set_t* wait_set) {
  // 将guard condition添加到rcl_wait_set中 (Add the guard condition to the rcl_wait_set)
  // 使用gc_成员变量作为guard condition (Using the gc_ member variable as the guard condition)
  detail::add_guard_condition_to_rcl_wait_set(*wait_set, gc_);
}

/**
 * @brief 获取主题名称 (Get the topic name)
 *
 * @return const char* 主题名称字符串 (Topic name string)
 */
const char* SubscriptionIntraProcessBase::get_topic_name() const {
  // 返回topic_name_成员变量的C风格字符串表示形式 (Return the C-style string representation of the
  // topic_name_ member variable)
  return topic_name_.c_str();
}

/**
 * @brief 获取实际QoS配置 (Get the actual QoS configuration)
 *
 * @return rclcpp::QoS 实际QoS配置对象 (Actual QoS configuration object)
 */
rclcpp::QoS SubscriptionIntraProcessBase::get_actual_qos() const {
  // 返回qos_profile_成员变量，表示实际的QoS配置 (Return the qos_profile_ member variable,
  // representing the actual QoS configuration)
  return qos_profile_;
}

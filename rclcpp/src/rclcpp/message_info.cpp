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

#include "rclcpp/message_info.hpp"

namespace rclcpp {

/**
 * @brief 构造函数，用于初始化 MessageInfo 类的实例 (Constructor for initializing an instance of the
 * MessageInfo class)
 *
 * @param rmw_message_info 一个包含消息信息的 rmw_message_info_t 结构体实例 (An instance of
 * rmw_message_info_t structure containing message information)
 */
MessageInfo::MessageInfo(const rmw_message_info_t& rmw_message_info)
    : rmw_message_info_(
          rmw_message_info)  // 初始化 rmw_message_info_ 成员变量为传入的 rmw_message_info 参数
                             // (Initialize the rmw_message_info_ member variable with the passed
                             // rmw_message_info parameter)
{}

/**
 * @brief 析构函数 (Destructor)
 */
MessageInfo::~MessageInfo() {}

/**
 * @brief 获取 rmw_message_info_ 的常量引用 (Get a constant reference to rmw_message_info_)
 *
 * @return 返回 rmw_message_info_ 的常量引用 (Return a constant reference to rmw_message_info_)
 */
const rmw_message_info_t& MessageInfo::get_rmw_message_info() const { return rmw_message_info_; }

/**
 * @brief 获取 rmw_message_info_ 的引用 (Get a reference to rmw_message_info_)
 *
 * @return 返回 rmw_message_info_ 的引用 (Return a reference to rmw_message_info_)
 */
rmw_message_info_t& MessageInfo::get_rmw_message_info() { return rmw_message_info_; }

}  // namespace rclcpp

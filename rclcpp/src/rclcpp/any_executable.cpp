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

#include "rclcpp/any_executable.hpp"

using rclcpp::AnyExecutable;

/**
 * @brief 构造函数，初始化 AnyExecutable 类的成员变量。
 * @brief Constructor, initializes the member variables of the AnyExecutable class.
 */
AnyExecutable::AnyExecutable()
    : subscription(nullptr),    // 初始化订阅者指针为空指针
      timer(nullptr),           // 初始化计时器指针为空指针
      service(nullptr),         // 初始化服务指针为空指针
      client(nullptr),          // 初始化客户端指针为空指针
      callback_group(nullptr),  // 初始化回调组指针为空指针
      node_base(nullptr)        // 初始化节点基类指针为空指针
{}

/**
 * @brief 析构函数，用于在销毁 AnyExecutable 对象时执行一些清理操作。
 *        Destructor, used to perform some cleanup operations when destroying an AnyExecutable
 * object.
 */
AnyExecutable::~AnyExecutable() {
  // 确保已丢弃（已取出但未执行）的 AnyExecutable 对象重置其回调组。
  // 这可能发生在执行器在取出 AnyExecutable 和执行它之间被取消的情况下。
  // Make sure that discarded (taken but not executed) AnyExecutable's have
  // their callback groups reset. This can happen when an executor is canceled
  // between taking an AnyExecutable and executing it.
  if (callback_group) {
    // 将回调组的 can_be_taken_from() 原子变量设置为 true，表示可以从该回调组中获取任何可执行对象。
    // Set the can_be_taken_from() atomic variable of the callback group to true, indicating that
    // any executable can be taken from this callback group.
    callback_group->can_be_taken_from().store(true);
  }
}

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

#ifndef RCLCPP__DETAIL__RESOLVE_USE_INTRA_PROCESS_HPP_
#define RCLCPP__DETAIL__RESOLVE_USE_INTRA_PROCESS_HPP_

#include <stdexcept>

#include "rclcpp/intra_process_setting.hpp"

namespace rclcpp {

namespace detail {

/**
 * @brief 解决是否启用内部进程通信，如果需要，解析 "NodeDefault"。 (Resolve whether or not intra
 * process communication is enabled, resolving "NodeDefault" if needed.)
 *
 * @tparam OptionsT 选项类型。 (Option type.)
 * @tparam NodeBaseT 节点基类类型。 (Node base class type.)
 * @param options 传入的选项参数。 (The input option parameters.)
 * @param node_base 传入的节点基类对象。 (The input node base object.)
 * @return 是否启用内部进程通信。 (Whether intra process communication is enabled or not.)
 */
template <typename OptionsT, typename NodeBaseT>
bool resolve_use_intra_process(const OptionsT& options, const NodeBaseT& node_base) {
  bool use_intra_process;  // 定义一个布尔值变量，用于表示是否启用内部进程通信。 (Define a boolean
                           // variable to indicate whether intra-process communication is enabled.)

  // 使用 switch 语句根据选项中的 use_intra_process_comm 值来设置 use_intra_process 变量。 (Use a
  // switch statement to set the use_intra_process variable based on the use_intra_process_comm
  // value in the options.)
  switch (options.use_intra_process_comm) {
    case IntraProcessSetting::Enable:  // 如果设置为 Enable，则启用内部进程通信。 (If set to Enable,
                                       // enable intra-process communication.)
      use_intra_process = true;
      break;
    case IntraProcessSetting::Disable:  // 如果设置为 Disable，则禁用内部进程通信。 (If set to
                                        // Disable, disable intra-process communication.)
      use_intra_process = false;
      break;
    case IntraProcessSetting::NodeDefault:  // 如果设置为
                                            // NodeDefault，则从节点基类中获取默认的内部进程通信设置。
                                            // (If set to NodeDefault, get the default intra-process
                                            // communication setting from the node base class.)
      use_intra_process = node_base.get_use_intra_process_default();
      break;
    default:  // 如果遇到未识别的 IntraProcessSetting 值，则抛出运行时错误。 (If an unrecognized
              // IntraProcessSetting value is encountered, throw a runtime error.)
      throw std::runtime_error("Unrecognized IntraProcessSetting value");
      break;
  }

  return use_intra_process;  // 返回是否启用内部进程通信。 (Return whether intra-process
                             // communication is enabled or not.)
}

}  // namespace detail

}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__RESOLVE_USE_INTRA_PROCESS_HPP_

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

#ifndef RCLCPP__DETAIL__RESOLVE_INTRA_PROCESS_BUFFER_TYPE_HPP_
#define RCLCPP__DETAIL__RESOLVE_INTRA_PROCESS_BUFFER_TYPE_HPP_

#include <stdexcept>

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/intra_process_buffer_type.hpp"

namespace rclcpp {

namespace detail {

/**
 * @brief 返回缓冲区类型，如果需要，将 "CallbackDefault" 类型解析为实际类型。
 *        (Return the buffer type, resolving the "CallbackDefault" type to an actual type if
 * needed.)
 *
 * @tparam CallbackMessageT 回调消息类型 (Callback message type)
 * @tparam AllocatorT 分配器类型 (Allocator type)
 * @param buffer_type 缓冲区类型 (Buffer type)
 * @param any_subscription_callback 任意订阅回调 (Any subscription callback)
 * @return rclcpp::IntraProcessBufferType 解析后的缓冲区类型 (Resolved buffer type)
 */
template <typename CallbackMessageT, typename AllocatorT>
rclcpp::IntraProcessBufferType resolve_intra_process_buffer_type(
    const rclcpp::IntraProcessBufferType buffer_type,
    const rclcpp::AnySubscriptionCallback<CallbackMessageT, AllocatorT>&
        any_subscription_callback) {
  // 初始化解析后的缓冲区类型为传入的缓冲区类型
  // (Initialize the resolved buffer type to the passed-in buffer type)
  rclcpp::IntraProcessBufferType resolved_buffer_type = buffer_type;

  // 如果用户没有为进程内缓冲区指定类型，则使用回调的类型
  // (If the user has not specified a type for the intra-process buffer, use the callback's type)
  if (resolved_buffer_type == IntraProcessBufferType::CallbackDefault) {
    // 如果使用 take_shared 方法，则将解析后的缓冲区类型设置为 SharedPtr
    // (If using the take_shared_method, set the resolved buffer type to SharedPtr)
    if (any_subscription_callback.use_take_shared_method()) {
      resolved_buffer_type = IntraProcessBufferType::SharedPtr;
    } else {
      // 否则，将解析后的缓冲区类型设置为 UniquePtr
      // (Otherwise, set the resolved buffer type to UniquePtr)
      resolved_buffer_type = IntraProcessBufferType::UniquePtr;
    }
  }

  // 返回解析后的缓冲区类型 (Return the resolved buffer type)
  return resolved_buffer_type;
}

}  // namespace detail

}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__RESOLVE_INTRA_PROCESS_BUFFER_TYPE_HPP_

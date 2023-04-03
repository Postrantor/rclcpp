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

#ifndef RCLCPP__MESSAGE_MEMORY_STRATEGY_HPP_
#define RCLCPP__MESSAGE_MEMORY_STRATEGY_HPP_

#include <memory>
#include <stdexcept>

#include "rcl/types.h"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/serialized_message.h"

namespace rclcpp {
namespace message_memory_strategy {

/// 默认订阅接收到的消息的分配策略。
/// Default allocation strategy for messages received by subscriptions.
/** 消息内存策略必须是订阅所属类型的模板。*/
/** A message memory strategy must be templated on the type of the subscription it belongs to. */
template <typename MessageT, typename Alloc = std::allocator<void>>
class MessageMemoryStrategy {
public:
  // 定义智能指针
  // Smart pointer definitions
  RCLCPP_SMART_PTR_DEFINITIONS(MessageMemoryStrategy)

  // 定义消息分配器相关类型
  // Define message allocator related types
  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, MessageT>;

  // 定义序列化消息分配器相关类型
  // Define serialized message allocator related types
  using SerializedMessageAllocTraits = allocator::AllocRebind<rclcpp::SerializedMessage, Alloc>;
  using SerializedMessageAlloc = typename SerializedMessageAllocTraits::allocator_type;
  using SerializedMessageDeleter =
      allocator::Deleter<SerializedMessageAlloc, rclcpp::SerializedMessage>;

  // 定义缓冲区分配器相关类型
  // Define buffer allocator related types
  using BufferAllocTraits = allocator::AllocRebind<char, Alloc>;
  using BufferAlloc = typename BufferAllocTraits::allocator_type;
  using BufferDeleter = allocator::Deleter<BufferAlloc, char>;

  // 默认构造函数
  // Default constructor
  MessageMemoryStrategy() {
    message_allocator_ = std::make_shared<MessageAlloc>();
    serialized_message_allocator_ = std::make_shared<SerializedMessageAlloc>();
    buffer_allocator_ = std::make_shared<BufferAlloc>();
    rcutils_allocator_ = allocator::get_rcl_allocator<char, BufferAlloc>(*buffer_allocator_.get());
  }

  // 显式构造函数
  // Explicit constructor
  explicit MessageMemoryStrategy(std::shared_ptr<Alloc> allocator) {
    message_allocator_ = std::make_shared<MessageAlloc>(*allocator.get());
    serialized_message_allocator_ = std::make_shared<SerializedMessageAlloc>(*allocator.get());
    buffer_allocator_ = std::make_shared<BufferAlloc>(*allocator.get());
    rcutils_allocator_ = allocator::get_rcl_allocator<char, BufferAlloc>(*buffer_allocator_.get());
  }

  // 虚析构函数
  // Virtual destructor
  virtual ~MessageMemoryStrategy() = default;

  /// 默认工厂方法
  /// Default factory method
  static SharedPtr create_default() {
    return std::make_shared<MessageMemoryStrategy<MessageT, Alloc>>(std::make_shared<Alloc>());
  }

  /// 默认情况下，动态分配一个新消息。
  /// By default, dynamically allocate a new message.
  /** \return 新消息的共享指针。 */
  /** \return Shared pointer to the new message. */
  virtual std::shared_ptr<MessageT> borrow_message() {
    return std::allocate_shared<MessageT, MessageAlloc>(*message_allocator_.get());
  }

  virtual std::shared_ptr<rclcpp::SerializedMessage> borrow_serialized_message(size_t capacity) {
    return std::make_shared<rclcpp::SerializedMessage>(capacity);
  }

  virtual std::shared_ptr<rclcpp::SerializedMessage> borrow_serialized_message() {
    return borrow_serialized_message(default_buffer_capacity_);
  }

  virtual void set_default_buffer_capacity(size_t capacity) { default_buffer_capacity_ = capacity; }

  /// 释放消息的所有权，如果没有更多的所有者，将对其进行取消分配。
  /// Release ownership of the message, which will deallocate it if it has no more owners.
  /** \param[in] msg 我们要返回的消息的共享指针。 */
  /** \param[in] msg Shared pointer to the message we are returning. */
  virtual void return_message(std::shared_ptr<MessageT>& msg) { msg.reset(); }

  virtual void return_serialized_message(
      std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg) {
    serialized_msg.reset();
  }

  std::shared_ptr<MessageAlloc> message_allocator_;
  MessageDeleter message_deleter_;

  std::shared_ptr<SerializedMessageAlloc> serialized_message_allocator_;
  SerializedMessageDeleter serialized_message_deleter_;

  std::shared_ptr<BufferAlloc> buffer_allocator_;
  BufferDeleter buffer_deleter_;
  size_t default_buffer_capacity_ = 0;

  rcutils_allocator_t rcutils_allocator_;
};

}  // namespace message_memory_strategy
}  // namespace rclcpp

#endif  // RCLCPP__MESSAGE_MEMORY_STRATEGY_HPP_

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

#ifndef RCLCPP__EXPERIMENTAL__CREATE_INTRA_PROCESS_BUFFER_HPP_
#define RCLCPP__EXPERIMENTAL__CREATE_INTRA_PROCESS_BUFFER_HPP_

#include <memory>
#include <stdexcept>
#include <utility>

#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "rclcpp/experimental/buffers/ring_buffer_implementation.hpp"
#include "rclcpp/intra_process_buffer_type.hpp"
#include "rclcpp/qos.hpp"

namespace rclcpp {
namespace experimental {

/**
 * @brief 创建一个内部进程缓冲区 (Create an intra-process buffer)
 *
 * @tparam MessageT 消息类型 (Message type)
 * @tparam Alloc 分配器类型，默认为 std::allocator<void> (Allocator type, default is
 * std::allocator<void>)
 * @tparam Deleter 删除器类型，默认为 std::default_delete<MessageT> (Deleter type, default is
 * std::default_delete<MessageT>)
 * @param buffer_type 内部进程缓冲区类型 (Intra-process buffer type)
 * @param qos Quality of Service 设置 (Quality of Service settings)
 * @param allocator 分配器实例的共享指针 (Shared pointer to the allocator instance)
 * @return rclcpp::experimental::buffers::IntraProcessBuffer<MessageT, Alloc, Deleter>::UniquePtr
 * 创建的内部进程缓冲区指针 (Pointer to the created intra-process buffer)
 */
template <
    typename MessageT,
    typename Alloc = std::allocator<void>,
    typename Deleter = std::default_delete<MessageT>>
typename rclcpp::experimental::buffers::IntraProcessBuffer<MessageT, Alloc, Deleter>::UniquePtr
create_intra_process_buffer(
    IntraProcessBufferType buffer_type, const rclcpp::QoS& qos, std::shared_ptr<Alloc> allocator) {
  // 定义消息共享指针和唯一指针类型 (Define message shared pointer and unique pointer types)
  using MessageSharedPtr = std::shared_ptr<const MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, Deleter>;

  // 获取缓冲区大小，即 QoS 中设置的深度 (Get buffer size, which is the depth set in QoS)
  size_t buffer_size = qos.depth();

  // 定义内部进程缓冲区类型别名 (Define intra-process buffer type alias)
  using rclcpp::experimental::buffers::IntraProcessBuffer;
  typename IntraProcessBuffer<MessageT, Alloc, Deleter>::UniquePtr buffer;

  // 根据 buffer_type 创建相应类型的内部进程缓冲区 (Create the corresponding intra-process buffer
  // based on buffer_type)
  switch (buffer_type) {
    case IntraProcessBufferType::SharedPtr: {
      // 使用共享指针作为缓冲区类型 (Use shared pointer as buffer type)
      using BufferT = MessageSharedPtr;

      // 创建环形缓冲区实现 (Create ring buffer implementation)
      auto buffer_implementation =
          std::make_unique<rclcpp::experimental::buffers::RingBufferImplementation<BufferT>>(
              buffer_size);

      // 构造内部进程缓冲区 (Construct the intra-process buffer)
      buffer = std::make_unique<rclcpp::experimental::buffers::TypedIntraProcessBuffer<
          MessageT, Alloc, Deleter, BufferT>>(std::move(buffer_implementation), allocator);

      break;
    }
    case IntraProcessBufferType::UniquePtr: {
      // 使用唯一指针作为缓冲区类型 (Use unique pointer as buffer type)
      using BufferT = MessageUniquePtr;

      // 创建环形缓冲区实现 (Create ring buffer implementation)
      auto buffer_implementation =
          std::make_unique<rclcpp::experimental::buffers::RingBufferImplementation<BufferT>>(
              buffer_size);

      // 构造内部进程缓冲区 (Construct the intra-process buffer)
      buffer = std::make_unique<rclcpp::experimental::buffers::TypedIntraProcessBuffer<
          MessageT, Alloc, Deleter, BufferT>>(std::move(buffer_implementation), allocator);

      break;
    }
    default: {
      // 抛出异常，未识别的内部进程缓冲区类型 (Throw exception, unrecognized intra-process buffer
      // type)
      throw std::runtime_error("Unrecognized IntraProcessBufferType value");
      break;
    }
  }

  // 返回创建的内部进程缓冲区指针 (Return the created intra-process buffer pointer)
  return buffer;
}

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__CREATE_INTRA_PROCESS_BUFFER_HPP_

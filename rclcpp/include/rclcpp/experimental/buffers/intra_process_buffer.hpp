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

#ifndef RCLCPP__EXPERIMENTAL__BUFFERS__INTRA_PROCESS_BUFFER_HPP_
#define RCLCPP__EXPERIMENTAL__BUFFERS__INTRA_PROCESS_BUFFER_HPP_

#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/experimental/buffers/buffer_implementation_base.hpp"
#include "rclcpp/macros.hpp"

namespace rclcpp {
namespace experimental {
namespace buffers {

/**
 * @class IntraProcessBufferBase
 * @brief 基础类，定义了一些纯虚函数。 (Base class, defining some pure virtual functions.)
 */
class IntraProcessBufferBase {
public:
  // 定义智能指针别名，方便使用。 (Define smart pointer aliases for easy use.)
  RCLCPP_SMART_PTR_ALIASES_ONLY(IntraProcessBufferBase)

  // 虚析构函数。 (Virtual destructor.)
  virtual ~IntraProcessBufferBase() {}

  // 清除缓冲区中的数据。 (Clear the data in the buffer.)
  virtual void clear() = 0;

  // 检查缓冲区是否有数据。 (Check if the buffer has data.)
  virtual bool has_data() const = 0;

  // 返回是否使用 take_shared 方法。 (Return whether to use the take_shared method.)
  virtual bool use_take_shared_method() const = 0;
};

/**
 * @class IntraProcessBuffer
 * @tparam MessageT 消息类型。 (Message type.)
 * @tparam Alloc 分配器类型。 (Allocator type.)
 * @tparam MessageDeleter 消息删除器。 (Message deleter.)
 * @brief 继承自 IntraProcessBufferBase 的模板类。 (Template class inheriting from
 * IntraProcessBufferBase.)
 */
template <
    typename MessageT,
    typename Alloc = std::allocator<void>,
    typename MessageDeleter = std::default_delete<MessageT>>
class IntraProcessBuffer : public IntraProcessBufferBase {
public:
  // 定义智能指针别名，方便使用。 (Define smart pointer aliases for easy use.)
  RCLCPP_SMART_PTR_ALIASES_ONLY(IntraProcessBuffer)

  // 虚析构函数。 (Virtual destructor.)
  virtual ~IntraProcessBuffer() {}

  // 定义消息的智能指针类型。 (Define smart pointer types for messages.)
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;

  // 添加共享指针类型的消息。 (Add a message of shared pointer type.)
  virtual void add_shared(MessageSharedPtr msg) = 0;

  // 添加唯一指针类型的消息。 (Add a message of unique pointer type.)
  virtual void add_unique(MessageUniquePtr msg) = 0;

  // 消费共享指针类型的消息。 (Consume a message of shared pointer type.)
  virtual MessageSharedPtr consume_shared() = 0;

  // 消费唯一指针类型的消息。 (Consume a message of unique pointer type.)
  virtual MessageUniquePtr consume_unique() = 0;
};

/**
 * @class TypedIntraProcessBuffer
 * @tparam MessageT 消息类型。 (Message type.)
 * @tparam Alloc 分配器类型。 (Allocator type.)
 * @tparam MessageDeleter 消息删除器。 (Message deleter.)
 * @tparam BufferT 缓冲区类型。 (Buffer type.)
 * @brief 继承自 IntraProcessBuffer 的模板类。 (Template class inheriting from IntraProcessBuffer.)
 */
template <
    typename MessageT,
    typename Alloc = std::allocator<void>,
    typename MessageDeleter = std::default_delete<MessageT>,
    typename BufferT = std::unique_ptr<MessageT>>
class TypedIntraProcessBuffer : public IntraProcessBuffer<MessageT, Alloc, MessageDeleter> {
public:
  // 定义智能指针。 (Define smart pointers.)
  RCLCPP_SMART_PTR_DEFINITIONS(TypedIntraProcessBuffer)

  // 定义消息分配器相关类型。 (Define message allocator related types.)
  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;

  /**
   * @brief 构造函数。 (Constructor.)
   * @param buffer_impl 缓冲区实现。 (Buffer implementation.)
   * @param allocator 分配器。 (Allocator.)
   */
  explicit TypedIntraProcessBuffer(
      std::unique_ptr<BufferImplementationBase<BufferT>> buffer_impl,
      std::shared_ptr<Alloc> allocator = nullptr) {
    // 检查 BufferT 是否有效。 (Check if BufferT is valid.)
    bool valid_type =
        (std::is_same<BufferT, MessageSharedPtr>::value ||
         std::is_same<BufferT, MessageUniquePtr>::value);
    if (!valid_type) {
      throw std::runtime_error("Creating TypedIntraProcessBuffer with not valid BufferT");
    }

    // 移动缓冲区实现。 (Move the buffer implementation.)
    buffer_ = std::move(buffer_impl);

    // 初始化消息分配器。 (Initialize the message allocator.)
    if (!allocator) {
      message_allocator_ = std::make_shared<MessageAlloc>();
    } else {
      message_allocator_ = std::make_shared<MessageAlloc>(*allocator.get());
    }
  }

  // 虚析构函数。 (Virtual destructor.)
  virtual ~TypedIntraProcessBuffer() {}

  // 实现 add_shared 方法。 (Implement the add_shared method.)
  void add_shared(MessageSharedPtr msg) override { add_shared_impl<BufferT>(std::move(msg)); }

  // 实现 add_unique 方法。 (Implement the add_unique method.)
  void add_unique(MessageUniquePtr msg) override { buffer_->enqueue(std::move(msg)); }

  // 实现 consume_shared 方法。 (Implement the consume_shared method.)
  MessageSharedPtr consume_shared() override { return consume_shared_impl<BufferT>(); }

  // 实现 consume_unique 方法。 (Implement the consume_unique method.)
  MessageUniquePtr consume_unique() override { return consume_unique_impl<BufferT>(); }

  // 实现 has_data 方法。 (Implement the has_data method.)
  bool has_data() const override { return buffer_->has_data(); }

  // 实现 clear 方法。 (Implement the clear method.)
  void clear() override { buffer_->clear(); }

  // 实现 use_take_shared_method 方法。 (Implement the use_take_shared_method method.)
  bool use_take_shared_method() const override {
    return std::is_same<BufferT, MessageSharedPtr>::value;
  }

private:
  // 缓冲区实现。 (Buffer implementation.)
  std::unique_ptr<BufferImplementationBase<BufferT>> buffer_;

  // 消息分配器。 (Message allocator.)
  std::shared_ptr<MessageAlloc> message_allocator_;

  // 添加共享消息的实现。 (Implementation of adding shared messages.)
  template <typename DestinationT>
  typename std::enable_if<std::is_same<DestinationT, MessageSharedPtr>::value>::type
  add_shared_impl(MessageSharedPtr shared_msg) {
    buffer_->enqueue(std::move(shared_msg));
  }

  /**
   * @brief 添加共享消息的实现。(Implementation of adding shared messages.)
   *
   * @tparam DestinationT 目标类型 (Destination type)
   * @param[in] shared_msg 共享消息指针 (Shared message pointer)
   */
  template <typename DestinationT>
  typename std::enable_if<std::is_same<DestinationT, MessageUniquePtr>::value>::type
  add_shared_impl(MessageSharedPtr shared_msg) {
    // 这里会无条件地进行一次复制，而实际上 intra-process manager
    // 可以根据缓冲区的数量和类型来决定是否需要复制。 (Here a copy is unconditionally made, while
    // the intra-process manager can decide whether a copy is needed depending on the number and
    // type of buffers.)
    MessageUniquePtr unique_msg;

    // 获取共享消息的删除器。 (Get the deleter of the shared message.)
    MessageDeleter* deleter = std::get_deleter<MessageDeleter, const MessageT>(shared_msg);

    // 使用消息分配器为消息分配内存。 (Allocate memory for the message using the message allocator.)
    auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);

    // 使用消息分配器构造消息对象。 (Construct the message object using the message allocator.)
    MessageAllocTraits::construct(*message_allocator_.get(), ptr, *shared_msg);

    // 如果有删除器，则使用删除器创建唯一消息指针。否则直接创建唯一消息指针。
    // (If there is a deleter, create a unique message pointer with the deleter. Otherwise, create a
    // unique message pointer directly.)
    if (deleter) {
      unique_msg = MessageUniquePtr(ptr, *deleter);
    } else {
      unique_msg = MessageUniquePtr(ptr);
    }

    // 将唯一消息指针加入缓冲区。 (Enqueue the unique message pointer into the buffer.)
    buffer_->enqueue(std::move(unique_msg));
  }

  // 消费共享消息的实现。 (Implementation of consuming shared messages.)
  template <typename OriginT>
  typename std::enable_if<std::is_same<OriginT, MessageSharedPtr>::value, MessageSharedPtr>::type
  consume_shared_impl() {
    return buffer_->dequeue();
  }

  // 消费共享消息的实现。 (Implementation of consuming shared messages.)
  template <typename OriginT>
  typename std::enable_if<(std::is_same<OriginT, MessageUniquePtr>::value), MessageSharedPtr>::type
  consume_shared_impl() {
    // 自动从唯一指针转换为共享指针。 (Automatic cast from unique pointer to shared pointer.)
    return buffer_->dequeue();
  }

  /**
   * @brief 消费唯一消息的实现 (Implementation of consuming unique messages.)
   *
   * @tparam OriginT 模板参数，用于判断原始消息类型 (Template parameter for determining the original
   * message type)
   * @return std::enable_if<(std::is_same<OriginT, MessageSharedPtr>::value),
   * MessageUniquePtr>::type 返回一个MessageUniquePtr类型的智能指针 (Returns a smart pointer of type
   * MessageUniquePtr)
   */
  template <typename OriginT>
  typename std::enable_if<(std::is_same<OriginT, MessageSharedPtr>::value), MessageUniquePtr>::type
  consume_unique_impl() {
    // 从缓冲区中取出一个共享指针类型的消息 (Dequeues a shared pointer type message from the buffer)
    MessageSharedPtr buffer_msg = buffer_->dequeue();

    // 定义一个唯一指针类型的消息变量 (Defines a unique pointer type message variable)
    MessageUniquePtr unique_msg;

    // 获取共享指针中的删除器 (Gets the deleter from the shared pointer)
    MessageDeleter* deleter = std::get_deleter<MessageDeleter, const MessageT>(buffer_msg);

    // 使用分配器为唯一指针分配内存 (Allocates memory for the unique pointer using the allocator)
    auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);

    // 使用分配器构造唯一指针的对象 (Constructs the unique pointer object using the allocator)
    MessageAllocTraits::construct(*message_allocator_.get(), ptr, *buffer_msg);

    // 如果有删除器，则使用删除器创建唯一指针 (If there is a deleter, create the unique pointer
    // using the deleter)
    if (deleter) {
      unique_msg = MessageUniquePtr(ptr, *deleter);
    } else {
      // 否则，使用默认删除器创建唯一指针 (Otherwise, create the unique pointer using the default
      // deleter)
      unique_msg = MessageUniquePtr(ptr);
    }

    // 返回唯一指针类型的消息 (Returns the unique pointer type message)
    return unique_msg;
  }

  /**
   * @brief 消费唯一消息的实现 (Implementation of consuming unique messages.)
   *
   * @tparam OriginT 模板参数，用于判断原始消息类型 (Template parameter for determining the original
   * message type)
   * @return std::enable_if<(std::is_same<OriginT, MessageUniquePtr>::value),
   * MessageUniquePtr>::type 返回一个MessageUniquePtr类型的智能指针 (Returns a smart pointer of type
   * MessageUniquePtr)
   */
  template <typename OriginT>
  typename std::enable_if<(std::is_same<OriginT, MessageUniquePtr>::value), MessageUniquePtr>::type
  consume_unique_impl() {
    // 直接从缓冲区中取出一个唯一指针类型的消息并返回 (Dequeues and returns a unique pointer type
    // message from the buffer directly)
    return buffer_->dequeue();
  }
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__INTRA_PROCESS_BUFFER_HPP_

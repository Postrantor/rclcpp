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

#ifndef RCLCPP__LOANED_MESSAGE_HPP_
#define RCLCPP__LOANED_MESSAGE_HPP_

#include <memory>
#include <utility>

#include "rcl/allocator.h"
#include "rcl/publisher.h"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher_base.hpp"

namespace rclcpp {

/**
 * @brief LoanedMessage 类模板，用于在 ROS2 的 rclcpp 中管理消息的内存分配。
 *
 * 该类支持自定义消息类型 MessageT 和分配器 AllocatorT（默认为 std::allocator<void>）。
 *
 * @tparam MessageT 自定义消息类型
 * @tparam AllocatorT 分配器类型，默认为 std::allocator<void>
 */
template <typename MessageT, typename AllocatorT = std::allocator<void>>
class LoanedMessage {
  using MessageAllocatorTraits = rclcpp::allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAllocator = typename MessageAllocatorTraits::allocator_type;

public:
  /**
   * @brief LoanedMessage 类构造函数。
   *
   * 此构造函数为给定的消息类型分配内存，并将其与给定的发布者关联。
   *
   * 根据发布者实例，执行一个区分案例，以决定底层中间件是否能够为此消息类型分配适当的内存。
   * 如果中间件无法借用消息，则使用传入的分配器实例在此类的范围内分配消息。
   * 否则，忽略分配器，并仅在底层中间件中使用其适当的分配策略执行分配。
   * 这样做的必要性在于用户代码可以明确针对能够借用消息的中间件编写。
   * 然而，即使在动态链接到不支持消息借用的中间件时，这些用户代码也应该是可用的，在这种情况下将使用分配器。
   *
   * @param[in] pub 与内存相关联的 rclcpp::Publisher 实例
   * @param[in] allocator 如果中间件无法分配消息，则使用分配器实例
   * @throws 可能抛出 rclcpp::exceptions::throw_from_rcl_error 的异常。
   */
  LoanedMessage(const rclcpp::PublisherBase &pub, std::allocator<MessageT> allocator)
      : pub_(pub), message_(nullptr), message_allocator_(std::move(allocator)) {
    // 判断发布者是否可以借用消息
    if (pub_.can_loan_messages()) {
      void *message_ptr = nullptr;
      auto ret = rcl_borrow_loaned_message(
          pub_.get_publisher_handle().get(),
          rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(), &message_ptr);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
      message_ = static_cast<MessageT *>(message_ptr);
    } else {
      RCLCPP_INFO_ONCE(
          rclcpp::get_logger("rclcpp"),
          "Currently used middleware can't loan messages. Local allocator will be used.");
      message_ = message_allocator_.allocate(1);
      new (message_) MessageT();
    }
  }

  /**
   * @brief LoanedMessage 类构造函数（已弃用）。
   *
   * 此构造函数为给定的消息类型分配内存，并将其与给定的发布者关联。
   *
   * 根据发布者实例，执行一个区分案例，以决定底层中间件是否能够为此消息类型分配适当的内存。
   * 如果中间件无法借用消息，则使用传入的分配器实例在此类的范围内分配消息。
   * 否则，忽略分配器，并仅在底层中间件中使用其适当的分配策略执行分配。
   * 这样做的必要性在于用户代码可以明确针对能够借用消息的中间件编写。
   * 然而，即使在动态链接到不支持消息借用的中间件时，这些用户代码也应该是可用的，在这种情况下将使用分配器。
   *
   * @param[in] pub 与内存相关联的 rclcpp::Publisher 实例
   * @param[in] allocator 如果中间件无法分配消息，则使用分配器实例
   * @throws 可能抛出 rclcpp::exceptions::throw_from_rcl_error 的异常。
   */
  [[deprecated(
      "used the LoanedMessage constructor that does not use a shared_ptr to the "
      "allocator")]] LoanedMessage(const rclcpp::PublisherBase *pub, std::shared_ptr<std::allocator<MessageT>> allocator)
      : LoanedMessage(*pub, *allocator) {}

  /**
   * @brief 移动构造函数，用于返回值优化 (RVO)。
   */
  LoanedMessage(LoanedMessage<MessageT> &&other)
      : pub_(std::move(other.pub_)),
        message_(std::move(other.message_)),
        message_allocator_(std::move(other.message_allocator_)) {
    other.message_ = nullptr;
  }

  /**
   * @brief LoanedMessage 类析构函数。
   *
   * 析构函数的明确任务是为其消息实例返回分配的内存。
   * 如果之前通过中间件分配了消息，则将消息返回给中间件以清除分配。
   * 如果使用了本地分配器实例，那么同一个实例将被用于销毁分配的内存。
   *
   * 这里的约定是，只要这个 LoanedMessage 实例还活着，这个消息的内存就有效。
   */
  virtual ~LoanedMessage() {
    auto error_logger = rclcpp::get_logger("LoanedMessage");

    if (message_ == nullptr) {
      return;
    }

    if (pub_.can_loan_messages()) {
      // 将分配的内存返回给中间件
      auto ret =
          rcl_return_loaned_message_from_publisher(pub_.get_publisher_handle().get(), message_);
      if (ret != RCL_RET_OK) {
        RCLCPP_ERROR(
            error_logger, "rcl_deallocate_loaned_message failed: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
    } else {
      // 在释放内存之前调用析构函数
      message_->~MessageT();
      message_allocator_.deallocate(message_, 1);
    }
    message_ = nullptr;
  }

  /**
   * @brief 验证消息是否正确分配。
   *
   * 分配的内存可能并不总是一致和有效的。
   * 导致这种情况的原因可能是分配步骤失败，例如 malloc
   * 可能会失败，或者先前分配的消息数量超过最大值，
   * 在这种情况下，借用的消息必须在能够分配新消息之前返回给中间件。
   */
  bool is_valid() const { return message_ != nullptr; }

  /**
   * @brief 访问 ROS 消息实例。
   *
   * 调用 `get()` 将返回对底层 ROS 消息实例的可变引用。
   * 这允许用户在发布消息之前修改消息内容。
   *
   * 如果复制了此引用，则该副本的内存不再由 LoanedMessage 实例管理，并且需要单独清理。
   */
  MessageT &get() const { return *message_; }

  /**
   * @brief 释放 ROS 消息实例的所有权。
   *
   * 调用 `release()` 将取消管理 ROS 消息的内存。
   * 这意味着，当此类的作用域结束时，析构函数将不会释放内存。
   * 如果消息是从中间件借用的，但没有发布，则用户需要手动调用
   * `rcl_return_loaned_message_from_publisher`。 如果内存来自本地分配器，那么当 unique_ptr
   * 失效时，内存将被释放。
   *
   * @return 指向消息实例的 std::unique_ptr。
   */
  std::unique_ptr<MessageT, std::function<void(MessageT *)>> release() {
    auto msg = message_;
    message_ = nullptr;

    if (pub_.can_loan_messages()) {
      return std::unique_ptr<MessageT, std::function<void(MessageT *)>>(msg, [](MessageT *) {});
    }

    return std::unique_ptr<MessageT, std::function<void(MessageT *)>>(
        msg, [allocator = message_allocator_](MessageT *msg_ptr) mutable {
          // 在释放内存之前调用析构函数
          msg_ptr->~MessageT();
          allocator.deallocate(msg_ptr, 1);
        });
  }

protected:
  const rclcpp::PublisherBase &pub_;

  MessageT *message_;

  MessageAllocator message_allocator_;

  /// 已删除的复制构造函数，以保持内存完整性。
  LoanedMessage(const LoanedMessage<MessageT> &other) = delete;
};

}  // namespace rclcpp

#endif  // RCLCPP__LOANED_MESSAGE_HPP_

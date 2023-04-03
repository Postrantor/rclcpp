// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__PUBLISHER_HPP_
#define RCLCPP__PUBLISHER_HPP_

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/publisher.h"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/detail/resolve_use_intra_process.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/get_message_type_support_handle.hpp"
#include "rclcpp/is_ros_compatible_type.hpp"
#include "rclcpp/loaned_message.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/type_adapter.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rosidl_runtime_cpp/traits.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp {

// 模板类 LoanedMessage，用于支持消息的内存管理。
template <typename MessageT, typename AllocatorT>
class LoanedMessage;

/// 一个发布者（publisher）可以将任何类型的消息发布到一个主题（topic）。
/**
 * MessageT 必须是以下之一：
 *
 * - 具有自己类型支持的 ROS 消息类型（例如：std_msgs::msgs::String），或者
 * - rclcpp::TypeAdapter<CustomType, ROSMessageType>
 *   （例如：rclcpp::TypeAdapter<std::string, std_msgs::msg::String>），或者
 * - 使用 RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(custom_type, ros_message_type)
 *   设置为 ROS 类型隐式类型的自定义类型
 *
 * 如果 MessageT 是 ROS 消息类型（例如：std_msgs::msg::String），
 * 那么 PublishedType 和 ROSMessageType 都将是该类型。
 * 如果 MessageT 是 TypeAdapter<CustomType, ROSMessageType> 类型
 * （例如：TypeAdapter<std::string, std_msgs::msg::String>），则 PublishedType 将是自定义类型，
 * 而 ROSMessageType 将是 ros 消息类型。
 *
 * 这是因为 TypeAdapter 的 "identity specialization"，如果它已经是一个 TypeAdapter，它将返回本身，
 * 默认的 specialization 允许 ROSMessageType 为 void。
 * \sa rclcpp::TypeAdapter for more details.
 */
template <typename MessageT, typename AllocatorT = std::allocator<void>>
class Publisher : public PublisherBase {
public:
  // 确保给定的消息类型与 ROS 兼容，可以在发布者中使用。
  static_assert(
      rclcpp::is_ros_compatible_type<MessageT>::value,
      "given message type is not compatible with ROS and cannot be used with a Publisher");

  using PublishedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;
  using ROSMessageType = typename rclcpp::TypeAdapter<MessageT>::ros_message_type;
  using PublishedTypeAllocatorTraits = allocator::AllocRebind<PublishedType, AllocatorT>;
  using PublishedTypeAllocator = typename PublishedTypeAllocatorTraits::allocator_type;
  using PublishedTypeDeleter = allocator::Deleter<PublishedTypeAllocator, PublishedType>;
  using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<ROSMessageType, AllocatorT>;
  using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
  using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;

  /// \brief 已弃用，改用 PublishedTypeAllocatorTraits。
  using MessageAllocatorTraits [[deprecated("use PublishedTypeAllocatorTraits")]] =
      PublishedTypeAllocatorTraits;
  /// \brief 已弃用，改用 PublishedTypeAllocator。
  using MessageAllocator [[deprecated("use PublishedTypeAllocator")]] = PublishedTypeAllocator;
  /// \brief 已弃用，改用 PublishedTypeDeleter。
  using MessageDeleter [[deprecated("use PublishedTypeDeleter")]] = PublishedTypeDeleter;
  /// \brief 已弃用，改用 std::unique_ptr<PublishedType, PublishedTypeDeleter>。
  using MessageUniquePtr
      [[deprecated("use std::unique_ptr<PublishedType, PublishedTypeDeleter>")]] =
          std::unique_ptr<PublishedType, PublishedTypeDeleter>;
  /// \brief 已弃用，改用 std::shared_ptr<const PublishedType>。
  using MessageSharedPtr [[deprecated("use std::shared_ptr<const PublishedType>")]] =
      std::shared_ptr<const PublishedType>;

  /// \brief 定义 Publisher 类的智能指针类型。
  RCLCPP_SMART_PTR_DEFINITIONS(Publisher<MessageT, AllocatorT>)

  /// 默认构造函数。 (Default constructor.)
  /**
   * 几乎从不直接调用发布者的构造函数。 (The constructor for a Publisher is almost never called
   * directly.) 相反，订阅应通过函数实例化。 (Instead, subscriptions should be instantiated through
   * the function) rclcpp::create_publisher().
   *
   * \param[in] node_base 用于设置过程中部分内容的 NodeBaseInterface 指针。 (NodeBaseInterface
   * pointer that is used in part of the setup.)
   * \param[in] topic 要发布到的主题名称。 (Name of the
   * topic to publish to.)
   * \param[in] qos 订阅的 QoS 配置文件。 (QoS profile for Subscription.)
   * \param[in] options 订阅的选项。 (Options for the subscription.)
   */
  Publisher(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      const std::string &topic,
      const rclcpp::QoS &qos,
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options)
      : PublisherBase(
            node_base,
            topic,
            rclcpp::get_message_type_support_handle<MessageT>(),
            options.template to_rcl_publisher_options<MessageT>(qos),
            // NOTE(methylDragon): Passing these args separately is necessary for event binding
            options.event_callbacks,
            options.use_default_callbacks),
        options_(options),
        published_type_allocator_(*options.get_allocator()),
        ros_message_type_allocator_(*options.get_allocator()) {
    // 设置发布类型和 ROS 消息类型的删除器的分配器。 (Set allocator for deleter of published type
    // and ROS message type.)
    allocator::set_allocator_for_deleter(&published_type_deleter_, &published_type_allocator_);
    allocator::set_allocator_for_deleter(&ros_message_type_deleter_, &ros_message_type_allocator_);
    // 在 post_init_setup() 方法中继续设置。 (Setup continues in the post construction method,
    // post_init_setup().)
  }

  /// 构造后调用，以便在 shared_from_this() 工作后继续构造。 (Called post construction, so that
  /// construction may continue after shared_from_this() works.)
  virtual void post_init_setup(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      const std::string &topic,
      const rclcpp::QoS &qos,
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options) {
    // 主题目前未使用。 (Topic is unused for now.)
    (void)topic;
    (void)options;

    // 如果需要，设置进程内通信。 (If needed, setup intra process communication.)
    if (rclcpp::detail::resolve_use_intra_process(options_, *node_base)) {
      auto context = node_base->get_context();
      // 获取此上下文的进程内管理器实例。 (Get the intra process manager instance for this context.)
      auto ipm = context->get_sub_context<rclcpp::experimental::IntraProcessManager>();
      // 将发布者注册到进程内管理器。 (Register the publisher with the intra process manager.)
      if (qos.history() != rclcpp::HistoryPolicy::KeepLast) {
        throw std::invalid_argument(
            "intraprocess communication allowed only with keep last history qos policy");
      }
      if (qos.depth() == 0) {
        throw std::invalid_argument(
            "intraprocess communication is not allowed with a zero qos history depth value");
      }
      if (qos.durability() != rclcpp::DurabilityPolicy::Volatile) {
        throw std::invalid_argument(
            "intraprocess communication allowed only with volatile durability");
      }
      uint64_t intra_process_publisher_id = ipm->add_publisher(this->shared_from_this());
      this->setup_intra_process(intra_process_publisher_id, ipm);
    }
  }

  virtual ~Publisher() {}

  /// 借用中间件的已分配 ROS 消息。
  /**
   * 如果中间件能够为 ROS 消息实例提供内存分配，则借用的消息将直接在中间件中分配。
   * 如果不能，则使用此 rclcpp::Publisher 实例的消息分配器。
   *
   * 使用 \sa `publish` 调用，LoanedMessage 实例会被返回到中间件或根据分配器进行释放。
   * 如果消息没有被发布而是以其他方式处理，那么这个类的析构函数
   * 将要么将消息返回给中间件，要么通过内部分配器进行释放。
   * 分配器。有关 LoanedMessage 类的详细信息，请参阅 rclcpp::LoanedMessage。
   *
   * \return 包含 ROSMessageType 类型的 ROS 消息内存的 LoanedMessage
   */
  rclcpp::LoanedMessage<ROSMessageType, AllocatorT> borrow_loaned_message() {
    return rclcpp::LoanedMessage<ROSMessageType, AllocatorT>(
        *this, this->get_ros_message_type_allocator());
  }

  /// 在主题上发布消息。
  /**
   * 如果 std::unique_ptr 的 element_type 是 ROS 消息类型，而不是 TypeAdapter 的 custom_type，并且
   * 该类型与创建发布者时给定的类型匹配。
   *
   * 此签名允许用户将消息的所有权交给 rclcpp，从而实现更高效的进程内通信优化。
   *
   * \param[in] msg 要发送的消息的唯一指针。
   */
  template <typename T>
  typename std::enable_if_t<
      rosidl_generator_traits::is_message<T>::value && std::is_same<T, ROSMessageType>::value>
  publish(std::unique_ptr<T, ROSMessageTypeDeleter> msg) {
    if (!intra_process_is_enabled_) {
      this->do_inter_process_publish(*msg);
      return;
    }
    // 如果存在进程间订阅，则将 unique_ptr 提升为 shared_ptr 并发布。
    // 这允许先进行进程内发布，然后进行进程间发布，从而降低发布到订阅的延迟。
    // 使用 unique_ptr 是不可能实现这一点的，
    // 因为 do_intra_process_publish 接管了消息的所有权。
    bool inter_process_publish_needed =
        get_subscription_count() > get_intra_process_subscription_count();

    if (inter_process_publish_needed) {
      auto shared_msg =
          this->do_intra_process_ros_message_publish_and_return_shared(std::move(msg));
      this->do_inter_process_publish(*shared_msg);
    } else {
      this->do_intra_process_ros_message_publish(std::move(msg));
    }
  }

  /// 在主题上发布消息。
  /**
   * 如果发布的对象是 ROS 消息类型，而不是 TypeAdapter 的 custom_type，并且
   * 该类型与创建发布者时给定的类型匹配。
   *
   * 此签名允许用户给出一个消息引用，该引用在没有修改的情况下复制到堆上，
   * rclcpp 并在稍后需要时移动拥有权。
   *
   * \param[in] msg 要发送的消息的常量引用。
   * \param[in] msg A const reference to the message to send.
   */
  template <typename T>
  typename std::enable_if_t<
      rosidl_generator_traits::is_message<T>::value && std::is_same<T, ROSMessageType>::value>
  publish(const T &msg) {
    // 当不使用进程内通信时避免分配。
    if (!intra_process_is_enabled_) {
      // 在这种情况下，我们没有使用进程内通信。
      return this->do_inter_process_publish(msg);
    }
    // 否则，我们必须在 unique_ptr 中分配内存并传递它。
    // 由于消息不是 const 的，所以应该进行复制。
    // 这里也可以构造 shared_ptr<const MessageT>。
    auto unique_msg = this->duplicate_ros_message_as_unique_ptr(msg);
    this->publish(std::move(unique_msg));
  }

  /// 发布一个消息到话题上。
  /**
   * 如果这个类是用 TypeAdapter 创建的，且 std::unique_ptr 的 element_type 与此类使用的
   * TypeAdapter 的 custom_type 相匹配，则启用此签名。
   *
   * 此签名允许用户将消息的所有权交给 rclcpp，从而实现更高效的进程内通信优化。
   *
   * \param[in] msg 要发送的消息的 unique_ptr。
   */
  template <typename T>
  typename std::enable_if_t<
      rclcpp::TypeAdapter<MessageT>::is_specialized::value && std::is_same<T, PublishedType>::value>
  publish(std::unique_ptr<T, PublishedTypeDeleter> msg) {
    // 当不使用进程内通信时避免分配。
    if (!intra_process_is_enabled_) {
      // 在这种情况下，我们不使用进程内通信。
      ROSMessageType ros_msg;
      rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(*msg, ros_msg);
      return this->do_inter_process_publish(ros_msg);
    }

    bool inter_process_publish_needed =
        get_subscription_count() > get_intra_process_subscription_count();

    if (inter_process_publish_needed) {
      ROSMessageType ros_msg;
      // TODO(clalancette): This is unnecessarily doing an additional conversion
      // that may have already been done in do_intra_process_publish_and_return_shared().
      // We should just reuse that effort.
      rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(*msg, ros_msg);
      this->do_intra_process_publish(std::move(msg));
      this->do_inter_process_publish(ros_msg);
    } else {
      this->do_intra_process_publish(std::move(msg));
    }
  }

  /// 发布一个消息到话题上。
  /**
   * 如果这个类是用 TypeAdapter 创建的，且给定类型与 TypeAdapter 的 custom_type
   * 相匹配，则启用此签名。
   *
   * 此签名允许用户提供一个消息引用，该引用在不修改的情况下复制到堆上，
   * 这样 rclcpp 可以拥有副本的所有权，并在需要时移动副本的所有权。
   *
   * \param[in] msg 要发送的消息的 const 引用。
   */
  template <typename T>
  typename std::enable_if_t<
      rclcpp::TypeAdapter<MessageT>::is_specialized::value && std::is_same<T, PublishedType>::value>
  publish(const T &msg) {
    // 当不使用进程内通信时避免双重分配。
    if (!intra_process_is_enabled_) {
      // 转换为 ROS 消息等效物并发布它。
      ROSMessageType ros_msg;
      rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(msg, ros_msg);
      // 在这种情况下，我们不使用进程内通信。
      return this->do_inter_process_publish(ros_msg);
    }

    // 否则，我们必须在 unique_ptr 中分配内存并传递它。
    // 由于消息不是 const 的，应该进行复制。
    // 这里也可以构造 shared_ptr<const MessageT>。
    auto unique_msg = this->duplicate_type_adapt_message_as_unique_ptr(msg);
    this->publish(std::move(unique_msg));
  }

  void publish(const rcl_serialized_message_t &serialized_msg) {
    return this->do_serialized_publish(&serialized_msg);
  }

  void publish(const SerializedMessage &serialized_msg) {
    return this->do_serialized_publish(&serialized_msg.get_rcl_serialized_message());
  }

  /* > [!NOTE]: LoanedMessage */

  /// 发布一个 LoanedMessage 实例。
  /**
   * 当发布一个借用的消息时，这个 ROS 消息的内存将在发布后被释放。
   * 此调用之后，借用的消息实例不再有效。
   *
   * \param loaned_msg 要发布的 LoanedMessage 实例。
   */
  void publish(rclcpp::LoanedMessage<ROSMessageType, AllocatorT> &&loaned_msg) {
    // 检查借用的消息是否有效
    if (!loaned_msg.is_valid()) {
      throw std::runtime_error("loaned message is not valid");
    }
    // 检查内部进程通信是否启用
    if (intra_process_is_enabled_) {
      // TODO(Karsten1987): 支持通过内部进程传递的借用消息
      throw std::runtime_error("storing loaned messages in intra process is not supported yet");
    }

    // 验证发布者是否支持借用消息
    // TODO(Karsten1987): 这种情况的分离必须在 rclcpp 中完成
    // 否则我们必须确保每个中间件都实现了
    // `rmw_publish_loaned_message`显式地与`rmw_publish`相同的方式
    // 通过获取 ROS 消息的副本。
    if (this->can_loan_messages()) {
      // 我们从 rclpp::LoanedMessage 实例中释放所有权
      // 让中间件清理内存。
      this->do_loaned_message_publish(std::move(loaned_msg.release()));
    } else {
      // 我们不释放所有权，让中间件复制 ROS 消息
      // 因此 rclcpp::LoanedMessage 的析构函数会清理内存。
      this->do_inter_process_publish(loaned_msg.get());
    }
  }

  [[deprecated(
      "use get_published_type_allocator() or get_ros_message_type_allocator() instead")]] std::
      shared_ptr<PublishedTypeAllocator>
      get_allocator() const {
    return std::make_shared<PublishedTypeAllocator>(published_type_allocator_);
  }

  // 获取已发布类型分配器
  PublishedTypeAllocator get_published_type_allocator() const { return published_type_allocator_; }

  // 获取 ROS 消息类型分配器
  ROSMessageTypeAllocator get_ros_message_type_allocator() const {
    return ros_message_type_allocator_;
  }

protected:
  /**
   * @brief 发布进程间消息（Publish an inter-process message）
   *
   * @param msg 要发布的消息（The message to be published）
   */
  void do_inter_process_publish(const ROSMessageType &msg) {
    // 追踪发布事件（Trace the publish event）
    TRACEPOINT(rclcpp_publish, nullptr, static_cast<const void *>(&msg));
    // 执行发布操作，将消息发布到ROS主题
    auto status = rcl_publish(publisher_handle_.get(), &msg, nullptr);

    // 检查发布者是否有效。如果无效，则尝试重置错误并检查上下文
    if (RCL_RET_PUBLISHER_INVALID == status) {
      rcl_reset_error();  // 下一次调用将在非上下文情况下重置错误消息
      if (rcl_publisher_is_valid_except_context(publisher_handle_.get())) {
        rcl_context_t *context = rcl_publisher_get_context(publisher_handle_.get());
        if (nullptr != context && !rcl_context_is_valid(context)) {
          // 发布者无效，因为上下文已关闭
          return;
        }
      }
    }
    // 如果发布操作失败，抛出异常（If the publish operation fails, throw an exception）
    if (RCL_RET_OK != status) {
      rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish message");
    }
  }

  /**
   * @brief 发布序列化消息（Publish a serialized message）
   *
   * @param serialized_msg 要发布的序列化消息（The serialized message to be published）
   */
  void do_serialized_publish(const rcl_serialized_message_t *serialized_msg) {
    // 如果启用了进程内通信，抛出异常（If intra-process communication is enabled, throw an
    // exception）
    if (intra_process_is_enabled_) {
      // TODO(Karsten1987): 支持通过进程内部传递序列化消息（support serialized message passed by
      // intraprocess）
      throw std::runtime_error("storing serialized messages in intra process is not supported yet");
    }
    // 执行发布序列化消息操作（Perform the publish serialized message operation）
    auto status = rcl_publish_serialized_message(publisher_handle_.get(), serialized_msg, nullptr);
    // 如果发布操作失败，抛出异常（If the publish operation fails, throw an exception）
    if (RCL_RET_OK != status) {
      rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish serialized message");
    }
  }

  /**
   * @brief 发布借用消息（Publish a loaned message）
   *
   * @param msg 要发布的借用消息（The loaned message to be published）
   */
  void do_loaned_message_publish(
      std::unique_ptr<ROSMessageType, std::function<void(ROSMessageType *)>> msg) {
    // 执行发布借用消息操作（Perform the publish loaned message operation）
    auto status = rcl_publish_loaned_message(publisher_handle_.get(), msg.get(), nullptr);

    // 检查发布者是否有效。如果无效，则尝试重置错误并检查上下文（Check if the publisher is valid. If
    // not, try to reset the error and check the context）
    if (RCL_RET_PUBLISHER_INVALID == status) {
      rcl_reset_error();  // 下一次调用将在非上下文情况下重置错误消息（next call will reset error
                          // message if not context）
      if (rcl_publisher_is_valid_except_context(publisher_handle_.get())) {
        rcl_context_t *context = rcl_publisher_get_context(publisher_handle_.get());
        if (nullptr != context && !rcl_context_is_valid(context)) {
          // 发布者无效，因为上下文已关闭（publisher is invalid due to context being shutdown）
          return;
        }
      }
    }
    // 如果发布操作失败，抛出异常（If the publish operation fails, throw an exception）
    if (RCL_RET_OK != status) {
      rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish message");
    }
  }

  /**
   * @brief 发布进程内消息（Publish an intra-process message）
   *
   * @param msg 要发布的消息（The message to be published）
   */
  void do_intra_process_publish(std::unique_ptr<PublishedType, PublishedTypeDeleter> msg) {
    // 获取对进程内管理器的弱引用（Get a weak reference to the intra-process manager）
    auto ipm = weak_ipm_.lock();
    // 如果进程内管理器不存在，则抛出异常（If the intra-process manager does not exist, throw an
    // exception）
    if (!ipm) {
      throw std::runtime_error(
          "intra process publish called after destruction of intra process manager");
    }
    // 如果消息为空指针，则抛出异常（If the message is a null pointer, throw an exception）
    if (!msg) {
      throw std::runtime_error("cannot publish msg which is a null pointer");
    }

    // 执行进程内发布操作（Perform the intra-process publish operation）
    ipm->template do_intra_process_publish<PublishedType, ROSMessageType, AllocatorT>(
        intra_process_publisher_id_, std::move(msg), published_type_allocator_);
  }

  /**
   * @brief 发布进程内ROS消息（Publish an intra-process ROS message）
   *
   * @param msg 要发布的消息（The message to be published）
   */
  void do_intra_process_ros_message_publish(
      std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> msg) {
    // 获取对进程内管理器的弱引用（Get a weak reference to the intra-process manager）
    auto ipm = weak_ipm_.lock();
    // 如果进程内管理器不存在，则抛出异常（If the intra-process manager does not exist, throw an
    // exception）
    if (!ipm) {
      throw std::runtime_error(
          "intra process publish called after destruction of intra process manager");
    }
    // 如果消息为空指针，则抛出异常（If the message is a null pointer, throw an exception）
    if (!msg) {
      throw std::runtime_error("cannot publish msg which is a null pointer");
    }

    // 执行进程内发布操作（Perform the intra-process publish operation）
    ipm->template do_intra_process_publish<ROSMessageType, ROSMessageType, AllocatorT>(
        intra_process_publisher_id_, std::move(msg), ros_message_type_allocator_);
  }

  /**
   * @brief 发布内部进程 ROS 消息并返回共享指针
   * @param[in] msg 唯一指针类型的 ROSMessageType 对象
   * @return std::shared_ptr<const ROSMessageType> 返回常量共享指针类型的 ROSMessageType
   * 对象
   *
   * 此函数用于发布内部进程 ROS 消息，并返回一个常量共享指针类型的 ROSMessageType 对象。
   */
  std::shared_ptr<const ROSMessageType> do_intra_process_ros_message_publish_and_return_shared(
      std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> msg) {
    // 获取弱引用的内部进程管理器（Get the weak reference of the intra-process manager）
    auto ipm = weak_ipm_.lock();

    // 如果内部进程管理器不存在，抛出异常（If the intra-process manager does not exist, throw an
    // exception）
    if (!ipm) {
      throw std::runtime_error(
          "intra process publish called after destruction of intra process manager");
    }

    // 如果消息为空指针，抛出异常（If the message is a null pointer, throw an exception）
    if (!msg) {
      throw std::runtime_error("cannot publish msg which is a null pointer");
    }

    // 调用内部进程管理器的 do_intra_process_publish_and_return_shared
    // 方法进行发布，并返回共享指针（Call the do_intra_process_publish_and_return_shared method of
    // the intra-process manager to publish and return the shared pointer）
    return ipm->template do_intra_process_publish_and_return_shared<
        ROSMessageType, ROSMessageType, AllocatorT>(
        intra_process_publisher_id_, std::move(msg), ros_message_type_allocator_);
  }

  /**
   * @brief 创建一个新的 ROSMessageType 类型的唯一指针
   * @return std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> 返回 ROSMessageType
   * 类型的唯一指针
   */
  std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> create_ros_message_unique_ptr() {
    // 使用 ROSMessageTypeAllocatorTraits 分配器分配内存空间
    auto ptr = ROSMessageTypeAllocatorTraits::allocate(ros_message_type_allocator_, 1);

    // 使用 ROSMessageTypeAllocatorTraits 分配器构造对象
    ROSMessageTypeAllocatorTraits::construct(ros_message_type_allocator_, ptr);

    // 返回 ROSMessageType 类型的唯一指针
    return std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>(ptr, ros_message_type_deleter_);
  }

  /**
   * @brief 复制给定的 ROS 消息为唯一指针
   * @param[in] msg 输入的 ROSMessageType 对象（Input ROSMessageType object）
   * @return std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> 返回复制后的 ROSMessageType
   * 类型的唯一指针（Return the duplicated unique_ptr of type ROSMessageType）
   */
  std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> duplicate_ros_message_as_unique_ptr(
      const ROSMessageType &msg) {
    // 使用 ROSMessageTypeAllocatorTraits 分配器分配内存空间（Allocate memory space using
    // ROSMessageTypeAllocatorTraits allocator）
    auto ptr = ROSMessageTypeAllocatorTraits::allocate(ros_message_type_allocator_, 1);

    // 使用 ROSMessageTypeAllocatorTraits 分配器构造对象并复制消息（Construct object and copy
    // message using ROSMessageTypeAllocatorTraits allocator）
    ROSMessageTypeAllocatorTraits::construct(ros_message_type_allocator_, ptr, msg);

    // 返回复制后的 ROSMessageType 类型的唯一指针（Return the duplicated unique_ptr of type
    // ROSMessageType）
    return std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>(ptr, ros_message_type_deleter_);
  }

  /**
   * @brief 复制给定的类型适配消息为唯一指针
   * @param[in] msg 输入的 PublishedType 对象（Input PublishedType object）
   * @return std::unique_ptr<PublishedType, PublishedTypeDeleter> 返回复制后的 PublishedType
   * 类型的唯一指针（Return the duplicated unique_ptr of type PublishedType）
   */
  std::unique_ptr<PublishedType, PublishedTypeDeleter> duplicate_type_adapt_message_as_unique_ptr(
      const PublishedType &msg) {
    // 使用 PublishedTypeAllocatorTraits 分配器分配内存空间（Allocate memory space using
    // PublishedTypeAllocatorTraits allocator）
    auto ptr = PublishedTypeAllocatorTraits::allocate(published_type_allocator_, 1);

    // 使用 PublishedTypeAllocatorTraits 分配器构造对象并复制消息（Construct object and copy message
    // using PublishedTypeAllocatorTraits allocator）
    PublishedTypeAllocatorTraits::construct(published_type_allocator_, ptr, msg);

    // 返回复制后的 PublishedType 类型的唯一指针（Return the duplicated unique_ptr of type
    // PublishedType）
    return std::unique_ptr<PublishedType, PublishedTypeDeleter>(ptr, published_type_deleter_);
  }

  /// 原始选项的副本，在构造过程中传递。
  /**
   * 保存这个副本非常重要，以便在发布者的整个生命周期内保持 rmw 载荷的活跃。
   */
  const rclcpp::PublisherOptionsWithAllocator<AllocatorT> options_;
  // 已发布类型分配器（用于分配已发布类型所需的内存）
  PublishedTypeAllocator published_type_allocator_;
  // 已发布类型删除器（用于删除已发布类型实例）
  PublishedTypeDeleter published_type_deleter_;
  // ROS 消息类型分配器（用于分配 ROS 消息类型所需的内存）
  ROSMessageTypeAllocator ros_message_type_allocator_;
  // ROS 消息类型删除器（用于删除 ROS 消息类型实例）
  ROSMessageTypeDeleter ros_message_type_deleter_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_HPP_

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

#ifndef RCLCPP__SUBSCRIPTION_HPP_
#define RCLCPP__SUBSCRIPTION_HPP_

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/subscription.h"
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/detail/resolve_intra_process_buffer_type.hpp"
#include "rclcpp/detail/resolve_use_intra_process.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/experimental/subscription_intra_process.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp/topic_statistics/subscription_topic_statistics.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"
#include "tracetools/tracetools.h"

namespace rclcpp {

namespace node_interfaces {
class NodeTopicsInterface;
}  // namespace node_interfaces

/// 订阅实现，模板化为接收的消息类型。
/// Subscription implementation, templated on the type of message this subscription receives.
template <
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    /// 如果 MessageT 是 TypeAdapter，则为 MessageT::custom_type，否则为 MessageT。
    /// MessageT::custom_type if MessageT is a TypeAdapter, otherwise just MessageT.
    typename SubscribedT = typename rclcpp::TypeAdapter<MessageT>::custom_type,
    /// 如果 MessageT 是 TypeAdapter，则为 MessageT::ros_message_type，否则为 MessageT。
    /// MessageT::ros_message_type if MessageT is a TypeAdapter, otherwise just MessageT.
    typename ROSMessageT = typename rclcpp::TypeAdapter<MessageT>::ros_message_type,
    typename MessageMemoryStrategyT =
        rclcpp::message_memory_strategy::MessageMemoryStrategy<ROSMessageT, AllocatorT>>
class Subscription : public SubscriptionBase {
  // 使 rclcpp::node_interfaces::NodeTopicsInterface 类成为友元类，以便访问其私有成员。
  // Make the rclcpp::node_interfaces::NodeTopicsInterface class a friend to access its private
  // members.
  friend class rclcpp::node_interfaces::NodeTopicsInterface;

public:
  // 在此重新声明这些类型，以便在类外部使用。
  // Redeclare these types here for use outside of the class.
  using SubscribedType = SubscribedT;
  using ROSMessageType = ROSMessageT;
  using MessageMemoryStrategyType = MessageMemoryStrategyT;

  // 定义 SubscribedType 的分配器特性和分配器类型。
  // Define allocator traits and allocator type for SubscribedType.
  using SubscribedTypeAllocatorTraits = allocator::AllocRebind<SubscribedType, AllocatorT>;
  using SubscribedTypeAllocator = typename SubscribedTypeAllocatorTraits::allocator_type;
  using SubscribedTypeDeleter = allocator::Deleter<SubscribedTypeAllocator, SubscribedType>;

  // 定义 ROSMessageType 的分配器特性和分配器类型。
  // Define allocator traits and allocator type for ROSMessageType.
  using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<ROSMessageType, AllocatorT>;
  using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
  using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;

  // 已弃用的别名，建议使用 ROSMessageTypeAllocatorTraits。
  // Deprecated alias, use ROSMessageTypeAllocatorTraits instead.
  using MessageAllocatorTraits [[deprecated("use ROSMessageTypeAllocatorTraits")]] =
      ROSMessageTypeAllocatorTraits;
  // 已弃用的别名，建议使用 ROSMessageTypeAllocator。
  // Deprecated alias, use ROSMessageTypeAllocator instead.
  using MessageAllocator [[deprecated("use ROSMessageTypeAllocator")]] = ROSMessageTypeAllocator;
  // 已弃用的别名，建议使用 ROSMessageTypeDeleter。
  // Deprecated alias, use ROSMessageTypeDeleter instead.
  using MessageDeleter [[deprecated("use ROSMessageTypeDeleter")]] = ROSMessageTypeDeleter;

  // 已弃用的共享指针类型。
  // Deprecated shared pointer type.
  using ConstMessageSharedPtr [[deprecated]] = std::shared_ptr<const ROSMessageType>;
  // 已弃用的唯一指针类型，建议使用 std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> 替代。
  // Deprecated unique pointer type, use std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>
  // instead.
  using MessageUniquePtr
      [[deprecated("use std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> instead")]] =
          std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>;

private:
  using SubscriptionTopicStatisticsSharedPtr =
      std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics<ROSMessageType>>;

public:
  // 定义智能指针类型
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  // 默认构造函数
  // 构造订阅者时几乎不会直接调用此构造函数
  // 而是通过 rclcpp::create_subscription() 函数实例化订阅者
  Subscription(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      const rosidl_message_type_support_t &type_support_handle,
      const std::string &topic_name,
      const rclcpp::QoS &qos,
      AnySubscriptionCallback<MessageT, AllocatorT> callback,
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options,
      typename MessageMemoryStrategyT::SharedPtr message_memory_strategy,
      SubscriptionTopicStatisticsSharedPtr subscription_topic_statistics = nullptr)
      : SubscriptionBase(
            node_base,
            type_support_handle,
            topic_name,
            options.to_rcl_subscription_options(qos),
            // NOTE(methylDragon): Passing these args separately is necessary for event binding
            options.event_callbacks,
            options.use_default_callbacks,
            callback.is_serialized_message_callback()),
        any_callback_(callback),
        options_(options),
        message_memory_strategy_(message_memory_strategy) {
    // 如果需要，设置内部进程发布
    if (rclcpp::detail::resolve_use_intra_process(options_, *node_base)) {
      using rclcpp::detail::resolve_intra_process_buffer_type;

      // 检查 QoS 是否与内部进程兼容
      auto qos_profile = get_actual_qos();
      if (qos_profile.history() != rclcpp::HistoryPolicy::KeepLast) {
        throw std::invalid_argument(
            "intraprocess communication allowed only with keep last history qos policy");
      }
      if (qos_profile.depth() == 0) {
        throw std::invalid_argument(
            "intraprocess communication is not allowed with 0 depth qos policy");
      }
      if (qos_profile.durability() != rclcpp::DurabilityPolicy::Volatile) {
        throw std::invalid_argument(
            "intraprocess communication allowed only with volatile durability");
      }

      // 定义内部进程订阅类型
      using SubscriptionIntraProcessT = rclcpp::experimental::SubscriptionIntraProcess<
          MessageT, SubscribedType, SubscribedTypeAllocator, SubscribedTypeDeleter, ROSMessageT,
          AllocatorT>;

      // 首先创建一个将提供给内部进程管理器的 SubscriptionIntraProcess
      auto context = node_base->get_context();
      subscription_intra_process_ = std::make_shared<SubscriptionIntraProcessT>(
          callback, options_.get_allocator(), context,
          this->get_topic_name(),  // 通过这种方式获取非常重要，因为它具有完全限定的名称
          qos_profile,
          resolve_intra_process_buffer_type(options_.intra_process_buffer_type, callback));
      TRACEPOINT(
          rclcpp_subscription_init, static_cast<const void *>(get_subscription_handle().get()),
          static_cast<const void *>(subscription_intra_process_.get()));

      // 将其添加到内部进程管理器中
      using rclcpp::experimental::IntraProcessManager;
      auto ipm = context->get_sub_context<IntraProcessManager>();
      uint64_t intra_process_subscription_id = ipm->add_subscription(subscription_intra_process_);
      this->setup_intra_process(intra_process_subscription_id, ipm);
    }

    // 如果提供了主题统计订阅，则设置主题统计订阅
    if (subscription_topic_statistics != nullptr) {
      this->subscription_topic_statistics_ = std::move(subscription_topic_statistics);
    }

    // 设置跟踪点
    TRACEPOINT(
        rclcpp_subscription_init, static_cast<const void *>(get_subscription_handle().get()),
        static_cast<const void *>(this));
    TRACEPOINT(
        rclcpp_subscription_callback_added, static_cast<const void *>(this),
        static_cast<const void *>(&any_callback_));
    // 回调对象会被复制，因此如果在这个点之前/过早地完成注册
    // （例如在 `AnySubscriptionCallback::set()` 中），其地址将无法匹配后续跟踪点中使用的任何地址
#ifndef TRACETOOLS_DISABLED
    any_callback_.register_callback_for_tracing();
#endif
  }

  /// \brief 在构造函数之后调用，继续进行需要 shared_from_this() 的设置。
  /// Called after construction to continue setup that requires shared_from_this().
  /// \param[in] node_base NodeBaseInterface 类型的指针，指向节点基类。
  /// \param[in] qos 消息服务质量 (Quality of Service) 参数。
  /// \param[in] options 订阅选项。
  template <typename AllocatorT>
  void post_init_setup(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      const rclcpp::QoS &qos,
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options) {
    (void)node_base;
    (void)qos;
    (void)options;
  }

  /// \brief 从跨进程订阅中获取下一条消息。
  /// Take the next message from the inter-process subscription.
  /// \param[out] message_out 输出消息，take 将数据拷贝到这里。
  /// \param[out] message_info_out 获取到的消息的信息。
  /// \return 如果获取到有效数据则返回 true，否则返回 false。
  /// \throws 来自 rcl_take 的任何 rcl 错误。
  template <typename ROSMessageType>
  bool take(ROSMessageType &message_out, rclcpp::MessageInfo &message_info_out) {
    return this->take_type_erased(static_cast<void *>(&message_out), message_info_out);
  }

  /// \brief 从跨进程订阅中获取下一条消息。
  /// Take the next message from the inter-process subscription.
  /// \tparam TakeT 使用 TypeAdapter 时，与 ROSMessageType 不同的 SubscribedType。
  /// \param[out] message_out 输出消息，take 将数据拷贝到这里。
  /// \param[out] message_info_out 获取到的消息的信息。
  /// \return 如果获取到有效数据则返回 true，否则返回 false。
  template <typename TakeT>
  std::enable_if_t<
      !rosidl_generator_traits::is_message<TakeT>::value && std::is_same_v<TakeT, SubscribedType>,
      bool>
  take(TakeT &message_out, rclcpp::MessageInfo &message_info_out) {
    ROSMessageType local_message;
    bool taken = this->take_type_erased(static_cast<void *>(&local_message), message_info_out);
    if (taken) {
      rclcpp::TypeAdapter<MessageT>::convert_to_custom(local_message, message_out);
    }
    return taken;
  }

  /// \brief 创建消息实例。
  /// Create a message instance.
  /// \return 返回 shared_ptr 类型的消息实例。
  std::shared_ptr<void> create_message() override {
    // 默认的消息内存策略在每次调用 create_message 时提供一个动态分配的消息，
    // 尽管也可以使用其他内存策略（例如重用预先分配的消息）。
    return message_memory_strategy_->borrow_message();
  }

  /// \brief 创建序列化消息实例。
  /// Create a serialized message instance.
  /// \return 返回 shared_ptr 类型的序列化消息实例。
  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override {
    return message_memory_strategy_->borrow_serialized_message();
  }

  /// \brief 处理收到的消息。
  /// Handle the received message.
  /// \param[in] message 收到的消息。
  /// \param[in] message_info 收到的消息的信息。
  void handle_message(
      std::shared_ptr<void> &message, const rclcpp::MessageInfo &message_info) override {
    if (matches_any_intra_process_publishers(&message_info.get_rmw_message_info().publisher_gid)) {
      // 在这种情况下，消息将通过内部进程传递，
      // 而我们应该忽略此副本。
      return;
    }
    auto typed_message = std::static_pointer_cast<ROSMessageType>(message);

    std::chrono::time_point<std::chrono::system_clock> now;
    if (subscription_topic_statistics_) {
      // 在执行回调之前获取当前时间，
      // 以便从主题统计结果中排除回调持续时间。
      now = std::chrono::system_clock::now();
    }

    any_callback_.dispatch(typed_message, message_info);

    if (subscription_topic_statistics_) {
      const auto nanos = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
      const auto time = rclcpp::Time(nanos.time_since_epoch().count());
      subscription_topic_statistics_->handle_message(*typed_message, time);
    }
  }

  /**
   * @brief 处理序列化消息 (Handle serialized message)
   *
   * @param[in] serialized_message 序列化后的消息 (Serialized message)
   * @param[in] message_info 消息信息 (Message information)
   */
  void handle_serialized_message(
      const std::shared_ptr<rclcpp::SerializedMessage> &serialized_message,
      const rclcpp::MessageInfo &message_info) override {
    // TODO(wjwwood): enable topic statistics for serialized messages
    // 分发序列化消息和消息信息 (Dispatch the serialized message and message information)
    any_callback_.dispatch(serialized_message, message_info);
  }

  /**
   * @brief 处理借用的消息 (Handle loaned message)
   *
   * @param[in] loaned_message 借用的消息指针 (Pointer to the loaned message)
   * @param[in] message_info 消息信息 (Message information)
   */
  void handle_loaned_message(
      void *loaned_message, const rclcpp::MessageInfo &message_info) override {
    // 检查是否与任何内部处理发布器匹配 (Check if it matches any intra-process publishers)
    if (matches_any_intra_process_publishers(&message_info.get_rmw_message_info().publisher_gid)) {
      // 在这种情况下，消息将通过内部处理传递，我们应该忽略此消息副本。
      // (In this case, the message will be delivered via intra process and
      // we should ignore this copy of the message.)
      return;
    }

    // 将借来的消息转换为特定类型 (Cast the loaned message to the specific type)
    auto typed_message = static_cast<ROSMessageType *>(loaned_message);
    // 消息是借来的，因此我们必须确保删除器不会释放消息
    // (The message is loaned, so we have to make sure that the deleter does not deallocate the
    // message)
    auto sptr =
        std::shared_ptr<ROSMessageType>(typed_message, [](ROSMessageType *msg) { (void)msg; });

    // 获取当前时间
    std::chrono::time_point<std::chrono::system_clock> now;
    if (subscription_topic_statistics_) {
      // 在执行回调之前获取当前时间，以将回调持续时间从主题统计结果中排除。
      // (Get the current time before executing the callback to
      // exclude the callback duration from the topic statistics result.)
      now = std::chrono::system_clock::now();
    }

    // 分发消息和消息信息 (Dispatch the message and message information)
    any_callback_.dispatch(sptr, message_info);

    if (subscription_topic_statistics_) {
      // 计算时间点的纳秒表示 (Compute the nanosecond representation of the time point)
      const auto nanos = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
      const auto time = rclcpp::Time(nanos.time_since_epoch().count());
      // 处理消息并更新订阅主题统计信息 (Handle the message and update subscription topic
      // statistics)
      subscription_topic_statistics_->handle_message(*typed_message, time);
    }
  }

  /**
   * @brief 返回借用的消息 (Return the borrowed message)
   *
   * @param[inout] message 要返回的消息 (Message to be returned)
   */
  void return_message(std::shared_ptr<void> &message) override {
    // 将消息转换为特定类型 (Cast the message to the specific type)
    auto typed_message = std::static_pointer_cast<ROSMessageType>(message);
    // 返回消息到内存策略 (Return the message to the memory strategy)
    message_memory_strategy_->return_message(typed_message);
  }

  /// 返回借用的序列化消息。
  /// Return the borrowed serialized message.
  /**
   * \param[inout] message 要返回的序列化消息
   * \param[inout] message serialized message to be returned
   */
  void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> &message) override {
    // 调用 message_memory_strategy_ 的 return_serialized_message 方法来归还序列化消息
    // Call the return_serialized_message method of message_memory_strategy_ to return the
    // serialized message
    message_memory_strategy_->return_serialized_message(message);
  }

  // 使用 take_shared 方法
  // Use take_shared method
  bool use_take_shared_method() const {
    // 调用 any_callback_ 的 use_take_shared_method 方法，获取是否使用 take_shared 方法的值
    // Call the use_take_shared_method method of any_callback_ to get whether to use the take_shared
    // method or not
    return any_callback_.use_take_shared_method();
  }

private:
  /**
   * @brief 禁用 Subscription 类的拷贝构造函数和赋值运算符 (Disable copy constructor and assignment
   * operator for Subscription class)
   */
  RCLCPP_DISABLE_COPY(Subscription)

  /// 任意订阅回调，用于处理订阅到的消息 (Any subscription callback, used to handle subscribed
  /// messages)
  AnySubscriptionCallback<MessageT, AllocatorT> any_callback_;

  /// 在构造时传递的原始选项的副本 (Copy of original options passed during construction)
  /**
   * @details 保存此副本非常重要，以便在订阅期间保持它可能包含的 rmw 载荷的生存周期
   *          (It is important to save a copy of this so that the rmw payload which it
   *           may contain is kept alive for the duration of the subscription)
   */
  const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> options_;

  /// 消息内存策略，用于管理订阅到的消息的内存 (Message memory strategy, used to manage memory of
  /// subscribed messages)
  typename message_memory_strategy::MessageMemoryStrategy<ROSMessageType, AllocatorT>::SharedPtr
      message_memory_strategy_;

  /// 计算并发布此订阅者的主题统计信息的组件 (Component which computes and publishes topic
  /// statistics for this subscriber)
  SubscriptionTopicStatisticsSharedPtr subscription_topic_statistics_{nullptr};
};

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_HPP_

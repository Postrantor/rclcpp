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

#ifndef RCLCPP__SUBSCRIPTION_BASE_HPP_
#define RCLCPP__SUBSCRIPTION_BASE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcl/event_callback.h"
#include "rcl/subscription.h"
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/detail/cpp_callback_trampoline.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/network_flow_endpoint.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription_content_filter_options.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/impl/cpp/demangle.hpp"
#include "rmw/rmw.h"

namespace rclcpp {

namespace node_interfaces {
class NodeBaseInterface;
}  // namespace node_interfaces

namespace experimental {
/**
 * @class IntraProcessManager
 * @brief IntraProcessManager 类在这里进行前向声明，避免了 `intra_process_manager.hpp` 和
 * `subscription_base.hpp` 之间的循环包含。 IntraProcessManager class is forward declared here,
 * avoiding a circular inclusion between `intra_process_manager.hpp` and `subscription_base.hpp`.
 */
class IntraProcessManager;
}  // namespace experimental

/// @brief 虚拟基类，用于订阅。这种模式允许我们在其他方面迭代不同的 Subscription 模板特化。
/// Virtual base class for subscriptions. This pattern allows us to iterate over different template
/// specializations of Subscription, among other things.
class SubscriptionBase : public std::enable_shared_from_this<SubscriptionBase> {
public:
  /// @brief 定义智能指针相关的类型别名和操作，使得 SubscriptionBase 不可复制。
  /// Define smart pointer-related type aliases and operations, making SubscriptionBase
  /// non-copyable.
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(SubscriptionBase)

  /// 构造函数。 (Constructor)
  /**
   * 本构造函数接受 rcl_subscription_options_t 而非 rclcpp::SubscriptionOptions，因为
   * rclcpp::SubscriptionOptions::to_rcl_subscription_options 取决于消息类型。
   * (This accepts rcl_subscription_options_t instead of rclcpp::SubscriptionOptions because
   * rclcpp::SubscriptionOptions::to_rcl_subscription_options depends on the message type.)
   *
   * \param[in] node_base 用于部分设置的 NodeBaseInterface 指针。 (NodeBaseInterface pointer used in
   * parts of the setup.)
   * \param[in] type_support_handle rosidl 类型支持结构，用于主题的 Message
   * 类型。 (rosidl type support struct, for the Message type of the topic.)
   * \param[in] topic_name
   * 要订阅的主题名称。 (Name of the topic to subscribe to.)
   * \param[in] subscription_options
   * 订阅选项。 (Options for the subscription.)
   * \param[in] event_callbacks 事件回调。 (Event
   * callbacks.)
   * \param[in] use_default_callbacks 是否使用默认回调。 (Whether to use default
   * callbacks.)
   * \param[in] is_serialized 如果消息将以序列化形式传递，则为 true。(is true if the
   * message will be delivered still serialized)
   */
  RCLCPP_PUBLIC
  SubscriptionBase(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      const rosidl_message_type_support_t &type_support_handle,
      const std::string &topic_name,
      const rcl_subscription_options_t &subscription_options,
      const SubscriptionEventCallbacks &event_callbacks,
      bool use_default_callbacks,
      bool is_serialized = false);

  /// 析构函数。 (Destructor)
  RCLCPP_PUBLIC
  virtual ~SubscriptionBase();

  /// 为传入的 event_callbacks 添加事件处理程序。 (Add event handlers for passed in event_callbacks)
  RCLCPP_PUBLIC
  void bind_event_callbacks(
      const SubscriptionEventCallbacks &event_callbacks, bool use_default_callbacks);

  /// 获取此订阅所订阅的主题。 (Get the topic that this subscription is subscribed on)
  RCLCPP_PUBLIC
  const char *get_topic_name() const;

  RCLCPP_PUBLIC
  std::shared_ptr<rcl_subscription_t> get_subscription_handle();

  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_subscription_t> get_subscription_handle() const;

  /// 获取与此订阅关联的所有 QoS 事件处理程序。 (Get all the QoS event handlers associated with this
  /// subscription)
  /** \return QoS事件处理程序映射。 (The map of QoS event handlers) */
  RCLCPP_PUBLIC
  const std::
      unordered_map<rcl_subscription_event_type_t, std::shared_ptr<rclcpp::QOSEventHandlerBase>> &
      get_event_handlers() const;

  /// 获取实际的 QoS 设置，确定默认值后。
  /// Get the actual QoS settings, after the defaults have been determined.
  /**
   * 使用 RMW_QOS_POLICY_*_SYSTEM_DEFAULT 时，实际应用的配置只能在订阅创建后解析，
   * 并且取决于底层的 rmw 实现。如果正在使用的底层设置无法用 ROS 术语表示，
   * 则将其设置为 RMW_QOS_POLICY_*_UNKNOWN。
   * 当出现意外错误时，可能会抛出 runtime_error。
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the subscription, and it
   * depends on the underlying rmw implementation. If the underlying setting in use
   * can't be represented in ROS terms, it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return 实际的 qos 设置。
   * \return The actual qos settings.
   * \throws 如果无法获取 qos 设置，则抛出 std::runtime_error
   * \throws std::runtime_error if failed to get qos settings
   */
  RCLCPP_PUBLIC
  rclcpp::QoS get_actual_qos() const;

  /// 以类型擦除指针的形式从订阅中获取下一个进程间消息。
  /// Take the next inter-process message from the subscription as a type erased pointer.
  /**
   * 关于此功能的详细信息，请参阅 Subscription::take()。
   * \sa Subscription::take() for details on how this function works.
   *
   * 唯一的区别是它采用类型擦除指针，而不是精确的消息类型引用。
   * The only difference is that it takes a type erased pointer rather than a
   * reference to the exact message type.
   *
   * 此类型擦除版本通过使用 SubscriptionBase::create_message() 和
   * SubscriptionBase::handle_message() 以类型不可知的方式使用订阅。
   * This type erased version facilitates using the subscriptions in a type
   * agnostic way using SubscriptionBase::create_message() and
   * SubscriptionBase::handle_message().
   *
   * \param[out] message_out 将采用数据复制到其中的类型擦除消息指针。
   * \param[out] message_out The type erased message pointer into which take
   *   will copy the data.
   * \param[out] message_info_out 已获取消息的消息信息。
   * \param[out] message_info_out The message info for the taken message.
   * \returns 如果数据已被采用且有效，则返回 true，否则返回 false
   * \returns true if data was taken and is valid, otherwise false
   * \throws 任何来自 rcl_take 的 rcl 错误，\sa rclcpp::exceptions::throw_from_rcl_error()
   * \throws any rcl errors from rcl_take, \sa rclcpp::exceptions::throw_from_rcl_error()
   */
  RCLCPP_PUBLIC
  bool take_type_erased(void *message_out, rclcpp::MessageInfo &message_info_out);

  /// 从订阅中获取下一个进程间消息的序列化形式。
  /// Take the next inter-process message, in its serialized form, from the subscription.
  /**
   * 目前，如果将数据（写入）message_out 和 message_info_out，
   * 则返回 true。与 Subscription::take() 不同，暂时无法通过进程内部
   * 采用序列化数据，因此无需在任何情况下去重复数据。
   * For now, if data is taken (written) into the message_out and
   * message_info_out then true will be returned. Unlike Subscription::take(),
   * taking data serialized is not possible via intra-process for the time being,
   * so it will not need to de-duplicate data in any case.
   *
   * \param[out] message_out 用于存储已获取消息的序列化消息数据结构。
   * \param[out] message_out The serialized message data structure used to
   *   store the taken message.
   * \param[out] message_info_out 已获取消息的消息信息。
   * \param[out] message_info_out The message info for the taken message.
   * \returns 如果数据已被采用且有效，则返回 true，否则返回 false
   * \returns true if data was taken and is valid, otherwise false
   * \throws 任何来自 rcl_take 的 rcl 错误，\sa rclcpp::exceptions::throw_from_rcl_error()
   * \throws any rcl errors from rcl_take, \sa rclcpp::exceptions::throw_from_rcl_error()
   */
  RCLCPP_PUBLIC
  bool take_serialized(
      rclcpp::SerializedMessage &message_out, rclcpp::MessageInfo &message_info_out);

  /// 借用一个新消息。
  /// Borrow a new message.
  /** \return 指向新消息的共享指针。 */
  /** \return Shared pointer to the fresh message. */
  RCLCPP_PUBLIC
  virtual std::shared_ptr<void> create_message() = 0;

  /// 借用一个新的序列化消息
  /// Borrow a new serialized message
  /** \return 指向 rcl_message_serialized_t 的共享指针。 */
  /** \return Shared pointer to a rcl_message_serialized_t. */
  RCLCPP_PUBLIC
  virtual std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() = 0;

  /// 检查是否需要处理消息，如果需要则执行回调。
  /// Check if we need to handle the message, and execute the callback if we do.
  /**
   * \param[in] message 需要处理的消息的共享指针。
   * \param[in] message_info 与此消息相关的元数据。
   * \param[in] message Shared pointer to the message to handle.
   * \param[in] message_info Metadata associated with this message.
   */
  RCLCPP_PUBLIC
  virtual void handle_message(
      std::shared_ptr<void> &message, const rclcpp::MessageInfo &message_info) = 0;

  /// 处理序列化消息。
  /// Handle serialized message.
  RCLCPP_PUBLIC
  virtual void handle_serialized_message(
      const std::shared_ptr<rclcpp::SerializedMessage> &serialized_message,
      const rclcpp::MessageInfo &message_info) = 0;

  /// 处理借用的消息。
  /// Handle loaned message.
  RCLCPP_PUBLIC
  virtual void handle_loaned_message(
      void *loaned_message, const rclcpp::MessageInfo &message_info) = 0;

  /// 返回在 create_message 中借用的消息。
  /// Return the message borrowed in create_message.
  /** \param[in] message 被返回消息的共享指针。
   * \param[in] message Shared pointer to the returned message. */
  RCLCPP_PUBLIC
  virtual void return_message(std::shared_ptr<void> &message) = 0;

  /// 返回在 create_serialized_message 中借用的消息。
  /// Return the message borrowed in create_serialized_message.
  /** \param[in] message 被返回消息的共享指针。
   * \param[in] message Shared pointer to the returned message. */
  RCLCPP_PUBLIC
  virtual void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> &message) = 0;

  /// 获取消息类型支持句柄。
  /// Get message type support handle.
  RCLCPP_PUBLIC
  const rosidl_message_type_support_t &get_message_type_support_handle() const;

  /// 判断订阅是否序列化。
  /// Check if the subscription is serialized.
  /**
   * \return 如果订阅已序列化，则返回 `true`，否则返回 `false`。
   * \return `true` if the subscription is serialized, `false` otherwise.
   */
  RCLCPP_PUBLIC
  bool is_serialized() const;

  /// 获取匹配的发布者数量。
  /// Get matching publisher count.
  /** \return 此主题上的发布者数量。
   * \return The number of publishers on this topic. */
  RCLCPP_PUBLIC
  size_t get_publisher_count() const;

  /// 检查订阅实例是否可以借用消息。
  /// Check if subscription instance can loan messages.
  /**
   * 根据中间件和消息类型，如果中间件可以分配 ROS 消息实例，则返回 true。
   * Depending on the middleware and the message type, this will return true if the middleware
   * can allocate a ROS message instance.
   *
   * \return 表示中间件是否可以借用消息的布尔标志。
   * \return boolean flag indicating if middleware can loan messages.
   */
  RCLCPP_PUBLIC
  bool can_loan_messages() const;

  using IntraProcessManagerWeakPtr = std::weak_ptr<rclcpp::experimental::IntraProcessManager>;

  /// 实现细节。
  /// Implemenation detail.
  RCLCPP_PUBLIC
  void setup_intra_process(
      uint64_t intra_process_subscription_id, IntraProcessManagerWeakPtr weak_ipm);

  /// 返回用于内部处理的等待对象。
  /// Return the waitable for intra-process.
  /**
   * \return 内部处理的等待对象共享指针，如果没有设置内部处理，则返回 nullptr。
   * \return the waitable sharedpointer for intra-process, or nullptr if intra-process is not setup.
   * \throws std::runtime_error 如果内部处理管理器被销毁。
   * \throws std::runtime_error if the intra process manager is destroyed.
   */
  RCLCPP_PUBLIC
  rclcpp::Waitable::SharedPtr get_intra_process_waitable() const;

  /// 交换订阅的一部分是否被等待集合使用的状态。
  /// Exchange state of whether or not a part of the subscription is used by a wait set.
  /**
   * 用于确保订阅的各个部分不会与多个等待集合同时使用。
   * Used to ensure parts of the subscription are not used with multiple wait
   * sets simultaneously.
   *
   * \param[in] pointer_to_subscription_part 订阅部分的地址。
   * \param[in] in_use_state 要交换的新状态，true 表示 "现在正在使用"，
   *   false 表示 "不再使用"。
   * \param[in] pointer_to_subscription_part address of a subscription part.
   * \param[in] in_use_state the new state to exchange, true means "now in use",
   *   and false means "no longer in use".
   * \returns 当前的 "正在使用" 状态。
   * \returns the current "in use" state.
   * \throws std::invalid_argument 如果 pointer_to_subscription_part 为 nullptr。
   * \throws std::invalid_argument If pointer_to_subscription_part is nullptr.
   * \throws std::runtime_error 如果给定的指针不是可与等待集合一起使用的订阅部分之一的指针。
   * \throws std::runtime_error If the pointer given is not a pointer to one of
   *   the parts of the subscription which can be used with a wait set.
   */
  RCLCPP_PUBLIC
  bool exchange_in_use_by_wait_set_state(void *pointer_to_subscription_part, bool in_use_state);

  /// 获取网络流端点 (Get network flow endpoints)
  /**
   * 描述此订阅正在接收消息的网络流端点 (Describes network flow endpoints that this subscription is
   * receiving messages on) \return NetworkFlowEndpoint 的向量 (vector of NetworkFlowEndpoint)
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::NetworkFlowEndpoint> get_network_flow_endpoints() const;

  /// 设置一个回调，当收到每个新消息时调用。 (Set a callback to be called when each new message is
  /// received.)
  /**
   * 回调接收一个 size_t，它是自上次调用此回调以来收到的消息数。 (The callback receives a size_t
   * which is the number of messages received since the last time this callback was called.)
   * 通常这是1，但如果在设置任何回调之前接收到了消息，则可以 > 1。 (Normally this is 1, but can be >
   * 1 if messages were received before any callback was set.)
   *
   * 由于此回调是从中间件调用的，您应该尽量使其快速且不阻塞。 (Since this callback is called from
   * the middleware, you should aim to make it fast and not blocking.)
   * 如果需要做很多工作或等待其他事件，您应该将其转移到另一个线程，否则您可能会阻止中间件。 (If you
   * need to do a lot of work or wait for some other event, you should spin it off to another
   * thread, otherwise you risk blocking the middleware.)
   *
   * 再次调用它将清除之前设置的任何回调。 (Calling it again will clear any previously set callback.)
   *
   * 此功能是线程安全的。 (This function is thread-safe.)
   *
   * 如果您希望在回调中提供更多信息，例如订阅或其他信息，您可以使用带有捕获的 lambda 或 std::bind。
   * (If you want more information available in the callback, like the subscription or other
   * information, you may use a lambda with captures or std::bind.)
   *
   * \sa rmw_subscription_set_on_new_message_callback
   * \sa rcl_subscription_set_on_new_message_callback
   *
   * \param[in] callback 收到新消息时要调用的函数对象 (functor to be called when a new message is
   * received)
   */
  void set_on_new_message_callback(std::function<void(size_t)> callback) {
    // 检查回调是否可调用 (Check if the callback is callable)
    if (!callback) {
      throw std::invalid_argument(
          "The callback passed to set_on_new_message_callback "
          "is not callable.");
    }

    // 创建一个新的回调 (Create a new callback)
    auto new_callback = [callback, this](size_t number_of_messages) {
      try {
        // 调用回调 (Call the callback)
        callback(number_of_messages);
      } catch (const std::exception &exception) {
        // 捕获并记录异常 (Catch and log the exception)
        RCLCPP_ERROR_STREAM(
            node_logger_,
            "rclcpp::SubscriptionBase@"
                << this << " caught " << rmw::impl::cpp::demangle(exception)
                << " exception in user-provided callback for the 'on new message' callback: "
                << exception.what());
      } catch (...) {
        // 捕获并记录未处理的异常 (Catch and log unhandled exception)
        RCLCPP_ERROR_STREAM(
            node_logger_, "rclcpp::SubscriptionBase@"
                              << this << " caught unhandled exception in user-provided callback "
                              << "for the 'on new message' callback");
      }
    };

    // 锁定回调互斥锁 (Lock the callback mutex)
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    // 将其临时设置为新回调，同时替换旧回调。 (Set it temporarily to the new callback, while we
    // replace the old one.) 这种两步设置可以防止出现这样的情况：旧的 std::function
    // 已经被替换，但中间件尚未得知新的回调。 (This two-step setting, prevents a gap where the old
    // std::function has been replaced but the middleware hasn't been told about the new one yet.)
    set_on_new_message_callback(
        rclcpp::detail::cpp_callback_trampoline<decltype(new_callback), const void *, size_t>,
        static_cast<const void *>(&new_callback));

    // 存储 std::function 以保持其作用域，也覆盖现有的。 (Store the std::function to keep it in
    // scope, also overwrites the existing one.)
    on_new_message_callback_ = new_callback;

    // 再次设置它，现在使用永久存储。 (Set it again, now using the permanent storage.)
    set_on_new_message_callback(
        rclcpp::detail::cpp_callback_trampoline<
            decltype(on_new_message_callback_), const void *, size_t>,
        static_cast<const void *>(&on_new_message_callback_));
  }

  /// 取消注册新消息的回调（如果有）。 (Unset the callback registered for new messages, if any.)
  void clear_on_new_message_callback() {
    // 锁定回调互斥锁 (Lock the callback mutex)
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    // 如果有回调，清除它 (If there's a callback, clear it)
    if (on_new_message_callback_) {
      set_on_new_message_callback(nullptr, nullptr);
      on_new_message_callback_ = nullptr;
    }
  }

  /// 设置一个回调，在收到每个新的内部进程消息时调用。
  /// Set a callback to be called when each new intra-process message is received.
  /**
   * 回调接收一个 size_t，表示自上次调用此回调以来收到的消息数。
   * The callback receives a size_t which is the number of messages received
   * since the last time this callback was called.
   * 通常这是1，但如果在设置任何回调之前收到了消息，则可以是 > 1。
   * Normally this is 1, but can be > 1 if messages were received before any
   * callback was set.
   *
   * 再次调用它将清除之前设置的任何回调。
   * Calling it again will clear any previously set callback.
   *
   * 此函数是线程安全的。
   * This function is thread-safe.
   *
   * 如果您希望在回调中提供更多信息，例如订阅或其他信息，可以使用带捕获的 lambda 或 std::bind。
   * If you want more information available in the callback, like the subscription
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \sa rclcpp::SubscriptionIntraProcessBase::set_on_ready_callback
   *
   * \param[in] callback 收到新消息时要调用的函数对象
   * \param[in] callback functor to be called when a new message is received
   */
  void set_on_new_intra_process_message_callback(std::function<void(size_t)> callback) {
    // 如果不使用内部进程通信，则发出警告
    // If not using intra-process communication, issue a warning
    if (!use_intra_process_) {
      RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"),
          "Calling set_on_new_intra_process_message_callback for subscription with IPC disabled");
      return;
    }

    // 如果回调不可调用，则抛出无效参数异常
    // If the callback is not callable, throw an invalid_argument exception
    if (!callback) {
      throw std::invalid_argument(
          "The callback passed to set_on_new_intra_process_message_callback "
          "is not callable.");
    }

    // on_ready_callback 签名有一个额外的 `int` 参数，用于在通用可等待对象中区分可能的不同实体。
    // The on_ready_callback signature has an extra `int` argument used to disambiguate between
    // possible different entities within a generic waitable.
    // 我们向此方法的用户隐藏了这个细节。
    // We hide that detail to users of this method.
    std::function<void(size_t, int)> new_callback = std::bind(callback, std::placeholders::_1);
    subscription_intra_process_->set_on_ready_callback(new_callback);
  }

  /// 取消注册新内部进程消息的回调（如果有）。
  /// Unset the callback registered for new intra-process messages, if any.
  void clear_on_new_intra_process_message_callback() {
    // 如果不使用内部进程通信，则发出警告
    // If not using intra-process communication, issue a warning
    if (!use_intra_process_) {
      RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"),
          "Calling clear_on_new_intra_process_message_callback for subscription with IPC disabled");
      return;
    }

    // 清除准备好的回调
    // Clear the on_ready callback
    subscription_intra_process_->clear_on_ready_callback();
  }

  /// 设置一个回调，在每次出现新的 qos 事件实例时调用。
  /// Set a callback to be called when each new qos event instance occurs.
  /**
   * 回调接收一个 size_t，表示自上次调用此回调以来发生的事件数。
   * The callback receives a size_t which is the number of events that occurred
   * since the last time this callback was called.
   * 通常这是1，但如果在设置任何回调之前发生了事件，则可以是 > 1。
   * Normally this is 1, but can be > 1 if events occurred before any
   * callback was set.
   *
   * 由于此回调是从中间件调用的，因此您应该尽量使其快速且不阻塞。
   * Since this callback is called from the middleware, you should aim to make
   * it fast and not blocking.
   * 如果您需要做很多工作或等待其他事件，您应该将其转移到另一个线程，否则您可能会阻塞中间件。
   * If you need to do a lot of work or wait for some other event, you should
   * spin it off to another thread, otherwise you risk blocking the middleware.
   *
   * 再次调用它将清除之前设置的任何回调。
   * Calling it again will clear any previously set callback.
   *
   * 如果回调不可调用，将抛出异常。
   * An exception will be thrown if the callback is not callable.
   *
   * 此函数是线程安全的。
   * This function is thread-safe.
   *
   * 如果您希望在回调中提供更多信息，例如 qos 事件或其他信息，可以使用带捕获的 lambda 或 std::bind。
   * If you want more information available in the callback, like the qos event
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \sa rclcpp::QOSEventHandlerBase::set_on_ready_callback
   *
   * \param[in] callback 发生新事件时要调用的函数对象
   * \param[in] callback functor to be called when a new event occurs
   * \param[in] event_type 我们想要将回调附加到的 qos 事件的标识符
   * \param[in] event_type identifier for the qos event we want to attach the callback to
   */
  void set_on_new_qos_event_callback(
      std::function<void(size_t)> callback, rcl_subscription_event_type_t event_type) {
    // 如果未注册 event_type，则发出警告
    // Issue a warning if the event_type is not registered
    if (event_handlers_.count(event_type) == 0) {
      RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"),
          "Calling set_on_new_qos_event_callback for non registered subscription event_type");
      return;
    }

    // 如果回调不可调用，则抛出无效参数异常
    // If the callback is not callable, throw an invalid_argument exception
    if (!callback) {
      throw std::invalid_argument(
          "The callback passed to set_on_new_qos_event_callback "
          "is not callable.");
    }

    // on_ready_callback 签名有一个额外的 `int` 参数，用于在通用可等待对象中区分可能的不同实体。
    // The on_ready_callback signature has an extra `int` argument used to disambiguate between
    // possible different entities within a generic waitable.
    // 我们向此方法的用户隐藏了这个细节。
    // We hide that detail to users of this method.
    std::function<void(size_t, int)> new_callback = std::bind(callback, std::placeholders::_1);
    event_handlers_[event_type]->set_on_ready_callback(new_callback);
  }

  /// 取消注册新 qos 事件的回调（如果有）。
  /// Unset the callback registered for new qos events, if any.
  /**
   * \param[in] event_type 要取消注册的事件类型。
   * \param[in] event_type The event type to unregister.
   */
  void clear_on_new_qos_event_callback(rcl_subscription_event_type_t event_type) {
    // 如果事件类型未注册，返回 0。
    // If the event type is not registered, return 0.
    if (event_handlers_.count(event_type) == 0) {
      // 打印警告日志，表示试图取消未注册的事件类型的回调。
      // Print a warning log indicating an attempt to clear the callback for a non-registered event
      // type.
      RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"),
          "Calling clear_on_new_qos_event_callback for non registered event_type");
      return;
    }

    // 清除指定事件类型的回调。
    // Clear the callback for the specified event type.
    event_handlers_[event_type]->clear_on_ready_callback();
  }

  /// 检查订阅实例的内容过滤主题功能是否启用。
  /// Check if content filtered topic feature of the subscription instance is enabled.
  /**
   * \return 返回布尔值，表示订阅的内容过滤主题是否启用。
   * \return A boolean flag indicating if the content filtered topic of this subscription is
   * enabled.
   */
  RCLCPP_PUBLIC
  bool is_cft_enabled() const;

  /// 为订阅设置过滤表达式和表达式参数。
  /// Set the filter expression and expression parameters for the subscription.
  /**
   * \param[in] filter_expression 要设置的过滤表达式。
   * \param[in] filter_expression A filter expression to set.
   *   \sa ContentFilterOptions::filter_expression
   *   空字符串（""）将清除订阅的内容过滤设置。
   *   An empty string ("") will clear the content filter setting of the subscription.
   * \param[in] expression_parameters 要设置的表达式参数数组。
   * \param[in] expression_parameters Array of expression parameters to set.
   *   \sa ContentFilterOptions::expression_parameters
   * \throws RCLBadAlloc 如果无法分配内存
   * \throws RCLBadAlloc if memory cannot be allocated
   * \throws RCLError 如果发生意外错误
   * \throws RCLError if an unexpected error occurs
   */
  RCLCPP_PUBLIC
  void set_content_filter(
      const std::string &filter_expression,
      const std::vector<std::string> &expression_parameters = {});

  /// 获取订阅的过滤表达式和表达式参数。
  /// Get the filter expression and expression parameters for the subscription.
  /**
   * \return 返回 rclcpp::ContentFilterOptions 类型的内容过滤选项。
   * \return rclcpp::ContentFilterOptions The content filter options to get.
   * \throws RCLBadAlloc 如果无法分配内存
   * \throws RCLBadAlloc if memory cannot be allocated
   * \throws RCLError 如果发生意外错误
   * \throws RCLError if an unexpected error occurs
   */
  RCLCPP_PUBLIC
  rclcpp::ContentFilterOptions get_content_filter() const;

protected:
  /**
   * @brief 添加事件处理器 (Add event handler)
   *
   * @tparam EventCallbackT 事件回调类型 (Event callback type)
   * @param[in] callback 事件回调函数 (Event callback function)
   * @param[in] event_type 订阅事件类型 (Subscription event type)
   */
  template <typename EventCallbackT>
  void add_event_handler(
      const EventCallbackT &callback, const rcl_subscription_event_type_t event_type) {
    // 创建一个 QOSEventHandler 实例，并将其与订阅句柄关联 (Create a QOSEventHandler instance and
    // associate it with the subscription handle)
    auto handler =
        std::make_shared<QOSEventHandler<EventCallbackT, std::shared_ptr<rcl_subscription_t>>>(
            callback, rcl_subscription_event_init, get_subscription_handle(), event_type);

    // 将创建的事件处理器插入 qos_events_in_use_by_wait_set_ 中 (Insert the created event handler
    // into qos_events_in_use_by_wait_set_)
    qos_events_in_use_by_wait_set_.insert(std::make_pair(handler.get(), false));

    // 将事件处理器插入 event_handlers_ 中 (Insert the event handler into event_handlers_)
    event_handlers_.insert(std::make_pair(event_type, handler));
  }

  // 默认不兼容 QoS 回调函数 (Default incompatible QoS callback function)
  RCLCPP_PUBLIC
  void default_incompatible_qos_callback(QOSRequestedIncompatibleQoSInfo &info) const;

  // 检查是否与任何内部进程发布者匹配 (Check if it matches any intra-process publishers)
  RCLCPP_PUBLIC
  bool matches_any_intra_process_publishers(const rmw_gid_t *sender_gid) const;

  // 设置新消息回调函数 (Set new message callback function)
  RCLCPP_PUBLIC
  void set_on_new_message_callback(rcl_event_callback_t callback, const void *user_data);

  // 节点基础接口指针 (Node base interface pointer)
  rclcpp::node_interfaces::NodeBaseInterface *const node_base_;

  // 节点句柄 (Node handle)
  std::shared_ptr<rcl_node_t> node_handle_;

  // 订阅句柄 (Subscription handle)
  std::shared_ptr<rcl_subscription_t> subscription_handle_;

  // 内部进程订阅句柄 (Intra-process subscription handle)
  std::shared_ptr<rcl_subscription_t> intra_process_subscription_handle_;

  // 节点日志记录器 (Node logger)
  rclcpp::Logger node_logger_;

  // 事件处理器映射 (Event handlers map)
  std::unordered_map<rcl_subscription_event_type_t, std::shared_ptr<rclcpp::QOSEventHandlerBase>>
      event_handlers_;

  // 是否使用内部进程通信 (Whether to use intra-process communication)
  bool use_intra_process_;

  // 弱指针，指向 IntraProcessManager 实例 (Weak pointer to IntraProcessManager instance)
  IntraProcessManagerWeakPtr weak_ipm_;

  // 内部进程订阅 ID (Intra-process subscription ID)
  uint64_t intra_process_subscription_id_;

  // 内部进程订阅实例 (Intra-process subscription instance)
  std::shared_ptr<rclcpp::experimental::SubscriptionIntraProcessBase> subscription_intra_process_;

  // 订阅事件回调集合 (Subscription event callbacks collection)
  const SubscriptionEventCallbacks event_callbacks_;

private:
  /**
   * @brief 禁用 SubscriptionBase 的拷贝构造函数和赋值操作符
   * @details Disable copy constructor and assignment operator for SubscriptionBase
   */
  RCLCPP_DISABLE_COPY(SubscriptionBase)

  // 定义消息类型支持的结构体变量
  // Define the message type support struct variable
  rosidl_message_type_support_t type_support_;

  // 表示订阅的消息是否已序列化的标志
  // Flag indicating whether the subscribed message is serialized or not
  bool is_serialized_;

  // 表示订阅是否被 wait set 使用的原子布尔变量
  // Atomic boolean variable indicating whether the subscription is in use by a wait set or not
  std::atomic<bool> subscription_in_use_by_wait_set_{false};

  // 表示内部进程订阅可等待对象是否被 wait set 使用的原子布尔变量
  // Atomic boolean variable indicating whether the intra-process subscription waitable is in use by
  // a wait set or not
  std::atomic<bool> intra_process_subscription_waitable_in_use_by_wait_set_{false};

  // 用于存储 QoS 事件处理器及其对应的原子布尔变量（表示事件处理器是否被 wait set 使用）
  // Unordered map to store QoS event handlers and their corresponding atomic boolean variables
  // (indicating whether the event handler is in use by a wait set or not)
  std::unordered_map<rclcpp::QOSEventHandlerBase *, std::atomic<bool>>
      qos_events_in_use_by_wait_set_;

  // 用于保护回调函数访问的递归互斥锁
  // Recursive mutex to protect access to the callback function
  std::recursive_mutex callback_mutex_;

  // 当有新消息时调用的回调函数，参数为消息大小
  // Callback function to be called when a new message arrives, with the message size as parameter
  std::function<void(size_t)> on_new_message_callback_{nullptr};
};

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_BASE_HPP_

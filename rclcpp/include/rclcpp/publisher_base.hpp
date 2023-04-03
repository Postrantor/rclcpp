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

#ifndef RCLCPP__PUBLISHER_BASE_HPP_
#define RCLCPP__PUBLISHER_BASE_HPP_

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcl/publisher.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/network_flow_endpoint.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcpputils/time.hpp"

namespace rclcpp {

// Forward declaration is used for friend statement.
namespace node_interfaces {
class NodeBaseInterface;
class NodeTopicsInterface;
}  // namespace node_interfaces

namespace experimental {
/**
 * 此处前向声明 IntraProcessManager 类，避免 `intra_process_manager.hpp` 和 `publisher_base.hpp`
 * 之间的循环包含。
 */
class IntraProcessManager;
}  // namespace experimental

/**
 * @class PublisherBase
 * @brief 基础发布者类 (Base publisher class)
 *
 * 这个类是 ROS2 rclcpp 中的基础发布者类，用于创建一个发布者对象。
 */
class PublisherBase : public std::enable_shared_from_this<PublisherBase> {
  // 允许 NodeTopicsInterface 访问此类的私有成员
  friend ::rclcpp::node_interfaces::NodeTopicsInterface;

public:
  /**
   * @brief 智能指针定义 (Smart pointer definitions)
   * 定义智能指针类型以方便使用。 (Define smart pointer types for easier use.)
   */
  RCLCPP_SMART_PTR_DEFINITIONS(PublisherBase)

  /// 默认构造函数 (Default constructor).
  /**
   * 通常情况下，发布者不是通过此方法创建的，而是通过调用 `Node::create_publisher` 创建的。
   * \param[in] node_base 父节点的 NodeBaseInterface 指针 (A pointer to the NodeBaseInterface for
   * the parent node).
   * \param[in] topic 这个发布者发布的主题 (The topic that this publisher publishes on).
   * \param[in] type_support 要发布的类型的类型支持结构 (The type support structure
   * for the type to be published).
   * \param[in] publisher_options 此发布者的 QoS 设置 (QoS settings for this publisher).
   * \param[in] event_callbacks 事件回调 (PublisherEventCallbacks)
   * \param[in] use_default_callbacks 是否使用默认回调 (Whether to use default callbacks or not)
   */
  RCLCPP_PUBLIC
  PublisherBase(
      rclcpp::node_interfaces::NodeBaseInterface* node_base,
      const std::string& topic,
      const rosidl_message_type_support_t& type_support,
      const rcl_publisher_options_t& publisher_options,
      const PublisherEventCallbacks& event_callbacks,
      bool use_default_callbacks);

  RCLCPP_PUBLIC
  virtual ~PublisherBase();

  /// 为传入的 event_callbacks 添加事件处理程序 (Add event handlers for passed in event_callbacks).
  RCLCPP_PUBLIC
  void bind_event_callbacks(
      const PublisherEventCallbacks& event_callbacks, bool use_default_callbacks);

  /// 获取此发布者发布的主题 (Get the topic that this publisher publishes on).
  /** \return 主题名称 (The topic name). */
  RCLCPP_PUBLIC
  const char* get_topic_name() const;

  /// 获取此发布者的队列大小 (Get the queue size for this publisher).
  /** \return 队列大小 (The queue size). */
  RCLCPP_PUBLIC
  size_t get_queue_size() const;

  /// 获取此发布者的全局标识符（在 rmw 和 DDS 中使用）
  /** \return gid (The gid). */
  RCLCPP_PUBLIC
  const rmw_gid_t& get_gid() const;

  /// 获取 rcl 发布者句柄 (Get the rcl publisher handle).
  /** \return rcl 发布者句柄 (The rcl publisher handle). */
  RCLCPP_PUBLIC
  std::shared_ptr<rcl_publisher_t> get_publisher_handle();

  /// 获取 rcl 发布者句柄 (Get the rcl publisher handle).
  /** \return rcl 发布者句柄 (The rcl publisher handle). */
  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_publisher_t> get_publisher_handle() const;

  /// 获取与此发布者关联的所有 QoS 事件处理程序 (Get all the QoS event handlers associated with this
  /// publisher).
  /** \return QoS 事件处理程序映射 (The map of QoS event handlers). */
  RCLCPP_PUBLIC
  const std::
      unordered_map<rcl_publisher_event_type_t, std::shared_ptr<rclcpp::QOSEventHandlerBase>>&
      get_event_handlers() const;

  /// 获取订阅计数 (Get subscription count)
  /** \return 订阅数量 (The number of subscriptions). */
  RCLCPP_PUBLIC
  size_t get_subscription_count() const;

  /// 获取内部进程订阅计数 (Get intraprocess subscription count)
  /** \return 内部进程订阅数量 (The number of intraprocess subscriptions). */
  RCLCPP_PUBLIC
  size_t get_intra_process_subscription_count() const;

  /// 手动声明此发布器是活动的（针对 RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC）。
  /// Manually assert that this Publisher is alive (for RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC).
  /**
   * 如果 rmw Liveliness 策略设置为 RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC，则此发布器的创建者
   * 可以在某个时间点手动调用 `assert_liveliness` 以向系统的其余部分发出该节点仍然活着的信号。
   * If the rmw Liveliness policy is set to RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC, the creator
   * of this publisher may manually call `assert_liveliness` at some point in time to signal to the
   * rest of the system that this Node is still alive.
   *
   * \return `true` 如果成功声明了活跃性，否则返回 `false`
   * \return `true` if the liveliness was asserted successfully, otherwise `false`
   */
  RCLCPP_PUBLIC
  RCUTILS_WARN_UNUSED
  bool assert_liveliness() const;

  /// 获取实际 QoS 设置，在确定默认值后。
  /// Get the actual QoS settings, after the defaults have been determined.
  /**
   * 使用 RMW_QOS_POLICY_*_SYSTEM_DEFAULT 时应用的实际配置，
   * 只有在创建发布器之后才能解析，并且它取决于底层的 rmw 实现。
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the publisher, and it
   * depends on the underlying rmw implementation.
   * 如果无法用 ROS 术语表示正在使用的底层设置，
   * 它将设置为 RMW_QOS_POLICY_*_UNKNOWN。
   * If the underlying setting in use can't be represented in ROS terms,
   * it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * 当发生意外错误时可能会抛出运行时错误。
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return 实际 qos 设置。
   * \return The actual qos settings.
   */
  RCLCPP_PUBLIC
  rclcpp::QoS get_actual_qos() const;

  /// 检查发布器实例是否可以借用消息。
  /// Check if publisher instance can loan messages.
  /**
   * 根据中间件和消息类型，如果中间件可以分配 ROS 消息实例，则返回 true。
   * Depending on the middleware and the message type, this will return true if the middleware
   * can allocate a ROS message instance.
   */
  RCLCPP_PUBLIC
  bool can_loan_messages() const;

  /// 将此发布器与 gid 进行比较。
  /// Compare this publisher to a gid.
  /**
   * 请注意，此函数调用下一个函数。
   * Note that this function calls the next function.
   * \param[in] gid 对 gid 的引用。
   * \param[in] gid Reference to a gid.
   * \return 如果发布器的 gid 与输入匹配，则为 True。
   * \return True if the publisher's gid matches the input.
   */
  RCLCPP_PUBLIC
  bool operator==(const rmw_gid_t& gid) const;

  /// 将此发布器与指针 gid 进行比较。
  /// Compare this publisher to a pointer gid.
  /**
   * 使用 rmw_compare_gids_equal 将此发布器的 gid 与输入进行比较的包装器。
   * A wrapper for comparing this publisher's gid to the input using rmw_compare_gids_equal.
   * \param[in] gid 指向 gid 的指针。
   * \param[in] gid A pointer to a gid.
   * \return 如果此发布器的 gid 与输入匹配，则为 True。
   * \return True if this publisher's gid matches the input.
   */
  RCLCPP_PUBLIC
  bool operator==(const rmw_gid_t* gid) const;

  using IntraProcessManagerSharedPtr = std::shared_ptr<rclcpp::experimental::IntraProcessManager>;
  /// 在创建后用于设置内部进程发布的实现实用程序函数。
  /// Implementation utility function used to setup intra process publishing after creation.
  RCLCPP_PUBLIC
  void setup_intra_process(uint64_t intra_process_publisher_id, IntraProcessManagerSharedPtr ipm);

  /// 获取网络流端点
  /**
   * 描述此发布器正在发送消息的网络流端点
   * \return NetworkFlowEndpoint 的向量
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::NetworkFlowEndpoint> get_network_flow_endpoints() const;

  /// 等待所有已发布的消息被确认或直到指定的超时时间过去。
  /**
   * 此方法等待所有已发布的消息被所有匹配的订阅者确认，或者给定的超时时间过去。
   *
   * 如果超时时间为负，则此方法将无限期阻塞，直到所有已发布的消息被确认。
   * 如果超时时间为零，则此方法不会阻塞，它将检查所有已发布的消息是否被确认并立即返回。
   * 如果超时时间大于零，此方法将等待所有已发布的消息被确认或超时时间过去。
   *
   * 仅当发布者的 QoS 配置文件为 RELIABLE 时，此方法才等待确认。否则，此方法将立即返回 `true`。
   *
   * \param[in] timeout 等待所有已发布消息被确认的持续时间。
   * \return 如果在给定的超时时间过去之前，所有已发布的消息都被确认，则返回 `true`，否则返回
   * `false`。
   * \throws rclcpp::exceptions::RCLError 如果中间件不支持或内部错误发生
   * \throws std::invalid_argument 如果超时时间大于 std::chrono::nanoseconds::max() 或小于
   * std::chrono::nanoseconds::min()
   */
  template <typename DurationRepT = int64_t, typename DurationT = std::milli>
  bool wait_for_all_acked(
      std::chrono::duration<DurationRepT, DurationT> timeout =
          std::chrono::duration<DurationRepT, DurationT>(-1)) const {
    // 将超时时间转换为纳秒值
    rcl_duration_value_t rcl_timeout = rcpputils::convert_to_nanoseconds(timeout).count();

    // 调用 rcl_publisher_wait_for_all_acked 函数并检查返回值
    rcl_ret_t ret = rcl_publisher_wait_for_all_acked(publisher_handle_.get(), rcl_timeout);
    if (ret == RCL_RET_OK) {
      return true;
    } else if (ret == RCL_RET_TIMEOUT) {
      return false;
    } else {
      // 当发生错误时，从 rcl 错误中抛出异常
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
  }

  /// 设置一个回调，当每个新的 qos 事件实例发生时调用。
  /**
   * 回调接收一个 size_t，它是自上次调用此回调以来发生的事件的数量。
   * 通常这是 1，但如果在设置任何回调之前发生了事件，则可能 > 1。
   *
   * 由于此回调是从中间件调用的，因此您应该尽量使其快速且不阻塞。
   * 如果您需要执行大量工作或等待其他事件，您应该将其分配给另一个线程，否则您可能会阻塞中间件。
   *
   * 再次调用它将清除先前设置的任何回调。
   * 如果回调不可调用，将抛出异常。
   *
   * 此函数是线程安全的。
   *
   * 如果您希望在回调中使用更多信息，如 qos 事件或其他信息，您可以使用带捕获的 lambda 或 std::bind。
   *
   * \sa rclcpp::QOSEventHandlerBase::set_on_ready_callback
   *
   * \param[in] callback 当新事件发生时要调用的函数对象
   * \param[in] event_type 我们想要将回调附加到的 qos 事件的标识符
   */
  void set_on_new_qos_event_callback(
      std::function<void(size_t)> callback, rcl_publisher_event_type_t event_type) {
    // 检查是否有已注册的事件处理程序与给定的事件类型匹配
    if (event_handlers_.count(event_type) == 0) {
      RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"),
          "Calling set_on_new_qos_event_callback for non registered publisher event_type");
      return;
    }

    // 如果回调不可调用，则抛出无效参数异常
    if (!callback) {
      throw std::invalid_argument(
          "The callback passed to set_on_new_qos_event_callback "
          "is not callable.");
    }

    // on_ready_callback 签名具有额外的 `int` 参数，用于消除通用 waitable
    // 内可能存在的不同实体之间的歧义。
    // 我们将此细节隐藏给此方法的用户。
    std::function<void(size_t, int)> new_callback = std::bind(callback, std::placeholders::_1);
    event_handlers_[event_type]->set_on_ready_callback(new_callback);
  }

  /**
   * @brief 清除已注册的新 QoS 事件回调函数（如果有的话）。
   * @param event_type 要清除回调的 rcl_publisher_event_type_t 类型的事件。
   */
  void clear_on_new_qos_event_callback(rcl_publisher_event_type_t event_type) {
    // 检查 event_handlers_ 是否包含给定的 event_type。
    if (event_handlers_.count(event_type) == 0) {
      // 如果 event_handlers_ 不包含给定的 event_type，打印警告信息。
      RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"),
          "Calling clear_on_new_qos_event_callback for non registered event_type");
      // 提前返回，不执行后续操作。
      return;
    }

    // 从 event_handlers_ 中获取指定 event_type 的事件处理器，并清除其就绪回调函数。
    event_handlers_[event_type]->clear_on_ready_callback();
  }

protected:
  /**
   * @brief 添加事件处理程序 (Add event handler)
   *
   * @tparam EventCallbackT 事件回调类型 (Event callback type)
   * @param[in] callback 事件回调函数 (Event callback function)
   * @param[in] event_type 要添加的事件类型 (Event type to add)
   */
  template <typename EventCallbackT>
  void add_event_handler(
      const EventCallbackT& callback, const rcl_publisher_event_type_t event_type) {
    // 创建一个事件处理程序，并使用传入的回调函数、初始化函数、发布者句柄和事件类型进行初始化
    auto handler =
        std::make_shared<QOSEventHandler<EventCallbackT, std::shared_ptr<rcl_publisher_t>>>(
            callback, rcl_publisher_event_init, publisher_handle_, event_type);

    // 将创建的事件处理程序插入到事件处理程序映射中
    event_handlers_.insert(std::make_pair(event_type, handler));
  }

  // 默认不兼容 QoS 回调函数声明
  RCLCPP_PUBLIC
  void default_incompatible_qos_callback(QOSOfferedIncompatibleQoSInfo& info) const;
  // 声明一个指向 rcl_node_t 类型的共享指针
  std::shared_ptr<rcl_node_t> rcl_node_handle_;
  // 声明一个指向 rcl_publisher_t 类型的共享指针
  std::shared_ptr<rcl_publisher_t> publisher_handle_;
  // 声明一个事件处理程序的无序映射
  std::unordered_map<rcl_publisher_event_type_t, std::shared_ptr<rclcpp::QOSEventHandlerBase>>
      event_handlers_;
  // 使用弱指针声明 IntraProcessManager 类型

  using IntraProcessManagerWeakPtr = std::weak_ptr<rclcpp::experimental::IntraProcessManager>;
  // 声明内部进程是否启用的布尔变量
  bool intra_process_is_enabled_;
  // 声明一个指向 IntraProcessManager 的弱指针
  IntraProcessManagerWeakPtr weak_ipm_;
  // 声明内部进程发布者 ID
  uint64_t intra_process_publisher_id_;
  // 声明 RMW 全局唯一标识符
  rmw_gid_t rmw_gid_;
  // 声明消息类型支持常量
  const rosidl_message_type_support_t type_support_;
  // 声明发布者事件回调常量
  const PublisherEventCallbacks event_callbacks_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_BASE_HPP_

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

#include "rclcpp/subscription_base.hpp"

#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/qos_event.hpp"
#include "rcpputils/scope_exit.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

using rclcpp::SubscriptionBase;

/**
 * @brief 构造函数，创建一个订阅器对象 (Constructor, creates a SubscriptionBase object)
 *
 * @param node_base 指向节点基类接口的指针 (Pointer to the NodeBaseInterface)
 * @param type_support_handle 消息类型支持句柄 (Message type support handle)
 * @param topic_name 订阅的主题名称 (Name of the topic to subscribe)
 * @param subscription_options 订阅选项 (Subscription options)
 * @param event_callbacks 事件回调 (Event callbacks)
 * @param use_default_callbacks 是否使用默认回调 (Whether to use default callbacks or not)
 * @param is_serialized 是否序列化 (Whether the message is serialized or not)
 */
SubscriptionBase::SubscriptionBase(
    rclcpp::node_interfaces::NodeBaseInterface *node_base,
    const rosidl_message_type_support_t &type_support_handle,
    const std::string &topic_name,
    const rcl_subscription_options_t &subscription_options,
    const SubscriptionEventCallbacks &event_callbacks,
    bool use_default_callbacks,
    bool is_serialized)
    : node_base_(node_base),
      node_handle_(node_base_->get_shared_rcl_node_handle()),
      // 初始化节点日志 (Initialize the node logger)
      node_logger_(rclcpp::get_node_logger(node_handle_.get())),
      use_intra_process_(false),
      intra_process_subscription_id_(0),
      event_callbacks_(event_callbacks),
      type_support_(type_support_handle),
      is_serialized_(is_serialized) {
  // 自定义删除器 (Custom deletor)
  auto custom_deletor = [node_handle = this->node_handle_](rcl_subscription_t *rcl_subs) {
    if (rcl_subscription_fini(rcl_subs, node_handle.get()) != RCL_RET_OK) {
      RCLCPP_ERROR(
          rclcpp::get_node_logger(node_handle.get()).get_child("rclcpp"),
          "Error in destruction of rcl subscription handle: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
    delete rcl_subs;
  };

  // 创建订阅句柄 (Create the subscription handle)
  subscription_handle_ =
      std::shared_ptr<rcl_subscription_t>(new rcl_subscription_t, custom_deletor);
  *subscription_handle_.get() = rcl_get_zero_initialized_subscription();

  // 初始化订阅器 (Initialize the subscription)
  rcl_ret_t ret = rcl_subscription_init(
      subscription_handle_.get(), node_handle_.get(), &type_support_handle, topic_name.c_str(),
      &subscription_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      auto rcl_node_handle = node_handle_.get();
      // this will throw on any validation problem
      rcl_reset_error();
      expand_topic_or_service_name(
          topic_name, rcl_node_get_name(rcl_node_handle), rcl_node_get_namespace(rcl_node_handle));
    }
    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create subscription");
  }

  // 绑定事件回调 (Bind event callbacks)
  bind_event_callbacks(event_callbacks_, use_default_callbacks);
}

/**
 * @brief 析构函数，销毁SubscriptionBase对象 (Destructor, destroys the SubscriptionBase object)
 */
SubscriptionBase::~SubscriptionBase() {
  // 如果不使用内部进程通信，则直接返回 (If not using intra-process communication, return directly)
  if (!use_intra_process_) {
    return;
  }
  // 尝试从弱引用中获取IntraProcessManager的共享指针 (Try to get a shared pointer of
  // IntraProcessManager from the weak reference)
  auto ipm = weak_ipm_.lock();
  // 如果无法获取IntraProcessManager，则发出警告并返回 (If unable to get IntraProcessManager, issue
  // a warning and return)
  if (!ipm) {
    // TODO(ivanpauno): should this raise an error?
    RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"), "Intra process manager died before than a subscription.");
    return;
  }
  // 从IntraProcessManager中移除订阅 (Remove the subscription from IntraProcessManager)
  ipm->remove_subscription(intra_process_subscription_id_);
}

/**
 * @brief 绑定事件回调函数 (Bind event callbacks)
 *
 * @param[in] event_callbacks 事件回调结构体 (Event callback structure)
 * @param[in] use_default_callbacks 是否使用默认回调函数 (Whether to use default callback functions)
 */
void SubscriptionBase::bind_event_callbacks(
    const SubscriptionEventCallbacks &event_callbacks, bool use_default_callbacks) {
  // 如果设置了deadline_callback，则添加相应的事件处理器 (If deadline_callback is set, add the
  // corresponding event handler)
  if (event_callbacks.deadline_callback) {
    this->add_event_handler(
        event_callbacks.deadline_callback, RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED);
  }
  // 如果设置了liveliness_callback，则添加相应的事件处理器 (If liveliness_callback is set, add the
  // corresponding event handler)
  if (event_callbacks.liveliness_callback) {
    this->add_event_handler(
        event_callbacks.liveliness_callback, RCL_SUBSCRIPTION_LIVELINESS_CHANGED);
  }
  // 如果设置了incompatible_qos_callback，则添加相应的事件处理器 (If incompatible_qos_callback is
  // set, add the corresponding event handler)
  if (event_callbacks.incompatible_qos_callback) {
    this->add_event_handler(
        event_callbacks.incompatible_qos_callback, RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS);
  } else if (use_default_callbacks) {
    // 当未指定回调函数时，注册默认回调函数 (Register default callback when not specified)
    try {
      this->add_event_handler(
          [this](QOSRequestedIncompatibleQoSInfo &info) {
            this->default_incompatible_qos_callback(info);
          },
          RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS);
    } catch (UnsupportedEventTypeException & /*exc*/) {
      // pass
    }
  }
  // 如果设置了message_lost_callback，则添加相应的事件处理器 (If message_lost_callback is set, add
  // the corresponding event handler)
  if (event_callbacks.message_lost_callback) {
    this->add_event_handler(event_callbacks.message_lost_callback, RCL_SUBSCRIPTION_MESSAGE_LOST);
  }
}

/**
 * @brief 获取主题名称 (Get topic name)
 *
 * @return 主题名称字符串 (Topic name string)
 */
const char *SubscriptionBase::get_topic_name() const {
  return rcl_subscription_get_topic_name(subscription_handle_.get());
}

// 获取订阅句柄 (Get the subscription handle)
std::shared_ptr<rcl_subscription_t> SubscriptionBase::get_subscription_handle() {
  // 返回订阅句柄 (Return the subscription handle)
  return subscription_handle_;
}

// 获取订阅句柄（常量方法）(Get the subscription handle (const method))
std::shared_ptr<const rcl_subscription_t> SubscriptionBase::get_subscription_handle() const {
  // 返回订阅句柄 (Return the subscription handle)
  return subscription_handle_;
}

// 获取事件处理器 (Get event handlers)
const std::
    unordered_map<rcl_subscription_event_type_t, std::shared_ptr<rclcpp::QOSEventHandlerBase>> &
    SubscriptionBase::get_event_handlers() const {
  // 返回事件处理器映射表 (Return the event handlers map)
  return event_handlers_;
}

// 获取实际的QoS设置 (Get the actual QoS settings)
rclcpp::QoS SubscriptionBase::get_actual_qos() const {
  // 从订阅句柄获取QoS配置 (Get QoS profile from the subscription handle)
  const rmw_qos_profile_t *qos = rcl_subscription_get_actual_qos(subscription_handle_.get());
  if (!qos) {
    auto msg = std::string("failed to get qos settings: ") + rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }

  // 返回QoS对象 (Return QoS object)
  return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*qos), *qos);
}

// 接收类型擦除消息 (Take type erased message)
bool SubscriptionBase::take_type_erased(void *message_out, rclcpp::MessageInfo &message_info_out) {
  // 接收消息 (Take the message)
  rcl_ret_t ret = rcl_take(
      this->get_subscription_handle().get(), message_out, &message_info_out.get_rmw_message_info(),
      nullptr  // rmw_subscription_allocation_t is unused here
  );
  TRACEPOINT(rclcpp_take, static_cast<const void *>(message_out));
  if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
    return false;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  // 检查是否匹配任何内部进程发布者 (Check if it matches any intra-process publishers)
  if (matches_any_intra_process_publishers(
          &message_info_out.get_rmw_message_info().publisher_gid)) {
    // 在这种情况下，消息将通过内部进程传递，
    // 我们应该忽略此副本的消息。 (In this case, the message will be delivered via intra-process and
    // we should ignore this copy of the message.)
    return false;
  }
  return true;
}

// 接收序列化消息 (Take serialized message)
bool SubscriptionBase::take_serialized(
    rclcpp::SerializedMessage &message_out, rclcpp::MessageInfo &message_info_out) {
  // 接收序列化消息 (Take the serialized message)
  rcl_ret_t ret = rcl_take_serialized_message(
      this->get_subscription_handle().get(), &message_out.get_rcl_serialized_message(),
      &message_info_out.get_rmw_message_info(), nullptr);
  if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
    return false;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  return true;
}

// 获取消息类型支持句柄 (Get message type support handle)
const rosidl_message_type_support_t &SubscriptionBase::get_message_type_support_handle() const {
  // 返回类型支持句柄 (Return the type support handle)
  return type_support_;
}

// 判断是否为序列化消息 (Check if it's a serialized message)
bool SubscriptionBase::is_serialized() const { return is_serialized_; }

// 获取发布者数量 (Get publisher count)
size_t SubscriptionBase::get_publisher_count() const {
  size_t inter_process_publisher_count = 0;

  // 从订阅句柄获取发布者数量 (Get publisher count from the subscription handle)
  rmw_ret_t status = rcl_subscription_get_publisher_count(
      subscription_handle_.get(), &inter_process_publisher_count);

  if (RCL_RET_OK != status) {
    rclcpp::exceptions::throw_from_rcl_error(status, "failed to get get publisher count");
  }
  return inter_process_publisher_count;
}

/**
 * @brief 设置订阅者的内部进程通信。
 * @param intra_process_subscription_id 内部进程订阅ID。
 * @param weak_ipm 内部进程管理器的弱引用。
 *
 * @brief Set up the intra-process communication for the subscriber.
 * @param intra_process_subscription_id The intra-process subscription ID.
 * @param weak_ipm A weak reference to the IntraProcessManager.
 */
void SubscriptionBase::setup_intra_process(
    uint64_t intra_process_subscription_id, IntraProcessManagerWeakPtr weak_ipm) {
  // 设置内部进程订阅ID
  // Set the intra-process subscription ID
  intra_process_subscription_id_ = intra_process_subscription_id;

  // 设置内部进程管理器的弱引用
  // Set the weak reference to the IntraProcessManager
  weak_ipm_ = weak_ipm;

  // 启用内部进程通信
  // Enable intra-process communication
  use_intra_process_ = true;
}

/**
 * @brief 检查是否可以借用消息。
 * @return 如果可以借用消息，则返回true，否则返回false。
 *
 * @brief Check if messages can be loaned.
 * @return Returns true if messages can be loaned, false otherwise.
 */
bool SubscriptionBase::can_loan_messages() const {
  return rcl_subscription_can_loan_messages(subscription_handle_.get());
}

/**
 * @brief 获取内部进程通信的可等待对象。
 * @return 返回一个共享指针，指向内部进程通信的可等待对象。
 *
 * @brief Get the waitable object for intra-process communication.
 * @return Returns a shared pointer to the waitable object for intra-process communication.
 */
rclcpp::Waitable::SharedPtr SubscriptionBase::get_intra_process_waitable() const {
  // 如果不使用内部进程通信，返回空指针
  // Return nullptr if not using intra-process communication
  if (!use_intra_process_) {
    return nullptr;
  }

  // 获取内部进程管理器
  // Get the intra-process manager
  auto ipm = weak_ipm_.lock();
  if (!ipm) {
    throw std::runtime_error(
        "SubscriptionBase::get_intra_process_waitable() called "
        "after destruction of intra process manager");
  }

  // 使用ID从内部进程管理器中检索订阅的内部进程
  // Retrieve the subscription intra-process from the intra-process manager using the ID
  return ipm->get_subscription_intra_process(intra_process_subscription_id_);
}

/**
 * @brief 默认的不兼容QoS回调函数。
 * @param event 请求不兼容QoS的事件信息。
 *
 * @brief Default incompatible QoS callback function.
 * @param event The event information of requested incompatible QoS.
 */
void SubscriptionBase::default_incompatible_qos_callback(
    rclcpp::QOSRequestedIncompatibleQoSInfo &event) const {
  std::string policy_name = qos_policy_name_from_kind(event.last_policy_kind);
  RCLCPP_WARN(
      rclcpp::get_logger(rcl_node_get_logger_name(node_handle_.get())),
      "New publisher discovered on topic '%s', offering incompatible QoS. "
      "No messages will be sent to it. "
      "Last incompatible policy: %s",
      get_topic_name(), policy_name.c_str());
}

/**
 * @brief 检查发送者是否与任何内部进程发布者匹配。
 * @param sender_gid 发送者的全局唯一ID。
 * @return 如果与任何内部进程发布者匹配，则返回true，否则返回false。
 *
 * @brief Check if the sender matches any intra-process publishers.
 * @param sender_gid The global unique ID of the sender.
 * @return Returns true if matches any intra-process publishers, false otherwise.
 */
bool SubscriptionBase::matches_any_intra_process_publishers(const rmw_gid_t *sender_gid) const {
  if (!use_intra_process_) {
    return false;
  }
  auto ipm = weak_ipm_.lock();
  if (!ipm) {
    throw std::runtime_error(
        "intra process publisher check called "
        "after destruction of intra process manager");
  }
  return ipm->matches_any_publishers(sender_gid);
}

/**
 * @brief 交换订阅的等待集状态 (Exchange the in-use state of a subscription part)
 *
 * @param[in] pointer_to_subscription_part 指向订阅部分的指针 (Pointer to the subscription part)
 * @param[in] in_use_state 新的使用状态 (The new in-use state)
 * @return bool 返回是否成功交换状态 (Returns whether the state is successfully exchanged)
 */
bool SubscriptionBase::exchange_in_use_by_wait_set_state(
    void *pointer_to_subscription_part, bool in_use_state) {
  // 检查传入的指针是否为 nullptr (Check if the passed pointer is nullptr)
  if (nullptr == pointer_to_subscription_part) {
    throw std::invalid_argument("pointer_to_subscription_part is unexpectedly nullptr");
  }
  // 如果指针与当前对象相同，则交换订阅的等待集状态 (If the pointer is the same as this object,
  // exchange the wait set state of the subscription)
  if (this == pointer_to_subscription_part) {
    return subscription_in_use_by_wait_set_.exchange(in_use_state);
  }
  // 如果指针与内部进程可等待对象相同，则交换内部进程订阅的等待集状态 (If the pointer is the same as
  // the intra-process waitable object, exchange the wait set state of the intra-process
  // subscription)
  if (get_intra_process_waitable().get() == pointer_to_subscription_part) {
    return intra_process_subscription_waitable_in_use_by_wait_set_.exchange(in_use_state);
  }
  // 遍历事件处理器，寻找匹配的指针并交换对应的等待集状态 (Iterate through event handlers, find the
  // matching pointer and exchange the corresponding wait set state)
  for (const auto &key_event_pair : event_handlers_) {
    auto qos_event = key_event_pair.second;
    if (qos_event.get() == pointer_to_subscription_part) {
      return qos_events_in_use_by_wait_set_[qos_event.get()].exchange(in_use_state);
    }
  }
  // 如果没有找到匹配的指针，抛出运行时错误 (If no matching pointer is found, throw a runtime error)
  throw std::runtime_error("given pointer_to_subscription_part does not match any part");
}

/**
 * @brief 获取网络流端点 (Get network flow endpoints)
 *
 * @return std::vector<rclcpp::NetworkFlowEndpoint> 返回网络流端点数组 (Returns a vector of network
 * flow endpoints)
 */
std::vector<rclcpp::NetworkFlowEndpoint> SubscriptionBase::get_network_flow_endpoints() const {
  // 获取默认分配器 (Get the default allocator)
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  // 初始化网络流端点数组 (Initialize the network flow endpoint array)
  rcl_network_flow_endpoint_array_t network_flow_endpoint_array =
      rcl_get_zero_initialized_network_flow_endpoint_array();
  // 从订阅句柄获取网络流端点 (Get network flow endpoints from the subscription handle)
  rcl_ret_t ret = rcl_subscription_get_network_flow_endpoints(
      subscription_handle_.get(), &allocator, &network_flow_endpoint_array);
  // 检查返回值并处理错误 (Check return value and handle errors)
  if (RCL_RET_OK != ret) {
    auto error_msg =
        std::string("Error obtaining network flows of subscription: ") + rcl_get_error_string().str;
    rcl_reset_error();
    if (RCL_RET_OK != rcl_network_flow_endpoint_array_fini(&network_flow_endpoint_array)) {
      error_msg +=
          std::string(". Also error cleaning up network flow array: ") + rcl_get_error_string().str;
      rcl_reset_error();
    }
    rclcpp::exceptions::throw_from_rcl_error(ret, error_msg);
  }

  // 将网络流端点数组转换为向量 (Convert the network flow endpoint array to a vector)
  std::vector<rclcpp::NetworkFlowEndpoint> network_flow_endpoint_vector;
  for (size_t i = 0; i < network_flow_endpoint_array.size; ++i) {
    network_flow_endpoint_vector.push_back(
        rclcpp::NetworkFlowEndpoint(network_flow_endpoint_array.network_flow_endpoint[i]));
  }

  // 清理网络流端点数组 (Clean up the network flow endpoint array)
  ret = rcl_network_flow_endpoint_array_fini(&network_flow_endpoint_array);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "error cleaning up network flow array");
  }

  // 返回网络流端点向量 (Return the network flow endpoint vector)
  return network_flow_endpoint_vector;
}

/**
 * @brief 设置新消息回调 (Set the callback for new messages)
 *
 * @param[in] callback 新消息的回调函数 (The callback function for new messages)
 * @param[in] user_data 用户数据指针 (Pointer to user data)
 */
void SubscriptionBase::set_on_new_message_callback(
    rcl_event_callback_t callback, const void *user_data) {
  // 设置订阅的新消息回调 (Set the new message callback for the subscription)
  rcl_ret_t ret =
      rcl_subscription_set_on_new_message_callback(subscription_handle_.get(), callback, user_data);

  // 检查返回值并处理错误 (Check return value and handle errors)
  if (RCL_RET_OK != ret) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(ret, "failed to set the on new message callback for subscription");
  }
}

/**
 * @brief 检查订阅是否启用了内容过滤主题 (Check if content filtered topic is enabled for the
 * subscription)
 *
 * @return bool 返回是否启用了内容过滤主题 (Returns whether content filtered topic is enabled)
 */
bool SubscriptionBase::is_cft_enabled() const {
  return rcl_subscription_is_cft_enabled(subscription_handle_.get());
}

/**
 * @brief 设置订阅器的内容过滤器
 * @param filter_expression 过滤表达式字符串
 * @param expression_parameters 表达式参数字符串列表
 *
 * Set the content filter for the subscription.
 * @param filter_expression A string representing the filter expression.
 * @param expression_parameters A vector of strings representing the expression parameters.
 */
void SubscriptionBase::set_content_filter(
    const std::string &filter_expression, const std::vector<std::string> &expression_parameters) {
  // 初始化内容过滤器选项结构体
  // Initialize the content filter options structure.
  rcl_subscription_content_filter_options_t options =
      rcl_get_zero_initialized_subscription_content_filter_options();

  // 将表达式参数转换为 C 类型字符串向量
  // Convert the expression parameters to a C-style string vector.
  std::vector<const char *> cstrings = get_c_vector_string(expression_parameters);

  // 初始化订阅器的内容过滤器选项
  // Initialize the content filter options for the subscription.
  rcl_ret_t ret = rcl_subscription_content_filter_options_init(
      subscription_handle_.get(), get_c_string(filter_expression), cstrings.size(), cstrings.data(),
      &options);

  // 检查初始化是否成功，如果失败则抛出异常
  // Check if the initialization was successful; throw an exception if it failed.
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(
        ret, "failed to init subscription content_filtered_topic option");
  }

  // 在作用域结束时清理内容过滤器选项
  // Clean up the content filter options when the scope ends.
  RCPPUTILS_SCOPE_EXIT({
    rcl_ret_t ret =
        rcl_subscription_content_filter_options_fini(subscription_handle_.get(), &options);
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"),
          "Failed to fini subscription content_filtered_topic option: %s",
          rcl_get_error_string().str);
      rcl_reset_error();
    }
  });

  // 设置订阅器的内容过滤器
  // Set the content filter for the subscription.
  ret = rcl_subscription_set_content_filter(subscription_handle_.get(), &options);

  // 检查设置是否成功，如果失败则抛出异常
  // Check if the setting was successful; throw an exception if it failed.
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to set cft expression parameters");
  }
}

/**
 * @brief 获取订阅器的内容过滤器选项
 * @return 返回一个 ContentFilterOptions 对象，包含过滤表达式和参数
 *
 * Get the content filter options for the subscription.
 * @return A ContentFilterOptions object containing the filter expression and parameters.
 */
rclcpp::ContentFilterOptions SubscriptionBase::get_content_filter() const {
  // 初始化返回的内容过滤器选项对象
  // Initialize the returned content filter options object.
  rclcpp::ContentFilterOptions ret_options;

  // 初始化内容过滤器选项结构体
  // Initialize the content filter options structure.
  rcl_subscription_content_filter_options_t options =
      rcl_get_zero_initialized_subscription_content_filter_options();

  // 获取订阅器的内容过滤器选项
  // Get the content filter options for the subscription.
  rcl_ret_t ret = rcl_subscription_get_content_filter(subscription_handle_.get(), &options);

  // 检查获取是否成功，如果失败则抛出异常
  // Check if the retrieval was successful; throw an exception if it failed.
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to get cft expression parameters");
  }

  // 在作用域结束时清理内容过滤器选项
  // Clean up the content filter options when the scope ends.
  RCPPUTILS_SCOPE_EXIT({
    rcl_ret_t ret =
        rcl_subscription_content_filter_options_fini(subscription_handle_.get(), &options);
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"),
          "Failed to fini subscription content_filtered_topic option: %s",
          rcl_get_error_string().str);
      rcl_reset_error();
    }
  });

  // 将底层的内容过滤器选项转换为 ContentFilterOptions 对象
  // Convert the underlying content filter options to a ContentFilterOptions object.
  rmw_subscription_content_filter_options_t &content_filter_options =
      options.rmw_subscription_content_filter_options;
  ret_options.filter_expression = content_filter_options.filter_expression;

  // 将表达式参数从 C 类型字符串向量转换为 std::vector<std::string>
  // Convert the expression parameters from a C-style string vector to a std::vector<std::string>.
  for (size_t i = 0; i < content_filter_options.expression_parameters.size; ++i) {
    ret_options.expression_parameters.push_back(
        content_filter_options.expression_parameters.data[i]);
  }

  // 返回内容过滤器选项对象
  // Return the content filter options object.
  return ret_options;
}

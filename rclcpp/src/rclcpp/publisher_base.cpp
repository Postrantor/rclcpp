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

#include "rclcpp/publisher_base.hpp"

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/network_flow_endpoint.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos_event.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/impl/cpp/demangle.hpp"

using rclcpp::PublisherBase;

/**
 * @brief 构造函数，用于设置 PublisherBase 对象的属性 (Constructor for setting the properties of a
 * PublisherBase object)
 *
 * @param node_base 指向 rclcpp::node_interfaces::NodeBaseInterface 类型的指针 (Pointer to an
 * rclcpp::node_interfaces::NodeBaseInterface type)
 * @param topic 要发布的话题名称 (Name of the topic to publish)
 * @param type_support 话题类型支持结构体 (Topic type support structure)
 * @param publisher_options 发布者选项 (Publisher options)
 * @param event_callbacks 事件回调结构体 (Event callback structure)
 * @param use_default_callbacks 是否使用默认回调 (Whether to use default callbacks)
 */
PublisherBase::PublisherBase(
    rclcpp::node_interfaces::NodeBaseInterface* node_base,
    const std::string& topic,
    const rosidl_message_type_support_t& type_support,
    const rcl_publisher_options_t& publisher_options,
    const PublisherEventCallbacks& event_callbacks,
    bool use_default_callbacks)
    : rcl_node_handle_(node_base->get_shared_rcl_node_handle()),
      intra_process_is_enabled_(false),
      intra_process_publisher_id_(0),
      type_support_(type_support),
      event_callbacks_(event_callbacks) {
  // 自定义删除器，用于在共享指针销毁时清理 rcl_publisher_t 资源
  auto custom_deleter = [node_handle = this->rcl_node_handle_](rcl_publisher_t* rcl_pub) {
    if (rcl_publisher_fini(rcl_pub, node_handle.get()) != RCL_RET_OK) {
      RCLCPP_ERROR(
          rclcpp::get_node_logger(node_handle.get()).get_child("rclcpp"),
          "Error in destruction of rcl publisher handle: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
    delete rcl_pub;
  };

  // 初始化 publisher_handle_ 为一个共享指针，并设置自定义删除器
  publisher_handle_ = std::shared_ptr<rcl_publisher_t>(new rcl_publisher_t, custom_deleter);
  *publisher_handle_.get() = rcl_get_zero_initialized_publisher();

  // 初始化 rcl publisher (Initialize the rcl publisher)
  rcl_ret_t ret = rcl_publisher_init(
      publisher_handle_.get(), rcl_node_handle_.get(), &type_support, topic.c_str(),
      &publisher_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      auto rcl_node_handle = rcl_node_handle_.get();
      // this will throw on any validation problem
      rcl_reset_error();
      expand_topic_or_service_name(
          topic, rcl_node_get_name(rcl_node_handle), rcl_node_get_namespace(rcl_node_handle));
    }

    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create publisher");
  }
  // 将对象的生命周期绑定到发布者句柄 (Bind the lifetime of this object to the publisher handle)
  rmw_publisher_t* publisher_rmw_handle = rcl_publisher_get_rmw_handle(publisher_handle_.get());
  if (!publisher_rmw_handle) {
    auto msg = std::string("failed to get rmw handle: ") + rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }
  if (rmw_get_gid_for_publisher(publisher_rmw_handle, &rmw_gid_) != RMW_RET_OK) {
    auto msg = std::string("failed to get publisher gid: ") + rmw_get_error_string().str;
    rmw_reset_error();
    throw std::runtime_error(msg);
  }

  // 绑定事件回调函数 (Bind event callbacks)
  bind_event_callbacks(event_callbacks_, use_default_callbacks);
}

/// Destructor for PublisherBase.
PublisherBase::~PublisherBase() {
  // 清除事件处理器列表 (Clear the list of event handlers)
  event_handlers_.clear();

  auto ipm = weak_ipm_.lock();

  // 如果内部进程通信未启用，则返回
  if (!intra_process_is_enabled_) {
    return;
  }
  // 如果无法获取 IntraProcessManager 的弱引用，则发出警告并返回
  if (!ipm) {
    // TODO(ivanpauno): should this raise an error?
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Intra process manager died before a publisher.");
    return;
  }
  // 从 IntraProcessManager 中移除发布者
  ipm->remove_publisher(intra_process_publisher_id_);
}

/// 获取主题名称 (Get the topic name)
const char* PublisherBase::get_topic_name() const {
  return rcl_publisher_get_topic_name(publisher_handle_.get());
}

/// 绑定事件回调函数 (Bind event callbacks)
void PublisherBase::bind_event_callbacks(
    const PublisherEventCallbacks& event_callbacks, bool use_default_callbacks) {
  // 如果存在 deadline_callback，将其添加到事件处理器中 (If deadline_callback exists, add it to the
  // event handler)
  if (event_callbacks.deadline_callback) {
    this->add_event_handler(
        event_callbacks.deadline_callback, RCL_PUBLISHER_OFFERED_DEADLINE_MISSED);
  }
  // 如果存在 liveliness_callback，将其添加到事件处理器中 (If liveliness_callback exists, add it to
  // the event handler)
  if (event_callbacks.liveliness_callback) {
    this->add_event_handler(event_callbacks.liveliness_callback, RCL_PUBLISHER_LIVELINESS_LOST);
  }
  // 如果存在 incompatible_qos_callback，将其添加到事件处理器中 (If incompatible_qos_callback
  // exists, add it to the event handler)
  if (event_callbacks.incompatible_qos_callback) {
    this->add_event_handler(
        event_callbacks.incompatible_qos_callback, RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS);
  } else if (use_default_callbacks) {
    // 当未指定回调时，注册默认回调 (Register default callback when not specified)
    try {
      this->add_event_handler(
          [this](QOSOfferedIncompatibleQoSInfo& info) {
            this->default_incompatible_qos_callback(info);
          },
          RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS);
    } catch (UnsupportedEventTypeException& /*exc*/) {
      // pass
    }
  }
}

/**
 * @brief 获取发布者队列的大小 (Get the queue size of the publisher)
 *
 * @return size_t 队列的大小 (The size of the queue)
 */
size_t PublisherBase::get_queue_size() const {
  // 获取发布者选项指针 (Get the pointer to publisher options)
  const rcl_publisher_options_t* publisher_options =
      rcl_publisher_get_options(publisher_handle_.get());
  // 如果获取失败，抛出运行时错误 (If failed, throw a runtime error)
  if (!publisher_options) {
    auto msg = std::string("failed to get publisher options: ") + rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }
  // 返回队列深度 (Return the queue depth)
  return publisher_options->qos.depth;
}

/**
 * @brief 获取发布者的 GID (Get the GID of the publisher)
 *
 * @return const rmw_gid_t& 发布者的 GID (The GID of the publisher)
 */
const rmw_gid_t& PublisherBase::get_gid() const { return rmw_gid_; }

/**
 * @brief 获取发布者句柄 (Get the publisher handle)
 *
 * @return std::shared_ptr<rcl_publisher_t> 发布者句柄 (Publisher handle)
 */
std::shared_ptr<rcl_publisher_t> PublisherBase::get_publisher_handle() { return publisher_handle_; }

/**
 * @brief 获取发布者句柄（常量版本）(Get the publisher handle (const version))
 *
 * @return std::shared_ptr<const rcl_publisher_t> 发布者句柄 (Publisher handle)
 */
std::shared_ptr<const rcl_publisher_t> PublisherBase::get_publisher_handle() const {
  return publisher_handle_;
}

/**
 * @brief 获取事件处理器 (Get the event handlers)
 *
 * @return const std::unordered_map<rcl_publisher_event_type_t,
 * std::shared_ptr<rclcpp::QOSEventHandlerBase>>& 事件处理器映射 (Event handlers map)
 */
const std::unordered_map<rcl_publisher_event_type_t, std::shared_ptr<rclcpp::QOSEventHandlerBase>>&
PublisherBase::get_event_handlers() const {
  return event_handlers_;
}

/**
 * @brief 获取订阅数量 (Get the subscription count)
 *
 * @return size_t 订阅数量 (Subscription count)
 */
size_t PublisherBase::get_subscription_count() const {
  size_t inter_process_subscription_count = 0;

  // 获取发布者的订阅数量 (Get the subscription count of the publisher)
  rcl_ret_t status = rcl_publisher_get_subscription_count(
      publisher_handle_.get(), &inter_process_subscription_count);

  // 如果发布者无效，检查上下文是否有效 (If the publisher is invalid, check if the context is valid)
  if (RCL_RET_PUBLISHER_INVALID == status) {
    rcl_reset_error(); /* next call will reset error message if not context */
    if (rcl_publisher_is_valid_except_context(publisher_handle_.get())) {
      rcl_context_t* context = rcl_publisher_get_context(publisher_handle_.get());
      if (nullptr != context && !rcl_context_is_valid(context)) {
        /* publisher is invalid due to context being shutdown */
        return 0;
      }
    }
  }
  // 如果状态不是 RCL_RET_OK，抛出异常 (If the status is not RCL_RET_OK, throw an exception)
  if (RCL_RET_OK != status) {
    rclcpp::exceptions::throw_from_rcl_error(status, "failed to get get subscription count");
  }
  // 返回订阅数量 (Return the subscription count)
  return inter_process_subscription_count;
}

/**
 * @brief 获取内部进程订阅的数量 (Get the count of intra-process subscriptions)
 *
 * @return size_t 内部进程订阅的数量 (The count of intra-process subscriptions)
 */
size_t PublisherBase::get_intra_process_subscription_count() const {
  // 尝试获取 weak_ipm_ 的共享指针 (Attempt to obtain a shared pointer from weak_ipm_)
  auto ipm = weak_ipm_.lock();

  // 如果内部进程未启用，则返回0 (If intra-process is not enabled, return 0)
  if (!intra_process_is_enabled_) {
    return 0;
  }

  // 如果无法获取到共享指针 (If unable to obtain a shared pointer)
  if (!ipm) {
    // TODO(ivanpauno): 是否应该静默返回？或者返回一个警告？
    //                  与 wjwwood 在 publisher_factory create_shared_publish_callback
    //                  中的评论相同。
    throw std::runtime_error(
        "intra process subscriber count called after "
        "destruction of intra process manager");
  }

  // 返回内部进程订阅的数量 (Return the count of intra-process subscriptions)
  return ipm->get_subscription_count(intra_process_publisher_id_);
}

/**
 * @brief 获取实际的 QoS (Quality of Service) 配置 (Get the actual QoS configuration)
 *
 * @return rclcpp::QoS 实际的 QoS 配置 (The actual QoS configuration)
 */
rclcpp::QoS PublisherBase::get_actual_qos() const {
  // 获取实际的 QoS 配置 (Get the actual QoS configuration)
  const rmw_qos_profile_t* qos = rcl_publisher_get_actual_qos(publisher_handle_.get());

  // 如果获取失败，抛出异常 (If failed to get, throw an exception)
  if (!qos) {
    auto msg = std::string("failed to get qos settings: ") + rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }

  // 返回实际的 QoS 配置 (Return the actual QoS configuration)
  return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*qos), *qos);
}

/**
 * @brief 断言生命周期 (Assert liveliness)
 *
 * @return bool 操作是否成功 (Whether the operation is successful)
 */
bool PublisherBase::assert_liveliness() const {
  // 断言生命周期并返回操作结果 (Assert liveliness and return the operation result)
  return RCL_RET_OK == rcl_publisher_assert_liveliness(publisher_handle_.get());
}

/**
 * @brief 判断是否可以借用消息 (Determine whether messages can be loaned)
 *
 * @return bool 是否可以借用消息 (Whether messages can be loaned)
 */
bool PublisherBase::can_loan_messages() const {
  // 判断是否可以借用消息并返回结果 (Determine whether messages can be loaned and return the result)
  return rcl_publisher_can_loan_messages(publisher_handle_.get());
}

/**
 * @brief 比较两个 rmw_gid_t 是否相等 (Compare whether two rmw_gid_t are equal)
 *
 * @param gid 要比较的 rmw_gid_t (The rmw_gid_t to compare)
 * @return bool 是否相等 (Whether they are equal)
 */
bool PublisherBase::operator==(const rmw_gid_t& gid) const { return *this == &gid; }

/**
 * @brief 比较两个 rmw_gid_t 是否相等 (Compare whether two rmw_gid_t are equal)
 *
 * @param gid 要比较的 rmw_gid_t 指针 (The pointer to the rmw_gid_t to compare)
 * @return bool 是否相等 (Whether they are equal)
 */
bool PublisherBase::operator==(const rmw_gid_t* gid) const {
  // 初始化结果变量 (Initialize result variable)
  bool result = false;

  // 比较两个 rmw_gid_t 是否相等 (Compare whether two rmw_gid_t are equal)
  auto ret = rmw_compare_gids_equal(gid, &this->get_gid(), &result);

  // 如果比较失败，抛出异常 (If the comparison fails, throw an exception)
  if (ret != RMW_RET_OK) {
    auto msg = std::string("failed to compare gids: ") + rmw_get_error_string().str;
    rmw_reset_error();
    throw std::runtime_error(msg);
  }

  // 返回比较结果 (Return the comparison result)
  return result;
}

/*!
 * \brief 设置内部进程通信的参数（Set up the intra-process communication parameters）
 *
 * \param[in] intra_process_publisher_id 内部进程发布者ID（Intra-process publisher ID）
 * \param[in] ipm 共享指针，指向IntraProcessManager对象（Shared pointer to an IntraProcessManager
 * object）
 */
void PublisherBase::setup_intra_process(
    uint64_t intra_process_publisher_id, IntraProcessManagerSharedPtr ipm) {
  // 设置内部进程发布者ID（Set the intra-process publisher ID）
  intra_process_publisher_id_ = intra_process_publisher_id;

  // 设置弱指针，指向IntraProcessManager对象（Set the weak pointer to the IntraProcessManager
  // object）
  weak_ipm_ = ipm;

  // 启用内部进程通信（Enable intra-process communication）
  intra_process_is_enabled_ = true;
}

/*!
 * \brief 默认不兼容QoS回调函数（Default callback for incompatible QoS events）
 *
 * \param[out] event 不兼容QoS事件信息（Incompatible QoS event information）
 */
void PublisherBase::default_incompatible_qos_callback(
    rclcpp::QOSOfferedIncompatibleQoSInfo& event) const {
  // 获取不兼容策略的名称（Get the name of the incompatible policy）
  std::string policy_name = qos_policy_name_from_kind(event.last_policy_kind);

  // 打印警告信息（Print a warning message）
  RCLCPP_WARN(
      rclcpp::get_logger(rcl_node_get_logger_name(rcl_node_handle_.get())),
      "New subscription discovered on topic '%s', requesting incompatible QoS. "
      "No messages will be sent to it. "
      "Last incompatible policy: %s",
      get_topic_name(), policy_name.c_str());
}

/*!
 * \brief 获取网络流终端列表（Get a list of network flow endpoints）
 *
 * \return std::vector<rclcpp::NetworkFlowEndpoint> 网络流终端列表（List of NetworkFlowEndpoint
 * objects）
 */
std::vector<rclcpp::NetworkFlowEndpoint> PublisherBase::get_network_flow_endpoints() const {
  // 获取默认分配器（Get the default allocator）
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  // 初始化网络流终端数组（Initialize the network flow endpoint array）
  rcl_network_flow_endpoint_array_t network_flow_endpoint_array =
      rcl_get_zero_initialized_network_flow_endpoint_array();

  // 获取发布者的网络流终端（Get the network flow endpoints for the publisher）
  rcl_ret_t ret = rcl_publisher_get_network_flow_endpoints(
      publisher_handle_.get(), &allocator, &network_flow_endpoint_array);

  // 检查返回值（Check the return value）
  if (RCL_RET_OK != ret) {
    auto error_msg =
        std::string("error obtaining network flows of publisher: ") + rcl_get_error_string().str;
    rcl_reset_error();
    if (RCL_RET_OK != rcl_network_flow_endpoint_array_fini(&network_flow_endpoint_array)) {
      error_msg +=
          std::string(", also error cleaning up network flow array: ") + rcl_get_error_string().str;
      rcl_reset_error();
    }
    rclcpp::exceptions::throw_from_rcl_error(ret, error_msg);
  }

  // 将网络流终端数组转换为向量（Convert the network flow endpoint array to a vector）
  std::vector<rclcpp::NetworkFlowEndpoint> network_flow_endpoint_vector;
  for (size_t i = 0; i < network_flow_endpoint_array.size; ++i) {
    network_flow_endpoint_vector.push_back(
        rclcpp::NetworkFlowEndpoint(network_flow_endpoint_array.network_flow_endpoint[i]));
  }

  // 清理网络流终端数组（Clean up the network flow endpoint array）
  ret = rcl_network_flow_endpoint_array_fini(&network_flow_endpoint_array);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "error cleaning up network flow array");
  }

  // 返回网络流终端向量（Return the network flow endpoint vector）
  return network_flow_endpoint_vector;
}

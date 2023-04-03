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

#include "rclcpp/service.hpp"

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"

using rclcpp::ServiceBase;

/**
 * @brief 构造函数，初始化 ServiceBase 对象 (Constructor, initializes the ServiceBase object)
 *
 * @param node_handle 共享指针，指向 rcl_node_t 类型的节点句柄 (Shared pointer to an rcl_node_t type
 * node handle)
 */
ServiceBase::ServiceBase(std::shared_ptr<rcl_node_t> node_handle)
    : node_handle_(node_handle), node_logger_(rclcpp::get_node_logger(node_handle_.get())) {}

/**
 * @brief 接收类型擦除的请求 (Take a type-erased request)
 *
 * @param[out] request_out 请求输出 (Request output)
 * @param[out] request_id_out 请求 ID 输出 (Request ID output)
 * @return bool 成功接收返回 true，否则返回 false (Returns true if successfully taken, otherwise
 * returns false)
 */
bool ServiceBase::take_type_erased_request(void* request_out, rmw_request_id_t& request_id_out) {
  rcl_ret_t ret = rcl_take_request(this->get_service_handle().get(), &request_id_out, request_out);
  if (RCL_RET_SERVICE_TAKE_FAILED == ret) {
    return false;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  return true;
}

/**
 * @brief 获取服务名称 (Get the service name)
 *
 * @return const char* 服务名称 (Service name)
 */
const char* ServiceBase::get_service_name() {
  return rcl_service_get_service_name(this->get_service_handle().get());
}

/**
 * @brief 获取服务句柄 (Get the service handle)
 *
 * @return std::shared_ptr<rcl_service_t> 服务句柄 (Service handle)
 */
std::shared_ptr<rcl_service_t> ServiceBase::get_service_handle() { return service_handle_; }

/**
 * @brief 获取服务句柄，常量版本 (Get the service handle, const version)
 *
 * @return std::shared_ptr<const rcl_service_t> 服务句柄 (Service handle)
 */
std::shared_ptr<const rcl_service_t> ServiceBase::get_service_handle() const {
  return service_handle_;
}

/**
 * @brief 获取 rcl_node_t 类型的节点句柄 (Get the rcl_node_t type node handle)
 *
 * @return rcl_node_t* 节点句柄 (Node handle)
 */
rcl_node_t* ServiceBase::get_rcl_node_handle() { return node_handle_.get(); }

/**
 * @brief 获取 rcl_node_t 类型的节点句柄，常量版本 (Get the rcl_node_t type node handle, const
 * version)
 *
 * @return const rcl_node_t* 节点句柄 (Node handle)
 */
const rcl_node_t* ServiceBase::get_rcl_node_handle() const { return node_handle_.get(); }

/**
 * @brief 交换等待集合中的使用状态 (Exchange the usage state in the wait set)
 *
 * @param[in] in_use_state 使用状态 (Usage state)
 * @return bool 原始状态值 (Original state value)
 */
bool ServiceBase::exchange_in_use_by_wait_set_state(bool in_use_state) {
  return in_use_by_wait_set_.exchange(in_use_state);
}

/**
 * @brief 获取实际响应发布器的 QoS 设置 (Get the actual QoS settings of the response publisher)
 *
 * @return rclcpp::QoS 响应发布器的 QoS 设置 (Response publisher's QoS settings)
 */
rclcpp::QoS ServiceBase::get_response_publisher_actual_qos() const {
  const rmw_qos_profile_t* qos =
      rcl_service_response_publisher_get_actual_qos(service_handle_.get());
  if (!qos) {
    auto msg = std::string("failed to get service's response publisher qos settings: ") +
               rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }

  rclcpp::QoS response_publisher_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*qos), *qos);

  return response_publisher_qos;
}

/**
 * @brief 获取实际请求订阅器的 QoS 设置 (Get the actual QoS settings of the request subscriber)
 *
 * @return rclcpp::QoS 请求订阅器的 QoS 设置 (Request subscriber's QoS settings)
 */
rclcpp::QoS ServiceBase::get_request_subscription_actual_qos() const {
  const rmw_qos_profile_t* qos =
      rcl_service_request_subscription_get_actual_qos(service_handle_.get());
  if (!qos) {
    auto msg = std::string("failed to get service's request subscription qos settings: ") +
               rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }

  rclcpp::QoS request_subscription_qos =
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*qos), *qos);

  return request_subscription_qos;
}

/**
 * @brief 设置新请求回调 (Set the callback for new requests)
 *
 * @param[in] callback 新请求回调函数 (New request callback function)
 * @param[in] user_data 用户数据 (User data)
 */
void ServiceBase::set_on_new_request_callback(
    rcl_event_callback_t callback, const void* user_data) {
  rcl_ret_t ret =
      rcl_service_set_on_new_request_callback(service_handle_.get(), callback, user_data);

  if (RCL_RET_OK != ret) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(ret, "failed to set the on new request callback for service");
  }
}

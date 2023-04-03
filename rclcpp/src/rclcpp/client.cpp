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

#include "rclcpp/client.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rcl/graph.h"
#include "rcl/node.h"
#include "rcl/wait.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/utilities.hpp"

using rclcpp::ClientBase;
using rclcpp::exceptions::InvalidNodeError;
using rclcpp::exceptions::throw_from_rcl_error;

/**
 * @brief 构造函数，初始化 ClientBase 类的对象 (Constructor, initializes an object of the ClientBase
 * class)
 *
 * @param node_base 节点基础接口指针 (Pointer to the NodeBaseInterface)
 * @param node_graph 节点图形接口共享指针 (Shared pointer to the NodeGraphInterface)
 */
ClientBase::ClientBase(
    rclcpp::node_interfaces::NodeBaseInterface* node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph)
    : node_graph_(node_graph),
      node_handle_(node_base->get_shared_rcl_node_handle()),
      context_(node_base->get_context()),
      node_logger_(rclcpp::get_node_logger(node_handle_.get())) {
  // 创建一个弱指针来引用节点句柄 (Create a weak pointer to reference the node handle)
  std::weak_ptr<rcl_node_t> weak_node_handle(node_handle_);

  // 分配并初始化一个新的 rcl_client_t 结构体 (Allocate and initialize a new rcl_client_t structure)
  rcl_client_t* new_rcl_client = new rcl_client_t;
  *new_rcl_client = rcl_get_zero_initialized_client();

  // 设置 client_handle_ 的自定义删除器 (Set the custom deleter for client_handle_)
  client_handle_.reset(new_rcl_client, [weak_node_handle](rcl_client_t* client) {
    auto handle = weak_node_handle.lock();
    if (handle) {
      if (rcl_client_fini(client, handle.get()) != RCL_RET_OK) {
        RCLCPP_ERROR(
            rclcpp::get_node_logger(handle.get()).get_child("rclcpp"),
            "Error in destruction of rcl client handle: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
    } else {
      RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"),
          "Error in destruction of rcl client handle: "
          "the Node Handle was destructed too early. You will leak memory");
    }
    delete client;
  });
}

/**
 * @brief 获取类型擦除的响应 (Take type-erased response)
 *
 * @param response_out 输出的响应指针 (Output response pointer)
 * @param request_header_out 输出的请求头 (Output request header)
 * @return 如果成功获取响应，则返回 true，否则返回 false (Returns true if the response is
 * successfully taken, otherwise returns false)
 */
bool ClientBase::take_type_erased_response(
    void* response_out, rmw_request_id_t& request_header_out) {
  rcl_ret_t ret =
      rcl_take_response(this->get_client_handle().get(), &request_header_out, response_out);
  if (RCL_RET_CLIENT_TAKE_FAILED == ret) {
    return false;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  return true;
}

/**
 * @brief 获取服务名称 (Get service name)
 *
 * @return 服务名称 (Service name)
 */
const char* ClientBase::get_service_name() const {
  return rcl_client_get_service_name(this->get_client_handle().get());
}

/**
 * @brief 获取客户端句柄 (Get client handle)
 *
 * @return 客户端句柄共享指针 (Shared pointer to the client handle)
 */
std::shared_ptr<rcl_client_t> ClientBase::get_client_handle() { return client_handle_; }

/**
 * @brief 获取客户端句柄 (Get client handle)
 *
 * @return 客户端句柄共享指针 (Shared pointer to the const client handle)
 */
std::shared_ptr<const rcl_client_t> ClientBase::get_client_handle() const { return client_handle_; }

/**
 * @brief 检查服务是否准备好 (Check if the service is ready)
 *
 * @return 如果服务准备好，则返回 true，否则返回 false (Returns true if the service is ready,
 * otherwise returns false)
 */
bool ClientBase::service_is_ready() const {
  bool is_ready;
  rcl_ret_t ret = rcl_service_server_is_available(
      this->get_rcl_node_handle(), this->get_client_handle().get(), &is_ready);
  if (RCL_RET_NODE_INVALID == ret) {
    const rcl_node_t* node_handle = this->get_rcl_node_handle();
    if (node_handle && !rcl_context_is_valid(node_handle->context)) {
      // context is shutdown, do a soft failure
      return false;
    }
  }
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "rcl_service_server_is_available failed");
  }
  return is_ready;
}

/**
 * @brief 等待服务的纳秒超时 (Wait for service with nanoseconds timeout)
 *
 * @param timeout 超时时间 (Timeout duration)
 * @return 如果在超时之前服务可用，则返回 true，否则返回 false (Returns true if the service becomes
 * available before the timeout, otherwise returns false)
 */
bool ClientBase::wait_for_service_nanoseconds(std::chrono::nanoseconds timeout) {
  // 获取当前时间 (Get the current time)
  auto start = std::chrono::steady_clock::now();
  // 创建一个可重用的事件，而不是每次都创建一个新的 (Create an event to reuse, rather than create a
  // new one each time)
  auto node_ptr = node_graph_.lock();
  // 如果节点指针为空，抛出异常 (If the node pointer is null, throw an exception)
  if (!node_ptr) {
    throw InvalidNodeError();
  }
  // 立即检查服务器是否准备好 (Check if the server is ready immediately)
  if (this->service_is_ready()) {
    return true;
  }
  // 如果超时为0，立即返回 (If the timeout is 0, return immediately)
  if (timeout == std::chrono::nanoseconds(0)) {
    // 非阻塞检查，立即返回 (Non-blocking check, return immediately)
    return false;
  }
  // 获取图形事件 (Get the graph event)
  auto event = node_ptr->get_graph_event();
  // 即使在第一次循环中也要更新时间，以考虑到第一次调用 this->server_is_ready() 中花费的时间 (Update
  // the time even on the first loop to account for time spent in the first call to
  // this->server_is_ready())
  std::chrono::nanoseconds time_to_wait = timeout > std::chrono::nanoseconds(0)
                                              ? timeout - (std::chrono::steady_clock::now() - start)
                                              : std::chrono::nanoseconds::max();
  // 如果等待时间小于0，将其设置为0 (If the waiting time is less than 0, set it to 0)
  if (time_to_wait < std::chrono::nanoseconds(0)) {
    // 当超时最初为正数时，不允许 time_to_wait 变为负数。将 time_to_wait 设置为 0
    // 将允许一个非阻塞等待，因为有 do-while。(Do not allow the time_to_wait to become negative when
    // the timeout was originally positive. Setting time_to_wait to 0 will allow one non-blocking
    // wait because of the do-while.)
    time_to_wait = std::chrono::nanoseconds(0);
  }
  // 循环直到超时或服务准备好 (Loop until the timeout or service is ready)
  do {
    // 如果上下文无效，返回 false (If the context is invalid, return false)
    if (!rclcpp::ok(this->context_)) {
      return false;
    }
    // 将每次等待限制为100ms，以解决特定于 Connext RMW
    // 实现的问题。一种竞争条件意味着可用服务的图形更改可能会触发等待集合唤醒，但之后可能不会立即报告为准备好
    // (Limit each wait to 100ms to workaround an issue specific to the Connext RMW implementation.
    // A race condition means that graph changes for services becoming available may trigger the
    // wait set to wake up, but then not be reported as ready immediately after the wake up)
    // 如果没有其他图形事件发生，等待集合将不会再次被触发，直到超时已到，尽管服务可用，因此我们人为限制等待时间以限制延迟。(If
    // no other graph events occur, the wait set will not be triggered again until the timeout has
    // been reached, despite the service being available, so we artificially limit the wait time to
    // limit the delay.)
    node_ptr->wait_for_graph_change(
        event, std::min(time_to_wait, std::chrono::nanoseconds(RCL_MS_TO_NS(100))));
    // 由于上述竞争条件，即使图形事件未触发，我们也会检查服务是否准备好。(Because of the
    // aforementioned race condition, we check if the service is ready even if the graph event
    // wasn't triggered.)
    event->check_and_clear();
    // 如果服务准备好了，返回 true (If the service is ready, return true)
    if (this->service_is_ready()) {
      return true;
    }
    // 服务器没有准备好，如果还有剩余时间就继续循环 (The server is not ready, loop if there is time
    // left)
    if (timeout > std::chrono::nanoseconds(0)) {
      time_to_wait = timeout - (std::chrono::steady_clock::now() - start);
    }
    // 如果超时为负数，time_to_wait 永远不会达到零 (If the timeout is negative, time_to_wait will
    // never reach zero)
  } while (time_to_wait > std::chrono::nanoseconds(0));
  // 超时时等待服务器准备好，返回 false (Timeout exceeded while waiting for the server to be ready,
  // return false)
  return false;
}

/**
 * @brief 获取 rcl_node_t 句柄 (Get the rcl_node_t handle)
 *
 * @return 返回 rcl_node_t 句柄 (Return the rcl_node_t handle)
 */
rcl_node_t* ClientBase::get_rcl_node_handle() {
  // 获取 node_handle_ 的原始指针并返回 (Get the raw pointer of node_handle_ and return it)
  return node_handle_.get();
}

/**
 * @brief 获取 rcl_node_t 句柄的常量版本 (Get the const version of the rcl_node_t handle)
 *
 * @return 返回 rcl_node_t 句柄的常量指针 (Return a const pointer to the rcl_node_t handle)
 */
const rcl_node_t* ClientBase::get_rcl_node_handle() const {
  // 获取 node_handle_ 的原始指针并返回 (Get the raw pointer of node_handle_ and return it)
  return node_handle_.get();
}

/**
 * @brief 改变 wait set 使用状态 (Change the in-use state by wait set)
 *
 * @param[in] in_use_state 新的使用状态 (New in-use state)
 * @return 返回之前的使用状态 (Return the previous in-use state)
 */
bool ClientBase::exchange_in_use_by_wait_set_state(bool in_use_state) {
  // 交换 in_use_by_wait_set_ 的状态并返回旧值 (Exchange the state of in_use_by_wait_set_ and return
  // the old value)
  return in_use_by_wait_set_.exchange(in_use_state);
}

/**
 * @brief 获取请求发布者的实际 QoS (Get the actual QoS of the request publisher)
 *
 * @return 返回请求发布者的实际 QoS (Return the actual QoS of the request publisher)
 */
rclcpp::QoS ClientBase::get_request_publisher_actual_qos() const {
  // 获取客户端请求发布者的实际 QoS 设置 (Get the actual QoS settings of the client request
  // publisher)
  const rmw_qos_profile_t* qos = rcl_client_request_publisher_get_actual_qos(client_handle_.get());
  if (!qos) {
    auto msg = std::string("failed to get client's request publisher qos settings: ") +
               rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }

  // 从 rmw_qos_profile_t 创建 rclcpp::QoS 对象 (Create an rclcpp::QoS object from
  // rmw_qos_profile_t)
  rclcpp::QoS request_publisher_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*qos), *qos);

  return request_publisher_qos;
}

/**
 * @brief 获取响应订阅者的实际 QoS (Get the actual QoS of the response subscriber)
 *
 * @return 返回响应订阅者的实际 QoS (Return the actual QoS of the response subscriber)
 */
rclcpp::QoS ClientBase::get_response_subscription_actual_qos() const {
  // 获取客户端响应订阅者的实际 QoS 设置 (Get the actual QoS settings of the client response
  // subscriber)
  const rmw_qos_profile_t* qos =
      rcl_client_response_subscription_get_actual_qos(client_handle_.get());
  if (!qos) {
    auto msg = std::string("failed to get client's response subscription qos settings: ") +
               rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }

  // 从 rmw_qos_profile_t 创建 rclcpp::QoS 对象 (Create an rclcpp::QoS object from
  // rmw_qos_profile_t)
  rclcpp::QoS response_subscription_qos =
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*qos), *qos);

  return response_subscription_qos;
}

/**
 * @brief 设置新响应回调函数 (Set the new response callback function)
 *
 * @param[in] callback 新的响应回调函数 (New response callback function)
 * @param[in] user_data 用户数据指针 (User data pointer)
 */
void ClientBase::set_on_new_response_callback(
    rcl_event_callback_t callback, const void* user_data) {
  // 设置客户端的新响应回调函数 (Set the new response callback function for the client)
  rcl_ret_t ret =
      rcl_client_set_on_new_response_callback(client_handle_.get(), callback, user_data);

  // 如果设置失败，抛出异常 (If it fails to set, throw an exception)
  if (RCL_RET_OK != ret) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(ret, "failed to set the on new response callback for client");
  }
}

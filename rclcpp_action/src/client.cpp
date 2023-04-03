// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "rclcpp_action/client.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <utility>

#include "rcl_action/action_client.h"
#include "rcl_action/wait.h"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp_action/exceptions.hpp"

namespace rclcpp_action {

/**
 * @class ClientBaseImpl
 * @brief 客户端基础实现类 (Client base implementation class)
 */
class ClientBaseImpl {
public:
  /**
   * @brief 构造函数，初始化 ClientBaseImpl 对象 (Constructor, initializes the ClientBaseImpl
   * object)
   *
   * @param node_base 一个共享指针，指向 NodeBaseInterface 对象 (A shared pointer to a
   * NodeBaseInterface object)
   * @param node_graph 一个共享指针，指向 NodeGraphInterface 对象 (A shared pointer to a
   * NodeGraphInterface object)
   * @param node_logging 一个共享指针，指向 NodeLoggingInterface 对象 (A shared pointer to a
   * NodeLoggingInterface object)
   * @param action_name action 的名称 (The name of the action)
   * @param type_support 指向 rosidl_action_type_support_t 结构体的指针 (A pointer to a
   * rosidl_action_type_support_t structure)
   * @param client_options rcl_action_client_options_t 结构体，包含客户端选项 (An
   * rcl_action_client_options_t structure containing client options)
   */
  ClientBaseImpl(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
      const std::string &action_name,
      const rosidl_action_type_support_t *type_support,
      const rcl_action_client_options_t &client_options)
      : node_graph_(node_graph),
        node_handle(node_base->get_shared_rcl_node_handle()),
        logger(node_logging->get_logger().get_child("rclcpp_action")),
        random_bytes_generator(std::random_device{}()) {
    // 创建一个弱指针，指向 rcl_node_t 类型的对象 (Create a weak pointer to an rcl_node_t object)
    std::weak_ptr<rcl_node_t> weak_node_handle(node_handle);

    // 初始化 client_handle，为 rcl_action_client_t 类型的共享指针 (Initialize client_handle, a
    // shared pointer to an rcl_action_client_t object)
    client_handle = std::shared_ptr<rcl_action_client_t>(
        new rcl_action_client_t, [weak_node_handle](rcl_action_client_t *client) {
          // 获取弱指针所指向的对象 (Get the object pointed to by the weak pointer)
          auto handle = weak_node_handle.lock();
          if (handle) {
            // 如果节点句柄有效，则销毁 rcl_action_client_t 对象 (If the node handle is valid,
            // destroy the rcl_action_client_t object)
            if (RCL_RET_OK != rcl_action_client_fini(client, handle.get())) {
              RCLCPP_ERROR(
                  rclcpp::get_logger(rcl_node_get_logger_name(handle.get()))
                      .get_child("rclcpp_action"),
                  "Error in destruction of rcl action client handle: %s",
                  rcl_get_error_string().str);
              rcl_reset_error();
            }
          } else {
            // 如果节点句柄无效，则报告内存泄漏错误 (If the node handle is invalid, report memory
            // leak error)
            RCLCPP_ERROR(
                rclcpp::get_logger("rclcpp_action"),
                "Error in destruction of rcl action client handle: "
                "the Node Handle was destructed too early. You will leak memory");
          }
          delete client;
        });

    // 初始化 rcl_action_client_t 对象 (Initialize the rcl_action_client_t object)
    *client_handle = rcl_action_get_zero_initialized_client();
    rcl_ret_t ret = rcl_action_client_init(
        client_handle.get(), node_handle.get(), type_support, action_name.c_str(), &client_options);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "could not initialize rcl action client");
    }

    // 获取 rcl_action_client_t 对象的详细信息 (Get details of the rcl_action_client_t object)
    ret = rcl_action_client_wait_set_get_num_entities(
        client_handle.get(), &num_subscriptions, &num_guard_conditions, &num_timers, &num_clients,
        &num_services);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "could not retrieve rcl action client details");
    }
  }

  // 订阅数量 (Number of subscriptions)
  size_t num_subscriptions{0u};
  // 守护条件数量 (Number of guard conditions)
  size_t num_guard_conditions{0u};
  // 定时器数量 (Number of timers)
  size_t num_timers{0u};
  // 客户端数量 (Number of clients)
  size_t num_clients{0u};
  // 服务数量 (Number of services)
  size_t num_services{0u};

  // 反馈是否准备好 (Is feedback ready)
  bool is_feedback_ready{false};
  // 状态是否准备好 (Is status ready)
  bool is_status_ready{false};
  // 目标响应是否准备好 (Is goal response ready)
  bool is_goal_response_ready{false};
  // 取消响应是否准备好 (Is cancel response ready)
  bool is_cancel_response_ready{false};
  // 结果响应是否准备好 (Is result response ready)
  bool is_result_response_ready{false};

  // ROS2上下文共享指针 (Shared pointer for ROS2 context)
  rclcpp::Context::SharedPtr context_;
  // 节点图接口弱指针 (Weak pointer for node graph interface)
  rclcpp::node_interfaces::NodeGraphInterface::WeakPtr node_graph_;
  // 需要在client_handle之后销毁node_handle以防止内存泄漏 (node_handle must be destroyed after
  // client_handle to prevent memory leak)
  std::shared_ptr<rcl_node_t> node_handle{nullptr};
  // 动作客户端共享指针 (Shared pointer for action client)
  std::shared_ptr<rcl_action_client_t> client_handle{nullptr};
  // 日志记录器 (Logger)
  rclcpp::Logger logger;

  // 响应回调类型定义 (Type definition for response callback)
  using ResponseCallback = std::function<void(std::shared_ptr<void> response)>;

  // 等待目标响应的映射 (Map for pending goal responses)
  std::map<int64_t, ResponseCallback> pending_goal_responses;
  // 目标请求互斥锁 (Mutex for goal requests)
  std::mutex goal_requests_mutex;

  // 等待结果响应的映射 (Map for pending result responses)
  std::map<int64_t, ResponseCallback> pending_result_responses;
  // 结果请求互斥锁 (Mutex for result requests)
  std::mutex result_requests_mutex;

  // 等待取消响应的映射 (Map for pending cancel responses)
  std::map<int64_t, ResponseCallback> pending_cancel_responses;
  // 取消请求互斥锁 (Mutex for cancel requests)
  std::mutex cancel_requests_mutex;

  // 随机字节生成器 (Random bytes generator)
  std::independent_bits_engine<std::default_random_engine, 8, unsigned int> random_bytes_generator;
};

/**
 * @brief 构造一个 ClientBase 对象 (Constructs a ClientBase object)
 *
 * @param node_base ROS 2 节点基础接口共享指针 (Shared pointer to the ROS 2 Node Base Interface)
 * @param node_graph ROS 2 节点图接口共享指针 (Shared pointer to the ROS 2 Node Graph Interface)
 * @param node_logging ROS 2 节点日志接口共享指针 (Shared pointer to the ROS 2 Node Logging
 * Interface)
 * @param action_name 操作名称字符串 (String containing the action name)
 * @param type_support 操作类型支持结构体指针 (Pointer to the action type support struct)
 * @param client_options 客户端选项结构体 (Client options struct)
 */
ClientBase::ClientBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string &action_name,
    const rosidl_action_type_support_t *type_support,
    const rcl_action_client_options_t &client_options)
    : pimpl_(new ClientBaseImpl(  // 创建一个新的 ClientBaseImpl 对象并初始化 pimpl_ (Create a new
                                  // ClientBaseImpl object and initialize pimpl_)
          node_base,
          node_graph,
          node_logging,
          action_name,
          type_support,
          client_options)) {}

/**
 * @brief 销毁 ClientBase 对象 (Destroys the ClientBase object)
 */
ClientBase::~ClientBase() {}

/**
 * @brief 检查动作服务器是否准备就绪 (Check if the action server is ready)
 *
 * @return 返回动作服务器的状态 (true: 就绪，false: 未就绪) (Return the status of the action server
 * (true: ready, false: not ready))
 */
bool ClientBase::action_server_is_ready() const {
  // 声明一个布尔变量用于存储动作服务器的状态（Declare a boolean variable to store the status of the
  // action server）
  bool is_ready;
  // 调用 rcl_action_server_is_available 函数检查动作服务器是否可用，并将结果存储在 is_ready
  // 中（Call the rcl_action_server_is_available function to check if the action server is available
  // and store the result in is_ready）
  rcl_ret_t ret = rcl_action_server_is_available(
      this->pimpl_->node_handle.get(), this->pimpl_->client_handle.get(), &is_ready);
  // 如果返回值为 RCL_RET_NODE_INVALID，则检查节点句柄和上下文是否有效（If the return value is
  // RCL_RET_NODE_INVALID, check if the node handle and context are valid）
  if (RCL_RET_NODE_INVALID == ret) {
    const rcl_node_t *node_handle = this->pimpl_->node_handle.get();
    // 如果节点句柄存在且上下文无效，则返回 false（If the node handle exists and the context is
    // invalid, return false）
    if (node_handle && !rcl_context_is_valid(node_handle->context)) {
      // 上下文已关闭，执行软故障（Context is shutdown, do a soft failure）
      return false;
    }
  }
  // 如果返回值不是 RCL_RET_OK，则抛出异常（If the return value is not RCL_RET_OK, throw an
  // exception）
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "rcl_action_server_is_available failed");
  }
  // 返回动作服务器的状态（Return the status of the action server）
  return is_ready;
}

/**
 * @brief 等待动作服务器在指定的纳秒超时时间内准备就绪 (Wait for the action server to be ready
 * within the specified nanoseconds timeout)
 *
 * @param timeout 超时时间，以纳秒为单位 (Timeout in nanoseconds)
 * @return 如果动作服务器在超时时间内准备就绪，则返回 true，否则返回 false (Returns true if the
 * action server is ready within the timeout, otherwise returns false)
 */
bool ClientBase::wait_for_action_server_nanoseconds(std::chrono::nanoseconds timeout) {
  // 获取当前时间点 (Get the current time point)
  auto start = std::chrono::steady_clock::now();

  // 创建一个可重用的事件，而不是每次都创建新事件 (Create a reusable event instead of creating a new
  // one each time)
  auto node_ptr = pimpl_->node_graph_.lock();
  // 如果节点指针无效，则抛出异常 (If the node pointer is invalid, throw an exception)
  if (!node_ptr) {
    throw rclcpp::exceptions::InvalidNodeError();
  }

  // 立即检查服务器是否已准备就绪 (Check if the server is ready immediately)
  if (this->action_server_is_ready()) {
    return true;
  }

  // 获取图形事件 (Get the graph event)
  auto event = node_ptr->get_graph_event();

  // 如果超时时间为 0，则立即返回 (If the timeout is 0, return immediately)
  if (timeout == std::chrono::nanoseconds(0)) {
    return false;
  }

  // 更新等待时间，考虑到第一次调用 this->server_is_ready() 所花费的时间 (Update the waiting time to
  // account for the time spent in the first call to this->server_is_ready())
  std::chrono::nanoseconds time_to_wait = timeout > std::chrono::nanoseconds(0)
                                              ? timeout - (std::chrono::steady_clock::now() - start)
                                              : std::chrono::nanoseconds::max();

  // 如果等待时间小于 0，则将其设置为 0 (If the waiting time is less than 0, set it to 0)
  if (time_to_wait < std::chrono::nanoseconds(0)) {
    time_to_wait = std::chrono::nanoseconds(0);
  }

  // 使用 do-while 循环进行等待 (Use a do-while loop to wait)
  do {
    // 检查上下文是否有效 (Check if the context is valid)
    if (!rclcpp::ok(this->pimpl_->context_)) {
      return false;
    }

    // 将每次等待限制为 100ms，以解决 Connext RMW 实现中的特定问题
    // 一个竞争条件意味着服务可用性的图形变化可能触发等待集唤醒，但在唤醒后不会立即报告就绪状态
    // (参见 https://github.com/ros2/rmw_connext/issues/201)
    // 如果没有其他图形事件发生，等待集将不会再次被触发，直到超时达到，尽管服务是可用的，因此我们人为限制等待时间以减少延迟。
    node_ptr->wait_for_graph_change(
        event, std::min(time_to_wait, std::chrono::nanoseconds(RCL_MS_TO_NS(100))));

    // 由于前面提到的竞争条件，即使图形事件没有被触发，我们也会检查服务是否已准备就绪。
    event->check_and_clear();

    // 如果动作服务器已准备就绪，则返回 true (If the action server is ready, return true)
    if (this->action_server_is_ready()) {
      return true;
    }

    // 如果服务器尚未准备好，且仍有剩余时间，则继续循环等待 (Continue to loop and wait if the server
    // is not ready and there is still time left)
    if (timeout > std::chrono::nanoseconds(0)) {
      time_to_wait = timeout - (std::chrono::steady_clock::now() - start);
    }

    // 如果超时为负数，time_to_wait 将永远不会达到零 (If the timeout is negative, time_to_wait will
    // never reach zero)
  } while (time_to_wait > std::chrono::nanoseconds(0));

  // 超时时间已过，但服务器尚未准备好 (The timeout has elapsed, but the server is not yet ready)
  return false;
}

/**
 * @brief 获取 ClientBase 的 logger 对象 (Get the logger object of ClientBase)
 *
 * @return rclcpp::Logger 返回 logger 对象 (Return the logger object)
 */
rclcpp::Logger ClientBase::get_logger() {
  // 返回 pimpl_ 结构体中的 logger 对象 (Return the logger object in the pimpl_ structure)
  return pimpl_->logger;
}

/**
 * @brief 获取准备好的订阅数量 (Get the number of ready subscriptions)
 *
 * @return size_t 准备好的订阅数量 (Number of ready subscriptions)
 */
size_t ClientBase::get_number_of_ready_subscriptions() {
  // 返回 pimpl_ 结构体中的 num_subscriptions 变量值 (Return the value of num_subscriptions in the
  // pimpl_ structure)
  return pimpl_->num_subscriptions;
}

/**
 * @brief 获取准备好的 guard conditions 数量 (Get the number of ready guard conditions)
 *
 * @return size_t 准备好的 guard conditions 数量 (Number of ready guard conditions)
 */
size_t ClientBase::get_number_of_ready_guard_conditions() {
  // 返回 pimpl_ 结构体中的 num_guard_conditions 变量值 (Return the value of num_guard_conditions in
  // the pimpl_ structure)
  return pimpl_->num_guard_conditions;
}

/**
 * @brief 获取准备好的定时器数量 (Get the number of ready timers)
 *
 * @return size_t 准备好的定时器数量 (Number of ready timers)
 */
size_t ClientBase::get_number_of_ready_timers() {
  // 返回 pimpl_ 结构体中的 num_timers 变量值 (Return the value of num_timers in the pimpl_
  // structure)
  return pimpl_->num_timers;
}

/**
 * @brief 获取准备好的客户端数量 (Get the number of ready clients)
 *
 * @return size_t 准备好的客户端数量 (Number of ready clients)
 */
size_t ClientBase::get_number_of_ready_clients() {
  // 返回 pimpl_ 结构体中的 num_clients 变量值 (Return the value of num_clients in the pimpl_
  // structure)
  return pimpl_->num_clients;
}

/**
 * @brief 获取准备好的服务数量 (Get the number of ready services)
 *
 * @return size_t 准备好的服务数量 (Number of ready services)
 */
size_t ClientBase::get_number_of_ready_services() {
  // 返回 pimpl_ 结构体中的 num_services 变量值 (Return the value of num_services in the pimpl_
  // structure)
  return pimpl_->num_services;
}

/**
 * @brief 添加客户端到等待集合 (Add the client to the wait set)
 *
 * @param wait_set 等待集合指针 (Pointer to the wait set)
 */
void ClientBase::add_to_wait_set(rcl_wait_set_t *wait_set) {
  // 将 action 客户端添加到等待集合中，并检查返回值 (Add the action client to the wait set and check
  // the return value)
  rcl_ret_t ret = rcl_action_wait_set_add_action_client(
      wait_set, pimpl_->client_handle.get(), nullptr, nullptr);
  if (RCL_RET_OK != ret) {
    // 如果返回值不是 RCL_RET_OK，则抛出异常 (If the return value is not RCL_RET_OK, throw an
    // exception)
    rclcpp::exceptions::throw_from_rcl_error(ret, "ClientBase::add_to_wait_set() failed");
  }
}

/**
 * @brief 检查客户端是否准备好 (Check if the client is ready)
 *
 * @param wait_set 等待集合指针 (Pointer to the wait set)
 * @return true 如果客户端准备好 (If the client is ready)
 * @return false 如果客户端未准备好 (If the client is not ready)
 */
bool ClientBase::is_ready(rcl_wait_set_t *wait_set) {
  // 检查等待集合中的实体是否准备好，并检查返回值 (Check if the entities in the wait set are ready
  // and check the return value)
  rcl_ret_t ret = rcl_action_client_wait_set_get_entities_ready(
      wait_set, pimpl_->client_handle.get(), &pimpl_->is_feedback_ready, &pimpl_->is_status_ready,
      &pimpl_->is_goal_response_ready, &pimpl_->is_cancel_response_ready,
      &pimpl_->is_result_response_ready);
  if (RCL_RET_OK != ret) {
    // 如果返回值不是 RCL_RET_OK，则抛出异常 (If the return value is not RCL_RET_OK, throw an
    // exception)
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to check for any ready entities");
  }
  // 返回客户端是否准备好的状态 (Return the readiness status of the client)
  return pimpl_->is_feedback_ready || pimpl_->is_status_ready || pimpl_->is_goal_response_ready ||
         pimpl_->is_cancel_response_ready || pimpl_->is_result_response_ready;
}

/**
 * @brief 处理目标响应 (Handle goal response)
 *
 * @param response_header 响应头 (Response header)
 * @param response 响应指针 (Pointer to the response)
 */
void ClientBase::handle_goal_response(
    const rmw_request_id_t &response_header, std::shared_ptr<void> response) {
  // 锁定互斥体以保护目标请求 (Lock the mutex to protect the goal requests)
  std::lock_guard<std::mutex> guard(pimpl_->goal_requests_mutex);
  const int64_t &sequence_number = response_header.sequence_number;
  if (pimpl_->pending_goal_responses.count(sequence_number) == 0) {
    // 如果找不到对应的序列号，记录错误并忽略 (If the corresponding sequence number is not found,
    // log an error and ignore)
    RCLCPP_ERROR(pimpl_->logger, "unknown goal response, ignoring...");
    return;
  }
  // 调用回调函数处理响应 (Call the callback function to handle the response)
  pimpl_->pending_goal_responses[sequence_number](response);
  // 从 pending_goal_responses 中删除已处理的序列号 (Remove the processed sequence number from
  // pending_goal_responses)
  pimpl_->pending_goal_responses.erase(sequence_number);
}

/**
 * @brief 发送目标请求 (Send goal request)
 *
 * @param request 请求指针 (Pointer to the request)
 * @param callback 响应回调函数 (Response callback function)
 */
void ClientBase::send_goal_request(std::shared_ptr<void> request, ResponseCallback callback) {
  // 锁定互斥体以保护目标请求 (Lock the mutex to protect the goal requests)
  std::unique_lock<std::mutex> guard(pimpl_->goal_requests_mutex);
  int64_t sequence_number;
  // 发送目标请求，并检查返回值 (Send the goal request and check the return value)
  rcl_ret_t ret =
      rcl_action_send_goal_request(pimpl_->client_handle.get(), request.get(), &sequence_number);
  if (RCL_RET_OK != ret) {
    // 如果返回值不是 RCL_RET_OK，则抛出异常 (If the return value is not RCL_RET_OK, throw an
    // exception)
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send goal request");
  }
  // 确保序列号在 pending_goal_responses 中不存在 (Ensure the sequence number does not exist in
  // pending_goal_responses)
  assert(pimpl_->pending_goal_responses.count(sequence_number) == 0);
  // 将回调函数与序列号关联并存储在 pending_goal_responses 中 (Associate the callback function with
  // the sequence number and store it in pending_goal_responses)
  pimpl_->pending_goal_responses[sequence_number] = callback;
}

/*!
 * \brief 处理结果响应
 * \param response_header 包含请求响应的元数据，例如序列号
 * \param response 结果响应的数据
 *
 * Handle the result response.
 * \param response_header Contains metadata about the request response, such as sequence number.
 * \param response The data of the result response.
 */
void ClientBase::handle_result_response(
    const rmw_request_id_t &response_header, std::shared_ptr<void> response) {
  // 对互斥锁进行加锁保护，防止多线程访问冲突
  // Lock the mutex to protect against multi-threaded access conflicts.
  std::lock_guard<std::mutex> guard(pimpl_->result_requests_mutex);

  // 获取响应头中的序列号
  // Get the sequence number from the response header.
  const int64_t &sequence_number = response_header.sequence_number;

  // 检查该序列号是否存在于待处理的结果响应映射中
  // Check if the sequence number exists in the pending result responses map.
  if (pimpl_->pending_result_responses.count(sequence_number) == 0) {
    // 如果不存在，则记录错误并忽略此响应
    // If not present, log an error and ignore this response.
    RCLCPP_ERROR(pimpl_->logger, "unknown result response, ignoring...");
    return;
  }

  // 调用与序列号关联的回调函数处理响应
  // Invoke the callback associated with the sequence number to handle the response.
  pimpl_->pending_result_responses[sequence_number](response);

  // 从映射中删除已处理的序列号
  // Remove the processed sequence number from the map.
  pimpl_->pending_result_responses.erase(sequence_number);
}

/*!
 * \brief 发送结果请求
 * \param request 要发送的结果请求数据
 * \param callback 处理结果响应的回调函数
 *
 * Send the result request.
 * \param request The data of the result request to be sent.
 * \param callback The callback function to handle the result response.
 */
void ClientBase::send_result_request(std::shared_ptr<void> request, ResponseCallback callback) {
  // 对互斥锁进行加锁保护，防止多线程访问冲突
  // Lock the mutex to protect against multi-threaded access conflicts.
  std::lock_guard<std::mutex> guard(pimpl_->result_requests_mutex);

  int64_t sequence_number;
  // 使用rcl_action_send_result_request发送结果请求，并获取序列号
  // Send the result request using rcl_action_send_result_request and get the sequence number.
  rcl_ret_t ret =
      rcl_action_send_result_request(pimpl_->client_handle.get(), request.get(), &sequence_number);

  // 检查发送是否成功
  // Check if the send was successful.
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send result request");
  }

  // 确保序列号在待处理的结果响应映射中不存在
  // Ensure the sequence number is not present in the pending result responses map.
  assert(pimpl_->pending_result_responses.count(sequence_number) == 0);

  // 将序列号与回调函数添加到待处理的结果响应映射中
  // Add the sequence number and callback to the pending result responses map.
  pimpl_->pending_result_responses[sequence_number] = callback;
}

/*!
 * \brief 处理取消响应
 * \param response_header 包含请求响应的元数据，例如序列号
 * \param response 取消响应的数据
 *
 * Handle the cancel response.
 * \param response_header Contains metadata about the request response, such as sequence number.
 * \param response The data of the cancel response.
 */
void ClientBase::handle_cancel_response(
    const rmw_request_id_t &response_header, std::shared_ptr<void> response) {
  // 对互斥锁进行加锁保护，防止多线程访问冲突
  // Lock the mutex to protect against multi-threaded access conflicts.
  std::lock_guard<std::mutex> guard(pimpl_->cancel_requests_mutex);

  // 获取响应头中的序列号
  // Get the sequence number from the response header.
  const int64_t &sequence_number = response_header.sequence_number;

  // 检查该序列号是否存在于待处理的取消响应映射中
  // Check if the sequence number exists in the pending cancel responses map.
  if (pimpl_->pending_cancel_responses.count(sequence_number) == 0) {
    // 如果不存在，则记录错误并忽略此响应
    // If not present, log an error and ignore this response.
    RCLCPP_ERROR(pimpl_->logger, "unknown cancel response, ignoring...");
    return;
  }

  // 调用与序列号关联的回调函数处理响应
  // Invoke the callback associated with the sequence number to handle the response.
  pimpl_->pending_cancel_responses[sequence_number](response);

  // 从映射中删除已处理的序列号
  // Remove the processed sequence number from the map.
  pimpl_->pending_cancel_responses.erase(sequence_number);
}

/**
 * @brief 发送取消请求 (Send cancel request)
 *
 * @param[in] request 取消请求的指针 (Pointer to the cancel request)
 * @param[in] callback 当收到响应时调用的回调函数 (Callback function to be called when a response is
 * received)
 */
void ClientBase::send_cancel_request(std::shared_ptr<void> request, ResponseCallback callback) {
  // 对 cancel_requests_mutex 上锁，防止多线程冲突 (Lock the cancel_requests_mutex to prevent
  // multi-threading conflicts)
  std::lock_guard<std::mutex> guard(pimpl_->cancel_requests_mutex);

  int64_t sequence_number;

  // 调用 rcl_action_send_cancel_request 函数发送取消请求 (Call the rcl_action_send_cancel_request
  // function to send the cancel request)
  rcl_ret_t ret =
      rcl_action_send_cancel_request(pimpl_->client_handle.get(), request.get(), &sequence_number);

  // 如果发送失败，抛出异常 (If the sending fails, throw an exception)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send cancel request");
  }

  // 检查序列号是否已经存在于 pending_cancel_responses 中 (Check if the sequence number already
  // exists in pending_cancel_responses)
  assert(pimpl_->pending_cancel_responses.count(sequence_number) == 0);

  // 将回调函数与序列号关联存储在 pending_cancel_responses 中 (Store the callback associated with
  // the sequence number in pending_cancel_responses)
  pimpl_->pending_cancel_responses[sequence_number] = callback;
}

/**
 * @brief 生成目标 ID (Generate goal ID)
 *
 * @return GoalUUID 生成的目标 ID (Generated goal ID)
 */
GoalUUID ClientBase::generate_goal_id() {
  GoalUUID goal_id;

  // TODO(hidmic): Do something better than this for UUID generation.
  // std::generate(
  //   goal_id.uuid.begin(), goal_id.uuid.end(),
  //   std::ref(pimpl_->random_bytes_generator));

  // 使用随机字节生成器生成目标 ID (Generate the goal ID using the random bytes generator)
  std::generate(goal_id.begin(), goal_id.end(), std::ref(pimpl_->random_bytes_generator));

  return goal_id;
}

/**
 * @brief 设置准备就绪回调函数 (Set on ready callback function)
 *
 * @param[in] callback 当实体准备好时调用的回调函数 (Callback function to be called when an entity
 * is ready)
 */
void ClientBase::set_on_ready_callback(std::function<void(size_t, int)> callback) {
  // 如果回调函数不可调用，抛出异常 (If the callback function is not callable, throw an exception)
  if (!callback) {
    throw std::invalid_argument(
        "The callback passed to set_on_ready_callback "
        "is not callable.");
  }

  // 为各种实体类型设置回调函数 (Set the callback function for various entity types)
  set_callback_to_entity(EntityType::GoalClient, callback);
  set_callback_to_entity(EntityType::ResultClient, callback);
  set_callback_to_entity(EntityType::CancelClient, callback);
  set_callback_to_entity(EntityType::FeedbackSubscription, callback);
  set_callback_to_entity(EntityType::StatusSubscription, callback);
}

/**
 * @brief 设置回调函数到实体 (Set the callback function to the entity)
 *
 * @param[in] entity_type 实体类型 (Entity type)
 * @param[in] callback 回调函数 (Callback function)
 */
void ClientBase::set_callback_to_entity(
    EntityType entity_type, std::function<void(size_t, int)> callback) {
  // 将 int 标识符参数绑定到此可等待的实体类型 (Bind the int identifier argument to this waitable's
  // entity types) 使用 lambda 表达式创建一个新的回调函数 (Create a new callback using a lambda
  // expression)
  auto new_callback = [callback, entity_type, this](size_t number_of_events) {
    try {
      // 调用回调函数并传入事件数量和实体类型 (Invoke the callback with the number of events and
      // entity type)
      callback(number_of_events, static_cast<int>(entity_type));
    } catch (const std::exception &exception) {
      // 当捕获到用户提供的回调中的异常时，记录错误信息 (Log an error when an exception is caught in
      // the user-provided callback)
      RCLCPP_ERROR_STREAM(
          pimpl_->logger,
          "rclcpp_action::ClientBase@"
              << this << " caught " << rmw::impl::cpp::demangle(exception)
              << " exception in user-provided callback for the 'on ready' callback: "
              << exception.what());
    } catch (...) {
      // 当捕获到未处理的异常时，记录错误信息 (Log an error when an unhandled exception is caught)
      RCLCPP_ERROR_STREAM(
          pimpl_->logger, "rclcpp_action::ClientBase@"
                              << this << " caught unhandled exception in user-provided callback "
                              << "for the 'on ready' callback");
    }
  };

  // 调用 set_on_ready_callback 设置新的回调函数 (Call set_on_ready_callback to set the new
  // callback) 这里使用了两步设置，以防止旧的 std::function
  // 被替换，但中间件还没有被通知到新的回调函数 (This two-step setting prevents a gap where the old
  // std::function has been replaced but the middleware hasn't been told about the new one yet)
  set_on_ready_callback(
      entity_type,
      rclcpp::detail::cpp_callback_trampoline<decltype(new_callback), const void *, size_t>,
      static_cast<const void *>(&new_callback));

  // 使用锁保护实体类型到回调函数的映射 (Use a lock to protect the mapping of entity type to
  // callback function)
  std::lock_guard<std::recursive_mutex> lock(listener_mutex_);
  // 将新的回调函数存储到映射中，覆盖现有的回调函数 (Store the new callback in the mapping,
  // overwriting the existing one)
  auto it = entity_type_to_on_ready_callback_.find(entity_type);

  if (it != entity_type_to_on_ready_callback_.end()) {
    it->second = new_callback;
  } else {
    entity_type_to_on_ready_callback_.emplace(entity_type, new_callback);
  }

  // 再次设置回调函数，这次使用永久存储 (Set the callback again, this time using permanent storage)
  it = entity_type_to_on_ready_callback_.find(entity_type);

  if (it != entity_type_to_on_ready_callback_.end()) {
    auto &cb = it->second;
    set_on_ready_callback(
        entity_type,
        rclcpp::detail::cpp_callback_trampoline<decltype(it->second), const void *, size_t>,
        static_cast<const void *>(&cb));
  }

  // 设置 on_ready_callback_set_ 标志为 true (Set the on_ready_callback_set_ flag to true)
  on_ready_callback_set_ = true;
}

/**
 * @brief 设置实体类型的准备回调
 *        Set the on ready callback for the specified entity type.
 *
 * @param entity_type 实体类型（目标客户端、结果客户端等）
 *                    Entity type (GoalClient, ResultClient, etc.)
 * @param callback    要设置的回调函数
 *                    The callback function to be set
 * @param user_data   用户数据，将传递给回调函数
 *                    User data that will be passed to the callback function
 */
void ClientBase::set_on_ready_callback(
    EntityType entity_type, rcl_event_callback_t callback, const void *user_data) {
  // 初始化返回值为错误
  // Initialize return value as error
  rcl_ret_t ret = RCL_RET_ERROR;

  // 根据实体类型执行相应操作
  // Perform corresponding operation based on entity type
  switch (entity_type) {
    case EntityType::GoalClient: {
      // 设置目标客户端回调
      // Set goal client callback
      ret = rcl_action_client_set_goal_client_callback(
          pimpl_->client_handle.get(), callback, user_data);
      break;
    }

    case EntityType::ResultClient: {
      // 设置结果客户端回调
      // Set result client callback
      ret = rcl_action_client_set_result_client_callback(
          pimpl_->client_handle.get(), callback, user_data);
      break;
    }

    case EntityType::CancelClient: {
      // 设置取消客户端回调
      // Set cancel client callback
      ret = rcl_action_client_set_cancel_client_callback(
          pimpl_->client_handle.get(), callback, user_data);
      break;
    }

    case EntityType::FeedbackSubscription: {
      // 设置反馈订阅回调
      // Set feedback subscription callback
      ret = rcl_action_client_set_feedback_subscription_callback(
          pimpl_->client_handle.get(), callback, user_data);
      break;
    }

    case EntityType::StatusSubscription: {
      // 设置状态订阅回调
      // Set status subscription callback
      ret = rcl_action_client_set_status_subscription_callback(
          pimpl_->client_handle.get(), callback, user_data);
      break;
    }

    default:
      // 抛出未知实体类型异常
      // Throw unknown entity type exception
      throw std::runtime_error("ClientBase::set_on_ready_callback: Unknown entity type.");
      break;
  }

  // 如果返回值不是 RCL_RET_OK，抛出异常
  // If the return value is not RCL_RET_OK, throw an exception
  if (RCL_RET_OK != ret) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(ret, "failed to set the on ready callback for action client");
  }
}

/**
 * @brief 清除已设置的就绪回调函数（Clears the set on ready callbacks）
 *
 * 该函数用于清除所有与实体类型相关的就绪回调函数。
 * (This function is used to clear all the on ready callbacks associated with entity types.)
 */
void ClientBase::clear_on_ready_callback() {
  // 使用 std::lock_guard 对 listener_mutex_ 进行加锁，以确保线程安全
  // (Lock the listener_mutex_ using std::lock_guard to ensure thread safety)
  std::lock_guard<std::recursive_mutex> lock(listener_mutex_);

  // 如果已设置就绪回调函数，则清除与各实体类型相关的回调函数
  // (If on ready callbacks are set, clear the callbacks associated with each entity type)
  if (on_ready_callback_set_) {
    // 清除目标客户端的就绪回调函数 (Clear the on ready callback for GoalClient)
    set_on_ready_callback(EntityType::GoalClient, nullptr, nullptr);
    // 清除结果客户端的就绪回调函数 (Clear the on ready callback for ResultClient)
    set_on_ready_callback(EntityType::ResultClient, nullptr, nullptr);
    // 清除取消客户端的就绪回调函数 (Clear the on ready callback for CancelClient)
    set_on_ready_callback(EntityType::CancelClient, nullptr, nullptr);
    // 清除反馈订阅的就绪回调函数 (Clear the on ready callback for FeedbackSubscription)
    set_on_ready_callback(EntityType::FeedbackSubscription, nullptr, nullptr);
    // 清除状态订阅的就绪回调函数 (Clear the on ready callback for StatusSubscription)
    set_on_ready_callback(EntityType::StatusSubscription, nullptr, nullptr);
    // 将 on_ready_callback_set_ 标志设置为 false，表示没有设置就绪回调函数
    // (Set the on_ready_callback_set_ flag to false, indicating that no on ready callbacks are set)
    on_ready_callback_set_ = false;
  }

  // 清除实体类型到就绪回调函数的映射
  // (Clear the mapping from entity types to on ready callbacks)
  entity_type_to_on_ready_callback_.clear();
}

/**
 * @brief 从 ClientBase 中获取数据（Take data from the ClientBase）
 *
 * @return std::shared_ptr<void> 返回一个包含请求结果的共享指针（Return a shared pointer containing
 * the request result）
 */
std::shared_ptr<void> ClientBase::take_data() {
  // 判断是否有反馈消息准备好（Check if feedback message is ready）
  // Determine if the feedback is ready
  if (pimpl_->is_feedback_ready) {
    // 创建一个反馈消息（Create a feedback message）
    std::shared_ptr<void> feedback_message = this->create_feedback_message();
    // 获取反馈消息（Get the feedback message）
    rcl_ret_t ret = rcl_action_take_feedback(pimpl_->client_handle.get(), feedback_message.get());
    // 返回包含反馈消息的元组（Return a tuple containing the feedback message）
    return std::static_pointer_cast<void>(
        std::make_shared<std::tuple<rcl_ret_t, std::shared_ptr<void>>>(ret, feedback_message));
  } else if (pimpl_
                 ->is_status_ready) {  // 判断状态消息是否准备好（Check if status message is ready）
    // 创建一个状态消息（Create a status message）
    std::shared_ptr<void> status_message = this->create_status_message();
    // 获取状态消息（Get the status message）
    rcl_ret_t ret = rcl_action_take_status(pimpl_->client_handle.get(), status_message.get());
    // 返回包含状态消息的元组（Return a tuple containing the status message）
    return std::static_pointer_cast<void>(
        std::make_shared<std::tuple<rcl_ret_t, std::shared_ptr<void>>>(ret, status_message));
  } else if (pimpl_->is_goal_response_ready) {  // 判断目标响应是否准备好（Check if goal response is
                                                // ready）
    rmw_request_id_t response_header;
    // 创建一个目标响应（Create a goal response）
    std::shared_ptr<void> goal_response = this->create_goal_response();
    // 获取目标响应（Get the goal response）
    rcl_ret_t ret = rcl_action_take_goal_response(
        pimpl_->client_handle.get(), &response_header, goal_response.get());
    // 返回包含目标响应的元组（Return a tuple containing the goal response）
    return std::static_pointer_cast<void>(
        std::make_shared<std::tuple<rcl_ret_t, rmw_request_id_t, std::shared_ptr<void>>>(
            ret, response_header, goal_response));
  } else if (pimpl_->is_result_response_ready) {  // 判断结果响应是否准备好（Check if result
                                                  // response is ready）
    rmw_request_id_t response_header;
    // 创建一个结果响应（Create a result response）
    std::shared_ptr<void> result_response = this->create_result_response();
    // 获取结果响应（Get the result response）
    rcl_ret_t ret = rcl_action_take_result_response(
        pimpl_->client_handle.get(), &response_header, result_response.get());
    // 返回包含结果响应的元组（Return a tuple containing the result response）
    return std::static_pointer_cast<void>(
        std::make_shared<std::tuple<rcl_ret_t, rmw_request_id_t, std::shared_ptr<void>>>(
            ret, response_header, result_response));
  } else if (pimpl_->is_cancel_response_ready) {  // 判断取消响应是否准备好（Check if cancel
                                                  // response is ready）
    rmw_request_id_t response_header;
    // 创建一个取消响应（Create a cancel response）
    std::shared_ptr<void> cancel_response = this->create_cancel_response();
    // 获取取消响应（Get the cancel response）
    rcl_ret_t ret = rcl_action_take_cancel_response(
        pimpl_->client_handle.get(), &response_header, cancel_response.get());
    // 返回包含取消响应的元组（Return a tuple containing the cancel response）
    return std::static_pointer_cast<void>(
        std::make_shared<std::tuple<rcl_ret_t, rmw_request_id_t, std::shared_ptr<void>>>(
            ret, response_header, cancel_response));
  } else {  // 如果没有任何消息准备好，抛出运行时错误（If no messages are ready, throw a runtime
            // error）
    throw std::runtime_error("Taking data from action client but nothing is ready");
  }
}

/**
 * @brief 根据实体ID获取数据 (Take data by entity ID)
 *
 * @param id 实体ID (Entity ID)
 * @return std::shared_ptr<void> 返回获取到的数据 (Return the taken data)
 */
std::shared_ptr<void> ClientBase::take_data_by_entity_id(size_t id) {
  // 将我们想要获取数据的实体标记为 ready (Mark as ready the entity from which we want to take data)
  switch (static_cast<EntityType>(id)) {
    case EntityType::GoalClient:
      // 标记目标客户端为 ready (Mark goal client as ready)
      pimpl_->is_goal_response_ready = true;
      break;
    case EntityType::ResultClient:
      // 标记结果客户端为 ready (Mark result client as ready)
      pimpl_->is_result_response_ready = true;
      break;
    case EntityType::CancelClient:
      // 标记取消客户端为 ready (Mark cancel client as ready)
      pimpl_->is_cancel_response_ready = true;
      break;
    case EntityType::FeedbackSubscription:
      // 标记反馈订阅为 ready (Mark feedback subscription as ready)
      pimpl_->is_feedback_ready = true;
      break;
    case EntityType::StatusSubscription:
      // 标记状态订阅为 ready (Mark status subscription as ready)
      pimpl_->is_status_ready = true;
      break;
  }

  // 获取数据并返回 (Take data and return)
  return take_data();
}

/**
 * @brief 执行客户端操作 (Execute client operations)
 *
 * @param data 存储操作数据的智能指针 (A shared pointer storing the operation data)
 */
void ClientBase::execute(std::shared_ptr<void> &data) {
  // 检查数据是否为空 (Check if the data is empty)
  // Check if 'data' is empty
  if (!data) {
    throw std::runtime_error("'data' is empty");
  }

  // 处理反馈消息 (Handling feedback messages)
  // Handle feedback message
  if (pimpl_->is_feedback_ready) {
    auto shared_ptr = std::static_pointer_cast<std::tuple<rcl_ret_t, std::shared_ptr<void>>>(data);
    auto ret = std::get<0>(*shared_ptr);
    pimpl_->is_feedback_ready = false;
    if (RCL_RET_OK == ret) {
      auto feedback_message = std::get<1>(*shared_ptr);
      this->handle_feedback_message(feedback_message);
    } else if (RCL_RET_ACTION_CLIENT_TAKE_FAILED != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "error taking feedback");
    }
    // 处理状态消息 (Handling status messages)
    // Handle status message
  } else if (pimpl_->is_status_ready) {
    auto shared_ptr = std::static_pointer_cast<std::tuple<rcl_ret_t, std::shared_ptr<void>>>(data);
    auto ret = std::get<0>(*shared_ptr);
    pimpl_->is_status_ready = false;
    if (RCL_RET_OK == ret) {
      auto status_message = std::get<1>(*shared_ptr);
      this->handle_status_message(status_message);
    } else if (RCL_RET_ACTION_CLIENT_TAKE_FAILED != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "error taking status");
    }
    // 处理目标响应 (Handling goal responses)
    // Handle goal response
  } else if (pimpl_->is_goal_response_ready) {
    auto shared_ptr =
        std::static_pointer_cast<std::tuple<rcl_ret_t, rmw_request_id_t, std::shared_ptr<void>>>(
            data);
    auto ret = std::get<0>(*shared_ptr);
    pimpl_->is_goal_response_ready = false;
    if (RCL_RET_OK == ret) {
      auto response_header = std::get<1>(*shared_ptr);
      auto goal_response = std::get<2>(*shared_ptr);
      this->handle_goal_response(response_header, goal_response);
    } else if (RCL_RET_ACTION_CLIENT_TAKE_FAILED != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "error taking goal response");
    }
    // 处理结果响应 (Handling result responses)
    // Handle result response
  } else if (pimpl_->is_result_response_ready) {
    auto shared_ptr =
        std::static_pointer_cast<std::tuple<rcl_ret_t, rmw_request_id_t, std::shared_ptr<void>>>(
            data);
    auto ret = std::get<0>(*shared_ptr);
    pimpl_->is_result_response_ready = false;
    if (RCL_RET_OK == ret) {
      auto response_header = std::get<1>(*shared_ptr);
      auto result_response = std::get<2>(*shared_ptr);
      this->handle_result_response(response_header, result_response);
    } else if (RCL_RET_ACTION_CLIENT_TAKE_FAILED != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "error taking result response");
    }
    // 处理取消响应 (Handling cancel responses)
    // Handle cancel response
  } else if (pimpl_->is_cancel_response_ready) {
    auto shared_ptr =
        std::static_pointer_cast<std::tuple<rcl_ret_t, rmw_request_id_t, std::shared_ptr<void>>>(
            data);
    auto ret = std::get<0>(*shared_ptr);
    pimpl_->is_cancel_response_ready = false;
    if (RCL_RET_OK == ret) {
      auto response_header = std::get<1>(*shared_ptr);
      auto cancel_response = std::get<2>(*shared_ptr);
      this->handle_cancel_response(response_header, cancel_response);
    } else if (RCL_RET_ACTION_CLIENT_TAKE_FAILED != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "error taking cancel response");
    }
  } else {
    throw std::runtime_error("Executing action client but nothing is ready");
  }
}

}  // namespace rclcpp_action

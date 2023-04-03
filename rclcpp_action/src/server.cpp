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

#include "rclcpp_action/server.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "rcl_action/action_server.h"
#include "rcl_action/wait.h"
#include "rclcpp/exceptions.hpp"
#include "rcpputils/scope_exit.hpp"

using rclcpp_action::GoalUUID;
using rclcpp_action::ServerBase;

namespace rclcpp_action {
/**
 * @class ServerBaseImpl
 * @brief 基本服务器实现类（Base server implementation class）
 */
class ServerBaseImpl {
public:
  /**
   * @brief 构造函数，用于初始化服务器基类实现（Constructor for initializing the base server
   * implementation）
   * @param clock 共享指针类型的时钟对象，用于处理时间相关操作（Shared pointer of clock object for
   * handling time-related operations）
   * @param logger 日志记录器对象，用于输出日志信息（Logger object for outputting log information）
   */
  ServerBaseImpl(rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
      : clock_(clock),
        logger_(logger)  // 初始化列表：将传入的参数分别赋值给成员变量 clock_ 和
                         // logger_（Initialization list: assign the input parameters to member
                         // variables clock_ and logger_ respectively）
  {}

  // 用于保护 action_server_ 的递归互斥锁。 A recursive mutex for protecting access to
  // action_server_.
  std::recursive_mutex action_server_reentrant_mutex_;

  // rclcpp 时钟共享指针。 Shared pointer to an rclcpp clock.
  rclcpp::Clock::SharedPtr clock_;

  // 不要在 clock_ 前声明此变量，因为它依赖于 clock_(参见 #1526)。 Do not declare this before
  // clock_, as it depends on clock_ (see issue #1526).
  std::shared_ptr<rcl_action_server_t> action_server_;

  // 订阅数量。 Number of subscriptions.
  size_t num_subscriptions_ = 0;
  // 计时器数量。 Number of timers.
  size_t num_timers_ = 0;
  // 客户端数量。 Number of clients.
  size_t num_clients_ = 0;
  // 服务数量。 Number of services.
  size_t num_services_ = 0;
  // 守卫条件数量。 Number of guard conditions.
  size_t num_guard_conditions_ = 0;

  // 表示目标请求是否准备好的原子布尔值。 Atomic bool indicating if goal request is ready.
  std::atomic<bool> goal_request_ready_{false};
  // 表示取消请求是否准备好的原子布尔值。 Atomic bool indicating if cancel request is ready.
  std::atomic<bool> cancel_request_ready_{false};
  // 表示结果请求是否准备好的原子布尔值。 Atomic bool indicating if result request is ready.
  std::atomic<bool> result_request_ready_{false};
  // 表示目标是否过期的原子布尔值。 Atomic bool indicating if the goal has expired.
  std::atomic<bool> goal_expired_{false};

  // 用于保护无序映射的递归互斥锁。 A recursive mutex for protecting access to unordered_maps.
  std::recursive_mutex unordered_map_mutex_;

  // 在达到终止状态后，将保留结果直到目标过期。 Results are kept until the goal expires after
  // reaching a terminal state.
  std::unordered_map<GoalUUID, std::shared_ptr<void>> goal_results_;
  // 结果请求将保留，直到结果可用。 Requests for results are kept until a result becomes available.
  std::unordered_map<GoalUUID, std::vector<rmw_request_id_t>> result_requests_;
  // 保留 rcl 目标句柄，以便在发送结果时不尝试访问已释放的内存。 RCL goal handles are kept so that
  // the API to send results does not try to access freed memory.
  std::unordered_map<GoalUUID, std::shared_ptr<rcl_action_goal_handle_t>> goal_handles_;

  // rclcpp 日志记录器。 RCLCPP logger.
  rclcpp::Logger logger_;
};
}  // namespace rclcpp_action

/**
 * @brief ServerBase 构造函数 (ServerBase constructor)
 *
 * @param[in] node_base NodeBaseInterface 的共享指针 (Shared pointer of NodeBaseInterface)
 * @param[in] node_clock NodeClockInterface 的共享指针 (Shared pointer of NodeClockInterface)
 * @param[in] node_logging NodeLoggingInterface 的共享指针 (Shared pointer of NodeLoggingInterface)
 * @param[in] name 服务名称 (Service name)
 * @param[in] type_support 操作类型支持 (Action type support)
 * @param[in] options 服务器选项 (Server options)
 */
ServerBase::ServerBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string &name,
    const rosidl_action_type_support_t *type_support,
    const rcl_action_server_options_t &options)
    : pimpl_(new ServerBaseImpl(
          node_clock->get_clock(), node_logging->get_logger().get_child("rclcpp_action"))) {
  // 定义一个删除器，用于清理资源 (Define a deleter for cleaning up resources)
  auto deleter = [node_base](rcl_action_server_t *ptr) {
    if (nullptr != ptr) {
      rcl_node_t *rcl_node = node_base->get_rcl_node_handle();
      rcl_ret_t ret = rcl_action_server_fini(ptr, rcl_node);
      if (RCL_RET_OK != ret) {
        RCLCPP_DEBUG(
            rclcpp::get_logger("rclcpp_action"), "failed to fini rcl_action_server_t in deleter");
      }
      delete ptr;
    }
  };

  // 为 action_server_ 分配内存并设置删除器 (Allocate memory for action_server_ and set the deleter)
  pimpl_->action_server_.reset(new rcl_action_server_t, deleter);
  *(pimpl_->action_server_) = rcl_action_get_zero_initialized_server();

  // 获取 RCL 节点和时钟句柄 (Get RCL node and clock handles)
  rcl_node_t *rcl_node = node_base->get_rcl_node_handle();
  rcl_clock_t *rcl_clock = pimpl_->clock_->get_clock_handle();

  // 初始化 action_server_ (Initialize action_server_)
  rcl_ret_t ret = rcl_action_server_init(
      pimpl_->action_server_.get(), rcl_node, rcl_clock, type_support, name.c_str(), &options);

  // 如果初始化失败，抛出异常 (Throw exception if initialization fails)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // 获取实体数量 (Get the number of entities)
  ret = rcl_action_server_wait_set_get_num_entities(
      pimpl_->action_server_.get(), &pimpl_->num_subscriptions_, &pimpl_->num_guard_conditions_,
      &pimpl_->num_timers_, &pimpl_->num_clients_, &pimpl_->num_services_);

  // 如果获取实体数量失败，抛出异常 (Throw exception if getting the number of entities fails)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

/**
 * @brief 析构函数 (Destructor)
 */
ServerBase::~ServerBase() {}

/**
 * @brief 获取准备好的订阅数量 (Get the number of ready subscriptions)
 * @return 准备好的订阅数量 (Number of ready subscriptions)
 */
size_t ServerBase::get_number_of_ready_subscriptions() { return pimpl_->num_subscriptions_; }

/**
 * @brief 获取准备好的定时器数量 (Get the number of ready timers)
 * @return 准备好的定时器数量 (Number of ready timers)
 */
size_t ServerBase::get_number_of_ready_timers() { return pimpl_->num_timers_; }

/**
 * @brief 获取准备好的客户端数量 (Get the number of ready clients)
 * @return 准备好的客户端数量 (Number of ready clients)
 */
size_t ServerBase::get_number_of_ready_clients() { return pimpl_->num_clients_; }

/**
 * @brief 获取准备好的服务数量 (Get the number of ready services)
 * @return 准备好的服务数量 (Number of ready services)
 */
size_t ServerBase::get_number_of_ready_services() { return pimpl_->num_services_; }

/**
 * @brief 获取准备好的保护条件数量 (Get the number of ready guard conditions)
 * @return 准备好的保护条件数量 (Number of ready guard conditions)
 */
size_t ServerBase::get_number_of_ready_guard_conditions() { return pimpl_->num_guard_conditions_; }

/**
 * @brief 将动作服务器添加到等待集 (Add the action server to the wait set)
 * @param wait_set 等待集指针 (Pointer to the wait set)
 */
void ServerBase::add_to_wait_set(rcl_wait_set_t *wait_set) {
  // 对动作服务器进行互斥访问 (Lock the action server for mutually exclusive access)
  std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);

  // 将动作服务器添加到等待集 (Add the action server to the wait set)
  rcl_ret_t ret =
      rcl_action_wait_set_add_action_server(wait_set, pimpl_->action_server_.get(), NULL);

  // 检查是否成功添加到等待集，如果不成功则抛出异常 (Check if successfully added to the wait set,
  // throw an exception if not)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "ServerBase::add_to_wait_set() failed");
  }
}

/**
 * @brief 判断服务器是否准备好处理请求（Determine if the server is ready to process requests）
 *
 * @param[in] wait_set 指向 rcl_wait_set_t 结构体的指针，用于检查实体是否准备好 (Pointer to an
 * rcl_wait_set_t structure for checking if entities are ready)
 * @return 返回服务器是否准备好处理请求 (Return true if the server is ready to process requests,
 * false otherwise)
 */
bool ServerBase::is_ready(rcl_wait_set_t *wait_set) {
  // 定义各种请求类型的就绪状态变量 (Define variables for the readiness status of various request
  // types)
  bool goal_request_ready;
  bool cancel_request_ready;
  bool result_request_ready;
  bool goal_expired;
  rcl_ret_t ret;

  // 使用互斥锁保护操作，避免多线程冲突 (Use a mutex lock to protect operations and avoid
  // multithreading conflicts)
  {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    // 获取 Action 服务器中各实体的就绪状态 (Get the readiness status of the entities in the Action
    // server)
    ret = rcl_action_server_wait_set_get_entities_ready(
        wait_set, pimpl_->action_server_.get(), &goal_request_ready, &cancel_request_ready,
        &result_request_ready, &goal_expired);
  }

  // 更新内部状态变量 (Update internal state variables)
  pimpl_->goal_request_ready_ = goal_request_ready;
  pimpl_->cancel_request_ready_ = cancel_request_ready;
  pimpl_->result_request_ready_ = result_request_ready;
  pimpl_->goal_expired_ = goal_expired;

  // 检查返回值，如果不是 RCL_RET_OK，则抛出异常 (Check the return value, and if it's not
  // RCL_RET_OK, throw an exception)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // 如果任何一个请求类型的实体准备好了，返回 true，否则返回 false (Return true if any of the
  // request type entities are ready, false otherwise)
  return pimpl_->goal_request_ready_.load() || pimpl_->cancel_request_ready_.load() ||
         pimpl_->result_request_ready_.load() || pimpl_->goal_expired_.load();
}

/**
 * @brief 从 Action Server 获取请求数据
 * @brief Retrieve request data from the Action Server
 *
 * @return std::shared_ptr<void> 包含请求数据的智能指针
 * @return std::shared_ptr<void> A smart pointer containing the request data
 */
std::shared_ptr<void> ServerBase::take_data() {
  // 检查目标请求是否准备好
  // Check if the goal request is ready
  if (pimpl_->goal_request_ready_.load()) {
    rcl_ret_t ret;
    // 初始化目标信息结构体
    // Initialize the goal info structure
    rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
    rmw_request_id_t request_header;

    // 使用递归互斥锁保护资源
    // Use a recursive mutex to protect resources
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);

    // 创建目标请求，并获取请求数据
    // Create the goal request and retrieve the request data
    std::shared_ptr<void> message = create_goal_request();
    ret =
        rcl_action_take_goal_request(pimpl_->action_server_.get(), &request_header, message.get());

    // 返回包含请求数据的智能指针
    // Return a smart pointer containing the request data
    return std::static_pointer_cast<void>(
        std::make_shared<
            std::tuple<rcl_ret_t, rcl_action_goal_info_t, rmw_request_id_t, std::shared_ptr<void>>>(
            ret, goal_info, request_header, message));
    // 检查取消请求是否准备好
    // Check if the cancel request is ready
  } else if (pimpl_->cancel_request_ready_.load()) {
    rcl_ret_t ret;
    rmw_request_id_t request_header;

    // 初始化取消请求
    // Initialize the cancel request
    auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();

    // 使用递归互斥锁保护资源
    // Use a recursive mutex to protect resources
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    ret = rcl_action_take_cancel_request(
        pimpl_->action_server_.get(), &request_header, request.get());

    // 返回包含请求数据的智能指针
    // Return a smart pointer containing the request data
    return std::static_pointer_cast<void>(
        std::make_shared<std::tuple<
            rcl_ret_t, std::shared_ptr<action_msgs::srv::CancelGoal::Request>, rmw_request_id_t>>(
            ret, request, request_header));
    // 检查结果请求是否准备好
    // Check if the result request is ready
  } else if (pimpl_->result_request_ready_.load()) {
    rcl_ret_t ret;
    // 获取结果请求消息
    // Get the result request message
    rmw_request_id_t request_header;
    std::shared_ptr<void> result_request = create_result_request();
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    ret = rcl_action_take_result_request(
        pimpl_->action_server_.get(), &request_header, result_request.get());

    // 返回包含请求数据的智能指针
    // Return a smart pointer containing the request data
    return std::static_pointer_cast<void>(
        std::make_shared<std::tuple<rcl_ret_t, std::shared_ptr<void>, rmw_request_id_t>>(
            ret, result_request, request_header));
    // 检查目标是否过期
    // Check if the goal has expired
  } else if (pimpl_->goal_expired_.load()) {
    return nullptr;
  } else {
    // 如果没有准备好的数据，抛出运行时错误
    // Throw a runtime error if no data is ready
    throw std::runtime_error("Taking data from action server but nothing is ready");
  }
}

/**
 * @brief 根据实体ID获取数据 (Take data by entity ID)
 *
 * @param id 实体ID (Entity ID)
 * @return std::shared_ptr<void> 返回共享指针类型的数据 (Return shared pointer type of data)
 */
std::shared_ptr<void> ServerBase::take_data_by_entity_id(size_t id) {
  // 标记我们想要获取数据的实体为准备好状态 (Mark as ready the entity from which we want to take
  // data)
  switch (static_cast<EntityType>(id)) {
    case EntityType::GoalService:
      // 设置目标请求为准备好状态 (Set goal request as ready)
      pimpl_->goal_request_ready_ = true;
      break;
    case EntityType::ResultService:
      // 设置结果请求为准备好状态 (Set result request as ready)
      pimpl_->result_request_ready_ = true;
      break;
    case EntityType::CancelService:
      // 设置取消请求为准备好状态 (Set cancel request as ready)
      pimpl_->cancel_request_ready_ = true;
      break;
  }

  // 获取数据 (Take data)
  return take_data();
}

/**
 * @brief 执行操作服务器 (Execute action server)
 *
 * @param data 共享指针类型的数据 (Shared pointer type of data)
 */
void ServerBase::execute(std::shared_ptr<void> &data) {
  // 如果数据为空且目标没有过期，则抛出异常 (If data is empty and goal is not expired, throw an
  // exception)
  if (!data && !pimpl_->goal_expired_.load()) {
    throw std::runtime_error("'data' is empty");
  }

  // 判断哪种请求类型已经准备好 (Determine which request type is ready)
  if (pimpl_->goal_request_ready_.load()) {
    // 执行目标请求接收处理 (Execute goal request received processing)
    execute_goal_request_received(data);
  } else if (pimpl_->cancel_request_ready_.load()) {
    // 执行取消请求接收处理 (Execute cancel request received processing)
    execute_cancel_request_received(data);
  } else if (pimpl_->result_request_ready_.load()) {
    // 执行结果请求接收处理 (Execute result request received processing)
    execute_result_request_received(data);
  } else if (pimpl_->goal_expired_.load()) {
    // 检查过期的目标并执行相应操作 (Check expired goals and execute corresponding actions)
    execute_check_expired_goals();
  } else {
    // 如果没有任何请求准备好，抛出异常 (If nothing is ready, throw an exception)
    throw std::runtime_error("Executing action server but nothing is ready");
  }
}

/**
 * @brief 执行收到的目标请求 (Execute received goal request)
 *
 * @param data 包含目标请求信息的共享指针 (A shared pointer containing goal request information)
 */
void ServerBase::execute_goal_request_received(std::shared_ptr<void> &data) {
  // 将输入数据转换为适当的类型 (Convert input data to appropriate type)
  auto shared_ptr = std::static_pointer_cast<
      std::tuple<rcl_ret_t, rcl_action_goal_info_t, rmw_request_id_t, std::shared_ptr<void>>>(data);

  // 获取返回值 (Get return value)
  rcl_ret_t ret = std::get<0>(*shared_ptr);

  // 检查操作服务器接收失败 (Check for action server take failure)
  if (RCL_RET_ACTION_SERVER_TAKE_FAILED == ret) {
    // 忽略接收失败，因为 Connext 在收到没有有效数据的样本时会失败。
    // 当客户端关闭，Connext 收到表示客户端不再存在的样本时，就会发生这种情况。
    // (Ignore take failure because Connext fails if it receives a sample without valid data.
    // This happens when a client shuts down and Connext receives a sample saying the client is
    // no longer alive.)
    return;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // 获取目标信息、请求头和消息 (Get goal info, request header and message)
  rcl_action_goal_info_t goal_info = std::get<1>(*shared_ptr);
  rmw_request_id_t request_header = std::get<2>(*shared_ptr);
  std::shared_ptr<void> message = std::get<3>(*shared_ptr);

  // 检查目标请求是否准备好 (Check if goal request is ready)
  bool expected = true;
  if (!pimpl_->goal_request_ready_.compare_exchange_strong(expected, false)) {
    return;
  }

  // 获取目标请求中的目标 ID (Get goal ID from goal request)
  GoalUUID uuid = get_goal_id_from_goal_request(message.get());
  convert(uuid, &goal_info);

  // 调用用户回调，获取用户响应和要发送回的 ROS 消息
  // (Call user's callback, getting the user's response and a ros message to send back)
  auto response_pair = call_handle_goal_callback(uuid, message);

  {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    ret = rcl_action_send_goal_response(
        pimpl_->action_server_.get(), &request_header, response_pair.second.get());
  }

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  const auto status = response_pair.first;

  // 如果目标被接受，创建一个目标句柄，并存储它
  // (If goal is accepted, create a goal handle, and store it)
  if (GoalResponse::ACCEPT_AND_EXECUTE == status || GoalResponse::ACCEPT_AND_DEFER == status) {
    RCLCPP_DEBUG(pimpl_->logger_, "Accepted goal %s", to_string(uuid).c_str());

    // rcl_action 将设置时间戳 (rcl_action will set time stamp)
    auto deleter = [](rcl_action_goal_handle_t *ptr) {
      if (nullptr != ptr) {
        rcl_ret_t fail_ret = rcl_action_goal_handle_fini(ptr);
        if (RCL_RET_OK != fail_ret) {
          RCLCPP_DEBUG(
              rclcpp::get_logger("rclcpp_action"),
              "failed to fini rcl_action_goal_handle_t in deleter");
        }
        delete ptr;
      }
    };
    rcl_action_goal_handle_t *rcl_handle;
    {
      std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
      rcl_handle = rcl_action_accept_new_goal(pimpl_->action_server_.get(), &goal_info);
    }
    if (!rcl_handle) {
      throw std::runtime_error("Failed to accept new goal\n");
    }

    std::shared_ptr<rcl_action_goal_handle_t> handle(new rcl_action_goal_handle_t, deleter);

    // 从操作服务器存储中复制目标句柄，因为当它完成时，操作服务器存储会消失
    // (Copy out goal handle since action server storage disappears when it is fini'd)
    *handle = *rcl_handle;

    {
      std::lock_guard<std::recursive_mutex> lock(pimpl_->unordered_map_mutex_);
      pimpl_->goal_handles_[uuid] = handle;
    }

    if (GoalResponse::ACCEPT_AND_EXECUTE == status) {
      // 改变状态为执行 (Change status to executing)
      ret = rcl_action_update_goal_state(handle.get(), GOAL_EVENT_EXECUTE);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }

    // 发布状态，因为目标的状态已经改变（已接受或已开始执行）
    // (Publish status since a goal's state has changed (was accepted or has begun execution))
    publish_status();

    // 告诉用户开始执行操作 (Tell user to start executing action)
    call_goal_accepted_callback(handle, uuid, message);
  }

  // 重置数据 (Reset data)
  data.reset();
}

/**
 * @brief 执行取消请求的处理函数
 * @param data 存储请求数据的共享指针
 *
 * @details 该函数处理 action server 收到的取消目标请求，尝试取消相关目标，并返回取消结果。
 */
void ServerBase::execute_cancel_request_received(std::shared_ptr<void> &data) {
  // 将通用指针转换为特定类型的元组指针
  // Convert the generic pointer to a specific tuple pointer
  auto shared_ptr = std::static_pointer_cast<std::tuple<
      rcl_ret_t, std::shared_ptr<action_msgs::srv::CancelGoal::Request>, rmw_request_id_t>>(data);

  // 获取操作结果
  // Get the operation result
  auto ret = std::get<0>(*shared_ptr);

  // 如果操作失败，则忽略并返回
  // If the operation fails, ignore and return
  if (RCL_RET_ACTION_SERVER_TAKE_FAILED == ret) {
    return;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // 获取请求和请求头信息
  // Get the request and request header information
  auto request = std::get<1>(*shared_ptr);
  auto request_header = std::get<2>(*shared_ptr);
  pimpl_->cancel_request_ready_ = false;

  // 将 C++ 消息转换为 C 消息
  // Convert C++ message to C message
  rcl_action_cancel_request_t cancel_request = rcl_action_get_zero_initialized_cancel_request();
  convert(request->goal_info.goal_id.uuid, &cancel_request.goal_info);
  cancel_request.goal_info.stamp.sec = request->goal_info.stamp.sec;
  cancel_request.goal_info.stamp.nanosec = request->goal_info.stamp.nanosec;

  // 获取需要尝试取消的目标信息列表
  // Get a list of goal info that should be attempted to be cancelled
  rcl_action_cancel_response_t cancel_response = rcl_action_get_zero_initialized_cancel_response();

  {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    ret = rcl_action_process_cancel_request(
        pimpl_->action_server_.get(), &cancel_request, &cancel_response);
  }

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  RCPPUTILS_SCOPE_EXIT({
    ret = rcl_action_cancel_response_fini(&cancel_response);
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(pimpl_->logger_, "Failed to fini cancel response");
    }
  });

  // 创建响应消息
  // Create response message
  auto response = std::make_shared<action_msgs::srv::CancelGoal::Response>();

  // 设置返回码
  // Set the return code
  response->return_code = cancel_response.msg.return_code;
  auto &goals = cancel_response.msg.goals_canceling;

  // 对于每个取消的目标，调用取消回调
  // For each canceled goal, call cancel callback
  for (size_t i = 0; i < goals.size; ++i) {
    const rcl_action_goal_info_t &goal_info = goals.data[i];
    GoalUUID uuid;
    convert(goal_info, &uuid);
    auto response_code = call_handle_cancel_callback(uuid);
    if (CancelResponse::ACCEPT == response_code) {
      action_msgs::msg::GoalInfo cpp_info;
      cpp_info.goal_id.uuid = uuid;
      cpp_info.stamp.sec = goal_info.stamp.sec;
      cpp_info.stamp.nanosec = goal_info.stamp.nanosec;
      response->goals_canceling.push_back(cpp_info);
    }
  }

  // 如果用户拒绝所有取消目标的请求，则认为顶层取消请求被拒绝
  // If the user rejects all individual requests to cancel goals,
  // then we consider the top-level cancel request as rejected.
  if (goals.size >= 1u && 0u == response->goals_canceling.size()) {
    response->return_code = action_msgs::srv::CancelGoal::Response::ERROR_REJECTED;
  }

  // 如果至少有一个目标状态发生了变化，发布新的状态消息
  // If at least one goal state changed, publish a new status message
  if (!response->goals_canceling.empty()) {
    publish_status();
  }

  {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    ret = rcl_action_send_cancel_response(
        pimpl_->action_server_.get(), &request_header, response.get());
  }

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // 重置数据指针
  // Reset data pointer
  data.reset();
}

/**
 * @brief 执行结果请求接收到时的回调函数
 * @param data 一个包含rcl_ret_t、结果请求指针和rmw_request_id_t的元组的共享指针
 *
 * @details
 * 当服务器收到结果请求时，此函数将被调用。它会检查目标是否存在，如果存在并且已经有结果可用，
 *          则立即发送结果。否则，它会将结果请求存储起来，以便稍后响应。
 */
void ServerBase::execute_result_request_received(std::shared_ptr<void> &data) {
  // 将data转换为包含rcl_ret_t、结果请求指针和rmw_request_id_t的元组的共享指针
  auto shared_ptr =
      std::static_pointer_cast<std::tuple<rcl_ret_t, std::shared_ptr<void>, rmw_request_id_t>>(
          data);
  auto ret = std::get<0>(*shared_ptr);
  if (RCL_RET_ACTION_SERVER_TAKE_FAILED == ret) {
    // 如果操作服务器接收失败，则忽略，因为Connext在接收到无效数据的样本时会失败。
    // 当客户端关闭，Connext接收到表明客户端不再活动的样本时，会发生这种情况。
    return;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  auto result_request = std::get<1>(*shared_ptr);
  auto request_header = std::get<2>(*shared_ptr);

  pimpl_->result_request_ready_ = false;
  std::shared_ptr<void> result_response;

  // 检查目标是否存在
  GoalUUID uuid = get_goal_id_from_result_request(result_request.get());
  rcl_action_goal_info_t goal_info;
  convert(uuid, &goal_info);
  bool goal_exists;
  {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    goal_exists = rcl_action_server_goal_exists(pimpl_->action_server_.get(), &goal_info);
  }
  if (!goal_exists) {
    // 目标不存在
    result_response = create_result_response(action_msgs::msg::GoalStatus::STATUS_UNKNOWN);
  } else {
    // 目标存在，检查结果是否已经可用
    std::lock_guard<std::recursive_mutex> lock(pimpl_->unordered_map_mutex_);
    auto iter = pimpl_->goal_results_.find(uuid);
    if (iter != pimpl_->goal_results_.end()) {
      result_response = iter->second;
    } else {
      // 存储请求以便稍后响应
      pimpl_->result_requests_[uuid].push_back(request_header);
    }
  }

  if (result_response) {
    // 现在发送结果
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    rcl_ret_t rcl_ret = rcl_action_send_result_response(
        pimpl_->action_server_.get(), &request_header, result_response.get());
    if (RCL_RET_OK != rcl_ret) {
      rclcpp::exceptions::throw_from_rcl_error(rcl_ret);
    }
  }
  data.reset();
}

/**
 * @brief 执行检查过期目标的操作 (Execute the operation of checking expired goals)
 *
 * @param[out] expired_goals 存储已过期目标信息的数组 (An array to store the information of expired
 * goals)
 * @param[in] num_expired 已过期目标的数量 (The number of expired goals)
 */
void ServerBase::execute_check_expired_goals() {
  // 为每次循环只预计有一个目标过期分配内存 (Allocate memory expecting only one goal to expire at a
  // time)
  rcl_action_goal_info_t expired_goals[1];
  size_t num_expired = 1;

  // 循环以防多个目标同时过期 (Loop in case more than 1 goal expired)
  while (num_expired > 0u) {
    rcl_ret_t ret;
    {
      // 使用锁保护 action_server 的数据结构 (Use lock to protect the data structure of
      // action_server)
      std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);

      // 检查并获取过期目标 (Check and get the expired goals)
      ret = rcl_action_expire_goals(pimpl_->action_server_.get(), expired_goals, 1, &num_expired);
    }

    // 检查返回值是否正确 (Check if the return value is correct)
    if (RCL_RET_OK != ret) {
      // 抛出异常 (Throw an exception)
      rclcpp::exceptions::throw_from_rcl_error(ret);
    } else if (num_expired) {
      // 有一个目标过期了 (A goal expired!)
      GoalUUID uuid;

      // 将过期目标转换为 UUID (Convert the expired goal to UUID)
      convert(expired_goals[0], &uuid);

      // 输出调试信息 (Output debug information)
      RCLCPP_DEBUG(pimpl_->logger_, "Expired goal %s", to_string(uuid).c_str());

      // 使用锁保护 unordered_map 的数据结构 (Use lock to protect the data structure of
      // unordered_map)
      std::lock_guard<std::recursive_mutex> lock(pimpl_->unordered_map_mutex_);

      // 从 goal_results_ 中删除过期目标 (Remove the expired goal from goal_results_)
      pimpl_->goal_results_.erase(uuid);

      // 从 result_requests_ 中删除过期目标 (Remove the expired goal from result_requests_)
      pimpl_->result_requests_.erase(uuid);

      // 从 goal_handles_ 中删除过期目标 (Remove the expired goal from goal_handles_)
      pimpl_->goal_handles_.erase(uuid);
    }
  }
}

/**
 * @brief 发布服务器状态 (Publish server status)
 */
void ServerBase::publish_status() {
  rcl_ret_t ret;

  // 我们需要在整个方法中持有锁，因为 rcl_action_server_get_goal_handles()
  // 返回指向目标数据的内部指针。 (We need to hold the lock across this entire method because
  // rcl_action_server_get_goal_handles() returns an internal pointer to the goal data.)
  std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);

  // 获取 C 动作服务器已知的所有目标句柄 (Get all goal handles known to C action server)
  rcl_action_goal_handle_t **goal_handles = NULL;
  size_t num_goals = 0;
  ret = rcl_action_server_get_goal_handles(pimpl_->action_server_.get(), &goal_handles, &num_goals);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  auto status_msg = std::make_shared<action_msgs::msg::GoalStatusArray>();
  status_msg->status_list.reserve(num_goals);
  // 使用目标及其状态填充 c++ 状态消息 (Populate a c++ status message with the goals and their
  // statuses)
  rcl_action_goal_status_array_t c_status_array =
      rcl_action_get_zero_initialized_goal_status_array();
  ret = rcl_action_get_goal_status_array(pimpl_->action_server_.get(), &c_status_array);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  RCPPUTILS_SCOPE_EXIT({
    ret = rcl_action_goal_status_array_fini(&c_status_array);
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(pimpl_->logger_, "Failed to fini status array message");
    }
  });

  for (size_t i = 0; i < c_status_array.msg.status_list.size; ++i) {
    auto &c_status_msg = c_status_array.msg.status_list.data[i];

    action_msgs::msg::GoalStatus msg;
    msg.status = c_status_msg.status;
    // 将 C 目标信息转换为 C++ 目标信息 (Convert C goal info to C++ goal info)
    convert(c_status_msg.goal_info, &msg.goal_info.goal_id.uuid);
    msg.goal_info.stamp.sec = c_status_msg.goal_info.stamp.sec;
    msg.goal_info.stamp.nanosec = c_status_msg.goal_info.stamp.nanosec;

    status_msg->status_list.push_back(msg);
  }

  // 通过状态发布器发布消息 (Publish the message through the status publisher)
  ret = rcl_action_publish_status(pimpl_->action_server_.get(), status_msg.get());

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

/**
 * @brief 发布目标结果 (Publish the goal result)
 *
 * @param uuid 目标的唯一标识符 (Unique identifier of the goal)
 * @param result_msg 结果消息的共享指针 (Shared pointer to the result message)
 */
void ServerBase::publish_result(const GoalUUID &uuid, std::shared_ptr<void> result_msg) {
  // 检查目标是否存在 (Check that the goal exists)
  rcl_action_goal_info_t goal_info;
  convert(uuid, &goal_info);  // 将 UUID 转换为 rcl_action_goal_info_t 类型 (Convert the UUID to an
                              // rcl_action_goal_info_t type)
  bool goal_exists;
  {
    // 加锁保护，防止多线程问题 (Lock guard for protection against multi-threading issues)
    std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
    // 判断目标是否存在 (Determine if the goal exists)
    goal_exists = rcl_action_server_goal_exists(pimpl_->action_server_.get(), &goal_info);
  }

  // 如果目标不存在，则抛出异常 (If the goal does not exist, throw an exception)
  if (!goal_exists) {
    throw std::runtime_error("Asked to publish result for goal that does not exist");
  }

  {
    /**
     * 注意：如果在其他块范围内同时锁定 unordered_map_mutex_ 和 action_server_reentrant_mutex_，
     * 可能会出现死锁问题。除非使用 std::scoped_lock，否则锁定顺序必须与当前一致。
     * (NOTE: There is a potential deadlock issue if both unordered_map_mutex_ and
     * action_server_reentrant_mutex_ locked in other block scopes. Unless using
     * std::scoped_lock, locking order must be consistent with the current.)
     *
     * 当前锁定顺序 (Current locking order):
     *
     *   1. unordered_map_mutex_
     *   2. action_server_reentrant_mutex_
     *
     */
    // 加锁保护 unordered_map (Lock guard for protection of unordered_map)
    std::lock_guard<std::recursive_mutex> unordered_map_lock(pimpl_->unordered_map_mutex_);
    // 将结果消息存储在目标结果映射中 (Store the result message in the goal results map)
    pimpl_->goal_results_[uuid] = result_msg;

    // 如果已经有客户端请求结果，则将结果发送给它们 (If there are clients who already asked for the
    // result, send it to them)
    auto iter = pimpl_->result_requests_.find(uuid);
    if (iter != pimpl_->result_requests_.end()) {
      // 加锁保护，防止多线程问题 (Lock guard for protection against multi-threading issues)
      std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);
      for (auto &request_header : iter->second) {
        // 发送结果响应 (Send the result response)
        rcl_ret_t ret = rcl_action_send_result_response(
            pimpl_->action_server_.get(), &request_header, result_msg.get());
        // 如果发送失败，抛出异常 (If sending fails, throw an exception)
        if (RCL_RET_OK != ret) {
          rclcpp::exceptions::throw_from_rcl_error(ret);
        }
      }
    }
  }
}

/**
 * @brief 通知目标终止状态 (Notify the goal terminal state)
 */
void ServerBase::notify_goal_terminal_state() {
  // 对互斥锁进行加锁，保证线程安全 (Lock the mutex to ensure thread safety)
  std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);

  // 通知目标已完成 (Notify that the goal is done)
  rcl_ret_t ret = rcl_action_notify_goal_done(pimpl_->action_server_.get());

  // 检查返回值是否为 RCL_RET_OK (Check if the return value is RCL_RET_OK)
  if (RCL_RET_OK != ret) {
    // 如果不是，则抛出异常 (If not, throw an exception)
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

/**
 * @brief 发布反馈信息 (Publish feedback message)
 *
 * @param feedback_msg 反馈信息指针 (Pointer to the feedback message)
 */
void ServerBase::publish_feedback(std::shared_ptr<void> feedback_msg) {
  // 对互斥锁进行加锁，保证线程安全 (Lock the mutex to ensure thread safety)
  std::lock_guard<std::recursive_mutex> lock(pimpl_->action_server_reentrant_mutex_);

  // 发布反馈信息 (Publish the feedback message)
  rcl_ret_t ret = rcl_action_publish_feedback(pimpl_->action_server_.get(), feedback_msg.get());

  // 检查返回值是否为 RCL_RET_OK (Check if the return value is RCL_RET_OK)
  if (RCL_RET_OK != ret) {
    // 如果不是，则抛出异常并附带错误信息 (If not, throw an exception with error information)
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to publish feedback");
  }
}

/**
 * @brief 设置准备好回调函数 (Set the on ready callback function)
 *
 * @param callback 回调函数 (Callback function)
 */
void ServerBase::set_on_ready_callback(std::function<void(size_t, int)> callback) {
  // 检查回调函数是否可调用 (Check if the callback function is callable)
  if (!callback) {
    // 如果不是，则抛出异常 (If not, throw an exception)
    throw std::invalid_argument(
        "The callback passed to set_on_ready_callback "
        "is not callable.");
  }

  // 分别为目标服务、结果服务和取消服务设置回调函数 (Set the callback function for GoalService,
  // ResultService, and CancelService respectively)
  set_callback_to_entity(EntityType::GoalService, callback);
  set_callback_to_entity(EntityType::ResultService, callback);
  set_callback_to_entity(EntityType::CancelService, callback);
}

/**
 * @brief 设置实体回调函数
 * @param entity_type 实体类型
 * @param callback 要设置的回调函数
 *
 * @details 该函数用于为实体设置回调函数，并在发生事件时调用。
 */
// Set the callback function for the entity
// @param entity_type The type of the entity
// @param callback The callback function to be set
//
// This function is used to set the callback function for the entity, and it will be called when an
// event occurs.
void ServerBase::set_callback_to_entity(
    EntityType entity_type, std::function<void(size_t, int)> callback) {
  // 绑定回调函数的整数标识符参数到此可等待对象的实体类型
  // Bind the int identifier argument to this waitable's entity types
  auto new_callback = [callback, entity_type, this](size_t number_of_events) {
    try {
      // 尝试调用回调函数，并将实体类型作为参数传递
      // Try to call the callback function and pass the entity type as a parameter
      callback(number_of_events, static_cast<int>(entity_type));
    } catch (const std::exception &exception) {
      // 如果捕获到异常，记录错误信息
      // If an exception is caught, log the error message
      RCLCPP_ERROR_STREAM(
          pimpl_->logger_,
          "rclcpp_action::ServerBase@"
              << this << " caught " << rmw::impl::cpp::demangle(exception)
              << " exception in user-provided callback for the 'on ready' callback: "
              << exception.what());
    } catch (...) {
      // 如果捕获到未处理的异常，记录错误信息
      // If an unhandled exception is caught, log the error message
      RCLCPP_ERROR_STREAM(
          pimpl_->logger_, "rclcpp_action::ServerBase@"
                               << this << " caught unhandled exception in user-provided callback "
                               << "for the 'on ready' callback");
    }
  };

  // 临时设置新的回调函数，同时替换旧的回调函数
  // This two-step setting prevents a gap where the old std::function has
  // been replaced but the middleware hasn't been told about the new one yet.
  set_on_ready_callback(
      entity_type,
      rclcpp::detail::cpp_callback_trampoline<decltype(new_callback), const void *, size_t>,
      static_cast<const void *>(&new_callback));

  std::lock_guard<std::recursive_mutex> lock(listener_mutex_);
  // 存储 std::function 以保持其作用域，并覆盖现有的回调函数
  // Store the std::function to keep it in scope and overwrite the existing callback function
  auto it = entity_type_to_on_ready_callback_.find(entity_type);

  if (it != entity_type_to_on_ready_callback_.end()) {
    it->second = new_callback;
  } else {
    entity_type_to_on_ready_callback_.emplace(entity_type, new_callback);
  }

  // 再次设置回调函数，现在使用永久存储
  // Set the callback function again, now using the permanent storage
  it = entity_type_to_on_ready_callback_.find(entity_type);

  if (it != entity_type_to_on_ready_callback_.end()) {
    auto &cb = it->second;
    set_on_ready_callback(
        entity_type,
        rclcpp::detail::cpp_callback_trampoline<decltype(it->second), const void *, size_t>,
        static_cast<const void *>(&cb));
  }

  // 标记回调函数已设置
  // Mark the callback function as set
  on_ready_callback_set_ = true;
}

/**
 * @brief 设置实体类型的回调函数 (Set the callback function for the specified entity type)
 *
 * @param entity_type 实体类型（目标服务、结果服务或取消服务）(Entity type: GoalService,
 * ResultService or CancelService)
 * @param callback 回调函数指针 (Callback function pointer)
 * @param user_data 用户数据指针，将传递给回调函数 (User data pointer that will be passed to the
 * callback function)
 */
void ServerBase::set_on_ready_callback(
    EntityType entity_type, rcl_event_callback_t callback, const void *user_data) {
  rcl_ret_t ret = RCL_RET_ERROR;  // 初始化返回值为错误 (Initialize return value as error)

  switch (entity_type) {          // 根据实体类型进行处理 (Handle based on entity type)
    case EntityType::GoalService: {
      // 设置目标服务回调 (Set goal service callback)
      ret = rcl_action_server_set_goal_service_callback(
          pimpl_->action_server_.get(), callback, user_data);
      break;
    }

    case EntityType::ResultService: {
      // 设置结果服务回调 (Set result service callback)
      ret = rcl_action_server_set_result_service_callback(
          pimpl_->action_server_.get(), callback, user_data);
      break;
    }

    case EntityType::CancelService: {
      // 设置取消服务回调 (Set cancel service callback)
      ret = rcl_action_server_set_cancel_service_callback(
          pimpl_->action_server_.get(), callback, user_data);
      break;
    }

    default:
      // 抛出未知实体类型异常 (Throw unknown entity type exception)
      throw std::runtime_error("ServerBase::set_on_ready_callback: Unknown entity type.");
      break;
  }

  // 检查返回值是否为 RCL_RET_OK (Check if return value is RCL_RET_OK)
  if (RCL_RET_OK != ret) {
    using rclcpp::exceptions::throw_from_rcl_error;
    // 抛出设置回调失败异常 (Throw exception for failed to set the callback)
    throw_from_rcl_error(ret, "failed to set the on ready callback for action client");
  }
}

/**
 * @brief 清除准备就绪的回调函数 (Clear the on_ready callbacks)
 */
void ServerBase::clear_on_ready_callback() {
  // 使用 lock_guard 对 listener_mutex_ 进行加锁，以确保线程安全 (Lock the listener_mutex_ with
  // lock_guard to ensure thread safety)
  std::lock_guard<std::recursive_mutex> lock(listener_mutex_);

  // 如果设置了 on_ready_callback_set_ (If on_ready_callback_set_ is set)
  if (on_ready_callback_set_) {
    // 将 GoalService 的回调函数设置为 nullptr (Set the callback of GoalService to nullptr)
    set_on_ready_callback(EntityType::GoalService, nullptr, nullptr);
    // 将 ResultService 的回调函数设置为 nullptr (Set the callback of ResultService to nullptr)
    set_on_ready_callback(EntityType::ResultService, nullptr, nullptr);
    // 将 CancelService 的回调函数设置为 nullptr (Set the callback of CancelService to nullptr)
    set_on_ready_callback(EntityType::CancelService, nullptr, nullptr);
    // 设置 on_ready_callback_set_ 为 false (Set on_ready_callback_set_ to false)
    on_ready_callback_set_ = false;
  }

  // 清空 entity_type_to_on_ready_callback_ 映射 (Clear the entity_type_to_on_ready_callback_ map)
  entity_type_to_on_ready_callback_.clear();
}

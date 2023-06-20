// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "lifecycle_node_interface_impl.hpp"

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/transition_description.hpp"
#include "lifecycle_msgs/msg/transition_event.h"  // for getting the c-typesupport
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rcl/error_handling.h"
#include "rcl/node.h"
#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rcl_lifecycle/transition_map.h"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/types.h"

namespace rclcpp_lifecycle {

/**
 * @brief 构造函数，用于初始化 LifecycleNodeInterfaceImpl 类的实例。
 * @param node_base_interface 一个共享指针，指向 NodeBaseInterface 类的实例。
 * @param node_services_interface 一个共享指针，指向 NodeServicesInterface 类的实例。
 */
LifecycleNode::LifecycleNodeInterfaceImpl::LifecycleNodeInterfaceImpl(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface)
    : node_base_interface_(node_base_interface),
      node_services_interface_(node_services_interface) {}

/**
 * @brief 析构函数，用于销毁 LifecycleNodeInterfaceImpl 类的实例。
 */
LifecycleNode::LifecycleNodeInterfaceImpl::~LifecycleNodeInterfaceImpl() {
  // 获取 rcl_node_t 类型的节点句柄
  rcl_node_t *node_handle = node_base_interface_->get_rcl_node_handle();

  rcl_ret_t ret;
  {
    // 对状态机互斥体进行加锁，以确保线程安全
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    // 销毁状态机
    ret = rcl_lifecycle_state_machine_fini(&state_machine_, node_handle);
  }

  // 检查销毁状态机是否成功，如果失败，输出致命错误日志
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_FATAL_NAMED("rclcpp_lifecycle", "failed to destroy rcl_state_machine");
  }
}

/**
 * @brief 初始化生命周期节点接口实现
 * @param enable_communication_interface 是否启用通信接口
 */
void LifecycleNode::LifecycleNodeInterfaceImpl::init(bool enable_communication_interface) {
  // 获取节点句柄
  rcl_node_t *node_handle = node_base_interface_->get_rcl_node_handle();

  // 获取节点选项
  const rcl_node_options_t *node_options =
      rcl_node_get_options(node_base_interface_->get_rcl_node_handle());

  // 获取默认状态机选项
  auto state_machine_options = rcl_lifecycle_get_default_state_machine_options();

  // 设置状态机选项的通信接口和分配器
  state_machine_options.enable_com_interface = enable_communication_interface;
  state_machine_options.allocator = node_options->allocator;

  // 初始化状态机，需要五个不同类型支持的发布者/服务
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  state_machine_ = rcl_lifecycle_get_zero_initialized_state_machine();

  // clang-format off
  // 调用初始化状态机函数
  // ROSIDL_GET_MSG_TYPE_SUPPORT函数是用于获取消息类型支持对象的函数，它的作用是为了获得消息类型支持库代码的起始指针。因为消息实例需要通
  // 过类型支持库（Type Support）来序列化和反序列化，所以在将消息发布到主题之前，需要先引用类型支持库来确保正确性和数据精度。ROSIDL_GET_MSG_TYPE_SUPPORT函数返回的是一个指针，该指针包含类型支持库的有关信息。这些信息可以用于在消息发布时指定所需的类型支持。因此，在ROS2系
  // 统中使用此函数来获得消息类型支持对象。具体使用方式请参考上面的示例代码。
  rcl_ret_t ret = rcl_lifecycle_state_machine_init(
      &state_machine_,  //
      node_handle,      //
      ROSIDL_GET_MSG_TYPE_SUPPORT(lifecycle_msgs, msg, TransitionEvent),
      rosidl_typesupport_cpp::get_service_type_support_handle<ChangeStateSrv>(),
      rosidl_typesupport_cpp::get_service_type_support_handle<GetStateSrv>(),
      rosidl_typesupport_cpp::get_service_type_support_handle<GetAvailableStatesSrv>(),
      rosidl_typesupport_cpp::get_service_type_support_handle<GetAvailableTransitionsSrv>(),
      rosidl_typesupport_cpp::get_service_type_support_handle<GetAvailableTransitionsSrv>(),
      &state_machine_options);
  // clang-format on

  // 检查状态机初始化是否成功
  if (ret != RCL_RET_OK) {
    throw std::runtime_error(
        std::string("Couldn't initialize state machine for node ") +
        node_base_interface_->get_name());
  }

  // 获取当前状态
  current_state_ = State(state_machine_.current_state);

  // 如果启用通信接口，创建相应的服务
  // 果然，从这个变量开启使能这个 lifecycle node 的能力
  if (enable_communication_interface) {
    {  // change_state
      auto cb = std::bind(
          &LifecycleNode::LifecycleNodeInterfaceImpl::on_change_state, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<ChangeStateSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_change_state_ = std::make_shared<rclcpp::Service<ChangeStateSrv>>(
          node_base_interface_->get_shared_rcl_node_handle(),
          &state_machine_.com_interface.srv_change_state, any_cb);
      node_services_interface_->add_service(
          std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_change_state_), nullptr);
    }

    {  // get_state
      auto cb = std::bind(
          &LifecycleNode::LifecycleNodeInterfaceImpl::on_get_state, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<GetStateSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_state_ = std::make_shared<rclcpp::Service<GetStateSrv>>(
          node_base_interface_->get_shared_rcl_node_handle(),
          &state_machine_.com_interface.srv_get_state, any_cb);
      node_services_interface_->add_service(
          std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_state_), nullptr);
    }

    {  // get_available_states
      auto cb = std::bind(
          &LifecycleNode::LifecycleNodeInterfaceImpl::on_get_available_states, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<GetAvailableStatesSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_available_states_ = std::make_shared<rclcpp::Service<GetAvailableStatesSrv>>(
          node_base_interface_->get_shared_rcl_node_handle(),
          &state_machine_.com_interface.srv_get_available_states, any_cb);
      node_services_interface_->add_service(
          std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_available_states_), nullptr);
    }

    {  // get_available_transitions
      auto cb = std::bind(
          &LifecycleNode::LifecycleNodeInterfaceImpl::on_get_available_transitions, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<GetAvailableTransitionsSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_available_transitions_ =
          std::make_shared<rclcpp::Service<GetAvailableTransitionsSrv>>(
              node_base_interface_->get_shared_rcl_node_handle(),
              &state_machine_.com_interface.srv_get_available_transitions, any_cb);
      node_services_interface_->add_service(
          std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_available_transitions_), nullptr);
    }

    {  // get_transition_graph
      auto cb = std::bind(
          &LifecycleNode::LifecycleNodeInterfaceImpl::on_get_transition_graph, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<GetAvailableTransitionsSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_transition_graph_ = std::make_shared<rclcpp::Service<GetAvailableTransitionsSrv>>(
          node_base_interface_->get_shared_rcl_node_handle(),
          &state_machine_.com_interface.srv_get_transition_graph, any_cb);
      node_services_interface_->add_service(
          std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_transition_graph_), nullptr);
    }
  }
}

/*!
 * \brief 注册回调函数
 * \param lifecycle_transition 生命周期转换的 ID
 * \param cb 回调函数，接收 State 类型参数并返回 CallbackReturn 类型结果
 * \return 返回注册成功状态(始终为 true)
 *
 * Register a callback function
 * \param lifecycle_transition The ID of the lifecycle transition
 * \param cb The callback function, which takes a State type parameter and returns a CallbackReturn
 * type result \return Return the registration success status (always true)
 */
bool LifecycleNode::LifecycleNodeInterfaceImpl::register_callback(
    std::uint8_t lifecycle_transition,
    std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)> &cb) {
  // 将回调函数存储到 cb_map_ 中，以便在生命周期转换时调用
  // Store the callback function in cb_map_ for invocation during lifecycle transitions
  cb_map_[lifecycle_transition] = cb;
  return true;
}

/*!
 * \brief 改变节点状态的处理函数
 * \param header 请求头
 * \param req 状态改变请求
 * \param resp 状态改变响应
 *
 * Handler function for changing node states
 * \param header Request header
 * \param req State change request
 * \param resp State change response
 */
void LifecycleNode::LifecycleNodeInterfaceImpl::on_change_state(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<ChangeStateSrv::Request> req,
    std::shared_ptr<ChangeStateSrv::Response> resp) {
  (void)header;
  std::uint8_t transition_id;
  {
    // 锁定状态机互斥量，防止多线程访问冲突
    // Lock the state machine mutex to prevent multi-threaded access conflicts
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      throw std::runtime_error("Can't get state. State machine is not initialized.");
    }

    // 从请求中获取转换 ID
    // Get the transition ID from the request
    transition_id = req->transition.id;
    // 如果请求中附带了标签，我们检查与此标签关联的转换
    // If there's a label attached to the request, we check the transition associated with this
    // label.
    if (req->transition.label.size() != 0) {
      auto rcl_transition = rcl_lifecycle_get_transition_by_label(
          state_machine_.current_state, req->transition.label.c_str());
      if (rcl_transition == nullptr) {
        resp->success = false;
        return;
      }
      // 将找到的转换 ID 赋值给 transition_id
      // Assign the found transition ID to transition_id
      transition_id = static_cast<std::uint8_t>(rcl_transition->id);
    }
  }

  node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code;
  // 调用 change_state 函数执行状态改变，并获取返回结果
  // Call the change_state function to perform the state change and get the return result
  auto ret = change_state(transition_id, cb_return_code);
  (void)ret;
  // TODO(karsten1987): Lifecycle msgs have to be extended to keep both returns
  // 1. return is the actual transition
  // 2. return is whether an error occurred or not
  // 根据回调函数的返回结果设置响应的成功状态
  // Set the success status of the response based on the return result of the callback function
  resp->success =
      (cb_return_code == node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

/**
 * @brief 获取当前状态的回调函数 (Callback function to get the current state)
 *
 * @param header 请求头信息 (Request header information)
 * @param req GetStateSrv::Request 类型的请求对象 (Request object of type GetStateSrv::Request)
 * @param resp GetStateSrv::Response 类型的响应对象 (Response object of type GetStateSrv::Response)
 */
void LifecycleNode::LifecycleNodeInterfaceImpl::on_get_state(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetStateSrv::Request> req,
    std::shared_ptr<GetStateSrv::Response> resp) const {
  (void)header;  // 忽略未使用的参数 (Ignore unused parameter)
  (void)req;     // 忽略未使用的参数 (Ignore unused parameter)

  // 对状态机互斥锁进行加锁 (Lock the state machine mutex)
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);

  // 检查状态机是否已初始化 (Check if the state machine is initialized)
  if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
    throw std::runtime_error("Can't get state. State machine is not initialized.");
  }

  // 设置响应中的当前状态 (Set the current state in the response)
  resp->current_state.id = static_cast<uint8_t>(state_machine_.current_state->id);
  resp->current_state.label = state_machine_.current_state->label;
}

/**
 * @brief 获取可用状态的回调函数 (Callback function to get available states)
 *
 * @param header 请求头信息 (Request header information)
 * @param req GetAvailableStatesSrv::Request 类型的请求对象 (Request object of type
 * GetAvailableStatesSrv::Request)
 * @param resp GetAvailableStatesSrv::Response 类型的响应对象 (Response object of type
 * GetAvailableStatesSrv::Response)
 */
void LifecycleNode::LifecycleNodeInterfaceImpl::on_get_available_states(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetAvailableStatesSrv::Request> req,
    std::shared_ptr<GetAvailableStatesSrv::Response> resp) const {
  (void)header;  // 忽略未使用的参数 (Ignore unused parameter)
  (void)req;     // 忽略未使用的参数 (Ignore unused parameter)

  // 对状态机互斥锁进行加锁 (Lock the state machine mutex)
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);

  // 检查状态机是否已初始化 (Check if the state machine is initialized)
  if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
    throw std::runtime_error("Can't get available states. State machine is not initialized.");
  }

  // 调整响应中可用状态向量的大小 (Resize the available states vector in the response)
  resp->available_states.resize(state_machine_.transition_map.states_size);

  // 遍历所有可用状态并设置响应 (Iterate through all available states and set the response)
  for (unsigned int i = 0; i < state_machine_.transition_map.states_size; ++i) {
    resp->available_states[i].id = static_cast<uint8_t>(state_machine_.transition_map.states[i].id);
    resp->available_states[i].label =
        static_cast<std::string>(state_machine_.transition_map.states[i].label);
  }
}

/**
 * @brief 获取当前状态可用的转换 (Get available transitions for the current state)
 *
 * @param header 请求头 (Request header)
 * @param req 请求 (Request)
 * @param resp 响应 (Response)
 */
void LifecycleNode::LifecycleNodeInterfaceImpl::on_get_available_transitions(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
    std::shared_ptr<GetAvailableTransitionsSrv::Response> resp) const {
  (void)header;               // 忽略 header 参数 (Ignore header parameter)
  (void)req;                  // 忽略 req 参数 (Ignore req parameter)
  std::lock_guard<std::recursive_mutex> lock(
      state_machine_mutex_);  // 加锁以保护状态机 (Lock to protect the state machine)

  // 检查状态机是否已初始化 (Check if the state machine is initialized)
  if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
    throw std::runtime_error("Can't get available transitions. State machine is not initialized.");
  }

  // 调整响应数组大小 (Resize the response array)
  resp->available_transitions.resize(state_machine_.current_state->valid_transition_size);

  // 遍历可用的转换 (Iterate through the available transitions)
  for (unsigned int i = 0; i < state_machine_.current_state->valid_transition_size; ++i) {
    lifecycle_msgs::msg::TransitionDescription &trans_desc = resp->available_transitions[i];

    // 获取 RCL 转换 (Get the RCL transition)
    auto rcl_transition = state_machine_.current_state->valid_transitions[i];

    // 设置转换描述 (Set the transition description)
    trans_desc.transition.id = static_cast<uint8_t>(rcl_transition.id);
    trans_desc.transition.label = rcl_transition.label;
    trans_desc.start_state.id = static_cast<uint8_t>(rcl_transition.start->id);
    trans_desc.start_state.label = rcl_transition.start->label;
    trans_desc.goal_state.id = static_cast<uint8_t>(rcl_transition.goal->id);
    trans_desc.goal_state.label = rcl_transition.goal->label;
  }
}

/**
 * @brief 获取所有状态的转换图 (Get the transition graph for all states)
 *
 * @param header 请求头 (Request header)
 * @param req 请求 (Request)
 * @param resp 响应 (Response)
 */
void LifecycleNode::LifecycleNodeInterfaceImpl::on_get_transition_graph(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
    std::shared_ptr<GetAvailableTransitionsSrv::Response> resp) const {
  (void)header;               // 忽略 header 参数 (Ignore header parameter)
  (void)req;                  // 忽略 req 参数 (Ignore req parameter)
  std::lock_guard<std::recursive_mutex> lock(
      state_machine_mutex_);  // 加锁以保护状态机 (Lock to protect the state machine)

  // 检查状态机是否已初始化 (Check if the state machine is initialized)
  if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
    throw std::runtime_error("Can't get available transitions. State machine is not initialized.");
  }

  // 调整响应数组大小 (Resize the response array)
  resp->available_transitions.resize(state_machine_.transition_map.transitions_size);

  // 遍历所有转换 (Iterate through all the transitions)
  for (unsigned int i = 0; i < state_machine_.transition_map.transitions_size; ++i) {
    lifecycle_msgs::msg::TransitionDescription &trans_desc = resp->available_transitions[i];

    // 获取 RCL 转换 (Get the RCL transition)
    auto rcl_transition = state_machine_.transition_map.transitions[i];

    // 设置转换描述 (Set the transition description)
    trans_desc.transition.id = static_cast<uint8_t>(rcl_transition.id);
    trans_desc.transition.label = rcl_transition.label;
    trans_desc.start_state.id = static_cast<uint8_t>(rcl_transition.start->id);
    trans_desc.start_state.label = rcl_transition.start->label;
    trans_desc.goal_state.id = static_cast<uint8_t>(rcl_transition.goal->id);
    trans_desc.goal_state.label = rcl_transition.goal->label;
  }
}

/**
 * @brief 获取当前状态 (Get the current state)
 *
 * @return 当前状态的引用 (A reference to the current state)
 */
const State &LifecycleNode::LifecycleNodeInterfaceImpl::get_current_state() const {
  // 返回当前状态 (Return the current state)
  return current_state_;
}

/**
 * @brief 获取可用状态列表 (Get the list of available states)
 *
 * @return 可用状态向量 (A vector of available states)
 */
std::vector<State> LifecycleNode::LifecycleNodeInterfaceImpl::get_available_states() const {
  // 创建状态向量 (Create a vector for states)
  std::vector<State> states;

  // 锁定状态机互斥锁 (Lock the state machine mutex)
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);

  // 预留状态向量空间 (Reserve space in the states vector)
  states.reserve(state_machine_.transition_map.states_size);

  // 遍历所有状态并添加到向量中 (Iterate through all states and add them to the vector)
  for (unsigned int i = 0; i < state_machine_.transition_map.states_size; ++i) {
    states.emplace_back(&state_machine_.transition_map.states[i]);
  }

  // 返回状态向量 (Return the states vector)
  return states;
}

/**
 * @brief 获取可用转换列表 (Get the list of available transitions)
 *
 * @return 可用转换向量 (A vector of available transitions)
 */
std::vector<Transition> LifecycleNode::LifecycleNodeInterfaceImpl::get_available_transitions()
    const {
  // 创建转换向量 (Create a vector for transitions)
  std::vector<Transition> transitions;

  // 锁定状态机互斥锁 (Lock the state machine mutex)
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);

  // 预留转换向量空间 (Reserve space in the transitions vector)
  transitions.reserve(state_machine_.current_state->valid_transition_size);

  // 遍历当前状态的所有有效转换并添加到向量中 (Iterate through all valid transitions for the current
  // state and add them to the vector)
  for (unsigned int i = 0; i < state_machine_.current_state->valid_transition_size; ++i) {
    transitions.emplace_back(&state_machine_.current_state->valid_transitions[i]);
  }

  // 返回转换向量 (Return the transitions vector)
  return transitions;
}

/**
 * @brief 获取转换图 (Get the transition graph)
 *
 * @return 转换向量 (A vector of transitions)
 */
std::vector<Transition> LifecycleNode::LifecycleNodeInterfaceImpl::get_transition_graph() const {
  // 创建转换向量 (Create a vector for transitions)
  std::vector<Transition> transitions;

  // 锁定状态机互斥锁 (Lock the state machine mutex)
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);

  // 预留转换向量空间 (Reserve space in the transitions vector)
  transitions.reserve(state_machine_.transition_map.transitions_size);

  // 遍历所有转换并添加到向量中 (Iterate through all transitions and add them to the vector)
  for (unsigned int i = 0; i < state_machine_.transition_map.transitions_size; ++i) {
    transitions.emplace_back(&state_machine_.transition_map.transitions[i]);
  }

  // 返回转换向量 (Return the transitions vector)
  return transitions;
}

/**
 * @brief 改变状态机的状态 (Change the state of the state machine)
 *
 * @param[in] transition_id 转换ID (Transition ID)
 * @param[out] cb_return_code 回调返回码 (Callback return code)
 * @return rcl_ret_t 返回状态 (Return status)
 */
rcl_ret_t LifecycleNode::LifecycleNodeInterfaceImpl::change_state(
    std::uint8_t transition_id,
    node_interfaces::LifecycleNodeInterface::CallbackReturn &cb_return_code) {
  // 是否发布状态更新 (Whether to publish status updates)
  constexpr bool publish_update = true;
  // 初始化状态变量 (Initialize state variables)
  State initial_state;
  unsigned int current_state_id;

  {
    // 使用递归互斥锁保护状态机 (Protect the state machine with a recursive mutex lock)
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    // 检查状态机是否已初始化 (Check if the state machine is initialized)
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      // 记录错误日志 (Log an error)
      RCUTILS_LOG_ERROR(
          "Unable to change state for state machine for %s: %s", node_base_interface_->get_name(),
          rcl_get_error_string().str);
      return RCL_RET_ERROR;
    }

    // 保留初始状态以传递给转换回调 (Keep the initial state to pass to a transition callback)
    initial_state = State(state_machine_.current_state);

    // 尝试触发指定ID的转换 (Attempt to trigger the transition with the specified ID)
    if (rcl_lifecycle_trigger_transition_by_id(&state_machine_, transition_id, publish_update) !=
        RCL_RET_OK) {
      // 记录错误日志 (Log an error)
      RCUTILS_LOG_ERROR(
          "Unable to start transition %u from current state %s: %s", transition_id,
          state_machine_.current_state->label, rcl_get_error_string().str);
      rcutils_reset_error();
      return RCL_RET_ERROR;
    }
    current_state_id = state_machine_.current_state->id;
  }

  // 更新内部的 current_state_ (Update the internal current_state_)
  current_state_ = State(state_machine_.current_state);

  // 定义获取回调返回码标签的函数 (Define a function to get the label for the callback return code)
  auto get_label_for_return_code =
      [](node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code) -> const char * {
    auto cb_id = static_cast<uint8_t>(cb_return_code);
    if (cb_id == lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS) {
      return rcl_lifecycle_transition_success_label;
    } else if (cb_id == lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE) {
      return rcl_lifecycle_transition_failure_label;
    }
    return rcl_lifecycle_transition_error_label;
  };

  // 执行回调并获取返回码 (Execute the callback and get the return code)
  cb_return_code = execute_callback(current_state_id, initial_state);
  auto transition_label = get_label_for_return_code(cb_return_code);

  {
    // 使用递归互斥锁保护状态机 (Protect the state machine with a recursive mutex lock)
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    // 尝试触发指定标签的转换 (Attempt to trigger the transition with the specified label)
    if (rcl_lifecycle_trigger_transition_by_label(
            &state_machine_, transition_label, publish_update) != RCL_RET_OK) {
      // 记录错误日志 (Log an error)
      RCUTILS_LOG_ERROR(
          "Failed to finish transition %u. Current state is now: %s (%s)", transition_id,
          state_machine_.current_state->label, rcl_get_error_string().str);
      rcutils_reset_error();
      return RCL_RET_ERROR;
    }
    current_state_id = state_machine_.current_state->id;
  }

  // 更新内部的 current_state_ (Update the internal current_state_)
  current_state_ = State(state_machine_.current_state);

  // 错误处理 (Error handling)
  // TODO(karsten1987): iterate over possible ret value
  if (cb_return_code == node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR) {
    // 记录警告日志 (Log a warning)
    RCUTILS_LOG_WARN("Error occurred while doing error handling.");

    // 执行回调并获取返回码 (Execute the callback and get the return code)
    auto error_cb_code = execute_callback(current_state_id, initial_state);
    auto error_cb_label = get_label_for_return_code(error_cb_code);
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    // 尝试触发指定标签的转换 (Attempt to trigger the transition with the specified label)
    if (rcl_lifecycle_trigger_transition_by_label(
            &state_machine_, error_cb_label, publish_update) != RCL_RET_OK) {
      // 记录错误日志 (Log an error)
      RCUTILS_LOG_ERROR("Failed to call cleanup on error state: %s", rcl_get_error_string().str);
      rcutils_reset_error();
      return RCL_RET_ERROR;
    }
  }

  // 更新内部的 current_state_ (Update the internal current_state_)
  current_state_ = State(state_machine_.current_state);

  // 如果实际回调成功或失败，都有有效的转换到新的主状态或错误状态 (If the actual callback was
  // successful or not, there is a valid transition to either a new primary state or error state)
  return RCL_RET_OK;
}

/**
 * @brief 执行生命周期回调函数 (Execute the lifecycle callback function)
 *
 * @param[in] cb_id 回调函数ID (Callback function ID)
 * @param[in] previous_state 上一个状态 (Previous state)
 * @return node_interfaces::LifecycleNodeInterface::CallbackReturn 返回执行结果 (Return execution
 * result)
 */
node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleNode::LifecycleNodeInterfaceImpl::execute_callback(
    unsigned int cb_id, const State &previous_state) const {
  // 如果没有附加回调，则直接转发 (Forward directly in case no callback was attached)
  auto cb_success = node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

  // 查找回调函数映射中的回调函数ID (Find the callback function ID in the callback function mapping)
  auto it = cb_map_.find(static_cast<uint8_t>(cb_id));
  if (it != cb_map_.end()) {
    auto callback = it->second;
    try {
      // 执行回调函数并获取执行结果 (Execute the callback function and get the execution result)
      cb_success = callback(State(previous_state));
    } catch (const std::exception &e) {
      // 捕获异常并记录错误日志 (Catch exception and log error)
      RCUTILS_LOG_ERROR("Caught exception in callback for transition %d", it->first);
      RCUTILS_LOG_ERROR("Original error: %s", e.what());
      cb_success = node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }
  return cb_success;
}

/**
 * @brief 触发状态转换 (Trigger state transition)
 *
 * @param[in] transition_label 转换标签 (Transition label)
 * @return const State& 当前状态 (Current state)
 */
const State &LifecycleNode::LifecycleNodeInterfaceImpl::trigger_transition(
    const char *transition_label) {
  node_interfaces::LifecycleNodeInterface::CallbackReturn error;
  return trigger_transition(transition_label, error);
}

/**
 * @brief 触发状态转换并返回回调执行结果 (Trigger state transition and return callback execution
 * result)
 *
 * @param[in] transition_label 转换标签 (Transition label)
 * @param[out] cb_return_code 回调执行结果 (Callback execution result)
 * @return const State& 当前状态 (Current state)
 */
const State &LifecycleNode::LifecycleNodeInterfaceImpl::trigger_transition(
    const char *transition_label,
    node_interfaces::LifecycleNodeInterface::CallbackReturn &cb_return_code) {
  const rcl_lifecycle_transition_t *transition;
  {
    // 加锁以避免多线程竞争条件 (Lock to avoid multi-threading race conditions)
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);

    // 通过转换标签获取转换 (Get the transition by transition label)
    transition =
        rcl_lifecycle_get_transition_by_label(state_machine_.current_state, transition_label);
  }
  if (transition) {
    // 改变状态并获取回调执行结果 (Change state and get callback execution result)
    change_state(static_cast<uint8_t>(transition->id), cb_return_code);
  }
  return get_current_state();
}

/**
 * @brief 触发生命周期节点的状态转换(Trigger a transition in the lifecycle node's state)
 * @param transition_id 转换的ID(The ID of the transition to trigger)
 * @return 返回当前状态(Return the current state)
 */
const State &LifecycleNode::LifecycleNodeInterfaceImpl::trigger_transition(uint8_t transition_id) {
  // 定义错误变量(Define error variable)
  node_interfaces::LifecycleNodeInterface::CallbackReturn error;

  // 改变状态(Change state)
  change_state(transition_id, error);

  // 忽略错误变量(Ignore error variable)
  (void)error;

  // 返回当前状态(Return the current state)
  return get_current_state();
}

/**
 * @brief 触发生命周期节点的状态转换，并返回回调代码(Trigger a transition in the lifecycle node's
 * state and return callback code)
 * @param transition_id 转换的ID(The ID of the transition to trigger)
 * @param cb_return_code 回调返回代码(Callback return code)
 * @return 返回当前状态(Return the current state)
 */
const State &LifecycleNode::LifecycleNodeInterfaceImpl::trigger_transition(
    uint8_t transition_id,
    node_interfaces::LifecycleNodeInterface::CallbackReturn &cb_return_code) {
  // 改变状态(Change state)
  change_state(transition_id, cb_return_code);

  // 返回当前状态(Return the current state)
  return get_current_state();
}

/**
 * @brief 添加管理实体(Add managed entity)
 * @param managed_entity 管理实体的弱指针(Weak pointer to the managed entity)
 */
void LifecycleNode::LifecycleNodeInterfaceImpl::add_managed_entity(
    std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface> managed_entity) {
  // 将管理实体添加到列表中(Add the managed entity to the list)
  weak_managed_entities_.push_back(managed_entity);
}

/**
 * @brief 添加定时器句柄(Add timer handle)
 * @param timer 定时器的共享指针(Shared pointer to the timer)
 */
void LifecycleNode::LifecycleNodeInterfaceImpl::add_timer_handle(
    std::shared_ptr<rclcpp::TimerBase> timer) {
  // 将定时器添加到列表中(Add the timer to the list)
  weak_timers_.push_back(timer);
}

/**
 * @brief 激活生命周期节点(Activate lifecycle node)
 */
void LifecycleNode::LifecycleNodeInterfaceImpl::on_activate() const {
  // 遍历所有弱管理实体(Iterate through all weak managed entities)
  for (const auto &weak_entity : weak_managed_entities_) {
    // 获取强实体指针(Get strong entity pointer)
    auto entity = weak_entity.lock();

    // 如果实体存在(If the entity exists)
    if (entity) {
      // 激活实体(Activate the entity)
      entity->on_activate();
    }
  }
}

/**
 * @brief 使生命周期节点的所有托管实体处于非活动状态 (Deactivate all managed entities of the
 * lifecycle node)
 *
 * @param[in] 无参数 (No parameters)
 * @return 无返回值 (No return value)
 */
void LifecycleNode::LifecycleNodeInterfaceImpl::on_deactivate() const {
  // 遍历 weak_managed_entities_ 中的所有弱引用实体 (Iterate through all weak reference entities in
  // weak_managed_entities_)
  for (const auto &weak_entity : weak_managed_entities_) {
    // 尝试从弱引用中获取实体的共享指针 (Attempt to obtain a shared pointer to the entity from the
    // weak reference)
    auto entity = weak_entity.lock();

    // 检查实体是否仍然存在 (Check if the entity still exists)
    if (entity) {
      // 如果实体存在，则调用其 on_deactivate() 方法将其置为非活动状态 (If the entity exists, call
      // its on_deactivate() method to set it to an inactive state)
      entity->on_deactivate();
    }
  }
}

}  // namespace rclcpp_lifecycle

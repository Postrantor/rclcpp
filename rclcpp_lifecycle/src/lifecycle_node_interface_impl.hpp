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

#ifndef LIFECYCLE_NODE_INTERFACE_IMPL_HPP_
#define LIFECYCLE_NODE_INTERFACE_IMPL_HPP_

#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rmw/types.h"

namespace rclcpp_lifecycle {

/**
 * @brief 生命周期节点接口实现类 (Lifecycle Node Interface Implementation class)
 */
class LifecycleNode::LifecycleNodeInterfaceImpl final {
  /// 使用 ChangeStateSrv 类型定义 ChangeStateSrv（用于更改状态的服务）(Using ChangeStateSrv type to
  /// define ChangeStateSrv (Service for changing state))
  using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;
  /// 使用 GetStateSrv 类型定义 GetStateSrv（用于获取当前状态的服务）(Using GetStateSrv type to
  /// define GetStateSrv (Service for getting current state))
  using GetStateSrv = lifecycle_msgs::srv::GetState;
  /// 使用 GetAvailableStatesSrv 类型定义 GetAvailableStatesSrv（用于获取可用状态的服务）(Using
  /// GetAvailableStatesSrv type to define GetAvailableStatesSrv (Service for getting available
  /// states))
  using GetAvailableStatesSrv = lifecycle_msgs::srv::GetAvailableStates;
  /// 使用 GetAvailableTransitionsSrv 类型定义
  /// GetAvailableTransitionsSrv（用于获取可用转换的服务）(Using GetAvailableTransitionsSrv type to
  /// define GetAvailableTransitionsSrv (Service for getting available transitions))
  using GetAvailableTransitionsSrv = lifecycle_msgs::srv::GetAvailableTransitions;
  /// 使用 TransitionEventMsg 类型定义 TransitionEventMsg（用于表示转换事件的消息）(Using
  /// TransitionEventMsg type to define TransitionEventMsg (Message representing transition event))
  using TransitionEventMsg = lifecycle_msgs::msg::TransitionEvent;

public:
  /**
   * @brief 生命周期节点接口实现类构造函数 (Lifecycle Node Interface Implementation class
   * constructor)
   *
   * @param node_base_interface 节点基础接口的共享指针 (Shared pointer to the Node Base Interface)
   * @param node_services_interface 节点服务接口的共享指针 (Shared pointer to the Node Services
   * Interface)
   */
  LifecycleNodeInterfaceImpl(
      std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
      std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface);

  /**
   * @brief 生命周期节点接口实现类析构函数 (Lifecycle Node Interface Implementation class
   * destructor)
   */
  ~LifecycleNodeInterfaceImpl();

  /**
   * @brief 初始化生命周期节点接口 (Initialize the Lifecycle Node Interface)
   *
   * @param enable_communication_interface 是否启用通信接口，默认为 true (Whether to enable
   * communication interface, default is true)
   */
  void init(bool enable_communication_interface = true);

  /**
   * @brief 注册回调函数 (Register callback function)
   *
   * @param lifecycle_transition 生命周期转换 (Lifecycle transition)
   * @param cb 回调函数 (Callback function)
   * @return 是否成功注册回调函数 (Whether the callback function is successfully registered)
   */
  bool register_callback(
      std::uint8_t lifecycle_transition,
      std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)> &cb);

  /**
   * @brief 获取当前状态 (Get current state)
   *
   * @return 当前状态的引用 (Reference to the current state)
   */
  const State &get_current_state() const;

  /**
   * @brief 获取可用状态 (Get available states)
   *
   * @return 可用状态的向量 (Vector of available states)
   */
  std::vector<State> get_available_states() const;

  /**
   * @brief 获取可用转换 (Get available transitions)
   *
   * @return 可用转换的向量 (Vector of available transitions)
   */
  std::vector<Transition> get_available_transitions() const;

  /**
   * @brief 获取转换图 (Get transition graph)
   *
   * @return 转换图的向量 (Vector of the transition graph)
   */
  std::vector<Transition> get_transition_graph() const;

  /**
   * @brief 触发转换 (Trigger transition)
   *
   * @param transition_id 转换 ID (Transition ID)
   * @return 状态的引用 (Reference to the state)
   */
  const State &trigger_transition(uint8_t transition_id);

  /**
   * @brief 触发转换并返回回调函数的返回代码 (Trigger transition and return callback function's
   * return code)
   *
   * @param transition_id 转换 ID (Transition ID)
   * @param cb_return_code 回调函数的返回代码 (Return code of the callback function)
   * @return 状态的引用 (Reference to the state)
   */
  const State &trigger_transition(
      uint8_t transition_id,
      node_interfaces::LifecycleNodeInterface::CallbackReturn &cb_return_code);

  /**
   * @brief 根据转换标签触发转换 (Trigger transition by transition label)
   *
   * @param transition_label 转换标签 (Transition label)
   * @return 状态的引用 (Reference to the state)
   */
  const State &trigger_transition(const char *transition_label);

  /**
   * @brief 根据转换标签触发转换并返回回调函数的返回代码 (Trigger transition by transition label and
   * return callback function's return code)
   *
   * @param transition_label 转换标签 (Transition label)
   * @param cb_return_code 回调函数的返回代码 (Return code of the callback function)
   * @return 状态的引用 (Reference to the state)
   */
  const State &trigger_transition(
      const char *transition_label,
      node_interfaces::LifecycleNodeInterface::CallbackReturn &cb_return_code);

  /**
   * @brief 激活状态回调 (Activate state callback)
   */
  void on_activate() const;

  /**
   * @brief 停用状态回调 (Deactivate state callback)
   */
  void on_deactivate() const;

  /**
   * @brief 添加托管实体 (Add managed entity)
   *
   * @param managed_entity 托管实体接口的弱指针 (Weak pointer to the ManagedEntityInterface)
   */
  void add_managed_entity(std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface> managed_entity);

  /**
   * @brief 添加定时器句柄 (Add timer handle)
   *
   * @param timer 定时器基类的共享指针 (Shared pointer to the TimerBase class)
   */
  void add_timer_handle(std::shared_ptr<rclcpp::TimerBase> timer);

private:
  /// 禁用 LifecycleNodeInterfaceImpl 的拷贝构造函数和赋值运算符。
  /// Disable copy constructor and assignment operator for LifecycleNodeInterfaceImpl.
  RCLCPP_DISABLE_COPY(LifecycleNodeInterfaceImpl)

  /**
   * @brief 改变状态的服务回调函数。
   * @param header 服务请求头，包含序列号和时间戳。
   * @param req 服务请求对象，包含要改变的状态信息。
   * @param resp 服务响应对象，包含状态改变结果。
   *
   * @brief Service callback function for changing state.
   * @param header Service request header, containing sequence number and timestamp.
   * @param req Service request object, containing the state information to change.
   * @param resp Service response object, containing the result of the state change.
   */
  void on_change_state(
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<ChangeStateSrv::Request> req,
      std::shared_ptr<ChangeStateSrv::Response> resp);

  /**
   * @brief 获取当前状态的服务回调函数。
   * @param header 服务请求头，包含序列号和时间戳。
   * @param req 服务请求对象。
   * @param resp 服务响应对象，包含当前状态信息。
   *
   * @brief Service callback function for getting current state.
   * @param header Service request header, containing sequence number and timestamp.
   * @param req Service request object.
   * @param resp Service response object, containing the current state information.
   */
  void on_get_state(
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<GetStateSrv::Request> req,
      std::shared_ptr<GetStateSrv::Response> resp) const;

  /**
   * @brief 获取可用状态的服务回调函数。
   * @param header 服务请求头，包含序列号和时间戳。
   * @param req 服务请求对象。
   * @param resp 服务响应对象，包含可用状态列表。
   *
   * @brief Service callback function for getting available states.
   * @param header Service request header, containing sequence number and timestamp.
   * @param req Service request object.
   * @param resp Service response object, containing the list of available states.
   */
  void on_get_available_states(
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<GetAvailableStatesSrv::Request> req,
      std::shared_ptr<GetAvailableStatesSrv::Response> resp) const;

  /**
   * @brief 获取可用转换的服务回调函数。
   * @param header 服务请求头，包含序列号和时间戳。
   * @param req 服务请求对象。
   * @param resp 服务响应对象，包含可用转换列表。
   *
   * @brief Service callback function for getting available transitions.
   * @param header Service request header, containing sequence number and timestamp.
   * @param req Service request object.
   * @param resp Service response object, containing the list of available transitions.
   */
  void on_get_available_transitions(
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
      std::shared_ptr<GetAvailableTransitionsSrv::Response> resp) const;

  /**
   * @brief 获取转换图的服务回调函数。
   * @param header 服务请求头，包含序列号和时间戳。
   * @param req 服务请求对象。
   * @param resp 服务响应对象，包含转换图信息。
   *
   * @brief Service callback function for getting transition graph.
   * @param header Service request header, containing sequence number and timestamp.
   * @param req Service request object.
   * @param resp Service response object, containing the transition graph information.
   */
  void on_get_transition_graph(
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
      std::shared_ptr<GetAvailableTransitionsSrv::Response> resp) const;

  /**
   * @brief 改变状态。
   * @param transition_id 要执行的转换 ID。
   * @param cb_return_code 回调函数的返回码。
   * @return 返回 RCL_RET_OK 或错误代码。
   *
   * @brief Change state.
   * @param transition_id The transition ID to execute.
   * @param cb_return_code The return code of the callback function.
   * @return Returns RCL_RET_OK or an error code.
   */
  rcl_ret_t change_state(
      std::uint8_t transition_id,
      node_interfaces::LifecycleNodeInterface::CallbackReturn &cb_return_code);

  /**
   * @brief 执行回调函数。
   * @param cb_id 要执行的回调函数 ID。
   * @param previous_state 前一个状态。
   * @return 返回回调函数的执行结果。
   *
   * @brief Execute callback function.
   * @param cb_id The callback function ID to execute.
   * @param previous_state The previous state.
   * @return Returns the execution result of the callback function.
   */
  node_interfaces::LifecycleNodeInterface::CallbackReturn execute_callback(
      unsigned int cb_id, const State &previous_state) const;

  /// 状态机互斥锁，用于保护状态机的访问。
  /// State machine mutex for protecting access to the state machine.
  mutable std::recursive_mutex state_machine_mutex_;

  /// 生命周期状态机对象。
  /// Lifecycle state machine object.
  rcl_lifecycle_state_machine_t state_machine_;

  /// 当前状态对象。
  /// Current state object.
  State current_state_;

  /// 回调函数映射，将转换 ID 映射到对应的回调函数。
  /// Callback function mapping, mapping transition IDs to their corresponding callback functions.
  std::map<
      std::uint8_t,
      std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)>>
      cb_map_;

  // 定义 NodeBasePtr 类型为 std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>
  // Define NodeBasePtr type as std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>
  using NodeBasePtr = std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>;
  // 定义 NodeServicesPtr 类型为 std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface>
  // Define NodeServicesPtr type as std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface>
  using NodeServicesPtr = std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface>;
  // 定义 ChangeStateSrvPtr 类型为 std::shared_ptr<rclcpp::Service<ChangeStateSrv>>
  // Define ChangeStateSrvPtr type as std::shared_ptr<rclcpp::Service<ChangeStateSrv>>
  using ChangeStateSrvPtr = std::shared_ptr<rclcpp::Service<ChangeStateSrv>>;
  // 定义 GetStateSrvPtr 类型为 std::shared_ptr<rclcpp::Service<GetStateSrv>>
  // Define GetStateSrvPtr type as std::shared_ptr<rclcpp::Service<GetStateSrv>>
  using GetStateSrvPtr = std::shared_ptr<rclcpp::Service<GetStateSrv>>;
  // 定义 GetAvailableStatesSrvPtr 类型为 std::shared_ptr<rclcpp::Service<GetAvailableStatesSrv>>
  // Define GetAvailableStatesSrvPtr type as std::shared_ptr<rclcpp::Service<GetAvailableStatesSrv>>
  using GetAvailableStatesSrvPtr = std::shared_ptr<rclcpp::Service<GetAvailableStatesSrv>>;
  // 定义 GetAvailableTransitionsSrvPtr 类型为
  // std::shared_ptr<rclcpp::Service<GetAvailableTransitionsSrv>> Define
  // GetAvailableTransitionsSrvPtr type as
  // std::shared_ptr<rclcpp::Service<GetAvailableTransitionsSrv>>
  using GetAvailableTransitionsSrvPtr =
      std::shared_ptr<rclcpp::Service<GetAvailableTransitionsSrv>>;
  // 定义 GetTransitionGraphSrvPtr 类型为
  // std::shared_ptr<rclcpp::Service<GetAvailableTransitionsSrv>> Define GetTransitionGraphSrvPtr
  // type as std::shared_ptr<rclcpp::Service<GetAvailableTransitionsSrv>>
  using GetTransitionGraphSrvPtr = std::shared_ptr<rclcpp::Service<GetAvailableTransitionsSrv>>;

  // 节点基本接口
  // Node base interface
  NodeBasePtr node_base_interface_;
  // 节点服务接口
  // Node services interface
  NodeServicesPtr node_services_interface_;
  // 改变状态服务指针
  // Change state service pointer
  ChangeStateSrvPtr srv_change_state_;
  // 获取状态服务指针
  // Get state service pointer
  GetStateSrvPtr srv_get_state_;
  // 获取可用状态服务指针
  // Get available states service pointer
  GetAvailableStatesSrvPtr srv_get_available_states_;
  // 获取可用转换服务指针
  // Get available transitions service pointer
  GetAvailableTransitionsSrvPtr srv_get_available_transitions_;
  // 获取转换图服务指针
  // Get transition graph service pointer
  GetTransitionGraphSrvPtr srv_get_transition_graph_;
  // to controllable things
  // 可控制的实体列表，使用弱指针存储
  // List of controllable entities, stored as weak pointers
  std::vector<std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface>> weak_managed_entities_;
  // 定时器列表，使用弱指针存储
  // List of timers, stored as weak pointers
  std::vector<std::weak_ptr<rclcpp::TimerBase>> weak_timers_;
};

}  // namespace rclcpp_lifecycle
#endif  // LIFECYCLE_NODE_INTERFACE_IMPL_HPP_

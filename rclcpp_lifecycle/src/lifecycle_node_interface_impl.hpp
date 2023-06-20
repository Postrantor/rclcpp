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
  /// 使用 ChangeStateSrv 类型定义 ChangeStateSrv（用于更改状态的服务）
  using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;
  /// 使用 GetStateSrv 类型定义 GetStateSrv（用于获取当前状态的服务）
  using GetStateSrv = lifecycle_msgs::srv::GetState;
  /// 使用 GetAvailableStatesSrv 类型定义 GetAvailableStatesSrv（用于获取可用状态的服务）
  using GetAvailableStatesSrv = lifecycle_msgs::srv::GetAvailableStates;
  /// 使用 GetAvailableTransitionsSrv 类型定义
  /// GetAvailableTransitionsSrv（用于获取可用转换的服务）
  using GetAvailableTransitionsSrv = lifecycle_msgs::srv::GetAvailableTransitions;
  /// 使用 TransitionEventMsg 类型定义 TransitionEventMsg（用于表示转换事件的消息）
  using TransitionEventMsg = lifecycle_msgs::msg::TransitionEvent;

public:
  /**
   * @brief 生命周期节点接口实现类构造函数
   * @param node_base_interface 节点基础接口的共享指针
   * @param node_services_interface 节点服务接口的共享指针
   */
  LifecycleNodeInterfaceImpl(
      std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
      std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface);

  /**
   * @brief 生命周期节点接口实现类析构函数
   */
  ~LifecycleNodeInterfaceImpl();

  /**
   * @brief 初始化生命周期节点接口
   * @param enable_communication_interface 是否启用通信接口，默认为 true
   */
  void init(bool enable_communication_interface = true);

  /**
   * @brief 注册回调函数 (Register callback function)
   * @param lifecycle_transition 生命周期转换 (Lifecycle transition)
   * @param cb 回调函数 (Callback function)
   * @return 是否成功注册回调函数 (Whether the callback function is successfully registered)
   */
  bool register_callback(
      std::uint8_t lifecycle_transition,
      std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)> &cb);

  /**
   * @brief 获取当前状态 (Get current state)
   * @return 当前状态的引用 (Reference to the current state)
   */
  const State &get_current_state() const;

  /**
   * @brief 获取可用状态 (Get available states)
   * @return 可用状态的向量 (Vector of available states)
   */
  std::vector<State> get_available_states() const;

  /**
   * @brief 获取可用转换 (Get available transitions)
   * @return 可用转换的向量 (Vector of available transitions)
   */
  std::vector<Transition> get_available_transitions() const;

  /**
   * @brief 获取转换图 (Get transition graph)
   * @return 转换图的向量 (Vector of the transition graph)
   */
  std::vector<Transition> get_transition_graph() const;

  // ========= ========= ========= //

  /**
   * @brief 触发转换 (Trigger transition)
   * @param transition_id 转换 ID (Transition ID)
   * @return 状态的引用 (Reference to the state)
   */
  const State &trigger_transition(uint8_t transition_id);

  /**
   * @brief 触发转换并返回回调函数的返回代码
   * @param transition_id 转换 ID (Transition ID)
   * @param cb_return_code 回调函数的返回代码 (Return code of the callback function)
   * @return 状态的引用 (Reference to the state)
   */
  const State &trigger_transition(
      uint8_t transition_id,
      node_interfaces::LifecycleNodeInterface::CallbackReturn &cb_return_code);

  /**
   * @brief 根据转换标签触发转换 (Trigger transition by transition label)
   * @param transition_label 转换标签 (Transition label)
   * @return 状态的引用 (Reference to the state)
   */
  const State &trigger_transition(const char *transition_label);

  /**
   * @brief 根据转换标签触发转换并返回回调函数的返回代码
   * @param transition_label 转换标签 (Transition label)
   * @param cb_return_code 回调函数的返回代码 (Return code of the callback function)
   * @return 状态的引用 (Reference to the state)
   */
  const State &trigger_transition(
      const char *transition_label,
      node_interfaces::LifecycleNodeInterface::CallbackReturn &cb_return_code);

  // ========= ========= ========= //

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
   * @param managed_entity 托管实体接口的弱指针 (Weak pointer to the ManagedEntityInterface)
   */
  void add_managed_entity(std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface> managed_entity);

  /**
   * @brief 添加定时器句柄 (Add timer handle)
   * @param timer 定时器基类的共享指针 (Shared pointer to the TimerBase class)
   */
  void add_timer_handle(std::shared_ptr<rclcpp::TimerBase> timer);

private:
  /// 禁用 LifecycleNodeInterfaceImpl 的拷贝构造函数和赋值运算符。
  RCLCPP_DISABLE_COPY(LifecycleNodeInterfaceImpl)

  /**
   * @brief 改变状态的服务回调函数。
   * @param header 服务请求头，包含序列号和时间戳。
   * @param req 服务请求对象，包含要改变的状态信息。
   * @param resp 服务响应对象，包含状态改变结果。
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
   */
  rcl_ret_t change_state(
      std::uint8_t transition_id,
      node_interfaces::LifecycleNodeInterface::CallbackReturn &cb_return_code);

  /**
   * @brief 执行回调函数。
   * @param cb_id 要执行的回调函数 ID。
   * @param previous_state 前一个状态。
   * @return 返回回调函数的执行结果。
   */
  node_interfaces::LifecycleNodeInterface::CallbackReturn execute_callback(
      unsigned int cb_id, const State &previous_state) const;

  /// 状态机互斥锁，用于保护状态机的访问。
  mutable std::recursive_mutex state_machine_mutex_;

  /// 生命周期状态机对象。
  rcl_lifecycle_state_machine_t state_machine_;

  /// 当前状态对象。
  State current_state_;

  /// 回调函数映射，将转换 ID 映射到对应的回调函数。
  std::map<
      std::uint8_t,
      std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)>>
      cb_map_;

  // 定义类型
  using NodeBasePtr = std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>;
  using NodeServicesPtr = std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface>;
  using ChangeStateSrvPtr = std::shared_ptr<rclcpp::Service<ChangeStateSrv>>;
  using GetStateSrvPtr = std::shared_ptr<rclcpp::Service<GetStateSrv>>;
  using GetAvailableStatesSrvPtr = std::shared_ptr<rclcpp::Service<GetAvailableStatesSrv>>;
  using GetAvailableTransitionsSrvPtr =
      std::shared_ptr<rclcpp::Service<GetAvailableTransitionsSrv>>;
  using GetTransitionGraphSrvPtr = std::shared_ptr<rclcpp::Service<GetAvailableTransitionsSrv>>;

  // 节点基本接口
  NodeBasePtr node_base_interface_;
  // 节点服务接口
  NodeServicesPtr node_services_interface_;
  // 改变状态服务指针
  ChangeStateSrvPtr srv_change_state_;
  // 获取状态服务指针
  GetStateSrvPtr srv_get_state_;
  // 获取可用状态服务指针
  GetAvailableStatesSrvPtr srv_get_available_states_;
  // 获取可用转换服务指针
  GetAvailableTransitionsSrvPtr srv_get_available_transitions_;
  // 获取转换图服务指针
  GetTransitionGraphSrvPtr srv_get_transition_graph_;
  // 可控制的实体列表，使用弱指针存储
  std::vector<std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface>> weak_managed_entities_;
  // 定时器列表，使用弱指针存储
  std::vector<std::weak_ptr<rclcpp::TimerBase>> weak_timers_;
};

}  // namespace rclcpp_lifecycle
#endif  // LIFECYCLE_NODE_INTERFACE_IMPL_HPP_

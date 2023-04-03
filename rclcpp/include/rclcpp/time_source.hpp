// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__TIME_SOURCE_HPP_
#define RCLCPP__TIME_SOURCE_HPP_

#include <memory>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rcl/time.h"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace rclcpp {
class Clock;

/**
 * 时间源，用于驱动附加的时钟。
 *
 * Time source that will drive the attached clocks.
 *
 * 如果附加节点的 `use_sim_time` 参数为 `true`，则附加的时钟将根据接收到的消息进行更新。
 *
 * If the attached node `use_sim_time` parameter is `true`, the attached clocks will
 * be updated based on messages received.
 *
 * 时间源创建的订阅时钟主题可以通过参数覆盖重新配置其QoS，特别是接受以下参数：
 *
 * The subscription to the clock topic created by the time source can have its qos reconfigured
 * using parameter overrides, particularly the following ones are accepted:
 *
 * - qos_overrides./clock.depth
 * - qos_overrides./clock.durability
 * - qos_overrides./clock.history
 * - qos_overrides./clock.reliability
 */
class TimeSource {
public:
  /// 构造函数
  /// Constructor
  /**
   * 节点将附加到时间源。
   *
   * The node will be attached to the time source.
   *
   * \param node std::shared指针，指向一个已初始化的节点
   * \param qos 创建 `/clock` 订阅时使用的 QoS。
   * \param use_clock_thread 是否在单独的线程中旋转附加的节点
   *
   * \param node std::shared pointer to a initialized node
   * \param qos QoS that will be used when creating a `/clock` subscription.
   * \param use_clock_thread whether to spin the attached node in a separate thread
   */
  RCLCPP_PUBLIC
  explicit TimeSource(
      rclcpp::Node::SharedPtr node,
      const rclcpp::QoS &qos = rclcpp::ClockQoS(),
      bool use_clock_thread = true);

  /// 空构造函数
  /// Empty constructor
  /**
   * 一个空的TimeSource类
   *
   * An Empty TimeSource class
   *
   * \param qos 创建 `/clock` 订阅时使用的 QoS。
   * \param use_clock_thread 是否在单独的线程中旋转附加的节点。
   *
   * \param qos QoS that will be used when creating a `/clock` subscription.
   * \param use_clock_thread whether to spin the attached node in a separate thread.
   */
  RCLCPP_PUBLIC
  explicit TimeSource(const rclcpp::QoS &qos = rclcpp::ClockQoS(), bool use_clock_thread = true);

  // TimeSource 不可复制
  // The TimeSource is uncopyable
  TimeSource(const TimeSource &) = delete;
  TimeSource &operator=(const TimeSource &) = delete;

  // TimeSource 可移动
  // The TimeSource is moveable
  TimeSource(TimeSource &&) = default;
  TimeSource &operator=(TimeSource &&) = default;

  /// 将节点附加到时间源。
  /// Attach node to the time source.
  /**
   * \param node std::shared指针，指向一个已初始化的节点
   *
   * \param node std::shared pointer to a initialized node
   */
  RCLCPP_PUBLIC
  void attachNode(rclcpp::Node::SharedPtr node);

  /// 将节点附加到时间源。
  /// Attach node to the time source.
  /**
   * 如果参数 `use_sim_time` 为 `true`，则源时间为模拟时间，否则源时间由系统定义。
   *
   * If the parameter `use_sim_time` is `true` then the source time is the simulation time,
   * otherwise the source time is defined by the system.
   *
   * \param node_base_interface 节点基本接口。
   * \param node_topics_interface 节点主题基本接口。
   * \param node_graph_interface 节点图形接口。
   * \param node_services_interface 节点服务接口。
   * \param node_logging_interface 节点日志接口。
   * \param node_clock_interface 节点时钟接口。
   * \param node_parameters_interface 节点参数接口。
   *
   * \param node_base_interface Node base interface.
   * \param node_topics_interface Node topic base interface.
   * \param node_graph_interface Node graph interface.
   * \param node_services_interface Node service interface.
   * \param node_logging_interface Node logging interface.
   * \param node_clock_interface Node clock interface.
   * \param node_parameters_interface Node parameters interface.
   */
  RCLCPP_PUBLIC
  void attachNode(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
      rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface);

  /// 从时间源分离节点
  /// Detach the node from the time source
  RCLCPP_PUBLIC
  void detachNode();

  /// 将时钟附加到时间源以进行更新
  /// Attach a clock to the time source to be updated
  /**
   * \param[in] 要附加到时间源的时钟
   * \throws std::invalid_argument 时间源必须是 RCL_ROS_TIME，否则抛出异常
   *
   * \param[in] clock to attach to the time source
   * \throws std::invalid_argument the time source must be a RCL_ROS_TIME otherwise throws an
   * exception
   */
  RCLCPP_PUBLIC
  void attachClock(rclcpp::Clock::SharedPtr clock);

  /// 从时间源分离时钟
  /// Detach a clock from the time source
  RCLCPP_PUBLIC
  void detachClock(rclcpp::Clock::SharedPtr clock);

  /// 获取是否使用单独的时钟线程
  /// Get whether a separate clock thread is used or not
  RCLCPP_PUBLIC
  bool get_use_clock_thread();

  /// 设置是否使用单独的时钟线程
  /// Set whether to use a separate clock thread or not
  RCLCPP_PUBLIC
  void set_use_clock_thread(bool use_clock_thread);

  /// 检查时钟线程是否可连接
  /// Check if the clock thread is joinable
  RCLCPP_PUBLIC
  bool clock_thread_is_joinable();

  /// TimeSource 析构函数
  /// TimeSource Destructor
  RCLCPP_PUBLIC
  ~TimeSource();

private:
  class NodeState;
  std::shared_ptr<NodeState> node_state_;

  // 保留构造函数接收到的参数，以便在运行时重用
  // Preserve the arguments received by the constructor for reuse at runtime
  bool constructed_use_clock_thread_;
  rclcpp::QoS constructed_qos_;
};

}  // namespace rclcpp

#endif  // RCLCPP__TIME_SOURCE_HPP_

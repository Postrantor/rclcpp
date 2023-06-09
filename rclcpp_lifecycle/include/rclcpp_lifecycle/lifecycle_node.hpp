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

/** \mainpage rclcpp_lifecycle: Package containing a prototype for lifecycle implementation.
 *
 * - Lifecycle states: Define the State class. There are 4 primary states: Unconfigured, Inactive,
 *   Active and Finalized. There are also 6 transition states which are intermediate states during
 *   a requested transition. Configuring, CleaningUp, ShuttingDown, Activating, Deactivating and
 *   ErrorProcessing.
 *   - rclcpp_lifecycle/state.hpp
 * - Lifecycle transitions Define the Transition class. There are 7 transitions exposed to a
 *   supervisory process, they are: create, configure, cleanup, activate, deactivate, shutdown and
 *   destroy.
 *   - rclcpp_lifecycle/transition.hpp
 * - Lifecycle publisher creates a publisher that allows enabling and disabling message publication.
 *   - rclcpp_lifecycle/publisher.hpp
 * - Lifecycle node: An optional interface class for life cycle node implementations.
 *   - rclcpp_lifecycle/lifecycle_node.hpp
 *
 * Some useful internal abstractions and utilities:
 * - Macros for controlling symbol visibility on the library
 *   - rclcpp_lifecycle/visibility_control.h
 */

/** \mainpage rclcpp_lifecycle: 包含生命周期实现的原型的包。
 *
 * - 生命周期状态：定义 State 类。有 4 个主要状态：Unconfigured(未配置), Inactive(非活动),
 *   Active(活动)和 Finalized(最终状态)。还有 6 个过渡状态，这些状态在请求过渡期间是中间状态。
 *   Configuring(配置中), CleaningUp(清理中), ShuttingDown(关闭中), Activating(激活中),
 *   Deactivating(停用中)和 ErrorProcessing(错误处理中)。
 *   - rclcpp_lifecycle/state.hpp
 * - 生命周期转换：定义 Transition 类。有 7 种转换暴露给监督过程，它们是：
 *   create(创建), configure(配置), cleanup(清理), activate(激活), deactivate(停用),
 *   shutdown(关闭)和 destroy(销毁)。
 *   - rclcpp_lifecycle/transition.hpp
 * - 生命周期发布器：创建一个发布器，允许启用和禁用消息发布。
 *   - rclcpp_lifecycle/publisher.hpp
 * - 生命周期节点：生命周期节点实现的可选接口类。
 *   - rclcpp_lifecycle/lifecycle_node.hpp
 *
 * 一些有用的内部抽象和实用程序：
 * - 控制库上符号可见性的宏
 *   - rclcpp_lifecycle/visibility_control.h
 */

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl/error_handling.h"
#include "rcl/node.h"
#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/event.hpp"
#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_time_source_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/transition.hpp"
#include "rclcpp_lifecycle/visibility_control.h"
#include "rcutils/macros.h"
#include "rmw/types.h"

namespace rclcpp_lifecycle {

// 在此处包含这些，以解决一个奇特的 Windows 错误，其中名称空间
// 不能在下面的函数声明中使用，否则会出现错误，如：
//   'rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>':
//     没有合适的默认构造函数可用
// Include these here to work around an esoteric Windows error where the namespace
// cannot be used in the function declaration below without getting an error like:
//   'rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>':
//     no appropriate default constructor available
template <typename AllocatorT>
using PublisherOptionsWithAllocator = rclcpp::PublisherOptionsWithAllocator<AllocatorT>;
template <typename AllocatorT>
using SubscriptionOptionsWithAllocator = rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>;

// 创建具有默认发布器选项的函数模板
// Create a function template for creating default publisher options
template <typename AllocatorT>
PublisherOptionsWithAllocator<AllocatorT> create_default_publisher_options() {
  // 返回默认的发布器选项
  // Return the default publisher options
  return rclcpp::PublisherOptionsWithAllocator<AllocatorT>();
}

// 创建具有默认订阅选项的函数模板
// Create a function template for creating default subscription options
template <typename AllocatorT>
SubscriptionOptionsWithAllocator<AllocatorT> create_default_subscription_options() {
  // 返回默认的订阅选项
  // Return the default subscription options
  return rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>();
}

/// 生命周期节点，用于创建生命周期组件 (LifecycleNode for creating lifecycle components)
/**
 * @brief 具有生命周期节点接口，用于配置此节点。
 *        (Has a lifecycle node interface for configuring this node.)
 *
 * 这个类继承了`node_interfaces::LifecycleNodeInterface`和
 * `std::enable_shared_from_this<LifecycleNode>`，实现了一个具有生命周期管理功能的节点。
 * (This class inherits from `node_interfaces::LifecycleNodeInterface` and
 * `std::enable_shared_from_this<LifecycleNode>`, implementing a node with lifecycle management
 * features.)
 */
class LifecycleNode : public node_interfaces::LifecycleNodeInterface,
                      public std::enable_shared_from_this<LifecycleNode> {
public:
  /// 定义智能指针类型别名，方便在其他地方使用。
  /// (Define smart pointer type aliases for easier use in other places.)
  RCLCPP_SMART_PTR_DEFINITIONS(LifecycleNode)

  /// 创建一个具有指定名称的新生命周期节点。
  /// Create a new lifecycle node with the specified name.
  /**
   * \param[in] node_name 节点的名称。
   * \param[in] options 控制创建节点的其他选项。
   * \param[in] enable_communication_interface 决定是否启用底层 rcl_lifecycle_node 的通信接口。
   *
   * \param[in] node_name Name of the node.
   * \param[in] options Additional options to control creation of the node.
   * \param[in] enable_communication_interface Deciding whether the communication interface of the
   * underlying rcl_lifecycle_node shall be enabled.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  explicit LifecycleNode(
      const std::string &node_name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
      bool enable_communication_interface = true);

  /// 根据节点名称创建一个节点
  /// Create a node based on the node name
  /**
   * \param[in] node_name 节点的名称。
   * \param[in] namespace_ 节点的命名空间。
   * \param[in] options 控制创建节点的其他选项。
   * \param[in] enable_communication_interface 决定是否启用底层 rcl_lifecycle_node 的通信接口。
   *
   * \param[in] node_name Name of the node.
   * \param[in] namespace_ Namespace of the node.
   * \param[in] options Additional options to control creation of the node.
   * \param[in] enable_communication_interface Deciding whether the communication interface of the
   * underlying rcl_lifecycle_node shall be enabled.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  LifecycleNode(
      const std::string &node_name,
      const std::string &namespace_,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
      bool enable_communication_interface = true);

  /// 生命周期节点的虚拟析构函数
  /// Virtual destructor of the LifecycleNode
  RCLCPP_LIFECYCLE_PUBLIC
  virtual ~LifecycleNode();

  /// 获取节点的名称。
  /// Get the name of the node.
  /**
   * \return 节点的名称。
   * \return The name of the node.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const char *get_name() const;

  /// 获取节点的命名空间 (Get the namespace of the node)
  /**
   * \return 节点的命名空间 (The namespace of the node)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const char *get_namespace() const;

  /// 获取节点的日志记录器 (Get the logger of the node)
  /**
   * \return 节点的日志记录器 (The logger of the node)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Logger get_logger() const;

  /// 创建并返回一个回调组 (Create and return a callback group)
  /**
   * \param[in] group_type 由此方法创建的回调组类型 (callback group type to create by this method)
   * \param[in] automatically_add_to_executor_with_node
   * 确定是否将回调组自动添加到与其关联的节点的执行器中 (A boolean that determines whether a
   * callback group is automatically added to an executor with the node with which it is associated)
   * \return 回调组 (a callback group)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::CallbackGroup::SharedPtr create_callback_group(
      rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node = true);

  /// 在节点中迭代回调组，对每个有效的回调组调用 func (Iterate over the callback groups in the node,
  /// calling func on each valid one)
  RCLCPP_LIFECYCLE_PUBLIC
  void for_each_callback_group(
      const rclcpp::node_interfaces::NodeBaseInterface::CallbackGroupFunction &func);

  /// 创建并返回一个发布者 (Create and return a Publisher)
  /**
   * \param[in] topic_name 此发布者要发布的主题 (The topic for this publisher to publish on)
   * \param[in] qos 此发布者的服务质量设置 (The Quality of Service settings for this publisher)
   * \param[in] options 此发布者的发布者选项 (The publisher options for this publisher)
   * \return 创建的生命周期发布者的共享指针 (Shared pointer to the created lifecycle publisher)
   */
  template <typename MessageT, typename AllocatorT = std::allocator<void>>
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>> create_publisher(
      const std::string &topic_name,
      const rclcpp::QoS &qos,
      const PublisherOptionsWithAllocator<AllocatorT> &options =
          (create_default_publisher_options<AllocatorT>()));

  /// 创建并返回一个Subscription。
  /// Create and return a Subscription.
  /**
   * \param[in] topic_name 要订阅的主题。
   * \param[in] callback 用户定义的回调函数。
   * \param[in] qos 此订阅的服务质量。
   * \param[in] options 此订阅的订阅选项。
   * \param[in] msg_mem_strat 用于分配消息的消息内存策略。
   * \return 创建的订阅的共享指针。
   * \param[in] topic_name The topic to subscribe on.
   * \param[in] callback The user-defined callback function.
   * \param[in] qos The quality of service for this subscription.
   * \param[in] options The subscription options for this subscription.
   * \param[in] msg_mem_strat The message memory strategy to use for allocating messages.
   * \return Shared pointer to the created subscription.
   */
  template <
      typename MessageT,
      typename CallbackT,
      typename AllocatorT = std::allocator<void>,
      typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
      typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType>
  std::shared_ptr<SubscriptionT> create_subscription(
      const std::string &topic_name,
      const rclcpp::QoS &qos,
      CallbackT &&callback,
      const SubscriptionOptionsWithAllocator<AllocatorT> &options =
          create_default_subscription_options<AllocatorT>(),
      typename MessageMemoryStrategyT::SharedPtr msg_mem_strat =
          (MessageMemoryStrategyT::create_default()));

  /// 创建一个使用墙壁时钟驱动回调的计时器。
  /// Create a timer that uses the wall clock to drive the callback.
  /**
   * \param[in] period 回调触发之间的时间间隔。
   * \param[in] period Time interval between triggers of the callback.
   * \param[in] callback 用户定义的回调函数。
   * \param[in] callback User-defined callback function.
   * \param[in] group 用于执行此计时器回调的回调组。
   * \param[in] group Callback group to execute this timer's callback in.
   */
  template <typename DurationRepT = int64_t, typename DurationT = std::milli, typename CallbackT>
  typename rclcpp::WallTimer<CallbackT>::SharedPtr create_wall_timer(
      std::chrono::duration<DurationRepT, DurationT> period,
      CallbackT callback,
      rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// 创建一个使用节点时钟驱动回调的定时器。 (Create a timer that uses the node clock to drive the
  /// callback.)
  /**
   * \param[in] period 回调触发之间的时间间隔。 (Time interval between triggers of the callback.)
   * \param[in] callback 用户定义的回调函数。 (User-defined callback function.)
   * \param[in] group 用于执行此定时器回调的回调组。 (Callback group to execute this timer's
   * callback in.)
   */
  template <typename DurationRepT = int64_t, typename DurationT = std::milli, typename CallbackT>
  typename rclcpp::GenericTimer<CallbackT>::SharedPtr create_timer(
      std::chrono::duration<DurationRepT, DurationT> period,
      CallbackT callback,
      rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// 创建并返回一个客户端。 (Create and return a Client.)
  /**
   * \sa rclcpp::Node::create_client
   * \deprecated 使用 rclcpp::QoS 而不是 rmw_qos_profile_t (use rclcpp::QoS instead of
   * rmw_qos_profile_t)
   */
  template <typename ServiceT>
  [[deprecated("use rclcpp::QoS instead of rmw_qos_profile_t")]]
  typename rclcpp::Client<ServiceT>::SharedPtr
  create_client(
      const std::string &service_name,
      const rmw_qos_profile_t &qos_profile,
      rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// 创建并返回一个客户端。 (Create and return a Client.)
  /**
   * \param[in] service_name 服务可访问的名称。 (The name on which the service is accessible.)
   * \param[in] qos 客户端的服务质量配置文件。 (Quality of service profile for client.)
   * \param[in] group 用于处理服务调用回复的回调组。 (Callback group to handle the reply to service
   * calls.) \return 创建的客户端的共享指针。 (Shared pointer to the created client.)
   */
  template <typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr create_client(
      const std::string &service_name,
      const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
      rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// 创建并返回一个 Service。
  /// Create and return a Service.
  /**
   * \sa rclcpp::Node::create_service
   * \deprecated 使用 rclcpp::QoS 代替 rmw_qos_profile_t
   * \deprecated use rclcpp::QoS instead of rmw_qos_profile_t
   */
  template <typename ServiceT, typename CallbackT>
  [[deprecated("使用 rclcpp::QoS 代替 rmw_qos_profile_t")]] [[deprecated(
      "use rclcpp::QoS instead of rmw_qos_profile_t")]]
  typename rclcpp::Service<ServiceT>::SharedPtr
  create_service(
      const std::string &service_name,       // 服务名称 (Service name)
      CallbackT &&callback,                  // 回调函数 (Callback function)
      const rmw_qos_profile_t &qos_profile,  // QoS 配置文件 (QoS profile)
      rclcpp::CallbackGroup::SharedPtr group =
          nullptr);  // 回调组，可选参数，默认为空指针 (Callback group, optional parameter, default
                     // is nullptr)

  /// 创建并返回一个 Service。
  /// Create and return a Service.
  /**
   * \sa rclcpp::Node::create_service
   */
  template <typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr create_service(
      const std::string &service_name,  // 服务名称 (Service name)
      CallbackT &&callback,             // 回调函数 (Callback function)
      const rclcpp::QoS &qos =
          rclcpp::ServicesQoS(),  // QoS 设置，默认为 rclcpp::ServicesQoS() (QoS settings, default
                                  // is rclcpp::ServicesQoS())
      rclcpp::CallbackGroup::SharedPtr group =
          nullptr);  // 回调组，可选参数，默认为空指针 (Callback group, optional parameter, default
                     // is nullptr)

  /// 创建并返回一个 GenericPublisher。
  /// Create and return a GenericPublisher.
  /**
   * \sa rclcpp::Node::create_generic_publisher
   */
  template <typename AllocatorT = std::allocator<void>>
  std::shared_ptr<rclcpp::GenericPublisher> create_generic_publisher(
      const std::string &topic_name,  // 主题名称 (Topic name)
      const std::string &topic_type,  // 主题类型 (Topic type)
      const rclcpp::QoS &qos,         // QoS 设置 (QoS settings)
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options =
          (rclcpp::PublisherOptionsWithAllocator<
              AllocatorT>()));  // 发布器选项，可选参数，默认为
                                // rclcpp::PublisherOptionsWithAllocator<AllocatorT>() (Publisher
                                // options, optional parameter, default is
                                // rclcpp::PublisherOptionsWithAllocator<AllocatorT>())

  /// 创建并返回一个通用订阅 (Create and return a GenericSubscription).
  /**
   * \sa rclcpp::Node::create_generic_subscription
   */
  template <typename AllocatorT = std::allocator<void>>
  std::shared_ptr<rclcpp::GenericSubscription> create_generic_subscription(
      const std::string &topic_name,  ///< 主题名称 (Topic name)
      const std::string &topic_type,  ///< 主题类型 (Topic type)
      const rclcpp::QoS &qos,         ///< 服务质量配置 (Quality of service configuration)
      std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)>
          callback,                   ///< 回调函数 (Callback function)
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>
          &options =                  ///< 订阅选项 (Subscription options)
      (rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()));

  /// 声明并初始化一个参数，返回有效值 (Declare and initialize a parameter, return the effective
  /// value).
  /**
   * \sa rclcpp::Node::declare_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const rclcpp::ParameterValue &declare_parameter(
      const std::string &name,                      ///< 参数名称 (Parameter name)
      const rclcpp::ParameterValue &default_value,  ///< 默认值 (Default value)
      const rcl_interfaces::msg::ParameterDescriptor
          &parameter_descriptor =                   ///< 参数描述符 (Parameter descriptor)
      rcl_interfaces::msg::ParameterDescriptor(),
      bool ignore_override = false);  ///< 是否忽略覆盖 (Whether to ignore override)

  /// 声明并初始化一个参数，返回有效值 (Declare and initialize a parameter, return the effective
  /// value).
  /**
   * \sa rclcpp::Node::declare_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const rclcpp::ParameterValue &declare_parameter(
      const std::string &name,        ///< 参数名称 (Parameter name)
      rclcpp::ParameterType type,     ///< 参数类型 (Parameter type)
      const rcl_interfaces::msg::ParameterDescriptor
          &parameter_descriptor =     ///< 参数描述符 (Parameter descriptor)
      rcl_interfaces::msg::ParameterDescriptor{},
      bool ignore_override = false);  ///< 是否忽略覆盖 (Whether to ignore override)

  /// 声明并使用类型初始化一个参数 (Declare and initialize a parameter with a type).
  /**
   * \sa rclcpp::Node::declare_parameter
   */
  template <typename ParameterT>
  auto declare_parameter(
      const std::string &name,          ///< 参数名称 (Parameter name)
      const ParameterT &default_value,  ///< 默认值 (Default value)
      const rcl_interfaces::msg::ParameterDescriptor
          &parameter_descriptor =       ///< 参数描述符 (Parameter descriptor)
      rcl_interfaces::msg::ParameterDescriptor(),
      bool ignore_override = false);    ///< 是否忽略覆盖 (Whether to ignore override)

  /// 声明并初始化具有类型的参数。
  /// Declare and initialize a parameter with a type.
  /**
   * 有关详细信息，请参阅此类上的非模板 declare_parameter()。
   * See the non-templated declare_parameter() on this class for details.
   */
  template <typename ParameterT>
  auto declare_parameter(
      const std::string &name,
      const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
          rcl_interfaces::msg::ParameterDescriptor(),
      bool ignore_override = false);

  /// 使用相同的命名空间和类型声明和初始化多个参数。
  /// Declare and initialize several parameters with the same namespace and type.
  /**
   * \sa rclcpp::Node::declare_parameters
   */
  template <typename ParameterT>
  std::vector<ParameterT> declare_parameters(
      const std::string &namespace_, const std::map<std::string, ParameterT> &parameters);

  /// 使用相同的命名空间和类型声明和初始化多个参数。
  /// Declare and initialize several parameters with the same namespace and type.
  /**
   * \sa rclcpp::Node::declare_parameters
   */
  template <typename ParameterT>
  std::vector<ParameterT> declare_parameters(
      const std::string &namespace_,
      const std::map<std::string, std::pair<ParameterT, rcl_interfaces::msg::ParameterDescriptor>>
          &parameters);

  /// 取消声明先前声明的参数。
  /// Undeclare a previously declared parameter.
  /**
   * \sa rclcpp::Node::undeclare_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  void undeclare_parameter(const std::string &name);

  /// 如果给定参数已声明，则返回true。
  /// Return true if a given parameter is declared.
  /**
   * \sa rclcpp::Node::has_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool has_parameter(const std::string &name) const;

  /// 设置单个参数 (Set a single parameter)
  /**
   * \param[in] parameter 需要设置的参数对象 (The parameter object to be set)
   * \return 返回设置参数结果 (Returns the result of setting the parameter)
   * \sa rclcpp::Node::set_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rcl_interfaces::msg::SetParametersResult set_parameter(const rclcpp::Parameter &parameter);

  /// 逐个设置一个或多个参数 (Set one or more parameters, one at a time)
  /**
   * \param[in] parameters 需要设置的参数对象列表 (A list of parameter objects to be set)
   * \return 返回设置每个参数的结果列表 (Returns a list of results for setting each parameter)
   * \sa rclcpp::Node::set_parameters
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
      const std::vector<rclcpp::Parameter> &parameters);

  /// 一次性设置一个或多个参数 (Set one or more parameters, all at once)
  /**
   * \param[in] parameters 需要设置的参数对象列表 (A list of parameter objects to be set)
   * \return 返回设置参数的结果 (Returns the result of setting the parameters)
   * \sa rclcpp::Node::set_parameters_atomically
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
      const std::vector<rclcpp::Parameter> &parameters);

  /// 通过给定名称返回参数 (Return the parameter by the given name)
  /**
   * \param[in] name 参数的名称 (The name of the parameter)
   * \return 返回获取到的参数对象 (Returns the obtained parameter object)
   * \sa rclcpp::Node::get_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Parameter get_parameter(const std::string &name) const;

  /// 通过给定名称获取参数值，并返回是否设置成功 (Get the value of a parameter by the given name,
  /// and return true if it was set)
  /**
   * \param[in] name 参数的名称 (The name of the parameter)
   * \param[out] parameter 返回获取到的参数对象 (The obtained parameter object is returned)
   * \return 如果设置成功则返回true，否则返回false (Returns true if set successfully, false
   * otherwise) \sa rclcpp::Node::get_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool get_parameter(const std::string &name, rclcpp::Parameter &parameter) const;

  /// 通过给定名称获取参数值，并返回是否设置成功 (Get the value of a parameter by the given name,
  /// and return true if it was set)
  /**
   * \tparam ParameterT 参数类型 (Parameter type)
   * \param[in] name 参数的名称 (The name of the parameter)
   * \param[out] parameter 返回获取到的参数对象 (The obtained parameter object is returned)
   * \return 如果设置成功则返回true，否则返回false (Returns true if set successfully, false
   * otherwise) \sa rclcpp::Node::get_parameter
   */
  template <typename ParameterT>
  bool get_parameter(const std::string &name, ParameterT &parameter) const;

  /// 获取参数值，如果没有设置，则将其赋给 "parameter"。
  /// Get the parameter value, or the "alternative_value" if not set, and assign it to "parameter".
  /**
   * \sa rclcpp::Node::get_parameter_or
   */
  template <typename ParameterT>
  bool get_parameter_or(
    const std::string & name, ///< 参数名(Parameter name)
    ParameterT & value, ///< 存储获取到的参数值的变量(Variable to store the obtained parameter value)
    const ParameterT & alternative_value ///< 如果参数未设置，则使用此替代值(Alternative value to be used if the parameter is not set)) const;

  /// 根据给定的参数名称返回参数。
  /// Return the parameters by the given parameter names.
  /**
   * \sa rclcpp::Node::get_parameters
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string> & names) const; ///< 参数名称列表(List of parameter names)

  /// 获取具有给定前缀的所有参数的参数值。
  /// Get the parameter values for all parameters that have a given prefix.
  /**
   * \sa rclcpp::Node::get_parameters
   */
  template <typename MapValueT>
  bool get_parameters(
    const std::string & prefix, ///< 参数名前缀(Parameter name prefix)
    std::map<std::string, MapValueT> & values) const; ///< 存储获取到的参数值的映射(Map to store the obtained parameter values)

  /// 返回给定参数名称的参数描述符。
  /// Return the parameter descriptor for the given parameter name.
  /**
   * \sa rclcpp::Node::describe_parameter
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rcl_interfaces::msg::ParameterDescriptor describe_parameter(const std::string & name) const; ///< 参数名称(Parameter name)

  /// 返回参数描述符向量，每个给定名称一个。
  /// Return a vector of parameter descriptors, one for each of the given names.
  /**
   * \sa rclcpp::Node::describe_parameters
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rcl_interfaces::msg::ParameterDescriptor>
  describe_parameters(const std::vector<std::string> & names) const; ///< 参数名称列表(List of parameter names)

  /// 返回参数类型向量，每个给定名称一个。
  /// Return a vector of parameter types, one for each of the given names.
  /**
   * \sa rclcpp::Node::get_parameter_types
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<uint8_t> get_parameter_types(const std::vector<std::string> & names) const; ///< 参数名称列表(List of parameter names)

  /// 返回具有任何给定前缀的参数列表，最大深度为给定值。
  /// Return a list of parameters with any of the given prefixes, up to the given depth.
  /**
   * \sa rclcpp::Node::list_parameters
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rcl_interfaces::msg::ListParametersResult
  list_parameters(
    const std::vector<std::string> & prefixes, ///< 参数名前缀列表(List of parameter name prefixes)
    uint64_t depth) const; ///< 列表参数的最大深度(Maximum depth for listing parameters)

  using PreSetParametersCallbackHandle = rclcpp::node_interfaces::PreSetParametersCallbackHandle;
  using PreSetParametersCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::PreSetParametersCallbackType;

  using OnSetParametersCallbackHandle = rclcpp::node_interfaces::OnSetParametersCallbackHandle;
  using OnSetParametersCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType;
  using OnParametersSetCallbackType [[deprecated("use OnSetParametersCallbackType instead")]] =
    OnSetParametersCallbackType;

  using PostSetParametersCallbackHandle = rclcpp::node_interfaces::PostSetParametersCallbackHandle;
  using PostSetParametersCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::PostSetParametersCallbackType;

  /// 添加一个在参数验证之前触发的回调。
  /// Add a callback that gets triggered before parameters are validated.
  /**
   * \param[in] callback 预设参数回调类型。
   * \param[in] callback Pre-set parameters callback type.
   *
   * \sa rclcpp::Node::add_pre_set_parameters_callback
   */
  RCLCPP_LIFECYCLE_PUBLIC
  RCUTILS_WARN_UNUSED
  rclcpp_lifecycle::LifecycleNode::PreSetParametersCallbackHandle::SharedPtr
  add_pre_set_parameters_callback(
    rclcpp_lifecycle::LifecycleNode::PreSetParametersCallbackType callback);

  /// 添加一个在设置参数时触发的回调。
  /// Add a callback for when parameters are being set.
  /**
   * \param[in] callback 设置参数回调类型。
   * \param[in] callback On-set parameters callback type.
   *
   * \sa rclcpp::Node::add_on_set_parameters_callback
   */
  RCLCPP_LIFECYCLE_PUBLIC
  RCUTILS_WARN_UNUSED
  rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr
  add_on_set_parameters_callback(
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackType callback);

  /// 添加一个在成功设置参数后触发的回调。
  /// Add a callback that gets triggered after parameters are set successfully.
  /**
   * \param[in] callback 后设参数回调类型。
   * \param[in] callback Post-set parameters callback type.
   *
   * \sa rclcpp::Node::add_post_set_parameters_callback
   */
  RCLCPP_LIFECYCLE_PUBLIC
  RCUTILS_WARN_UNUSED
  rclcpp_lifecycle::LifecycleNode::PostSetParametersCallbackHandle::SharedPtr
  add_post_set_parameters_callback(
    rclcpp_lifecycle::LifecycleNode::PostSetParametersCallbackType callback);

  /// 移除使用 `add_pre_set_parameters_callback` 注册的回调。
  /// Remove a callback registered with `add_pre_set_parameters_callback`.
  /**
   * \param[in] handler 预设参数回调句柄。
   * \param[in] handler Pre-set parameters callback handle.
   *
   * \sa rclcpp::Node::remove_pre_set_parameters_callback
   */
  RCLCPP_LIFECYCLE_PUBLIC
  void remove_pre_set_parameters_callback(
    const rclcpp_lifecycle::LifecycleNode::PreSetParametersCallbackHandle * const handler);

  /// 移除使用 `add_on_set_parameters_callback` 注册的回调。
  /// Remove a callback registered with `add_on_set_parameters_callback`.
  /**
   * \param[in] handler 设置参数回调句柄。
   * \param[in] handler On-set parameters callback handle.
   *
   * \sa rclcpp::Node::remove_on_set_parameters_callback
   */
  RCLCPP_LIFECYCLE_PUBLIC
  void remove_on_set_parameters_callback(
    const rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle * const handler);

  /// 删除通过 `add_post_set_parameters_callback` 注册的回调函数。
  /// Remove a callback registered with `add_post_set_parameters_callback`.
  /**
   * \sa rclcpp::Node::remove_post_set_parameters_callback
   */
  RCLCPP_LIFECYCLE_PUBLIC
  void remove_post_set_parameters_callback(
    const rclcpp_lifecycle::LifecycleNode::PostSetParametersCallbackHandle * const handler);

  /// 返回现有节点名称(字符串)的向量。
  /// Return a vector of existing node names (string).
  /**
   * \sa rclcpp::Node::get_node_names
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<std::string> get_node_names() const;

  /// 返回现有主题名称到主题类型列表的映射。
  /// Return a map of existing topic names to list of topic types.
  /**
   * \sa rclcpp::Node::get_topic_names_and_types
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types(bool no_demangle = false) const;

  /// 返回现有服务名称到服务类型列表的映射。
  /// Return a map of existing service names to list of topic types.
  /**
   * \sa rclcpp::Node::get_service_names_and_types
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::map<std::string, std::vector<std::string>> get_service_names_and_types() const;

  /// 返回特定节点上现有服务名称到服务类型列表的映射。
  /// Return a map of existing service names to list of service types for a specific node.
  /**
   * 此功能仅考虑服务 - 不包括客户端。
   * This function only considers services - not clients.
   *
   * \param[in] node_name 节点名称
   * \param[in] namespace_ 节点的命名空间
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::map<std::string, std::vector<std::string>> get_service_names_and_types_by_node(
    const std::string & node_name, const std::string & namespace_) const;

  /// 返回在给定主题上宣传的发布者数量。
  /// Return the number of publishers that are advertised on a given topic.
  /**
   * \sa rclcpp::Node::count_publishers
   */
  RCLCPP_LIFECYCLE_PUBLIC
  size_t count_publishers(const std::string & topic_name) const;

  /// 返回已为给定主题创建订阅的订阅者数量。 (Return the number of subscribers who have created a subscription for a given topic.)
  /**
   * \sa rclcpp::Node::count_subscribers
   */
  RCLCPP_LIFECYCLE_PUBLIC
  size_t count_subscribers(const std::string & topic_name) const;

  /// 返回给定主题上发布者的主题端点信息。 (Return the topic endpoint information about publishers on a given topic.)
  /**
   * \sa rclcpp::Node::get_publishers_info_by_topic
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rclcpp::TopicEndpointInfo>
  get_publishers_info_by_topic(const std::string & topic_name, bool no_mangle = false) const;

  /// 返回给定主题上订阅的主题端点信息。 (Return the topic endpoint information about subscriptions on a given topic.)
  /**
   * \sa rclcpp::Node::get_subscriptions_info_by_topic
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<rclcpp::TopicEndpointInfo>
  get_subscriptions_info_by_topic(const std::string & topic_name, bool no_mangle = false) const;

  /// 返回一个图形事件，该事件将在发生图形更改时设置。 (Return a graph event, which will be set anytime a graph change occurs.)
  /* 图形事件对象是一个贷款，必须归还。 (The graph Event object is a loan which must be returned.)
  * 事件对象是有范围的，因此要归还负载只需让其超出范围。 (The Event object is scoped and therefore to return the load just let it go out of scope.)
  */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Event::SharedPtr get_graph_event();

  /// 通过等待事件设置来等待图形事件发生。 (Wait for a graph event to occur by waiting on an Event to become set.)
  /* 必须通过get_graph_event()方法获取给定的事件。 (The given Event must be acquire through the get_graph_event() method.)
  *
  * \throws InvalidEventError 如果给定的事件是nullptr (if the given event is nullptr)
  * \throws EventNotRegisteredError 如果给定的事件未使用get_graph_event()获取 (if the given event was not acquired with get_graph_event().)
  */
  RCLCPP_LIFECYCLE_PUBLIC
  void wait_for_graph_change(rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout);

  /// 获取节点管理的非const共享指针时钟。 (Get a clock as a non-const shared pointer which is managed by the node.)
  /**
   * \sa rclcpp::node_interfaces::NodeClock::get_clock
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Clock::SharedPtr get_clock();

  /// 获取由节点管理的常量共享指针时钟。
  /// Get a clock as a const shared pointer which is managed by the node.
  /**
   * \sa rclcpp::node_interfaces::NodeClock::get_clock
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Clock::ConstSharedPtr get_clock() const;

  /// 从指定的 clock_type 时间源返回当前时间。
  /// Returns current time from the time source specified by clock_type.
  /**
   * \sa rclcpp::Clock::now
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::Time now() const;

  /// 返回节点的内部 NodeBaseInterface 实现。
  /// Return the Node's internal NodeBaseInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_base_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

  /// 返回节点的内部 NodeClockInterface 实现。
  /// Return the Node's internal NodeClockInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_clock_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr get_node_clock_interface();

  /// 返回节点的内部 NodeGraphInterface 实现。
  /// Return the Node's internal NodeGraphInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_graph_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr get_node_graph_interface();

  /// 返回节点的内部 NodeLoggingInterface 实现。
  /// Return the Node's internal NodeLoggingInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_logging_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr get_node_logging_interface();

  /// 返回节点的内部 NodeTimersInterface 实现。
  /// Return the Node's internal NodeTimersInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_timers_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr get_node_timers_interface();

  /// 返回节点的内部 NodeTopicsInterface 实现。
  /// Return the Node's internal NodeTopicsInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_topics_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface();

  /// 返回节点的内部 NodeServicesInterface 实现。
  /// Return the Node's internal NodeServicesInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_services_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr get_node_services_interface();

  /// 返回节点的内部 NodeParametersInterface 实现。
  /// Return the Node's internal NodeParametersInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_parameters_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface();

  /// 返回节点的内部 NodeTimeSourceInterface 实现。
  /// Return the Node's internal NodeTimeSourceInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_time_source_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr get_node_time_source_interface();

  /// 返回节点的内部 NodeWaitablesInterface 实现。
  /// Return the Node's internal NodeWaitablesInterface implementation.
  /**
   * \sa rclcpp::Node::get_node_waitables_interface
   */
  RCLCPP_LIFECYCLE_PUBLIC
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr get_node_waitables_interface();

  /// 返回创建此节点时使用的 NodeOptions。
  /// Return the NodeOptions used when creating this node.
  /**
   * \sa rclcpp::Node::get_node_options
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const rclcpp::NodeOptions & get_node_options() const;

  //
  // 生命周期组件
  // LIFECYCLE COMPONENTS
  //
  /// 返回当前状态。
  /// Return the current State.
  /**
   * \return 当前状态
   * \return the current state
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & get_current_state() const;

  /// 返回一个包含所有可用状态的列表。
  /// Return a list with the available states.
  /**
   * \return 包含所有可用状态的列表。
   * \return list with the available states.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<State> get_available_states() const;

  /// 返回一个包含当前可用转换的列表。
  /// Return a list with the current available transitions.
  /**
   * \return 包含当前可用转换的列表。
   * \return list with the current available transitions.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<Transition> get_available_transitions() const;

  /// 返回一个包含所有转换的列表。
  /// Return a list with all the transitions.
  /**
   * \return 转换图中的所有转换列表。
   * \return list with all the transitions in the transition graph.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::vector<Transition> get_transition_graph() const;

  /// 触发指定的转换。
  /// Trigger the specified transition.
  /*
  * \return 此转换后的新状态。
  * \return the new state after this transition.
  */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & trigger_transition(const Transition & transition);

  /// 触发指定的转换并获取回调返回代码。
  /// Trigger the specified transition and get the callback return code.
  /*
  * \param[out] cb_return_code 转换回调返回代码。
  * \param[out] cb_return_code transition callback return code.
  * \return 此转换后的新状态。
  * \return the new state after this transition.
  */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & trigger_transition(
    const Transition & transition, LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// 根据 ID 触发指定的转换。
  /// Trigger the specified transition based on an id.
  /*
  * \return 此转换后的新状态。
  * \return the new state after this transition.
  */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & trigger_transition(uint8_t transition_id);

  /// 触发指定的基于 id 的转换并获取回调返回代码。 (Trigger the specified transition based on an id and get the callback return code.)
  /**
   * \param[out] cb_return_code 转换回调返回代码 (transition callback return code)
   * \return 此转换后的新状态 (the new state after this transition)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & trigger_transition(
    uint8_t transition_id, LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// 触发配置转换 (Trigger the configure transition)
  /**
   * \param[out] cb_return_code 转换回调返回代码 (transition callback return code)
   * \return 此转换后的新状态 (the new state after this transition)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & configure();

  /// 触发配置转换并获取回调返回代码。 (Trigger the configure transition and get the callback return code.)
  /**
   * \param[out] cb_return_code 转换回调返回代码 (transition callback return code)
   * \return 此转换后的新状态 (the new state after this transition)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & configure(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// 触发清理转换 (Trigger the cleanup transition)
  /**
   * \return 此转换后的新状态 (the new state after this transition)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & cleanup();

  /// 触发清理转换并获取回调返回代码。 (Trigger the cleanup transition and get the callback return code.)
  /**
   * \param[out] cb_return_code 转换回调返回代码 (transition callback return code)
   * \return 此转换后的新状态 (the new state after this transition)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & cleanup(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// 触发激活转换 (Trigger the activate transition)
  /**
   * \return 此转换后的新状态 (the new state after this transition)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & activate();

  /// 触发激活转换并获取回调返回代码。 (Trigger the activate transition and get the callback return code.)
  /**
   * \param[out] cb_return_code 转换回调返回代码 (transition callback return code)
   * \return 此转换后的新状态 (the new state after this transition)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & activate(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// 触发停用转换 (Trigger the deactivate transition)
  /**
   * \return 此转换后的新状态 (the new state after this transition)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & deactivate();

  /// 触发停用转换并获取回调返回代码。 (Trigger the deactivate transition and get the callback return code.)
  /**
   * \param[out] cb_return_code 转换回调返回代码 (transition callback return code)
   * \return 此转换后的新状态 (the new state after this transition)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & deactivate(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// 触发关闭过渡
  /// Trigger the shutdown transition
  /**
   * \return 返回此过渡后的新状态
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & shutdown();

  /// 触发关闭过渡并获取回调返回代码
  /// Trigger the shutdown transition and get the callback return code.
  /**
   * \param[out] cb_return_code 过渡回调返回代码
   * \param[out] cb_return_code transition callback return code
   * \return 返回此过渡后的新状态
   * \return the new state after this transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  const State & shutdown(LifecycleNodeInterface::CallbackReturn & cb_return_code);

  /// 注册配置回调
  /// Register the configure callback
  /**
   * 当触发到此状态的过渡时，将调用此回调
   * This callback will be called when the transition to this state is triggered
   * \param[in] fcn 要调用的回调函数
   * \param[in] fcn callback function to call
   * \return 始终为 true
   * \return always true
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_configure(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  /// 注册清理回调
  /// Register the cleanup callback
  /**
   * 当触发到此状态的过渡时，将调用此回调
   * This callback will be called when the transition to this state is triggered
   * \param[in] fcn 要调用的回调函数
   * \param[in] fcn callback function to call
   * \return 始终为 true
   * \return always true
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_cleanup(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  /// 注册关闭回调
  /// Register the shutdown callback
  /**
   * 当触发到此状态的过渡时，将调用此回调
   * This callback will be called when the transition to this state is triggered
   * \param[in] fcn 要调用的回调函数
   * \param[in] fcn callback function to call
   * \return 始终为 true
   * \return always true
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_shutdown(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  /// 注册激活回调 (Register the activate callback)
  /**
   * 当触发此状态的过渡时，将调用此回调 (This callback will be called when the transition to this state is triggered)
   * \param[in] fcn 要调用的回调函数 (callback function to call)
   * \return 总是返回 true (always true)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_activate(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  /// 注册停用回调 (Register the deactivate callback)
  /**
   * 当触发此状态的过渡时，将调用此回调 (This callback will be called when the transition to this state is triggered)
   * \param[in] fcn 要调用的回调函数 (callback function to call)
   * \return 总是返回 true (always true)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool
  register_on_deactivate(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  /// 注册错误回调 (Register the error callback)
  /**
   * 当触发此状态的过渡时，将调用此回调 (This callback will be called when the transition to this state is triggered)
   * \param[in] fcn 要调用的回调函数 (callback function to call)
   * \return 总是返回 true (always true)
   */
  RCLCPP_LIFECYCLE_PUBLIC
  bool register_on_error(std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn);

  // 重写 on_activate 方法 (Override on_activate method)
  RCLCPP_LIFECYCLE_PUBLIC
  CallbackReturn on_activate(const State & previous_state) override;

  // 重写 on_deactivate 方法 (Override on_deactivate method)
  RCLCPP_LIFECYCLE_PUBLIC
  CallbackReturn on_deactivate(const State & previous_state) override;

  protected:
  // 添加托管实体 (Add managed entity)
  RCLCPP_LIFECYCLE_PUBLIC
  void add_managed_entity(std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface> managed_entity);

  // 添加定时器句柄 (Add timer handle)
  RCLCPP_LIFECYCLE_PUBLIC
  void add_timer_handle(std::shared_ptr<rclcpp::TimerBase> timer);

private:
  RCLCPP_DISABLE_COPY(LifecycleNode)

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr node_time_source_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_;

  const rclcpp::NodeOptions node_options_;

  class LifecycleNodeInterfaceImpl;
  std::unique_ptr<LifecycleNodeInterfaceImpl> impl_;
};

}  // namespace rclcpp_lifecycle

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_NODE_IMPL_HPP_
// Template implementations
#include "rclcpp_lifecycle/lifecycle_node_impl.hpp"
#endif

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_

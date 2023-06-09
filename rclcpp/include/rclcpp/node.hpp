// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_HPP_
#define RCLCPP__NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rcl/error_handling.h"
#include "rcl/node.h"
#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/context.hpp"
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
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcutils/macros.h"

namespace rclcpp {

/// \class Node
/// \brief Node 是用于创建发布者和订阅者的单一入口点。 (Node is the single point of entry for
/// creating publishers and subscribers.)
class Node : public std::enable_shared_from_this<Node> {
public:
  /// \brief RCLCPP_SMART_PTR_DEFINITIONS 定义了智能指针类型，以便在类外部使用。
  /// (RCLCPP_SMART_PTR_DEFINITIONS defines smart pointer types for use outside the class.)
  RCLCPP_SMART_PTR_DEFINITIONS(Node)

  /// 创建一个具有指定名称的新节点。
  /// Create a new node with the specified name.
  /**
   * \param[in] node_name 节点的名称。
   * \param[in] options 附加选项，用于控制节点创建。
   * \throws InvalidNamespaceError 如果命名空间无效
   *
   * \param[in] node_name Name of the node.
   * \param[in] options Additional options to control creation of the node.
   * \throws InvalidNamespaceError if the namespace is invalid
   */
  RCLCPP_PUBLIC
  explicit Node(const std::string& node_name, const NodeOptions& options = NodeOptions());

  /// 创建一个具有指定名称的新节点。
  /// Create a new node with the specified name.
  /**
   * \param[in] node_name 节点的名称。
   * \param[in] namespace_ 节点的命名空间。
   * \param[in] options 附加选项，用于控制节点创建。
   * \throws InvalidNamespaceError 如果命名空间无效
   *
   * \param[in] node_name Name of the node.
   * \param[in] namespace_ Namespace of the node.
   * \param[in] options Additional options to control creation of the node.
   * \throws InvalidNamespaceError if the namespace is invalid
   */
  RCLCPP_PUBLIC
  explicit Node(
      const std::string& node_name,
      const std::string& namespace_,
      const NodeOptions& options = NodeOptions());

  // 节点的析构函数。
  // Destructor for the node.
  RCLCPP_PUBLIC
  virtual ~Node();

  /// 获取节点的名称。
  /// Get the name of the node.
  /** \return 节点的名称。
   *          The name of the node. */
  RCLCPP_PUBLIC
  const char* get_name() const;

  /// 获取节点的命名空间。
  /// Get the namespace of the node.
  /**
   * 此命名空间是“节点”的命名空间，因此不受可能影响使用此实例创建的实体的任何子命名空间的影响。
   * 使用 get_effective_namespace() 获取实体使用的完整命名空间。
   *
   * \sa get_sub_namespace()
   * \sa get_effective_namespace()
   * \return 节点的命名空间。
   *
   * This namespace is the "node's" namespace, and therefore is not affected
   * by any sub-namespace's that may affect entities created with this instance.
   * Use get_effective_namespace() to get the full namespace used by entities.
   *
   * \sa get_sub_namespace()
   * \sa get_effective_namespace()
   * \return The namespace of the node.
   */
  RCLCPP_PUBLIC
  const char* get_namespace() const;

  /// 获得节点的完全限定名。
  /// Get the fully-qualified name of the node.
  /**
   * 完全限定名包括节点的本地命名空间和名称。
   * The fully-qualified name includes the local namespace and name of the node.
   * \return 节点的完全限定名。
   * \return fully-qualified name of the node.
   */
  RCLCPP_PUBLIC
  const char* get_fully_qualified_name() const;

  /// 获取节点的日志记录器。
  /// Get the logger of the node.
  /** \return 节点的日志记录器。
   * \return The logger of the node. */
  RCLCPP_PUBLIC
  rclcpp::Logger get_logger() const;

  /// 创建并返回一个回调组。
  /// Create and return a callback group.
  RCLCPP_PUBLIC
  rclcpp::CallbackGroup::SharedPtr create_callback_group(
      rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node = true);

  /// 遍历节点中的回调组，对每个有效的回调组调用给定的函数。
  /// Iterate over the callback groups in the node, calling the given function on each valid one.
  /**
   * 以线程安全的方式调用此方法，并确保仅对仍然有效的项目调用给定的函数。
   * This method is called in a thread-safe way, and also makes sure to only call the given
   * function on those items that are still valid.
   *
   * \param[in] func 在每个有效回调组上调用的回调函数。
   * \param[in] func The callback function to call on each valid callback group.
   */
  RCLCPP_PUBLIC
  void for_each_callback_group(
      const node_interfaces::NodeBaseInterface::CallbackGroupFunction& func);

  /// 创建并返回一个发布者。
  /// Create and return a Publisher.
  /**
   * rclcpp::QoS 有几个方便的构造函数，包括 size_t 的转换构造函数，它模仿旧的
   * API，允许只用一个字符串和 size_t 来创建发布者。 The rclcpp::QoS has several convenient
   * constructors, including a conversion constructor for size_t, which mimics older API's that
   * allows just a string and size_t to create a publisher.
   *
   * 例如，所有这些情况都可以工作：
   * For example, all of these cases will work:
   *
   * ```cpp
   * pub = node->create_publisher<MsgT>("chatter", 10);  // implicitly KeepLast
   * pub = node->create_publisher<MsgT>("chatter", QoS(10));  // implicitly KeepLast
   * pub = node->create_publisher<MsgT>("chatter", QoS(KeepLast(10)));
   * pub = node->create_publisher<MsgT>("chatter", QoS(KeepAll()));
   * pub = node->create_publisher<MsgT>("chatter", QoS(1).best_effort().durability_volatile());
   * {
   *   rclcpp::QoS custom_qos(KeepLast(10), rmw_qos_profile_sensor_data);
   *   pub = node->create_publisher<MsgT>("chatter", custom_qos);
   * }
   * ```
   *
   * 发布者选项可以作为上述任何情况的第三个参数传递。
   * The publisher options may optionally be passed as the third argument for
   * any of the above cases.
   *
   * \param[in] topic_name 此发布者要发布的主题。
   * \param[in] qos 发布者的服务质量设置。
   * \param[in] options 创建的 Publisher 的其他选项。
   * \return 创建的发布者的共享指针。
   * \return Shared pointer to the created publisher.
   */
  template <
      typename MessageT,
      typename AllocatorT = std::allocator<void>,
      typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>>
  std::shared_ptr<PublisherT> create_publisher(
      const std::string& topic_name,
      const rclcpp::QoS& qos,
      const PublisherOptionsWithAllocator<AllocatorT>& options =
          PublisherOptionsWithAllocator<AllocatorT>());

  /// 创建并返回一个 Subscription（订阅者）。
  /**
   * \param[in] topic_name 要订阅的主题。
   * \param[in] qos 订阅者的 QoS 配置。
   * \param[in] callback 接收消息的用户定义回调函数。
   * \param[in] options 创建订阅者的附加选项。
   * \param[in] msg_mem_strat 用于分配消息的消息内存策略。
   * \return 创建的订阅者的共享指针。
   */
  /// Create and return a Subscription.
  /**
   * \param[in] topic_name The topic to subscribe on.
   * \param[in] qos QoS profile for Subcription.
   * \param[in] callback The user-defined callback function to receive a message
   * \param[in] options Additional options for the creation of the Subscription.
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
      const std::string& topic_name,
      const rclcpp::QoS& qos,
      CallbackT&& callback,
      const SubscriptionOptionsWithAllocator<AllocatorT>& options =
          SubscriptionOptionsWithAllocator<AllocatorT>(),
      typename MessageMemoryStrategyT::SharedPtr msg_mem_strat =
          (MessageMemoryStrategyT::create_default()));

  /// 使用墙时钟创建一个定时器，以驱动回调。
  /**
   * \param[in] period 回调触发之间的时间间隔。
   * \param[in] callback 用户定义的回调函数。
   * \param[in] group 执行此定时器回调的回调组。
   */
  /// Create a wall timer that uses the wall clock to drive the callback.
  /**
   * \param[in] period Time interval between triggers of the callback.
   * \param[in] callback User-defined callback function.
   * \param[in] group Callback group to execute this timer's callback in.
   */
  template <typename DurationRepT = int64_t, typename DurationT = std::milli, typename CallbackT>
  typename rclcpp::WallTimer<CallbackT>::SharedPtr create_wall_timer(
      std::chrono::duration<DurationRepT, DurationT> period,
      CallbackT callback,
      rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// 创建一个使用节点时钟驱动回调的定时器。
  /**
   * \param[in] period 回调触发之间的时间间隔。
   * \param[in] callback 用户定义的回调函数。
   * \param[in] group 执行此定时器回调的回调组。
   */
  /// Create a timer that uses the node clock to drive the callback.
  /**
   * \param[in] period Time interval between triggers of the callback.
   * \param[in] callback User-defined callback function.
   * \param[in] group Callback group to execute this timer's callback in.
   */
  template <typename DurationRepT = int64_t, typename DurationT = std::milli, typename CallbackT>
  typename rclcpp::GenericTimer<CallbackT>::SharedPtr create_timer(
      std::chrono::duration<DurationRepT, DurationT> period,
      CallbackT callback,
      rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// 创建并返回一个 Client（客户端）。
  /**
   * \param[in] service_name 要为其提供服务的主题。
   * \param[in] qos_profile 客户端的 rmw_qos_profile_t 服务质量配置。
   * \param[in] group 调用服务的回调组。
   * \return 创建的客户端的共享指针。
   * \deprecated 使用 rclcpp::QoS 代替 rmw_qos_profile_t
   */
  /// Create and return a Client.
  /**
   * \param[in] service_name The topic to service on.
   * \param[in] qos_profile rmw_qos_profile_t Quality of service profile for client.
   * \param[in] group Callback group to call the service.
   * \return Shared pointer to the created client.
   * \deprecated use rclcpp::QoS instead of rmw_qos_profile_t
   */
  template <typename ServiceT>
  [[deprecated("use rclcpp::QoS instead of rmw_qos_profile_t")]]
  typename rclcpp::Client<ServiceT>::SharedPtr
  create_client(
      const std::string& service_name,
      const rmw_qos_profile_t& qos_profile,
      rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// 创建并返回一个 Client。
  /// Create and return a Client.
  /**
   * \param[in] service_name 可访问服务的名称。
   * \param[in] service_name The name on which the service is accessible.
   * \param[in] qos 客户端的服务质量配置文件。
   * \param[in] qos Quality of service profile for client.
   * \param[in] group 用于处理对服务调用的回复的回调组。
   * \param[in] group Callback group to handle the reply to service calls.
   * \return 创建的客户端的共享指针。
   * \return Shared pointer to the created client.
   */
  template <typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr create_client(
      const std::string& service_name,
      const rclcpp::QoS& qos = rclcpp::ServicesQoS(),
      rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// 创建并返回一个 Service。
  /// Create and return a Service.
  /**
   * \param[in] service_name 要在其上提供服务的主题。
   * \param[in] service_name The topic to service on.
   * \param[in] callback 用户定义的回调函数。
   * \param[in] callback User-defined callback function.
   * \param[in] qos_profile 客户端的 rmw_qos_profile_t 服务质量配置文件。
   * \param[in] qos_profile rmw_qos_profile_t Quality of service profile for client.
   * \param[in] group 用于调用服务的回调组。
   * \param[in] group Callback group to call the service.
   * \return 创建的服务的共享指针。
   * \return Shared pointer to the created service.
   * \deprecated 使用 rclcpp::QoS 代替 rmw_qos_profile_t
   * \deprecated use rclcpp::QoS instead of rmw_qos_profile_t
   */
  template <typename ServiceT, typename CallbackT>
  [[deprecated("use rclcpp::QoS instead of rmw_qos_profile_t")]]
  typename rclcpp::Service<ServiceT>::SharedPtr
  create_service(
      const std::string& service_name,
      CallbackT&& callback,
      const rmw_qos_profile_t& qos_profile,
      rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// 创建并返回一个 Service。
  /// Create and return a Service.
  /**
   * \param[in] service_name 要在其上提供服务的主题。
   * \param[in] service_name The topic to service on.
   * \param[in] callback 用户定义的回调函数。
   * \param[in] callback User-defined callback function.
   * \param[in] qos 服务的服务质量配置文件。
   * \param[in] qos Quality of service profile for the service.
   * \param[in] group 用于调用服务的回调组。
   * \param[in] group Callback group to call the service.
   * \return 创建的服务的共享指针。
   * \return Shared pointer to the created service.
   */
  template <typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr create_service(
      const std::string& service_name,
      CallbackT&& callback,
      const rclcpp::QoS& qos = rclcpp::ServicesQoS(),
      rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// 创建并返回一个 GenericPublisher。
  /**
   * 返回的指针永远不会为空，但此函数可能抛出各种异常，
   * 例如当消息包在 AMENT_PREFIX_PATH 上找不到时。
   *
   * Create and return a GenericPublisher.
   * The returned pointer will never be empty, but this function can throw various exceptions, for
   * instance when the message's package can not be found on the AMENT_PREFIX_PATH.
   *
   * \param[in] topic_name 主题名称 (Topic name)
   * \param[in] topic_type 主题类型 (Topic type)
   * \param[in] qos QoS 设置 (%QoS settings)
   * \param options 发布者选项。(%Publisher options.)
   * 并非所有发布者选项都受到尊重，对于这个发布者来说，唯一相关的选项是
   * `event_callbacks`、`use_default_callbacks` 和 `%callback_group`。 Not all publisher options are
   * currently respected, the only relevant options for this publisher are `event_callbacks`,
   * `use_default_callbacks`, and `%callback_group`. \return 创建的通用发布者的共享指针。 (Shared
   * pointer to the created generic publisher.)
   */
  template <typename AllocatorT = std::allocator<void>>
  std::shared_ptr<rclcpp::GenericPublisher> create_generic_publisher(
      const std::string& topic_name,
      const std::string& topic_type,
      const rclcpp::QoS& qos,
      const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options =
          (rclcpp::PublisherOptionsWithAllocator<AllocatorT>()));

  /// 创建并返回一个 GenericSubscription。
  /**
   * 返回的指针永远不会为空，但此函数可能抛出各种异常，
   * 例如当消息包在 AMENT_PREFIX_PATH 上找不到时。
   *
   * Create and return a GenericSubscription.
   * The returned pointer will never be empty, but this function can throw various exceptions, for
   * instance when the message's package can not be found on the AMENT_PREFIX_PATH.
   *
   * \param[in] topic_name 主题名称 (Topic name)
   * \param[in] topic_type 主题类型 (Topic type)
   * \param[in] qos QoS 设置 (%QoS settings)
   * \param[in] callback 序列化形式的新消息回调 (Callback for new messages of serialized form)
   * \param[in] options 订阅选项。(%Subscription options.)
   * 并非所有订阅选项都受到尊重，对于这个订阅来说，唯一相关的选项是
   * `event_callbacks`、`use_default_callbacks`、`ignore_local_publications` 和 `%callback_group`。
   * Not all subscription options are currently respected, the only relevant options for this
   * subscription are `event_callbacks`, `use_default_callbacks`, `ignore_local_publications`, and
   * `%callback_group`.
   * \return 创建的通用订阅的共享指针。 (Shared pointer to the created generic subscription.)
   */
  template <typename AllocatorT = std::allocator<void>>
  std::shared_ptr<rclcpp::GenericSubscription> create_generic_subscription(
      const std::string& topic_name,
      const std::string& topic_type,
      const rclcpp::QoS& qos,
      std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options =
          (rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()));

  /// 声明并初始化一个参数，返回有效值。
  /**
   * 此方法用于声明此节点上存在参数。
   * 如果在运行时用户提供了初始值，则该值将在此方法中设置，否则将设置给定的 default_value。
   * 无论如何，都会返回生成的值，无论它是基于默认值还是用户提供的初始值。
   *
   * Declare and initialize a parameter, return the effective value.
   * This method is used to declare that a parameter exists on this node.
   * If, at run-time, the user has provided an initial value then it will be
   * set in this method, otherwise the given default_value will be set.
   * In either case, the resulting value is returned, whether or not it is
   * based on the default value or the user provided initial value.
   *
   * 如果没有给出 parameter_descriptor，则将使用来自消息定义的默认值，例如 read_only 将为 false。
   * If no parameter_descriptor is given, then the default values from the
   * message definition will be used, e.g. read_only will be false.
   *
   * 在给定的 rcl_interfaces::msg::ParameterDescriptor 中的名称和类型将被忽略，
   * 应该使用此功能的名称参数和默认值类型指定。
   * The name and type in the given rcl_interfaces::msg::ParameterDescriptor
   * are ignored, and should be specified using the name argument to this
   * function and the default value's type instead.
   *
   * 如果 `ignore_override` 为 `true`，参数覆盖将被忽略。
   * If `ignore_override` is `true`, the parameter override will be ignored.
   *
   * 此方法将导致使用 `add_on_set_parameters_callback` 和 `add_post_set_parameters_callback`
   * 注册的任何回调被调用以设置参数。
   * This method will result in any callback registered with
   * `add_on_set_parameters_callback` and `add_post_set_parameters_callback`
   * to be called for the parameter being set.
   *
   * 如果之前使用 `add_on_set_parameters_callback` 注册了回调，
   * 则在为节点设置参数之前将调用它。
   * 如果该回调阻止了参数的初始值设置，则抛出 rclcpp::exceptions::InvalidParameterValueException。
   * If a callback was registered previously with `add_on_set_parameters_callback`,
   * it will be called prior to setting the parameter for the node.
   * If that callback prevents the initial value for the parameter from being
   * set then rclcpp::exceptions::InvalidParameterValueException is thrown.
   *
   * 如果之前使用 `add_post_set_parameters_callback` 注册了回调，
   * 则在成功为节点设置参数后将调用它。
   * If a callback was registered previously with `add_post_set_parameters_callback`,
   * it will be called after setting the parameter successfully for the node.
   *
   * 此方法不会导致使用 `add_pre_set_parameters_callback` 注册的任何回调被调用。
   * This method will _not_ result in any callbacks registered with
   * `add_pre_set_parameters_callback` to be called.
   *
   * 返回的引用将保持有效，直到参数未声明。
   * The returned reference will remain valid until the parameter is
   * undeclared.
   *
   * \param[in] name 参数名称。 (The name of the parameter.)
   * \param[in] default_value 如果在运行时用户没有覆盖它，则使用初始值。 (An initial value to be
   * used if at run-time user did not override it.) \param[in] parameter_descriptor
   * 可选的参数自定义描述。 (An optional, custom description for the parameter.) \param[in]
   * ignore_override 当为 `true` 时，忽略参数覆盖。默认为 `false`。 (When `true`, the parameter
   * override is ignored. Default to `false`.) \return 参数值的常量引用。 (A const reference to the
   * value of the parameter.) \throws rclcpp::exceptions::ParameterAlreadyDeclaredException
   * 如果参数已经声明。 (if parameter has already been declared.) \throws
   * rclcpp::exceptions::InvalidParametersException 如果参数名称无效。 (if a parameter name is
   * invalid.) \throws rclcpp::exceptions::InvalidParameterValueException 如果初始值无法设置。 (if
   * initial value fails to be set.) \throws rclcpp::exceptions::InvalidParameterTypeException
   * 如果默认值或覆盖值的类型错误。 (if the type of the default value or override is wrong.)
   */
  RCLCPP_PUBLIC
  const rclcpp::ParameterValue& declare_parameter(
      const std::string& name,
      const rclcpp::ParameterValue& default_value,
      const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
          rcl_interfaces::msg::ParameterDescriptor(),
      bool ignore_override = false);

  /// 声明并初始化一个参数，返回有效值。
  /// Declare and initialize a parameter, return the effective value.
  /**
   * 与前一个方法相同，但未提供默认值，用户必须提供正确类型的参数覆盖。
   * Same as the previous one, but a default value is not provided and the user
   * must provide a parameter override of the correct type.
   *
   * \param[in] name 参数名称。The name of the parameter.
   * \param[in] type 参数的期望类型，在运行时将被强制执行。Desired type of the parameter, which will
   * be enforced at runtime. \param[in] parameter_descriptor 参数的可选自定义描述。An optional,
   * custom description for the parameter. \param[in] ignore_override 当为 `true`
   * 时，忽略参数覆盖，默认为 `false`。When `true`, the parameter override is ignored. Default to
   * `false`. \return 参数值的常量引用。A const reference to the value of the parameter. \throws
   * 与之前接受默认值的重载相同的异常。Same as the previous overload taking a default value. \throws
   * rclcpp::exceptions::InvalidParameterTypeException 如果未提供覆盖或提供的覆盖类型错误。if an
   * override is not provided or the provided override is of the wrong type.
   */
  RCLCPP_PUBLIC
  const rclcpp::ParameterValue& declare_parameter(
      const std::string& name,
      rclcpp::ParameterType type,
      const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
          rcl_interfaces::msg::ParameterDescriptor{},
      bool ignore_override = false);

  /// 使用类型声明并初始化一个参数。
  /// Declare and initialize a parameter with a type.
  /**
   * 有关详细信息，请参阅此类上的非模板化 declare_parameter()。
   * See the non-templated declare_parameter() on this class for details.
   *
   * 如果默认值的类型（因此返回值的类型）与节点选项中提供的初始值不同，
   * 则可能会抛出 rclcpp::exceptions::InvalidParameterTypeException 异常。
   * 要避免这种情况，请使用返回 rclcpp::ParameterValue 的 declare_parameter() 方法。
   * If the type of the default value, and therefore also the type of return
   * value, differs from the initial value provided in the node options, then
   * a rclcpp::exceptions::InvalidParameterTypeException may be thrown.
   * To avoid this, use the declare_parameter() method which returns an
   * rclcpp::ParameterValue instead.
   *
   * 注意，此方法不能返回 const 引用，因为临时变量的生命周期只能在成员初始化器中递归扩展，
   * 并且不能扩展到返回的类的成员。
   * 此类的返回值是由其他版本的 declare_parameter() 返回的 ParameterValue 的成员的副本。
   * 另请参阅：
   * Note, this method cannot return a const reference, because extending the
   * lifetime of a temporary only works recursively with member initializers,
   * and cannot be extended to members of a class returned.
   * The return value of this class is a copy of the member of a ParameterValue
   * which is returned by the other version of declare_parameter().
   * See also:
   *
   *   - https://en.cppreference.com/w/cpp/language/lifetime
   *   - https://herbsutter.com/2008/01/01/gotw-88-a-candidate-for-the-most-important-const/
   *   - https://www.youtube.com/watch?v=uQyT-5iWUow (cppnow 2018 presentation)
   */
  template <typename ParameterT>
  auto declare_parameter(
      const std::string& name,
      const ParameterT& default_value,
      const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
          rcl_interfaces::msg::ParameterDescriptor(),
      bool ignore_override = false);

  /// 使用类型声明并初始化一个参数。
  /// Declare and initialize a parameter with a type.
  /**
   * 有关详细信息，请参阅此类上的非模板化 declare_parameter()。
   * See the non-templated declare_parameter() on this class for details.
   */
  template <typename ParameterT>
  auto declare_parameter(
      const std::string& name,
      const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
          rcl_interfaces::msg::ParameterDescriptor(),
      bool ignore_override = false);

  /// 使用相同的命名空间和类型声明并初始化多个参数。
  /// Declare and initialize several parameters with the same namespace and type.
  /**
   * 对于映射中的每个键，将设置一个名为 "namespace.key" 的参数，其值为映射中的值。
   * 每个声明参数的结果值将被返回。
   * For each key in the map, a parameter with a name of "namespace.key"
   * will be set to the value in the map.
   * The resulting value for each declared parameter will be returned.
   *
   * 名称扩展是简单的，所以如果将命名空间设置为 "foo."，
   * 那么生成的参数名称将类似于 "foo..key"。
   * 但是，如果命名空间为空字符串，则在每个键之前不会放置前导 '.'，
   * 这将是简单地扩展 "namespace.key" 时的情况。
   * 这允许您一次声明多个没有命名空间的参数。
   * The name expansion is naive, so if you set the namespace to be "foo.",
   * then the resulting parameter names will be like "foo..key".
   * However, if the namespace is an empty string, then no leading '.' will be
   * placed before each key, which would have been the case when naively
   * expanding "namespace.key".
   * This allows you to declare several parameters at once without a namespace.
   *
   * 映射包含参数的默认值。
   * 还有另一个重载，它采用具有默认值和描述符的 std::pair。
   * The map contains default values for parameters.
   * There is another overload which takes the std::pair with the default value
   * and descriptor.
   *
   * 如果 `ignore_overrides` 为 `true`，则函数调用声明的所有参数的覆盖都将被忽略。
   * If `ignore_overrides` is `true`, all the overrides of the parameters declared
   * by the function call will be ignored.
   *
   * 此方法将导致使用
   * `add_on_set_parameters_callback` 和 `add_post_set_parameters_callback`
   * 注册的任何回调被调用一次。
   * This method will result in any callback registered with
   * `add_on_set_parameters_callback` and `add_post_set_parameters_callback`
   * to be called once for each parameter.
   *
   * 如果成功，此方法将导致使用 `add_on_set_parameters_callback` 注册的任何回调被调用，
   * 每个参数一次。
   * 如果该回调阻止任何参数的初始值被设置，则抛出 rclcpp::exceptions::InvalidParameterValueException
   * 异常。 This method, if successful, will result in any callback registered with
   * `add_on_set_parameters_callback` to be called, once for each parameter.
   * If that callback prevents the initial value for any parameter from being
   * set then rclcpp::exceptions::InvalidParameterValueException is thrown.
   *
   * 如果之前使用 `add_post_set_parameters_callback` 注册了回调，
   * 则在为节点成功设置参数后，将对每个参数调用一次。
   * If a callback was registered previously with `add_post_set_parameters_callback`,
   * it will be called after setting the parameters successfully for the node,
   * once for each parameter.
   *
   * 此方法 _不会_ 导致使用 `add_pre_set_parameters_callback` 注册的任何回调被调用。
   * This method will _not_ result in any callbacks registered with
   * `add_pre_set_parameters_callback` to be called.
   *
   * \param[in] namespace_ 声明参数的命名空间。The namespace in which to declare the parameters.
   * \param[in] parameters 要在给定命名空间中设置的参数。The parameters to set in the given
   * namespace. \param[in] ignore_overrides 当为 `true` 时，忽略参数覆盖，默认为 `false`。When
   * `true`, the parameters overrides are ignored. Default to `false`. \throws
   * rclcpp::exceptions::ParameterAlreadyDeclaredException 如果参数已经声明。if parameter has
   * already been declared. \throws rclcpp::exceptions::InvalidParametersException
   * 如果参数名称无效。if a parameter name is invalid. \throws
   * rclcpp::exceptions::InvalidParameterValueException 如果初始值无法设置。if initial value fails
   * to be set.
   */
  template <typename ParameterT>
  std::vector<ParameterT> declare_parameters(
      const std::string& namespace_,
      const std::map<std::string, ParameterT>& parameters,
      bool ignore_overrides = false);

  /// 声明并初始化具有相同命名空间和类型的多个参数。
  /// Declare and initialize several parameters with the same namespace and type.
  /**
   * 该版本将采用一个映射，其中值是一个对，第一个项为默认参数值，第二个项为参数描述符。
   * This version will take a map where the value is a pair, with the default
   * parameter value as the first item and a parameter descriptor as the second.
   *
   * 有关更多详细信息，请参阅此类上的简化版 declare_parameters() 。
   * See the simpler declare_parameters() on this class for more details.
   */
  template <typename ParameterT>
  std::vector<ParameterT> declare_parameters(
      const std::string& namespace_,
      const std::map<std::string, std::pair<ParameterT, rcl_interfaces::msg::ParameterDescriptor>>&
          parameters,
      bool ignore_overrides = false);

  /// 取消声明以前声明的参数。
  /// Undeclare a previously declared parameter.
  /**
   * 此方法将 _not_ 导致使用任何 `add_pre_set_parameters_callback`、`add_on_set_parameters_callback`
   * 和 `add_post_set_parameters_callback` 注册的回调被调用。 This method will _not_ cause a
   * callback registered with any of the `add_pre_set_parameters_callback`,
   * `add_on_set_parameters_callback` and `add_post_set_parameters_callback` to be called.
   *
   * \param[in] name 要取消声明的参数的名称。
   * \param[in] name The name of the parameter to be undeclared.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException 如果参数尚未声明。
   * \throws rclcpp::exceptions::ParameterNotDeclaredException if the parameter
   *   has not been declared.
   * \throws rclcpp::exceptions::ParameterImmutableException 如果参数是只读（不可变）的。
   * \throws rclcpp::exceptions::ParameterImmutableException if the parameter
   *   was created as read_only (immutable).
   */
  RCLCPP_PUBLIC
  void undeclare_parameter(const std::string& name);

  /// 如果给定参数已声明，则返回 true。
  /// Return true if a given parameter is declared.
  /**
   * \param[in] name 要检查是否已声明的参数的名称。
   * \param[in] name The name of the parameter to check for being declared.
   * \return 如果参数名称已声明，则为 true，否则为 false。
   * \return true if the parameter name has been declared, otherwise false.
   */
  RCLCPP_PUBLIC
  bool has_parameter(const std::string& name) const;

  /// 设置单个参数。
  /// Set a single parameter.
  /**
   * 设置给定参数，然后返回设置操作的结果。
   * Set the given parameter and then return result of the set action.
   *
   * 如果参数尚未声明，此函数可能会抛出 rclcpp::exceptions::ParameterNotDeclaredException 异常，
   * 但仅当节点没有使用 rclcpp::NodeOptions::allow_undeclared_parameters 设置为 true 创建时。
   * If the parameter has not been declared this function may throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception, but only if
   * the node was not created with the
   * rclcpp::NodeOptions::allow_undeclared_parameters set to true.
   * 如果允许未声明的参数，则在设置之前，参数将使用默认参数元数据隐式声明。
   * If undeclared parameters are allowed, then the parameter is implicitly
   * declared with the default parameter meta data before being set.
   * set_parameter 忽略参数覆盖。
   * Parameter overrides are ignored by set_parameter.
   *
   * 此方法将导致使用 `add_pre_set_parameters_callback`、add_on_set_parameters_callback` 和
   * `add_post_set_parameters_callback` 注册的任何回调被调用一次，以设置参数。
   * This method will result in any callback registered with
   * `add_pre_set_parameters_callback`, add_on_set_parameters_callback` and
   * `add_post_set_parameters_callback` to be called once for the parameter
   * being set.
   *
   * 此方法将导致使用 `add_on_set_parameters_callback` 注册的任何回调被调用。
   * This method will result in any callback registered with
   * `add_on_set_parameters_callback` to be called.
   * 如果回调阻止了参数设置，则它将反映在返回的 SetParametersResult 中，但不会抛出异常。
   * If the callback prevents the parameter from being set, then it will be
   * reflected in the SetParametersResult that is returned, but no exception
   * will be thrown.
   *
   * 如果先前使用 `add_pre_set_parameters_callback` 注册了一个回调，
   * 则在验证节点的参数之前，该回调将被调用一次。
   * If a callback was registered previously with `add_pre_set_parameters_callback`,
   * it will be called once prior to the validation of the parameter for the node.
   * 如果此回调使修改后的参数列表为空，则将反映在返回的结果中；在这种情况下不会引发异常。
   * If this callback makes modified parameter list empty, then it will be reflected
   * in the returned result; no exceptions will be raised in this case.
   *
   * 如果先前使用 `add_post_set_parameters_callback` 注册了一个回调，
   * 则在节点成功设置参数后，该回调将被调用一次。
   * If a callback was registered previously with `add_post_set_parameters_callback`,
   * it will be called once after setting the parameter successfully for the node.
   *
   * 如果参数的值类型为 rclcpp::PARAMETER_NOT_SET，并且现有参数类型为其他类型，
   * 则参数将隐式取消声明。
   * If the value type of the parameter is rclcpp::PARAMETER_NOT_SET, and the
   * existing parameter type is something else, then the parameter will be
   * implicitly undeclared.
   * 这将导致参数事件表明参数已删除。
   * This will result in a parameter event indicating that the parameter was
   * deleted.
   *
   * \param[in] parameter 要设置的参数。
   * \param[in] parameter The parameter to be set.
   * \return 设置操作的结果。
   * \return The result of the set action.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException
   * 如果参数尚未声明，且不允许未声明的参数。 \throws
   * rclcpp::exceptions::ParameterNotDeclaredException if the parameter has not been declared and
   * undeclared parameters are not allowed.
   */
  RCLCPP_PUBLIC
  rcl_interfaces::msg::SetParametersResult set_parameter(const rclcpp::Parameter& parameter);

  /// 设置一个或多个参数，逐个设置。
  ///
  /// Set one or more parameters, one at a time.
  /**
   * 依次设置给定的参数，然后返回每个设置操作的结果。
   *
   * Set the given parameters, one at a time, and then return result of each set action.
   *
   * 参数按照输入向量中给定的顺序进行设置。
   *
   * Parameters are set in the order they are given within the input vector.
   *
   * 与 set_parameter 类似，如果要设置的任何参数都没有先声明，
   * 并且不允许使用未声明的参数（默认情况下），
   * 那么此方法将抛出 rclcpp::exceptions::ParameterNotDeclaredException。
   *
   * Like set_parameter, if any of the parameters to be set have not first been
   * declared, and undeclared parameters are not allowed (the default), then
   * this method will throw rclcpp::exceptions::ParameterNotDeclaredException.
   *
   * 如果由于未声明而导致设置参数失败，
   * 则已经设置的参数将保持设置状态，并且不会尝试设置之后的参数。
   *
   * If setting a parameter fails due to not being declared, then the
   * parameters which have already been set will stay set, and no attempt will
   * be made to set the parameters which come after.
   *
   * 如果由于其他原因导致参数设置失败，例如被用户的回调拒绝
   * （基本上是除了没有事先声明以外的任何原因），那么这将反映在
   * 此函数返回的向量中相应的 SetParametersResult 中。
   *
   * If a parameter fails to be set due to any other reason, like being
   * rejected by the user's callback (basically any reason other than not
   * having been declared beforehand), then that is reflected in the
   * corresponding SetParametersResult in the vector returned by this function.
   *
   * 此方法将导致使用
   * `add_pre_set_parameters_callback`、`add_on_set_parameters_callback` 和
   * `add_post_set_parameters_callback` 注册的任何回调被调用一次。
   *
   * This method will result in any callback registered with
   * `add_pre_set_parameters_callback`, `add_on_set_parameters_callback` and
   * `add_post_set_parameters_callback` to be called once for each parameter.

  * 如果先前使用 `add_pre_set_parameters_callback` 注册了回调，
  * 则在验证节点参数之前，每个参数会调用一次此回调。
  * 如果此回调使修改后的参数列表为空，则将反映在返回的结果中；
  * 在这种情况下不会引发任何异常。
  *
  * If a callback was registered previously with `add_pre_set_parameters_callback`,
  * it will be called prior to the validation of parameters for the node,
  * once for each parameter.
  * If this callback makes modified parameter list empty, then it will be reflected
  * in the returned result; no exceptions will be raised in this case.
  *
  * 此方法将导致使用
  * `add_on_set_parameters_callback` 注册的任何回调被调用一次。
  * 如果回调阻止参数设置，则如前所述，它将反映在返回的相应 SetParametersResult 中，
  * 但不会抛出异常。
  *
  * This method will result in any callback registered with
  * `add_on_set_parameters_callback` to be called, once for each parameter.
  * If the callback prevents the parameter from being set, then, as mentioned
  * before, it will be reflected in the corresponding SetParametersResult
  * that is returned, but no exception will be thrown.
  *
  * 如果先前使用 `add_post_set_parameters_callback` 注册了回调，
  * 则在成功设置节点参数后，每个参数会调用一次此回调。
  *
  * If a callback was registered previously with `add_post_set_parameters_callback`,
  * it will be called after setting the parameters successfully for the node,
  * once for each parameter.
  *
  * 与 set_parameter() 类似，此方法将隐式地取消声明类型为 rclcpp::PARAMETER_NOT_SET 的参数。
  *
  * Like set_parameter() this method will implicitly undeclare parameters
  * with the type rclcpp::PARAMETER_NOT_SET.
  *
  * \param[in] parameters 要设置的参数向量。
  *
  * \param[in] parameters The vector of parameters to be set.
  * \return 每个设置操作的结果作为向量。
  *
  * \return The results for each set action as a vector.
  * \throws rclcpp::exceptions::ParameterNotDeclaredException 如果任何参数
  *   尚未声明且不允许使用未声明的参数。
  *
  * \throws rclcpp::exceptions::ParameterNotDeclaredException if any parameter
  *   has not been declared and undeclared parameters are not allowed.
  */
  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
      const std::vector<rclcpp::Parameter>& parameters);

  /// 一次性设置一个或多个参数。
  ///
  /// Set one or more parameters, all at once.
  /**
   * 一次性设置给定的参数，然后聚合结果。
   *
   * Set the given parameters, all at one time, and then aggregate result.
   *
   * 行为类似于 set_parameter，只是它设置多个参数，
   * 如果其中一个参数设置失败，则所有参数都会失败。
   * 要么设置所有参数，要么一个都不设置。
   *
   * Behaves like set_parameter, except that it sets multiple parameters,
   * failing all if just one of the parameters are unsuccessfully set.
   * Either all of the parameters are set or none of them are set.
   *
   * 与 set_parameter 和 set_parameters 类似，如果要设置的任何参数
   * 都没有先声明，此方法可能会抛出 rclcpp::exceptions::ParameterNotDeclaredException 异常。
   * 如果抛出异常，则不会设置任何参数。
   *
   * Like set_parameter and set_parameters, this method may throw an
   * rclcpp::exceptions::ParameterNotDeclaredException exception if any of the
   * parameters to be set have not first been declared.
   * If the exception is thrown then none of the parameters will have been set.
   *
   * 此方法将导致使用
   * `add_pre_set_parameters_callback`、`add_on_set_parameters_callback` 和
   * `add_post_set_parameters_callback` 注册的任何回调只对所有参数调用“一次”。
   *
   * This method will result in any callback registered with
   * `add_pre_set_parameters_callback`, `add_on_set_parameters_callback` and
   * `add_post_set_parameters_callback` to be called only 'once' for all parameters.
   *
   * 如果先前使用 `add_pre_set_parameters_callback` 注册了回调，
   * 则在验证节点参数之前，所有参数只调用此回调一次。
   * 如果此回调使修改后的参数列表为空，则将反映在返回的结果中；
   * 在这种情况下不会引发任何异常。
   *
   * If a callback was registered previously with `add_pre_set_parameters_callback`,
   * it will be called prior to the validation of node parameters, just one time
   * for all parameters.
   * If this callback makes modified parameter list empty, then it will be reflected
   * in the returned result; no exceptions will be raised in this case.
   *
   * 此方法将导致使用
   * 'add_on_set_parameters_callback' 注册的任何回调仅调用一次。
   * 如果回调阻止参数设置，则它将反映在返回的 SetParametersResult 中，
   * 但不会抛出异常。
   *
   * This method will result in any callback registered with
   * 'add_on_set_parameters_callback' to be called, just one time.
   * If the callback prevents the parameters from being set, then it will be
   * reflected in the SetParametersResult which is returned, but no exception
   * will be thrown.
   *
   * 如果先前使用 `add_post_set_parameters_callback` 注册了回调，
   * 则在成功设置节点参数后，所有参数只调用此回调一次。
   *
   * If a callback was registered previously with `add_post_set_parameters_callback`,
   * it will be called after setting the node parameters successfully, just one time
   * for all parameters.
   *
   * 如果传递多个具有相同名称的 rclcpp::Parameter 实例，
   * 那么向量中的最后一个（正向迭代）将被设置。
   *
   * If you pass multiple rclcpp::Parameter instances with the same name, then
   * only the last one in the vector (forward iteration) will be set.
   *
   * 与 set_parameter() 类似，此方法将隐式地取消声明类型为 rclcpp::PARAMETER_NOT_SET 的参数。
   *
   * Like set_parameter() this method will implicitly undeclare parameters
   * with the type rclcpp::PARAMETER_NOT_SET.
   *
   * \param[in] parameters 要设置的参数向量。
   *
   * \param[in] parameters The vector of parameters to be set.
   * \return 设置所有参数的聚合结果。
   *
   * \return The aggregate result of setting all the parameters atomically.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException 如果任何参数
   *   尚未声明且不允许使用未声明的参数。
   *
   * \throws rclcpp::exceptions::ParameterNotDeclaredException if any parameter
   *   has not been declared and undeclared parameters are not allowed.
   */
  RCLCPP_PUBLIC
  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
      const std::vector<rclcpp::Parameter>& parameters);

  /// 返回给定名称的参数。
  /// Return the parameter by the given name.
  /**
   * 如果参数尚未声明，则此方法可能会抛出 rclcpp::exceptions::ParameterNotDeclaredException 异常。
   * If the parameter has not been declared, then this method may throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception.
   *
   * 如果参数尚未初始化，则此方法可能会抛出 rclcpp::exceptions::ParameterUninitializedException
   * 异常。 If the parameter has not been initialized, then this method may throw the
   * rclcpp::exceptions::ParameterUninitializedException exception.
   *
   * 如果允许使用未声明的参数，请参见节点选项 rclcpp::NodeOptions::allow_undeclared_parameters，
   * 那么此方法将不会抛出 rclcpp::exceptions::ParameterNotDeclaredException
   * 异常，而是返回默认初始化的 rclcpp::Parameter， 它具有 rclcpp::ParameterType::PARAMETER_NOT_SET
   * 类型。 If undeclared parameters are allowed, see the node option
   * rclcpp::NodeOptions::allow_undeclared_parameters, then this method will
   * not throw the rclcpp::exceptions::ParameterNotDeclaredException exception,
   * and instead return a default initialized rclcpp::Parameter, which has a type of
   * rclcpp::ParameterType::PARAMETER_NOT_SET.
   *
   * \param[in] name 要获取的参数的名称。
   * \param[in] name The name of the parameter to get.
   * \return 请求的参数在 rclcpp 参数对象内。
   * \return The requested parameter inside of a rclcpp parameter object.
   * \throws rclcpp::exceptions::ParameterNotDeclaredException
   * 如果参数尚未声明且不允许使用未声明的参数。 \throws
   * rclcpp::exceptions::ParameterNotDeclaredException if the parameter has not been declared and
   * undeclared parameters are not allowed. \throws
   * rclcpp::exceptions::ParameterUninitializedException 如果参数尚未初始化。 \throws
   * rclcpp::exceptions::ParameterUninitializedException if the parameter has not been initialized.
   */
  RCLCPP_PUBLIC
  rclcpp::Parameter get_parameter(const std::string& name) const;

  /// 根据给定名称获取参数值，并在设置时返回 true。
  /// Get the value of a parameter by the given name, and return true if it was set.
  /**
   * 此方法永远不会抛出 rclcpp::exceptions::ParameterNotDeclaredException 异常，
   * 但如果参数尚未先前声明，将返回 false。
   * This method will never throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception, but will
   * instead return false if the parameter has not be previously declared.
   *
   * 如果参数未声明，则此方法称为 "parameter" 的输出参数将不会被赋值。
   * If the parameter was not declared, then the output argument for this
   * method which is called "parameter" will not be assigned a value.
   * 如果参数已声明，因此具有值，则将其分配到此方法的 "parameter" 参数中。
   * If the parameter was declared, and therefore has a value, then it is
   * assigned into the "parameter" argument of this method.
   *
   * \param[in] name 要获取的参数的名称。
   * \param[in] name The name of the parameter to get.
   * \param[out] parameter 用于存储正在检索的参数的输出。
   * \param[out] parameter The output storage for the parameter being retrieved.
   * \return 如果先前声明了参数，则为 true，否则为 false。
   * \return true if the parameter was previously declared, otherwise false.
   */
  RCLCPP_PUBLIC
  bool get_parameter(const std::string& name, rclcpp::Parameter& parameter) const;

  /// 根据给定名称获取参数值，并在设置时返回 true。
  /// Get the value of a parameter by the given name, and return true if it was set.
  /**
   * 与此方法的非模板版本相同，只是在分配名为 "parameter" 的输出参数时，
   * 此方法将尝试将参数值强制转换为由给定模板参数请求的类型，这可能会失败并抛出异常。
   * Identical to the non-templated version of this method, except that when
   * assigning the output argument called "parameter", this method will attempt
   * to coerce the parameter value into the type requested by the given
   * template argument, which may fail and throw an exception.
   *
   * 如果参数尚未声明，它将不会尝试将值强制转换为请求的类型，因为已知类型尚未设置。
   * If the parameter has not been declared, it will not attempt to coerce the
   * value into the requested type, as it is known that the type is not set.
   *
   * \throws rclcpp::ParameterTypeException 如果请求的类型与存储的参数值不匹配。
   * \throws rclcpp::ParameterTypeException if the requested type does not
   *   match the value of the parameter which is stored.
   */
  template <typename ParameterT>
  bool get_parameter(const std::string& name, ParameterT& parameter) const;

  /// 获取参数值，如果未设置，则获取 "alternative_value" 并将其分配给 "parameter"。
  /// Get the parameter value, or the "alternative_value" if not set, and assign it to "parameter".
  /**
   * 如果参数未设置，则 "parameter" 参数被分配 "alternative_value"。
   * If the parameter was not set, then the "parameter" argument is assigned
   * the "alternative_value".
   *
   * 与返回 bool 的 get_parameter() 版本一样，此方法不会抛出
   * rclcpp::exceptions::ParameterNotDeclaredException 异常。 Like the version of get_parameter()
   * which returns a bool, this method will not throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception.
   *
   * 在所有情况下，参数从未在节点中设置或声明。
   * In all cases, the parameter is never set or declared within the node.
   *
   * \param[in] name 要获取的参数的名称。
   * \param[in] name The name of the parameter to get.
   * \param[out] parameter 应分配参数值的输出。
   * \param[out] parameter The output where the value of the parameter should be assigned.
   * \param[in] alternative_value 如果参数未设置，则应存储在输出中的值。
   * \param[in] alternative_value Value to be stored in output if the parameter was not set.
   * \returns 如果参数已设置，则为 true，否则为 false。
   * \returns true if the parameter was set, false otherwise.
   */
  template <typename ParameterT>
  bool get_parameter_or(
      const std::string& name, ParameterT& parameter, const ParameterT& alternative_value) const;

  /// 返回参数值，如果未设置，则返回 "alternative_value"。
  /// (Return the parameter value, or the "alternative_value" if not set.)
  /**
   * 如果参数未设置，则返回 "alternative_value" 参数。
   * (If the parameter was not set, then the "alternative_value" argument is returned.)
   *
   * 此方法不会抛出 rclcpp::exceptions::ParameterNotDeclaredException 异常。
   * (This method will not throw the rclcpp::exceptions::ParameterNotDeclaredException exception.)
   *
   * 在所有情况下，节点内都不会设置或声明参数。
   * (In all cases, the parameter is never set or declared within the node.)
   *
   * \param[in] name 要获取的参数的名称。(The name of the parameter to get.)
   * \param[in] alternative_value 如果参数未设置，则存储在输出中的值。(Value to be stored in output
   * if the parameter was not set.) \returns 参数的值。(The value of the parameter.)
   */
  template <typename ParameterT>
  ParameterT get_parameter_or(const std::string& name, const ParameterT& alternative_value) const;

  /// 根据给定的参数名称返回参数。
  /// (Return the parameters by the given parameter names.)
  /**
   * 与 get_parameter(const std::string &) 类似，如果请求的参数尚未声明且不允许使用未声明的参数，
   * 则此方法可能会抛出 rclcpp::exceptions::ParameterNotDeclaredException 异常，并可能抛出
   * rclcpp::exceptions::ParameterUninitializedException 异常。
   * (Like get_parameter(const std::string &), this method may throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception if the
   * requested parameter has not been declared and undeclared parameters are
   * not allowed, and may throw the rclcpp::exceptions::ParameterUninitializedException exception.)
   *
   * 同样，如果允许使用未声明的参数且参数尚未声明，
   * 则相应的 rclcpp::Parameter 将默认初始化，因此具有
   * rclcpp::ParameterType::PARAMETER_NOT_SET 类型。
   * (Also like get_parameter(const std::string &), if undeclared parameters are allowed and the
   * parameter has not been declared, then the corresponding rclcpp::Parameter
   * will be default initialized and therefore have the type
   * rclcpp::ParameterType::PARAMETER_NOT_SET.)
   *
   * \param[in] names 要检索的参数的名称。(The names of the parameters to be retrieved.)
   * \return 检索到的参数。(The parameters that were retrieved.)
   * \throws rclcpp::exceptions::ParameterNotDeclaredException
   * 如果任何参数尚未声明且不允许使用未声明的参数。 (if any of the parameters have not been declared
   * and undeclared parameters are not allowed.) \throws
   * rclcpp::exceptions::ParameterUninitializedException 如果任何参数尚未初始化。 (if any of the
   * parameters have not been initialized.)
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string>& names) const;

  /// 获取所有具有给定前缀的参数的参数值。
  /// (Get the parameter values for all parameters that have a given prefix.)
  /**
   * "prefix" 参数用于列出具有该前缀的参数，请参阅 list_parameters()。
   * (The "prefix" argument is used to list the parameters which are prefixed
   * with that prefix, see also list_parameters().)
   *
   * 结果参数名称列表用于获取参数值。
   * (The resulting list of parameter names are used to get the values of the
   * parameters.)
   *
   * 作为键的名称在 values 映射中具有前缀删除。
   * 例如，如果使用前缀 "foo" 和参数 "foo.ping" 和 "foo.pong" 存在，
   * 则返回的映射将具有键 "ping" 和 "pong"。
   * (The names which are used as keys in the values map have the prefix removed.
   * For example, if you use the prefix "foo" and the parameters "foo.ping" and
   * "foo.pong" exist, then the returned map will have the keys "ping" and
   * "pong".)
   *
   * 前缀为空字符串将匹配所有参数。
   * (An empty string for the prefix will match all parameters.)
   *
   * 如果找不到具有前缀的参数，则输出参数 "values" 将保持不变，并返回 false。
   * 否则，参数名称和值将存储在映射中，并返回 true 以指示 "values" 已更改。
   * (If no parameters with the prefix are found, then the output parameter
   * "values" will be unchanged and false will be returned.
   * Otherwise, the parameter names and values will be stored in the map and
   * true will be returned to indicate "values" was mutated.)
   *
   * 此方法永远不会抛出 rclcpp::exceptions::ParameterNotDeclaredException 异常，
   * 因为列出参数的操作与获取值同时进行，因此只有在已声明的情况下才会列出它们，
   * 并且在检索之前无法取消声明。
   * (This method will never throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception because the
   * action of listing the parameters is done atomically with getting the
   * values, and therefore they are only listed if already declared and cannot
   * be undeclared before being retrieved.)
   *
   * 与模板 get_parameter() 变体一样，此方法将尝试将参数值强制转换为给定模板参数请求的类型，
   * 这可能会失败并抛出异常。
   * (Like the templated get_parameter() variant, this method will attempt to
   * coerce the parameter values into the type requested by the given
   * template argument, which may fail and throw an exception.)
   *
   * \param[in] prefix 要获取的参数的前缀。(The prefix of the parameters to get.)
   * \param[out] values 用于存储参数名称和值的映射，每个匹配前缀的参数一个条目。
   * (The map used to store the parameter names and values,
   *   respectively, with one entry per parameter matching prefix.)
   * \returns 如果输出 "values" 已更改，则为 true；否则为 false。
   * (true if output "values" was changed, false otherwise.)
   * \throws rclcpp::ParameterTypeException 如果请求的类型与存储的参数值不匹配。
   * (if the requested type does not match the value of the parameter which is stored.)
   */
  template <typename ParameterT>
  bool get_parameters(const std::string& prefix, std::map<std::string, ParameterT>& values) const;

  /// 返回给定参数名称的参数描述符。
  /// Return the parameter descriptor for the given parameter name.
  /**
   * 与 get_parameters() 类似，如果请求的参数未声明且不允许未声明的参数，此方法可能会抛出
   * rclcpp::exceptions::ParameterNotDeclaredException 异常。
   * Like get_parameters(), this method may throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception if the
   * requested parameter has not been declared and undeclared parameters are
   * not allowed.
   *
   * 如果允许未声明的参数，则返回默认初始化的描述符。
   * If undeclared parameters are allowed, then a default initialized
   * descriptor will be returned.
   *
   * \param[in] name 要描述的参数的名称。The name of the parameter to describe.
   * \return 给定参数名称的描述符。The descriptor for the given parameter name.
   * \throws 如果参数未声明且不允许未声明的参数，则抛出
   * rclcpp::exceptions::ParameterNotDeclaredException。
   *   rclcpp::exceptions::ParameterNotDeclaredException if the
   *   parameter has not been declared and undeclared parameters are not
   *   allowed.
   * \throws 如果描述的参数数量超过一个，则抛出 std::runtime_error。
   *   std::runtime_error if the number of described parameters is more than one
   */
  RCLCPP_PUBLIC
  rcl_interfaces::msg::ParameterDescriptor describe_parameter(const std::string& name) const;

  /// 返回参数描述符向量，每个给定名称有一个。
  /// Return a vector of parameter descriptors, one for each of the given names.
  /**
   * 与 get_parameters() 类似，如果请求的任何参数未声明且不允许未声明的参数，此方法可能会抛出
   * rclcpp::exceptions::ParameterNotDeclaredException 异常。
   * Like get_parameters(), this method may throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception if any of the
   * requested parameters have not been declared and undeclared parameters are
   * not allowed.
   *
   * 如果允许未声明的参数，则对于未声明参数的描述符，将返回默认初始化的描述符。
   * If undeclared parameters are allowed, then a default initialized
   * descriptor will be returned for the undeclared parameter's descriptor.
   *
   * 如果 names 向量为空，则返回空向量。
   * If the names vector is empty, then an empty vector will be returned.
   *
   * \param[in] names 要描述的参数名称列表。The list of parameter names to describe.
   * \return 参数描述符列表，每个给定参数一个。A list of parameter descriptors, one for each
   * parameter given. \throws 如果任何参数未声明且不允许未声明的参数，则抛出
   * rclcpp::exceptions::ParameterNotDeclaredException。
   *   rclcpp::exceptions::ParameterNotDeclaredException if any of the
   *   parameters have not been declared and undeclared parameters are not
   *   allowed.
   * \throws 如果描述的参数数量超过一个，则抛出 std::runtime_error。
   *   std::runtime_error if the number of described parameters is more than one
   */
  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
      const std::vector<std::string>& names) const;

  /// 返回参数类型向量，每个给定名称有一个。
  /// Return a vector of parameter types, one for each of the given names.
  /**
   * 与 get_parameters() 类似，如果请求的任何参数未声明且不允许未声明的参数，此方法可能会抛出
   * rclcpp::exceptions::ParameterNotDeclaredException 异常。
   * Like get_parameters(), this method may throw the
   * rclcpp::exceptions::ParameterNotDeclaredException exception if any of the
   * requested parameters have not been declared and undeclared parameters are
   * not allowed.
   *
   * 如果允许未声明的参数，则将返回默认类型 rclcpp::ParameterType::PARAMETER_NOT_SET。
   * If undeclared parameters are allowed, then the default type
   * rclcpp::ParameterType::PARAMETER_NOT_SET will be returned.
   *
   * \param[in] names 要获取类型的参数名称列表。The list of parameter names to get the types.
   * \return 参数类型列表，每个给定参数一个。A list of parameter types, one for each parameter
   * given. \throws 如果任何参数未声明且不允许未声明的参数，则抛出
   * rclcpp::exceptions::ParameterNotDeclaredException。
   *   rclcpp::exceptions::ParameterNotDeclaredException if any of the
   *   parameters have not been declared and undeclared parameters are not
   *   allowed.
   */
  RCLCPP_PUBLIC
  std::vector<uint8_t> get_parameter_types(const std::vector<std::string>& names) const;

  /// 返回具有给定前缀的参数列表，最多到给定深度。
  /// Return a list of parameters with any of the given prefixes, up to the given depth.
  /**
   * \todo: 正确编写文档并测试此方法。
   * \todo: properly document and test this method.
   */
  RCLCPP_PUBLIC
  rcl_interfaces::msg::ListParametersResult list_parameters(
      const std::vector<std::string>& prefixes, uint64_t depth) const;

  // 定义各种回调处理句柄和类型
  // Define various callback handle and types
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

  /// 在验证参数之前触发的回调
  /// Add a callback that gets triggered before parameters are validated.
  /**
   * 该回调可用于修改用户设置的原始参数列表。
   * This callback can be used to modify the original list of parameters being
   * set by the user.
   *
   * 修改后的参数列表将转发给 "on set parameter" 回调进行验证。
   * The modified list of parameters is then forwarded to the "on set parameter"
   * callback for validation.
   *
   * 当调用任何 `set_parameter*` 方法或接收到设置参数服务请求时，将调用该回调。
   * The callback is called whenever any of the `set_parameter*` methods are called
   * or when a set parameter service request is received.
   *
   * 回调接受一个要设置的参数向量的引用。
   * The callback takes a reference to the vector of parameters to be set.
   *
   * 回调可以修改参数向量。
   * The vector of parameters may be modified by the callback.
   *
   * “预设回调”的一个用例可以是根据参数更改更新附加参数。
   * One of the use case of "pre set callback" can be updating additional parameters
   * conditioned on changes to a parameter.
   *
   * 用户应保留返回的共享指针副本，因为只有在智能指针活动期间回调才有效。
   * Users should retain a copy of the returned shared pointer, as the callback
   * is valid only as long as the smart pointer is alive.
   *
   * 例如回调：
   * For an example callback:
   *
   *```cpp
   * void
   * preSetParameterCallback(std::vector<rclcpp::Parameter> & parameters)
   * {
   *  for (auto & param : parameters) {
   *    if (param.get_name() == "param1") {
   *      parameters.push_back(rclcpp::Parameter("param2", 4.0));
   *    }
   *  }
   * }
   * ```
   * 如果用户正在设置 'param1'，则上述回调会将 'param2' 附加到要设置的参数列表中。
   * The above callback appends 'param2' to the list of parameters to be set if
   * 'param1' is being set by the user.
   *
   * 向量中的所有参数都将原子地设置。
   * All parameters in the vector will be set atomically.
   *
   * 请注意，仅在使用 `set_parameter`、 `set_parameters`、 `set_parameters_atomically`
   *设置参数时调用回调， 或者通过参数服务外部设置。 Note that the callback is only called while
   *setting parameters with `set_parameter`, `set_parameters`, `set_parameters_atomically`, or
   *externally with a parameters service.
   *
   * 使用 `declare_parameter` 或 `declare_parameters` 声明参数时不会调用回调。
   * The callback is not called when parameters are declared with `declare_parameter`
   * or `declare_parameters`.
   *
   * 使用 `undeclare_parameter` 取消声明参数时不会调用回调。
   * The callback is not called when parameters are undeclared with `undeclare_parameter`.
   *
   * 回调返回空的修改后参数列表将导致 “set_parameter*” 返回不成功的结果。
   * An empty modified parameter list from the callback will result in "set_parameter*"
   * returning an unsuccessful result.
   *
   * 可以使用 `remove_pre_set_parameters_callback` 注销回调。
   * The `remove_pre_set_parameters_callback` can be used to deregister the callback.
   *
   * \param callback 要注册的回调。
   * \param callback The callback to register.
   * \returns 一个共享指针。只要智能指针活动，回调就有效。
   * \returns A shared pointer. The callback is valid as long as the smart pointer is alive.
   * \throws std::bad_alloc 如果 PreSetParametersCallbackHandle 的分配失败。
   * \throws std::bad_alloc if the allocation of the PreSetParametersCallbackHandle fails.
   */
  RCLCPP_PUBLIC
  RCUTILS_WARN_UNUSED
  PreSetParametersCallbackHandle::SharedPtr add_pre_set_parameters_callback(
      PreSetParametersCallbackType callback);

  /// 添加一个回调函数，用于在参数设置之前进行验证。
  /// Add a callback to validate parameters before they are set.
  /**
   * 回调签名设计用于处理上述任何 `set_parameter*` 或 `declare_parameter*` 方法，
   * 因此它接受一个待设置参数的 vector 的 const 引用，并返回一个
   * rcl_interfaces::msg::SetParametersResult 实例，
   * 以指示是否应该设置参数，如果不设置，则说明原因。
   * The callback signature is designed to allow handling of any of the above
   * `set_parameter*` or `declare_parameter*` methods, and so it takes a const
   * reference to a vector of parameters to be set, and returns an instance of
   * rcl_interfaces::msg::SetParametersResult to indicate whether or not the
   * parameter should be set or not, and if not why.
   *
   * 用户应保留返回的共享指针的副本，因为只要智能指针还活着，回调就是有效的。
   * Users should retain a copy of the returned shared pointer, as the callback
   * is valid only as long as the smart pointer is alive.
   *
   * 示例回调：
   * For an example callback:
   *
   * ```cpp
   * rcl_interfaces::msg::SetParametersResult
   * my_callback(const std::vector<rclcpp::Parameter> & parameters)
   * {
   *   rcl_interfaces::msg::SetParametersResult result;
   *   result.successful = true;
   *   for (const auto & parameter : parameters) {
   *     if (!some_condition) {
   *       result.successful = false;
   *       result.reason = "the reason it could not be allowed";
   *     }
   *   }
   *   return result;
   * }
   * ```
   *
   * 可以看到，SetParametersResult 是一个表示成功的布尔标志，
   * 以及一个可选的原因，该原因可在失败时用于错误报告。
   * You can see that the SetParametersResult is a boolean flag for success
   * and an optional reason that can be used in error reporting when it fails.
   *
   * 这允许节点开发者控制哪些参数可以更改。
   * This allows the node developer to control which parameters may be changed.
   *
   * 拒绝对 "未知" 参数进行更改被认为是一种不好的做法，因为这会阻止
   * 节点的其他部分（可能了解这些参数）处理它们。
   * It is considered bad practice to reject changes for "unknown" parameters as this prevents
   * other parts of the node (that may be aware of these parameters) from handling them.
   *
   * 请注意，当调用 declare_parameter() 及其变体时，将调用回调，
   * 因此您不能假设在此回调之前已经设置了参数，因此在检查新值与现有值时，
   * 您必须考虑到参数尚未设置的情况。
   * Note that the callback is called when declare_parameter() and its variants
   * are called, and so you cannot assume the parameter has been set before
   * this callback, so when checking a new value against the existing one, you
   * must account for the case where the parameter is not yet set.
   *
   * 使用 `undeclare_parameter` 取消声明参数时，不会调用回调。
   * The callback is not called when parameters are undeclared with `undeclare_parameter`.
   *
   * 在调用回调之前，将强制执行一些约束条件，如 read_only。
   * Some constraints like read_only are enforced before the callback is called.
   *
   * 回调可以自省已经设置的其他参数（通过调用任何 {get,list,describe}_parameter() 方法），
   * 但不能修改其他参数（通过调用任何 {set,declare}_parameter() 方法）
   * 或修改已注册的回调本身（通过调用 add_on_set_parameters_callback() 方法）。
   * 如果回调尝试执行后者中的任何操作，
   * 将抛出 rclcpp::exceptions::ParameterModifiedInCallbackException 异常。
   * The callback may introspect other already set parameters (by calling any
   * of the {get,list,describe}_parameter() methods), but may *not* modify
   * other parameters (by calling any of the {set,declare}_parameter() methods)
   * or modify the registered callback itself (by calling the
   * add_on_set_parameters_callback() method).  If a callback tries to do any
   * of the latter things,
   * rclcpp::exceptions::ParameterModifiedInCallbackException will be thrown.
   *
   * 只要返回的智能指针有效，回调函数就必须保持有效。
   * 返回的智能指针可以升级为共享版本。
   * The callback functions must remain valid as long as the
   * returned smart pointer is valid.
   * The returned smart pointer can be promoted to a shared version.
   *
   * 重置或让智能指针超出范围将取消注册回调。
   * 也可以使用 `remove_on_set_parameters_callback`。
   * Resetting or letting the smart pointer go out of scope unregisters the callback.
   * `remove_on_set_parameters_callback` can also be used.
   *
   * 当参数被设置时，将调用已注册的回调。
   * 当回调返回不成功的结果时，剩余的回调将不会被调用。
   * 回调的顺序与注册顺序相反。
   * The registered callbacks are called when a parameter is set.
   * When a callback returns a not successful result, the remaining callbacks aren't called.
   * The order of the callback is the reverse from the registration order.
   *
   * \param callback 要注册的回调。
   * \param callback The callback to register.
   * \returns 一个共享指针。只要智能指针还活着，回调就是有效的。
   * \returns A shared pointer. The callback is valid as long as the smart pointer is alive.
   * \throws std::bad_alloc 如果 OnSetParametersCallbackHandle 的分配失败。
   * \throws std::bad_alloc if the allocation of the OnSetParametersCallbackHandle fails.
   */
  RCLCPP_PUBLIC
  RCUTILS_WARN_UNUSED
  OnSetParametersCallbackHandle::SharedPtr add_on_set_parameters_callback(
      OnSetParametersCallbackType callback);

  /// 添加一个在参数设置成功后触发的回调。
  /// Add a callback that gets triggered after parameters are set successfully.
  /**
   * 当任何 `set_parameter*` 或 `declare_parameter*` 方法成功时，回调被调用。
   * The callback is called when any of the `set_parameter*` or `declare_parameter*` methods are
   * successful.
   *
   * 用户应保留返回的共享指针的副本，因为只要智能指针存活，回调就有效。
   * Users should retain a copy of the returned shared pointer, as the callback is valid only as
   * long as the smart pointer is alive.
   *
   * 回调接受一个已成功设置的参数的const向量引用。
   * The callback takes a reference to a const vector of parameters that have been set successfully.
   *
   * post回调可以作为根据参数更改产生副作用的地方。
   * The post callback can be valuable as a place to cause side-effects based on parameter changes.
   * 例如，在参数成功更改后更新内部跟踪的类属性。
   * For instance updating internally tracked class attributes once parameters have been changed
   * successfully.
   *
   * 示例回调：
   * For an example callback:
   *
   * ```cpp
   * void
   * postSetParameterCallback(const std::vector<rclcpp::Parameter> & parameters)
   * {
   *  for(const auto & param:parameters) {
   *   // 在param1或param2成功更改后，可以更改内部类成员
   *   // the internal class member can be changed after successful change to param1 or param2
   *    if(param.get_name() == "param1") {
   *      internal_tracked_class_parameter_1_ = param.get_value<double>();
   *    }
   *    else if(param.get_name() == "param2") {
   *      internal_tracked_class_parameter_2_ = param.get_value<double>();
   *    }
   *  }
   * }
   * ```
   *
   * 上述回调接受一个已成功设置的参数列表的const引用，并根据此更新内部跟踪的类属性
   * `internal_tracked_class_parameter_1_` 和 `internal_tracked_class_parameter_2_`。
   * The above callback takes a const reference to list of parameters that have been set
   * successfully and as a result of this updates the internally tracked class attributes
   * `internal_tracked_class_parameter_1_` and `internal_tracked_class_parameter_2_` respectively.
   *
   * 此回调不应修改参数。
   * This callback should not modify parameters.
   *
   * 当使用 `declare_parameter` 或 `declare_parameters` 声明参数时，会调用回调。请参见上面的
   * `declare_parameter` 或 `declare_parameters`。 The callback is called when parameters are
   * declared with `declare_parameter` or `declare_parameters`. See `declare_parameter` or
   * `declare_parameters` above.
   *
   * 使用 `undeclare_parameter` 取消声明参数时，不会调用回调。
   * The callback is not called when parameters are undeclared with `undeclare_parameter`.
   *
   * 如果您想根据对其他参数的更改来更改参数，请使用 `add_pre_set_parameters_callback`。
   * If you want to make changes to parameters based on changes to another, use
   * `add_pre_set_parameters_callback`.
   *
   * 可以使用 `remove_post_set_parameters_callback` 注销回调。
   * The `remove_post_set_parameters_callback` can be used to deregister the callback.
   *
   * \param callback 要注册的回调。
   * \param callback The callback to register.
   * \returns 一个共享指针。只要智能指针存活，回调就有效。
   * \returns A shared pointer. The callback is valid as long as the smart pointer is alive.
   * \throws std::bad_alloc 如果 OnSetParametersCallbackHandle 的分配失败。
   * \throws std::bad_alloc if the allocation of the OnSetParametersCallbackHandle fails.
   */
  RCLCPP_PUBLIC
  RCUTILS_WARN_UNUSED
  PostSetParametersCallbackHandle::SharedPtr add_post_set_parameters_callback(
      PostSetParametersCallbackType callback);

  /// 删除使用 `add_pre_set_parameters_callback` 注册的回调。
  /// Remove a callback registered with `add_pre_set_parameters_callback`.
  /**
   * 删除由 `add_pre_set_parameters_callback` 返回的处理程序。
   * Delete a handler returned by `add_pre_set_parameters_callback`.
   *
   * \param handler 要删除的回调处理程序。
   * \param handler The callback handler to remove.
   * \throws std::runtime_error 如果处理程序不是通过 `add_pre_set_parameters_callback` 创建的，
   *   或者在之前已经被删除。
   * \throws std::runtime_error if the handler was not created with
   * `add_pre_set_parameters_callback`, or if it has been removed before.
   */
  RCLCPP_PUBLIC
  void remove_pre_set_parameters_callback(const PreSetParametersCallbackHandle* const handler);

  /// 删除使用 `add_on_set_parameters_callback` 注册的回调。
  /// Remove a callback registered with `add_on_set_parameters_callback`.
  /**
   * 删除由 `add_on_set_parameters_callback` 返回的处理程序。
   * Delete a handler returned by `add_on_set_parameters_callback`.
   *
   * 例如：
   * e.g.:
   *
   *    `remove_on_set_parameters_callback(scoped_callback.get())`
   *
   * 或者，可以重置智能指针：
   * As an alternative, the smart pointer can be reset:
   *
   *    `scoped_callback.reset()`
   *
   * 假设 `scoped_callback` 是唯一的所有者。
   * Supposing that `scoped_callback` was the only owner.
   *
   * 使用相同的处理程序多次调用 `remove_on_set_parameters_callback`，
   * 或在共享指针被重置后调用它是错误的。
   * Calling `remove_on_set_parameters_callback` more than once with the same handler,
   * or calling it after the shared pointer has been reset is an error.
   * 在调用 `remove_on_set_parameters_callback` 之后重置或让智能指针超出范围不是问题。
   * Resetting or letting the smart pointer go out of scope after calling
   * `remove_on_set_parameters_callback` is not a problem.
   *
   * \param handler 要删除的回调处理程序。
   * \param handler The callback handler to remove.
   * \throws std::runtime_error 如果处理程序不是通过 `add_on_set_parameters_callback` 创建的，
   *   或者在之前已经被删除。
   * \throws std::runtime_error if the handler was not created with
   * `add_on_set_parameters_callback`, or if it has been removed before.
   */
  RCLCPP_PUBLIC
  void remove_on_set_parameters_callback(const OnSetParametersCallbackHandle* const handler);

  /// 删除通过 `add_post_set_parameters_callback` 注册的回调。
  /// Remove a callback registered with `add_post_set_parameters_callback`.
  /**
   * 删除由 `add_post_set_parameters_callback` 返回的处理程序。
   * Delete a handler returned by `add_post_set_parameters_callback`.
   *
   * \param handler 要删除的回调处理程序。 The callback handler to remove.
   * \throws std::runtime_error 如果处理程序不是通过 `add_post_set_parameters_callback` 创建的，
   *   或者之前已经被删除了。 if the handler was not created with
   * `add_post_set_parameters_callback`, or if it has been removed before.
   */
  RCLCPP_PUBLIC
  void remove_post_set_parameters_callback(const PostSetParametersCallbackHandle* const handler);

  /// 获取所有可用节点的完全限定名称。
  /// Get the fully-qualified names of all available nodes.
  /**
   * 完全限定名称包括节点的本地命名空间和名称。
   * The fully-qualified name includes the local namespace and name of the node.
   * \return 节点的完全限定名称的向量。 A vector of fully-qualified names of nodes.
   */
  RCLCPP_PUBLIC
  std::vector<std::string> get_node_names() const;

  /// 返回现有主题名称到主题类型列表的映射。
  /// Return a map of existing topic names to list of topic types.
  /**
   * \return 现有主题名称到主题类型列表的映射。 a map of existing topic names to list of topic
   * types. \throws std::runtime_error 任何 rcl_error 可以抛出的异常。 anything that rcl_error can
   * throw
   */
  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>> get_topic_names_and_types() const;

  /// 返回现有服务名称到服务类型列表的映射。
  /// Return a map of existing service names to list of service types.
  /**
   * \return 现有服务名称到服务类型列表的映射。 a map of existing service names to list of service
   * types. \throws std::runtime_error 任何 rcl_error 可以抛出的异常。 anything that rcl_error can
   * throw
   */
  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>> get_service_names_and_types() const;

  /// 返回特定节点的现有服务名称到服务类型列表的映射。
  /// Return a map of existing service names to list of service types for a specific node.
  /**
   * 此函数仅考虑服务 - 不包括客户端。
   * This function only considers services - not clients.
   * 返回的名称是实际使用的名称，不会应用重映射规则。
   * The returned names are the actual names used and do not have remap rules applied.
   *
   * \param[in] node_name 节点的名称。 name of the node.
   * \param[in] namespace_ 节点的命名空间。 namespace of the node.
   * \return 现有服务名称到服务类型列表的映射。 a map of existing service names to list of service
   * types. \throws std::runtime_error 任何 rcl_error 可以抛出的异常。 anything that rcl_error can
   * throw.
   */
  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>> get_service_names_and_types_by_node(
      const std::string& node_name, const std::string& namespace_) const;

  /// 返回为给定主题创建的发布者数量。
  /// Return the number of publishers created for a given topic.
  /**
   * \param[in] topic_name 实际使用的主题名称；它不会自动重映射。the actual topic name used; it will
   * not be automatically remapped. \return 为给定主题创建的发布者数量。 number of publishers that
   * have been created for the given topic. \throws std::runtime_error 如果无法计算发布者数量。 if
   * publishers could not be counted
   */
  RCLCPP_PUBLIC
  size_t count_publishers(const std::string& topic_name) const;

  /// 返回为给定主题创建的订阅者数量。
  /// Return the number of subscribers created for a given topic.
  /**
   * \param[in] topic_name 实际使用的主题名称；它不会自动重映射。the actual topic name used; it will
   * not be automatically remapped. \return 为给定主题创建的订阅者数量。 number of subscribers that
   * have been created for the given topic. \throws std::runtime_error 如果无法计算订阅者数量。 if
   * subscribers could not be counted
   */
  RCLCPP_PUBLIC
  size_t count_subscribers(const std::string& topic_name) const;

  /// 返回给定主题上发布者的主题端点信息。
  /// Return the topic endpoint information about publishers on a given topic.
  /**
   * 返回的参数是一个主题端点信息列表，其中每个项目都将包含
   * 节点名称、节点命名空间、主题类型、端点类型、主题端点的 GID 和其 QoS 配置文件。
   * The returned parameter is a list of topic endpoint information, where each item will contain
   * the node name, node namespace, topic type, endpoint type, topic endpoint's GID, and its QoS
   * profile.
   *
   * 当 `no_mangle` 参数为 `true` 时，提供的 `topic_name` 应该是中间件的有效主题
   * 名称（在将 ROS 与本地中间件（例如 DDS）应用程序结合使用时有用）。
   * When the `no_mangle` parameter is `true`, the provided `topic_name` should be a valid topic
   * name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
   * 当 `no_mangle` 参数为 `false` 时，提供的 `topic_name` 应遵循
   * ROS 主题命名约定。
   * When the `no_mangle` parameter is `false`, the provided `topic_name` should follow
   * ROS topic name conventions.
   *
   * `topic_name` 可能是相对的、私有的或完全限定的主题名称。
   * A relative or private topic will be expanded using this node's namespace and name.
   * 查询的 `topic_name` 不会被重新映射。
   * The queried `topic_name` is not remapped.
   *
   * \param[in] topic_name 实际使用的主题名称；它不会自动重新映射。
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   * \param[in] no_mangle 如果为 `true`，`topic_name` 需要是有效的中间件主题名称，
   *   否则应该是有效的 ROS 主题名称。默认为 `false`。
   * \param[in] no_mangle if `true`, `topic_name` needs to be a valid middleware topic name,
   *   otherwise it should be a valid ROS topic name. Defaults to `false`.
   * \return 一个表示此主题上所有发布者的 TopicEndpointInfo 列表。
   * \return a list of TopicEndpointInfo representing all the publishers on this topic.
   * \throws InvalidTopicNameError 如果给定的 topic_name 无效。
   * \throws InvalidTopicNameError if the given topic_name is invalid.
   * \throws std::runtime_error 如果发生内部错误。
   * \throws std::runtime_error if internal error happens.
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::TopicEndpointInfo> get_publishers_info_by_topic(
      const std::string& topic_name, bool no_mangle = false) const;

  /// 返回给定主题上订阅者的主题端点信息。
  /// Return the topic endpoint information about subscriptions on a given topic.
  /**
   * 返回的参数是一个主题端点信息列表，其中每个项目都将包含
   * 节点名称、节点命名空间、主题类型、端点类型、主题端点的 GID 和其 QoS 配置文件。
   * The returned parameter is a list of topic endpoint information, where each item will contain
   * the node name, node namespace, topic type, endpoint type, topic endpoint's GID, and its QoS
   * profile.
   *
   * 当 `no_mangle` 参数为 `true` 时，提供的 `topic_name` 应该是中间件的有效主题
   * 名称（在将 ROS 与本地中间件（例如 DDS）应用程序结合使用时有用）。
   * When the `no_mangle` parameter is `true`, the provided `topic_name` should be a valid topic
   * name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
   * 当 `no_mangle` 参数为 `false` 时，提供的 `topic_name` 应遵循
   * ROS 主题命名约定。
   * When the `no_mangle` parameter is `false`, the provided `topic_name` should follow
   * ROS topic name conventions.
   *
   * `topic_name` 可能是相对的、私有的或完全限定的主题名称。
   * A relative or private topic will be expanded using this node's namespace and name.
   * 查询的 `topic_name` 不会被重新映射。
   * The queried `topic_name` is not remapped.
   *
   * \param[in] topic_name 实际使用的主题名称；它不会自动重新映射。
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   * \param[in] no_mangle 如果为 `true`，`topic_name` 需要是有效的中间件主题名称，
   *   否则应该是有效的 ROS 主题名称。默认为 `false`。
   * \param[in] no_mangle if `true`, `topic_name` needs to be a valid middleware topic name,
   *   otherwise it should be a valid ROS topic name. Defaults to `false`.
   * \return 一个表示此主题上所有订阅者的 TopicEndpointInfo 列表。
   * \return a list of TopicEndpointInfo representing all the subscriptions on this topic.
   * \throws InvalidTopicNameError 如果给定的 topic_name 无效。
   * \throws InvalidTopicNameError if the given topic_name is invalid.
   * \throws std::runtime_error 如果发生内部错误。
   * \throws std::runtime_error if internal error happens.
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::TopicEndpointInfo> get_subscriptions_info_by_topic(
      const std::string& topic_name, bool no_mangle = false) const;

  /// 返回一个图形事件，每当发生图形更改时都会设置该事件。
  /// Return a graph event, which will be set anytime a graph change occurs.
  /* 图形事件对象是一个贷款，必须归还。
   * The graph Event object is a loan which must be returned.
   * 事件对象是范围限定的，因此要返回贷款只需让其超出范围即可。
   * The Event object is scoped and therefore to return the loan just let it go
   * out of scope.
   */
  RCLCPP_PUBLIC
  rclcpp::Event::SharedPtr get_graph_event();

  /// 通过等待事件变为已设置来等待图形事件发生。
  /// Wait for a graph event to occur by waiting on an Event to become set.
  /**
   * 必须通过 get_graph_event() 方法获取给定的事件。
   * The given Event must be acquire through the get_graph_event() method.
   *
   * \param[in] event 要等待的事件指针
   * \param[in] event pointer to an Event to wait for
   * \param[in] timeout 等待事件更改状态的纳秒数
   * \param[in] timeout nanoseconds to wait for the Event to change the state
   *
   * \throws InvalidEventError 如果给定的事件为 nullptr
   * \throws InvalidEventError if the given event is nullptr
   * \throws EventNotRegisteredError 如果未使用
   *   get_graph_event() 获取给定的事件。
   * \throws EventNotRegisteredError if the given event was not acquired with
   *   get_graph_event().
   */
  RCLCPP_PUBLIC
  void wait_for_graph_change(rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout);

  /// 获取由节点管理的非常量共享指针时钟。
  /// Get a clock as a non-const shared pointer which is managed by the node.
  /**
   * \sa rclcpp::node_interfaces::NodeClock::get_clock
   */
  RCLCPP_PUBLIC
  rclcpp::Clock::SharedPtr get_clock();

  /// 获取由节点管理的常量共享指针时钟。
  /// Get a clock as a const shared pointer which is managed by the node.
  /**
   * \sa rclcpp::node_interfaces::NodeClock::get_clock
   */
  RCLCPP_PUBLIC
  rclcpp::Clock::ConstSharedPtr get_clock() const;

  /// 从 clock_type 指定的时间源返回当前时间。
  /// Returns current time from the time source specified by clock_type.
  /**
   * \sa rclcpp::Clock::now
   */
  RCLCPP_PUBLIC
  Time now() const;

  /// 返回节点的内部 NodeBaseInterface 实现。(Return the Node's internal NodeBaseInterface
  /// implementation.)
  /**
   * @brief 获取节点的内部 NodeBaseInterface 实现。(Get the internal NodeBaseInterface
   * implementation of the node.)
   * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr 节点的内部 NodeBaseInterface
   * 实现。(The internal NodeBaseInterface implementation of the node.)
   */
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

  /// 返回节点的内部 NodeClockInterface 实现。(Return the Node's internal NodeClockInterface
  /// implementation.)
  /**
   * @brief 获取节点的内部 NodeClockInterface 实现。(Get the internal NodeClockInterface
   * implementation of the node.)
   * @return rclcpp::node_interfaces::NodeClockInterface::SharedPtr 节点的内部 NodeClockInterface
   * 实现。(The internal NodeClockInterface implementation of the node.)
   */
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr get_node_clock_interface();

  /// 返回节点的内部 NodeGraphInterface 实现。(Return the Node's internal NodeGraphInterface
  /// implementation.)
  /**
   * @brief 获取节点的内部 NodeGraphInterface 实现。(Get the internal NodeGraphInterface
   * implementation of the node.)
   * @return rclcpp::node_interfaces::NodeGraphInterface::SharedPtr 节点的内部 NodeGraphInterface
   * 实现。(The internal NodeGraphInterface implementation of the node.)
   */
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr get_node_graph_interface();

  /// 返回节点的内部 NodeLoggingInterface 实现。(Return the Node's internal NodeLoggingInterface
  /// implementation.)
  /**
   * @brief 获取节点的内部 NodeLoggingInterface 实现。(Get the internal NodeLoggingInterface
   * implementation of the node.)
   * @return rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr 节点的内部
   * NodeLoggingInterface 实现。(The internal NodeLoggingInterface implementation of the node.)
   */
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr get_node_logging_interface();

  /// 返回节点的内部 NodeTimersInterface 实现。(Return the Node's internal NodeTimersInterface
  /// implementation.)
  /**
   * @brief 获取节点的内部 NodeTimersInterface 实现。(Get the internal NodeTimersInterface
   * implementation of the node.)
   * @return rclcpp::node_interfaces::NodeTimersInterface::SharedPtr 节点的内部 NodeTimersInterface
   * 实现。(The internal NodeTimersInterface implementation of the node.)
   */
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr get_node_timers_interface();

  /// 返回节点的内部 NodeTopicsInterface 实现。(Return the Node's internal NodeTopicsInterface
  /// implementation.)
  /**
   * @brief 获取节点的内部 NodeTopicsInterface 实现。(Get the internal NodeTopicsInterface
   * implementation of the node.)
   * @return rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr 节点的内部 NodeTopicsInterface
   * 实现。(The internal NodeTopicsInterface implementation of the node.)
   */
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface();

  /// 返回节点的内部 NodeServicesInterface 实现。(Return the Node's internal NodeServicesInterface
  /// implementation.)
  /**
   * @brief 获取节点的内部 NodeServicesInterface 实现。(Get the internal NodeServicesInterface
   * implementation of the node.)
   * @return rclcpp::node_interfaces::NodeServicesInterface::SharedPtr 节点的内部
   * NodeServicesInterface 实现。(The internal NodeServicesInterface implementation of the node.)
   */
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr get_node_services_interface();

  /// 返回节点的内部 NodeWaitablesInterface 实现。(Return the Node's internal NodeWaitablesInterface
  /// implementation.)
  /**
   * @brief 获取节点的内部 NodeWaitablesInterface 实现。(Get the internal NodeWaitablesInterface
   * implementation of the node.)
   * @return rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr 节点的内部
   * NodeWaitablesInterface 实现。(The internal NodeWaitablesInterface implementation of the node.)
   */
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr get_node_waitables_interface();

  /// 返回节点的内部 NodeParametersInterface 实现。(Return the Node's internal
  /// NodeParametersInterface implementation.)
  /**
   * @brief 获取节点的内部 NodeParametersInterface 实现。(Get the internal NodeParametersInterface
   * implementation of the node.)
   * @return rclcpp::node_interfaces::NodeParametersInterface::SharedPtr 节点的内部
   * NodeParametersInterface 实现。(The internal NodeParametersInterface implementation of the
   * node.)
   */
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface();

  /// 返回节点的内部 NodeTimeSourceInterface 实现。(Return the Node's internal
  /// NodeTimeSourceInterface implementation.)
  /**
   * @brief 获取节点的内部 NodeTimeSourceInterface 实现。(Get the internal NodeTimeSourceInterface
   * implementation of the node.)
   * @return rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr 节点的内部
   * NodeTimeSourceInterface 实现。(The internal NodeTimeSourceInterface implementation of the
   * node.)
   */
  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr get_node_time_source_interface();

  /// 返回子命名空间，如果这是一个子节点，否则返回空字符串。
  /// Return the sub-namespace, if this is a sub-node, otherwise an empty string.
  /**
   * 返回的子命名空间是累积的子命名空间，这些子命名空间是通过一对多的 create_sub_node() 调用给出的，
   * 或者如果这是一个原始节点实例（即不是子节点），则返回空字符串。
   * The returned sub-namespace is either the accumulated sub-namespaces which
   * were given to one-to-many create_sub_node() calls, or an empty string if
   * this is an original node instance, i.e. not a sub-node.
   *
   * 例如，考虑：
   * For example, consider:
   *
   * ```cpp
   * auto node = std::make_shared<rclcpp::Node>("my_node", "my_ns");
   * node->get_sub_namespace();  // -> ""
   * auto sub_node1 = node->create_sub_node("a");
   * sub_node1->get_sub_namespace();  // -> "a"
   * auto sub_node2 = sub_node1->create_sub_node("b");
   * sub_node2->get_sub_namespace();  // -> "a/b"
   * auto sub_node3 = node->create_sub_node("foo");
   * sub_node3->get_sub_namespace();  // -> "foo"
   * node->get_sub_namespace();  // -> ""
   * ```
   *
   * get_namespace() 将返回原始节点命名空间，如果存在子命名空间，将不包括子命名空间。
   * get_namespace() will return the original node namespace, and will not
   * include the sub-namespace if one exists.
   * 若要获取该子命名空间，需要调用 get_effective_namespace() 方法。
   * To get that you need to call the get_effective_namespace() method.
   *
   * \sa get_namespace()
   * \sa get_effective_namespace()
   * \return 子命名空间字符串，不包括节点的原始命名空间
   * \return the sub-namespace string, not including the node's original namespace
   */
  RCLCPP_PUBLIC
  const std::string& get_sub_namespace() const;

  /// 返回创建实体时使用的有效命名空间。
  /// Return the effective namespace that is used when creating entities.
  /**
   * 返回的命名空间是节点命名空间和累积子命名空间的连接，当创建具有相对名称的实体时将作为命名空间使用。
   * The returned namespace is a concatenation of the node namespace and the
   * accumulated sub-namespaces, which is used as the namespace when creating
   * entities which have relative names.
   *
   * 例如，考虑：
   * For example, consider:
   *
   * ```cpp
   * auto node = std::make_shared<rclcpp::Node>("my_node", "my_ns");
   * node->get_effective_namespace();  // -> "/my_ns"
   * auto sub_node1 = node->create_sub_node("a");
   * sub_node1->get_effective_namespace();  // -> "/my_ns/a"
   * auto sub_node2 = sub_node1->create_sub_node("b");
   * sub_node2->get_effective_namespace();  // -> "/my_ns/a/b"
   * auto sub_node3 = node->create_sub_node("foo");
   * sub_node3->get_effective_namespace();  // -> "/my_ns/foo"
   * node->get_effective_namespace();  // -> "/my_ns"
   * ```
   *
   * \sa get_namespace()
   * \sa get_sub_namespace()
   * \return 子命名空间字符串，不包括节点的原始命名空间
   * \return the sub-namespace string, not including the node's original namespace
   */
  RCLCPP_PUBLIC
  const std::string& get_effective_namespace() const;

  /// 创建一个子节点，它将扩展所有与其创建的实体的命名空间。
  /// Create a sub-node, which will extend the namespace of all entities created with it.
  /**
   * 子节点（简称下级节点）是使用此类的现有实例创建的此类的实例，
   * 但是它还具有与之关联的附加子命名空间（简称下级命名空间）。
   * A sub-node (short for subordinate node) is an instance of this class
   * which has been created using an existing instance of this class, but which
   * has an additional sub-namespace (short for subordinate namespace)
   * associated with it.
   *
   * 子命名空间将扩展节点的命名空间，以便创建其他实体，例如发布者、订阅者、服务客户端和服务器等。
   * The sub-namespace will extend the node's namespace for the purpose of
   * creating additional entities, such as Publishers, Subscriptions, Service
   * Clients and Servers, and so on.
   *
   * 默认情况下，当使用公共构造函数之一创建此类的实例时，它没有与之关联的子命名空间，因此不是子节点。
   * By default, when an instance of this class is created using one of the
   * public constructors, it has no sub-namespace associated with it, and
   * therefore is not a sub-node.
   *
   * 然而，可以使用 "普通"
   * 节点实例来创建基于原始实例的此类的进一步实例，这些实例具有与之关联的附加子命名空间。 That
   * "normal" node instance may, however, be used to create further instances of this class, based
   * on the original instance, which have an additional sub-namespace associated with them.
   *
   * 这可以通过使用此方法 `create_sub_node()` 来完成。
   * This may be done by using this method, create_sub_node().
   *
   * 此外，子节点可用于创建其他子节点，在这种情况下，传递给此函数的子命名空间将进一步扩展现有子节点的子命名空间。
   * Furthermore, a sub-node may be used to create additional sub-node's, in
   * which case the sub-namespace passed to this function will further
   * extend the sub-namespace of the existing sub-node.
   *
   * 请参考 get_sub_namespace() 和 get_effective_namespace() 示例。
   * See get_sub_namespace() and get_effective_namespace() for examples.
   *
   * 请注意，使用绝对名称的实体不受任何命名空间影响，无论是普通节点命名空间还是子命名空间。
   * Note that entities which use absolute names are not affected by any
   * namespaces, neither the normal node namespace nor any sub-namespace.
   *
   * 还要注意，完全限定的节点名称不受子命名空间的影响。
   * Note also that the fully qualified node name is unaffected by a
   * sub-namespace.
   *
   * 子命名空间应该是相对的，如果子命名空间是绝对的，即以前导 '/' 开头，则会抛出异常。
   * The sub-namespace should be relative, and an exception will be thrown if
   * the sub-namespace is absolute, i.e. if it starts with a leading '/'.
   *
   * \sa get_sub_namespace()
   * \sa get_effective_namespace()
   * \param[in] sub_namespace 子节点的子命名空间。
   * \param[in] sub_namespace sub-namespace of the sub-node.
   * \return 新创建的子节点
   * \return newly created sub-node
   * \throws NameValidationError 如果子命名空间是绝对的，即以前导 '/' 开头。
   * \throws NameValidationError if the sub-namespace is absolute, i.e. starts
   *   with a leading '/'.
   */
  RCLCPP_PUBLIC
  rclcpp::Node::SharedPtr create_sub_node(const std::string& sub_namespace);

  /// 返回创建此节点时使用的 NodeOptions。
  /// Return the NodeOptions used when creating this node.
  RCLCPP_PUBLIC
  const rclcpp::NodeOptions& get_node_options() const;

protected:
  /// 构造一个子节点，它将扩展用其创建的所有实体的命名空间。
  /// Construct a sub-node, which will extend the namespace of all entities created with it.
  /**
   * \sa create_sub_node()
   *
   * \param[in] other 从这个节点创建一个新的子节点。
   * \param[in] other The node from which a new sub-node is created.
   * \param[in] sub_namespace 子节点的子命名空间。
   * \param[in] sub_namespace The sub-namespace of the sub-node.
   */
  RCLCPP_PUBLIC
  Node(const Node& other, const std::string& sub_namespace);

private:
  /**
   * @brief 禁用 Node 类的拷贝构造函数和赋值运算符 (Disable copy constructor and assignment operator
   * for Node class)
   */
  RCLCPP_DISABLE_COPY(Node)

  /// @brief node_base_ 是一个 NodeBaseInterface
  /// 的智能指针，用于管理节点基本功能，例如节点生命周期、名称和命名空间等（A shared pointer of
  /// NodeBaseInterface to manage the basic functionalities of a node, such as node lifecycle, name,
  /// and namespace）
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  /// @brief node_graph_ 是一个 NodeGraphInterface
  /// 的智能指针，用于管理节点图相关功能，例如查询发布者、订阅者和服务等的拓扑关系（A shared pointer
  /// of NodeGraphInterface to manage the graph-related functionalities of a node, such as querying
  /// topology relationships of publishers, subscribers, and services）
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  /// @brief node_logging_ 是一个 NodeLoggingInterface
  /// 的智能指针，用于管理节点日志功能，例如记录不同级别的日志信息（A shared pointer of
  /// NodeLoggingInterface to manage the logging functionalities of a node, such as logging messages
  /// at different severity levels）
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  /// @brief node_timers_ 是一个 NodeTimersInterface
  /// 的智能指针，用于管理节点定时器功能，例如创建、启动和停止定时器（A shared pointer of
  /// NodeTimersInterface to manage the timer functionalities of a node, such as creating, starting,
  /// and stopping timers）
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  /// @brief node_topics_ 是一个 NodeTopicsInterface
  /// 的智能指针，用于管理节点话题功能，例如创建发布者和订阅者以及处理消息通信（A shared pointer of
  /// NodeTopicsInterface to manage the topic functionalities of a node, such as creating publishers
  /// and subscribers and handling message communication）
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  /// @brief node_services_ 是一个 NodeServicesInterface
  /// 的智能指针，用于管理节点服务功能，例如创建服务服务器和客户端以及处理服务请求和响应（A shared
  /// pointer of NodeServicesInterface to manage the service functionalities of a node, such as
  /// creating service servers and clients and handling service requests and responses）
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
  /// @brief node_clock_ 是一个 NodeClockInterface 的智能指针，用于管理节点时钟功能，例如提供 ROS
  /// 时间、系统时间和仿真时间等（A shared pointer of NodeClockInterface to manage the clock
  /// functionalities of a node, such as providing ROS time, system time, and simulated time）
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  /// @brief node_parameters_ 是一个 NodeParametersInterface
  /// 的智能指针，用于管理节点参数功能，例如设置、获取和更新节点参数（A shared pointer of
  /// NodeParametersInterface to manage the parameter functionalities of a node, such as setting,
  /// getting, and updating node parameters）
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  /// @brief node_time_source_ 是一个 NodeTimeSourceInterface
  /// 的智能指针，用于管理节点时间源功能，例如为节点提供时间信息和时间同步（A shared pointer of
  /// NodeTimeSourceInterface to manage the time source functionalities of a node, such as providing
  /// time information and time synchronization for the node）
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr node_time_source_;
  /// @brief node_waitables_ 是一个 NodeWaitablesInterface
  /// 的智能指针，用于管理节点可等待对象功能，例如创建和销毁可等待对象以及在执行器中处理这些对象（A
  /// shared pointer of NodeWaitablesInterface to manage the waitable object functionalities of a
  /// node, such as creating and destroying waitable objects and handling them in executors）
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_;

  /// @brief node_options_ 是一个 NodeOptions 类型的常量，用于存储节点选项（A constant of type
  /// NodeOptions to store the node options）
  const rclcpp::NodeOptions node_options_;
  /// @brief sub_namespace_ 是一个字符串类型的常量，用于存储子命名空间（A constant of type string to
  /// store the sub-namespace）
  const std::string sub_namespace_;
  /// @brief effective_namespace_ 是一个字符串类型的常量，用于存储有效命名空间（A constant of type
  /// string to store the effective namespace）
  const std::string effective_namespace_;
};

}  // namespace rclcpp

#ifndef RCLCPP__NODE_IMPL_HPP_
// Template implementations
#include "node_impl.hpp"
#endif

#endif  // RCLCPP__NODE_HPP_

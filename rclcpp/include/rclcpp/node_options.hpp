// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_OPTIONS_HPP_
#define RCLCPP__NODE_OPTIONS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rcl/node_options.h"
#include "rcl/time.h"
#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/// Encapsulation of options for node initialization.
class NodeOptions {
public:
  /// 使用默认值创建 NodeOptions ，可以选择性地指定要使用的分配器。
  /// Create NodeOptions with default values, optionally specifying the allocator to use.
  /**
   * 节点选项的默认值：
   * Default values for the node options:
   *
   *   - context = rclcpp::contexts::get_global_default_context()
   *   - arguments = {}
   *   - parameter_overrides = {}
   *   - use_global_arguments = true
   *   - use_intra_process_comms = false
   *   - enable_topic_statistics = false
   *   - start_parameter_services = true
   *   - start_parameter_event_publisher = true
   *   - clock_type = RCL_ROS_TIME
   *   - clock_qos = rclcpp::ClockQoS()
   *   - use_clock_thread = true
   *   - rosout_qos = rclcpp::RosoutQoS()
   *   - parameter_event_qos = rclcpp::ParameterEventQoS
   *     - with history setting and depth from rmw_qos_profile_parameter_events
   *   - parameter_event_publisher_options = rclcpp::PublisherOptionsBase
   *   - allow_undeclared_parameters = false
   *   - automatically_declare_parameters_from_overrides = false
   *   - allocator = rcl_get_default_allocator()
   *
   * \param[in] allocator 构造 NodeOptions 时使用的分配器。
   * \param[in] allocator Allocator to use in construction of NodeOptions.
   */
  RCLCPP_PUBLIC
  explicit NodeOptions(rcl_allocator_t allocator = rcl_get_default_allocator());

  /// 析构函数。
  /// Destructor.
  RCLCPP_PUBLIC
  virtual ~NodeOptions() = default;

  /// 拷贝构造函数。
  /// Copy constructor.
  RCLCPP_PUBLIC
  NodeOptions(const NodeOptions &other);

  /// 赋值运算符。
  /// Assignment operator.
  RCLCPP_PUBLIC
  NodeOptions &operator=(const NodeOptions &other);

  /// 返回节点使用的 rcl_node_options。
  /// Return the rcl_node_options used by the node.
  /**
   * 此数据结构在第一次调用此函数时被懒惰地创建。
   * This data structure is created lazily, on the first call to this function.
   * 除非更改了其中一个输入设置，如 arguments、use_global_arguments 或 rcl
   * 分配器，否则重复调用不会重新生成它。 Repeated calls will not regenerate it unless one of the
   * input settings changed, like arguments, use_global_arguments, or the rcl allocator.
   *
   * \return 节点使用的 const rcl_node_options_t 结构
   * \return a const rcl_node_options_t structure used by the node
   * \throws exceptions::UnknownROSArgsError 如果有未知的 ROS 参数
   * \throws exceptions::UnknownROSArgsError if there are unknown ROS arguments
   */
  RCLCPP_PUBLIC
  const rcl_node_options_t *get_rcl_node_options() const;

  /// 返回节点要使用的上下文。
  /// Return the context to be used by the node.
  RCLCPP_PUBLIC
  rclcpp::Context::SharedPtr context() const;

  /// 设置上下文，返回此参数用于参数习语。
  /// Set the context, return this for parameter idiom.
  RCLCPP_PUBLIC
  NodeOptions &context(rclcpp::Context::SharedPtr context);

  /// 返回节点参数列表的引用。
  /// Return a reference to the list of arguments for the node.
  RCLCPP_PUBLIC
  const std::vector<std::string> &arguments() const;

  /// 设置参数，返回此参数用于参数习语。
  /// Set the arguments, return this for parameter idiom.
  /**
   * 这些参数用于提取节点使用的重映射和其他ROS特定设置，以及用户定义的非ROS参数。
   * These arguments are used to extract remappings used by the node and other ROS specific
   * settings, as well as user defined non-ROS arguments.
   *
   * 这将导致内部 rcl_node_options_t 结构无效。
   * This will cause the internal rcl_node_options_t struct to be invalidated.
   */
  RCLCPP_PUBLIC
  NodeOptions &arguments(const std::vector<std::string> &arguments);

  /// 返回参数覆盖列表的引用 (Return a reference to the list of parameter overrides)
  RCLCPP_PUBLIC
  std::vector<rclcpp::Parameter> &parameter_overrides();

  /// 返回参数覆盖列表的常量引用 (Return a const reference to the list of parameter overrides)
  RCLCPP_PUBLIC
  const std::vector<rclcpp::Parameter> &parameter_overrides() const;

  /// 设置参数覆盖，返回此参数习语 (Set the parameters overrides, return this for parameter idiom)
  /**
   * 这些参数覆盖用于更改节点内声明参数的初始值，
   * 如果需要，可以覆盖硬编码的默认值。
   * (These parameter overrides are used to change the initial value
   * of declared parameters within the node, overriding hard coded default
   * values if necessary.)
   */
  RCLCPP_PUBLIC
  NodeOptions &parameter_overrides(const std::vector<rclcpp::Parameter> &parameter_overrides);

  /// 添加单个参数覆盖，参数习语风格 (Append a single parameter override, parameter idiom style)
  template <typename ParameterT>
  NodeOptions &append_parameter_override(const std::string &name, const ParameterT &value) {
    // 在参数覆盖列表中添加新的参数 (Add new parameter to the parameter_overrides list)
    this->parameter_overrides().emplace_back(name, rclcpp::ParameterValue(value));
    return *this;
  }

  /// 返回 use_global_arguments 标志 (Return the use_global_arguments flag)
  RCLCPP_PUBLIC
  bool use_global_arguments() const;

  /// 设置 use_global_arguments 标志，返回此参数习语 (Set the use_global_arguments flag, return this
  /// for parameter idiom)
  /**
   * 如果为 true，则节点的行为将受到 "全局" 参数的影响，
   * 即非针对特定节点的参数以及针对当前节点的参数。
   * (If true this will cause the node's behavior to be influenced by "global"
   * arguments, i.e. arguments not targeted at specific nodes, as well as the
   * arguments targeted at the current node.)
   *
   * 这将使内部 rcl_node_options_t 结构失效 (This will cause the internal rcl_node_options_t struct
   * to be invalidated)
   */
  RCLCPP_PUBLIC
  NodeOptions &use_global_arguments(bool use_global_arguments);

  /// 返回 enable_rosout 标志 (Return the enable_rosout flag)
  RCLCPP_PUBLIC
  bool enable_rosout() const;

  /// 设置 enable_rosout 标志，返回此参数习语 (Set the enable_rosout flag, return this for parameter
  /// idiom)
  /**
   * 如果为 false，则节点将不使用 rosout 日志记录。
   * (If false this will cause the node not to use rosout logging.)
   *
   * 目前默认为 true，因为仍有一些情况下需要使用它。
   * (Defaults to true for now, as there are still some cases where it is
   * desirable.)
   */
  RCLCPP_PUBLIC
  NodeOptions &enable_rosout(bool enable_rosout);

  /// 返回 use_intra_process_comms 标志 (Return the use_intra_process_comms flag)
  RCLCPP_PUBLIC
  bool use_intra_process_comms() const;

  /// 设置 use_intra_process_comms 标志，返回此参数习语 (Set the use_intra_process_comms flag,
  /// return this for parameter idiom)
  /**
   * 如果为 true，此上下文中发布和订阅的主题消息将通过特殊的进程内通信代码
   * 路径，从而避免序列化和反序列化、不必要的拷贝，并在某些情况下实现较低的延迟。
   * (If true, messages on topics which are published and subscribed to within
   * this context will go through a special intra-process communication code
   * code path which can avoid serialization and deserialization, unnecessary
   * copies, and achieve lower latencies in some cases.)
   *
   * 目前默认为 false，因为仍有一些情况下可能不需要使用它。
   * (Defaults to false for now, as there are still some cases where it is not
   * desirable.)
   */
  RCLCPP_PUBLIC
  NodeOptions &use_intra_process_comms(bool use_intra_process_comms);

  /// 返回 enable_topic_statistics 标志。 (Return the enable_topic_statistics flag.)
  RCLCPP_PUBLIC
  bool enable_topic_statistics() const;

  /// 设置 enable_topic_statistics 标志，返回此参数用于参数习语。(Set the enable_topic_statistics
  /// flag, return this for parameter idiom.)
  /**
   * 如果为 true，则启用所有订阅的主题统计信息收集和发布。(If true, topic statistics collection and
   * publication will be enabled for all subscriptions.) 可以使用此选项覆盖全局主题统计设置。(This
   * can be used to override the global topic statistics setting.)
   *
   * 默认为 false。(Defaults to false.)
   */
  RCLCPP_PUBLIC
  NodeOptions &enable_topic_statistics(bool enable_topic_statistics);

  /// 返回 start_parameter_services 标志。(Return the start_parameter_services flag.)
  RCLCPP_PUBLIC
  bool start_parameter_services() const;

  /// 设置 start_parameter_services 标志，返回此参数用于参数习语。(Set the start_parameter_services
  /// flag, return this for parameter idiom.)
  /**
   * 如果为 true，则创建 ROS 服务以允许外部节点列出、获取和请求设置此节点的参数。(If true, ROS
   * services are created to allow external nodes to list, get, and request to set parameters of
   * this node.)
   *
   * 如果为 false，则参数仍将在本地工作，但无法远程访问。(If false, parameters will still work
   * locally, but will not be accessible remotely.)
   *
   * \sa start_parameter_event_publisher()
   */
  RCLCPP_PUBLIC
  NodeOptions &start_parameter_services(bool start_parameter_services);

  /// 返回 start_parameter_event_publisher 标志。(Return the start_parameter_event_publisher flag.)
  RCLCPP_PUBLIC
  bool start_parameter_event_publisher() const;

  /// 设置 start_parameter_event_publisher 标志，返回此参数用于参数习语。(Set the
  /// start_parameter_event_publisher flag, return this for parameter idiom.)
  /**
   * 如果为 true，则每次参数状态发生变化时，在其中创建一个发布者并发布事件消息。(If true, a
   * publisher is created on which an event message is published each time a parameter's state
   * changes.) 这用于记录和自省，但可与其他参数服务分开配置。(This is used for recording and
   * introspection, but is configurable separately from the other parameter services.)
   */
  RCLCPP_PUBLIC
  NodeOptions &start_parameter_event_publisher(bool start_parameter_event_publisher);

  /// 返回对时钟类型的引用。 (Return a reference to the clock type.)
  RCLCPP_PUBLIC
  const rcl_clock_type_t &clock_type() const;

  /// 设置时钟类型。(Set the clock type.)
  /**
   * 节点要使用的时钟类型。(The clock type to be used by the node.)
   */
  RCLCPP_PUBLIC
  NodeOptions &clock_type(const rcl_clock_type_t &clock_type);

  /// 返回对时钟 QoS 的引用。 (Return a reference to the clock QoS.)
  RCLCPP_PUBLIC
  const rclcpp::QoS &clock_qos() const;

  /// 设置时钟 QoS。(Set the clock QoS.)
  /**
   * 如果启用了 /clock 主题上的发布者，则使用的 QoS 设置。(The QoS settings to be used for the
   * publisher on /clock topic, if enabled.)
   */
  RCLCPP_PUBLIC
  NodeOptions &clock_qos(const rclcpp::QoS &clock_qos);

  /// 返回 use_clock_thread 标志。 (Return the use_clock_thread flag.)
  /**
   * @return 是否使用时钟线程的布尔值。(A boolean value indicating whether to use clock thread or
   * not.)
   */
  RCLCPP_PUBLIC
  bool use_clock_thread() const;

  /// 设置 use_clock_thread 标志，返回 this 以实现参数惯用法。 (Set the use_clock_thread flag,
  /// return this for parameter idiom.)
  /**
   * 如果为 true，则将使用专用线程订阅 "/clock" 主题。 (If true, a dedicated thread will be used to
   * subscribe to "/clock" topic.)
   *
   * @param[in] use_clock_thread 一个布尔值，表示是否使用时钟线程。(A boolean value indicating
   * whether to use clock thread or not.)
   * @return 当前 NodeOptions 对象的引用，以便进行链式调用。(A reference to the current NodeOptions
   * object for chaining calls.)
   */
  RCLCPP_PUBLIC
  NodeOptions &use_clock_thread(bool use_clock_thread);

  /// 返回对 parameter_event_qos QoS 的引用。 (Return a reference to the parameter_event_qos QoS.)
  /**
   * @return 参数事件 QoS 的常量引用。 (A constant reference to the parameter event QoS.)
   */
  RCLCPP_PUBLIC
  const rclcpp::QoS &parameter_event_qos() const;

  /// 设置 parameter_event_qos QoS，返回 this 以实现参数惯用法。 (Set the parameter_event_qos QoS,
  /// return this for parameter idiom.)
  /**
   * 如果启用，则用于参数事件发布器的 QoS 设置。 (The QoS settings to be used for the parameter
   * event publisher, if enabled.)
   *
   * @param[in] parameter_event_qos 参数事件 QoS 的引用。(A reference to the parameter event QoS.)
   * @return 当前 NodeOptions 对象的引用，以便进行链式调用。 (A reference to the current NodeOptions
   * object for chaining calls.)
   */
  RCLCPP_PUBLIC
  NodeOptions &parameter_event_qos(const rclcpp::QoS &parameter_event_qos);

  /// 返回对 rosout QoS 的引用。 (Return a reference to the rosout QoS.)
  /**
   * @return rosout QoS 的常量引用。 (A constant reference to the rosout QoS.)
   */
  RCLCPP_PUBLIC
  const rclcpp::QoS &rosout_qos() const;

  /// 设置 rosout QoS。 (Set the rosout QoS.)
  /**
   * 如果启用，则用于 /rosout 主题上的发布器的 QoS 设置。 (The QoS settings to be used for the
   * publisher on /rosout topic, if enabled.)
   *
   * @param[in] rosout_qos rosout QoS 的引用。 (A reference to the rosout QoS.)
   * @return 当前 NodeOptions 对象的引用，以便进行链式调用。 (A reference to the current NodeOptions
   * object for chaining calls.)
   */
  RCLCPP_PUBLIC
  NodeOptions &rosout_qos(const rclcpp::QoS &rosout_qos);

  /// 返回对 parameter_event_publisher_options 的引用。 (Return a reference to the
  /// parameter_event_publisher_options.)
  /**
   * @return 参数事件发布器选项的常量引用。 (A constant reference to the parameter event publisher
   * options.)
   */
  RCLCPP_PUBLIC
  const rclcpp::PublisherOptionsBase &parameter_event_publisher_options() const;

  /// 设置 parameter_event_publisher_options，返回 this 以实现参数惯用法。
  /// Set the parameter_event_publisher_options, return this for parameter idiom.
  /**
   * 如果启用，将使用的 QoS 设置作为参数事件发布器。
   * The QoS settings to be used for the parameter event publisher, if enabled.
   *
   * \todo(wjwwood): 让此处接收/存储一个 rclcpp::PublisherOptionsWithAllocator<Allocator> 的实例，
   *   但要做到这一点需要 NodeOptions 也基于分配器类型进行模板化。
   * \todo(wjwwood): make this take/store an instance of
   *   rclcpp::PublisherOptionsWithAllocator<Allocator>, but to do that requires
   *   NodeOptions to also be templated based on the Allocator type.
   */
  RCLCPP_PUBLIC
  NodeOptions &parameter_event_publisher_options(
      const rclcpp::PublisherOptionsBase &parameter_event_publisher_options);

  /// 返回 allow_undeclared_parameters 标志。
  /// Return the allow_undeclared_parameters flag.
  RCLCPP_PUBLIC
  bool allow_undeclared_parameters() const;

  /// 设置 allow_undeclared_parameters，返回 this 以实现参数惯用法。
  /// Set the allow_undeclared_parameters, return this for parameter idiom.
  /**
   * 如果为 true，则允许在节点上设置任何参数名称，而无需先声明。
   * If true, allow any parameter name to be set on the node without first
   * being declared.
   * 否则，设置未声明的参数将引发异常。
   * Otherwise, setting an undeclared parameter will raise an exception.
   *
   * 此选项为 true 不会影响 parameter_overrides，因为第一次设置操作将隐式声明参数，
   *   因此会考虑任何参数覆盖。
   * This option being true does not affect parameter_overrides, as the first
   * set action will implicitly declare the parameter and therefore consider
   * any parameter overrides.
   */
  RCLCPP_PUBLIC
  NodeOptions &allow_undeclared_parameters(bool allow_undeclared_parameters);

  /// 返回 automatically_declare_parameters_from_overrides 标志。
  /// Return the automatically_declare_parameters_from_overrides flag.
  RCLCPP_PUBLIC
  bool automatically_declare_parameters_from_overrides() const;

  /// 设置 automatically_declare_parameters_from_overrides，返回 this。
  /// Set the automatically_declare_parameters_from_overrides, return this.
  /**
   * 如果为 true，则自动遍历节点的参数覆盖并隐式声明尚未声明的任何参数。
   * If true, automatically iterate through the node's parameter overrides and
   * implicitly declare any that have not already been declared.
   * 否则，传递给节点的参数重载和/或全局参数（例如来自 YAML
   * 文件的参数重载）中未明确声明的参数将根本不会出现在节点上， 即使 `allow_undeclared_parameters`
   * 为 true。 Otherwise, parameters passed to the node's parameter_overrides, and/or the global
   * arguments (e.g. parameter overrides from a YAML file), which are not explicitly declared will
   * not appear on the node at all, even if `allow_undeclared_parameters` is true.
   * 参数声明从覆盖开始是在节点的基本构造函数中完成的，因此用户必须注意检查参数是否已经（例如自动）声明，然后再声明它们。
   * Parameter declaration from overrides is done in the node's base constructor,
   * so the user must take care to check if the parameter is already (e.g.
   * automatically) declared before declaring it themselves.
   * 已声明的参数将不会被重新声明，以这种方式声明的参数将使用默认构造的 ParameterDescriptor。
   * Already declared parameters will not be re-declared, and parameters
   * declared in this way will use the default constructed ParameterDescriptor.
   */
  RCLCPP_PUBLIC
  NodeOptions &automatically_declare_parameters_from_overrides(
      bool automatically_declare_parameters_from_overrides);

  /// 返回要使用的 rcl_allocator_t。
  /// Return the rcl_allocator_t to be used.
  RCLCPP_PUBLIC
  const rcl_allocator_t &allocator() const;

  /// 设置要使用的 rcl_allocator_t，可能导致现有 rcl_node_options_t 的释放。
  /// Set the rcl_allocator_t to be used, may cause deallocation of existing rcl_node_options_t.
  /**
   * 这将导致内部 rcl_node_options_t 结构无效。
   * This will cause the internal rcl_node_options_t struct to be invalidated.
   */
  RCLCPP_PUBLIC
  NodeOptions &allocator(rcl_allocator_t allocator);

private:
  // 这是可变的，以允许使用懒惰创建节点选项实例的 const 访问器。
  // This is mutable to allow for a const accessor which lazily creates the node options instance.
  /// 底层 rcl_node_options 结构。
  /// Underlying rcl_node_options structure.
  mutable std::unique_ptr<rcl_node_options_t, void (*)(rcl_node_options_t *)> node_options_;

  // 重要：如果更改了任何这些默认值，请更新此类中的文档。
  // IMPORTANT: if any of these default values are changed, please update the
  // documentation in this class.

  rclcpp::Context::SharedPtr context_{rclcpp::contexts::get_global_default_context()};
  std::vector<std::string> arguments_{};
  std::vector<rclcpp::Parameter> parameter_overrides_{};
  bool use_global_arguments_{true};
  bool enable_rosout_{true};
  bool use_intra_process_comms_{false};
  bool enable_topic_statistics_{false};
  bool start_parameter_services_{true};
  bool start_parameter_event_publisher_{true};
  rcl_clock_type_t clock_type_{RCL_ROS_TIME};
  rclcpp::QoS clock_qos_ = rclcpp::ClockQoS();
  bool use_clock_thread_{true};
  rclcpp::QoS parameter_event_qos_ = rclcpp::ParameterEventsQoS(
      rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events));
  rclcpp::QoS rosout_qos_ = rclcpp::RosoutQoS();
  rclcpp::PublisherOptionsBase parameter_event_publisher_options_ = rclcpp::PublisherOptionsBase();
  bool allow_undeclared_parameters_{false};
  bool automatically_declare_parameters_from_overrides_{false};
  rcl_allocator_t allocator_{rcl_get_default_allocator()};
};

}  // namespace rclcpp

#endif  // RCLCPP__NODE_OPTIONS_HPP_

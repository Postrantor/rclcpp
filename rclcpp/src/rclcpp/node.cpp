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

#include "rclcpp/node.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "./detail/resolve_parameter_overrides.hpp"
#include "rcl/arguments.h"
#include "rclcpp/detail/qos_parameters.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/node_interfaces/node_clock.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"
#include "rclcpp/node_interfaces/node_parameters.hpp"
#include "rclcpp/node_interfaces/node_services.hpp"
#include "rclcpp/node_interfaces/node_time_source.hpp"
#include "rclcpp/node_interfaces/node_timers.hpp"
#include "rclcpp/node_interfaces/node_topics.hpp"
#include "rclcpp/node_interfaces/node_waitables.hpp"
#include "rclcpp/qos_overriding_options.hpp"
#include "rmw/validate_namespace.h"

using rclcpp::Node;
using rclcpp::NodeOptions;
using rclcpp::exceptions::throw_from_rcl_error;

namespace {

/**
 * @brief 扩展子命名空间
 * @param existing_sub_namespace 已存在的子命名空间
 * @param extension 要添加的子命名空间扩展
 * @return 返回新的子命名空间字符串
 * @throws rclcpp::exceptions::NameValidationError 如果子命名空间扩展为空或以'/'开头
 *
 * @brief Extend sub-namespace
 * @param existing_sub_namespace The existing sub-namespace
 * @param extension The sub-namespace extension to be added
 * @return Returns the new sub-namespace string
 * @throws rclcpp::exceptions::NameValidationError if the sub-namespace extension is empty or starts
 * with '/'
 */
RCLCPP_LOCAL
std::string extend_sub_namespace(
    const std::string& existing_sub_namespace, const std::string& extension) {
  // 假设已经检查过现有的子命名空间，因为它在之前使用此函数设置时已经被检查过了
  // Assumption is that the existing_sub_namespace does not need checking
  // because it would be checked already when it was set with this function.

  if (extension.empty()) {
    throw rclcpp::exceptions::NameValidationError(
        "sub_namespace", extension.c_str(),
        "sub-nodes should not extend nodes by an empty sub-namespace", 0);
  } else if (extension.front() == '/') {
    // 检查新的子命名空间扩展是否是绝对的
    // check if the new sub-namespace extension is absolute
    throw rclcpp::exceptions::NameValidationError(
        "sub_namespace", extension.c_str(), "a sub-namespace should not have a leading /", 0);
  }

  std::string new_sub_namespace;
  if (existing_sub_namespace.empty()) {
    new_sub_namespace = extension;
  } else {
    new_sub_namespace = existing_sub_namespace + "/" + extension;
  }

  // 删除任何尾随的'/'，以便新的扩展不会导致'//'
  // remove any trailing `/` so that new extensions do not result in `//`
  if (new_sub_namespace.back() == '/') {
    new_sub_namespace = new_sub_namespace.substr(0, new_sub_namespace.size() - 1);
  }

  return new_sub_namespace;
}

/**
 * @brief 创建有效命名空间
 * @param node_namespace 节点命名空间
 * @param sub_namespace 子命名空间
 * @return 返回组合后的有效命名空间字符串
 *
 * @brief Create effective namespace
 * @param node_namespace The node namespace
 * @param sub_namespace The sub-namespace
 * @return Returns the combined effective namespace string
 */
RCLCPP_LOCAL
std::string create_effective_namespace(
    const std::string& node_namespace, const std::string& sub_namespace) {
  // 假设node_namespace和sub_namespace都符合要求，不需要修剪'/'等，因为它们已经在其他函数中验证过了
  // Assumption is that both the node_namespace and sub_namespace are conforming
  // and do not need trimming of `/` and other things, as they were validated
  // in other functions already.

  // 如果节点没有子命名空间（即不是子节点），则直接返回原始命名空间
  // A node may not have a sub_namespace if it is no sub_node. In this case,
  // just return the original namespace
  if (sub_namespace.empty()) {
    return node_namespace;
  } else if (node_namespace.back() == '/') {
    // 这是node_namespace只是'/'的特殊情况
    // this is the special case where node_namespace is just `/`
    return node_namespace + sub_namespace;
  } else {
    return node_namespace + "/" + sub_namespace;
  }
}

}  // namespace

/**
 * @brief 构造函数，创建一个节点 (Constructor, creates a node)
 *
 * @param node_name 节点名称 (Node name)
 * @param options 节点选项 (Node options)
 */
Node::Node(const std::string& node_name, const NodeOptions& options)
    : Node(node_name, "", options) {
  // 空实现，仅调用另一个构造函数 (Empty implementation, just calls another constructor)
}

/**
 * @brief 获取参数事件的 QoS (Get the QoS for parameter events)
 *
 * @param node_base 节点基础接口引用 (Reference to the node base interface)
 * @param options 节点选项 (Node options)
 * @return rclcpp::QoS 参数事件的 QoS (QoS for parameter events)
 */
static rclcpp::QoS get_parameter_events_qos(
    rclcpp::node_interfaces::NodeBaseInterface& node_base, const rclcpp::NodeOptions& options) {
  // 获取用户提供的 QoS 或默认 QoS (Get user-provided QoS or default QoS)
  auto final_qos = options.parameter_event_qos();

  // 初始化全局参数指针 (Initialize global arguments pointer)
  const rcl_arguments_t* global_args = nullptr;

  // 获取 RCL 节点选项 (Get RCL node options)
  auto* rcl_options = options.get_rcl_node_options();

  // 如果使用全局参数 (If using global arguments)
  if (rcl_options->use_global_arguments) {
    // 获取 RCL 上下文 (Get RCL context)
    auto context_ptr = node_base.get_context()->get_rcl_context();

    // 设置全局参数 (Set global arguments)
    global_args = &(context_ptr->global_arguments);
  }

  // 解析参数覆盖 (Resolve parameter overrides)
  auto parameter_overrides = rclcpp::detail::resolve_parameter_overrides(
      node_base.get_fully_qualified_name(), options.parameter_overrides(), &rcl_options->arguments,
      global_args);

  // 解析最终的主题名称 (Resolve the final topic name)
  auto final_topic_name = node_base.resolve_topic_or_service_name("/parameter_events", false);

  // 设置 QoS 覆盖前缀 (Set QoS override prefix)
  auto prefix = "qos_overrides." + final_topic_name + ".";

  // 定义要检查的 QoS 策略 (Define QoS policies to check)
  std::array<rclcpp::QosPolicyKind, 4> policies = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
  };

  // 遍历策略并应用覆盖 (Iterate through policies and apply overrides)
  for (const auto& policy : policies) {
    // 构建参数名称 (Build parameter name)
    auto param_name = prefix + rclcpp::qos_policy_kind_to_cstr(policy);

    // 查找参数覆盖 (Find parameter override)
    auto it = parameter_overrides.find(param_name);

    // 获取值 (Get value)
    auto value = it != parameter_overrides.end() ? it->second
                                                 : rclcpp::detail::get_default_qos_param_value(
                                                       policy, options.parameter_event_qos());

    // 应用 QoS 覆盖 (Apply QoS override)
    rclcpp::detail::apply_qos_override(policy, value, final_qos);
  }

  // 返回最终的 QoS (Return the final QoS)
  return final_qos;
}

/**
 * @brief 构造函数，用于创建一个新的节点实例 (Constructor for creating a new Node instance)
 *
 * @param node_name 节点的名称 (Name of the node)
 * @param namespace_ 节点所属的命名空间 (Namespace the node belongs to)
 * @param options 节点选项，包括上下文、通信方式等 (Node options, including context, communication
 * method, etc.)
 */
Node::Node(const std::string& node_name, const std::string& namespace_, const NodeOptions& options)
    : node_base_(new rclcpp::node_interfaces::NodeBase(  // 创建节点基础接口实例 (Create a NodeBase
                                                         // interface instance)
          node_name,
          namespace_,
          options.context(),                  // 获取节点上下文 (Get the node context)
          *(options.get_rcl_node_options()),  // 获取RCL节点选项 (Get RCL node options)
          options.use_intra_process_comms(),  // 是否使用进程内通信 (Whether to use intra-process
                                              // communication)
          options.enable_topic_statistics())),  // 是否启用主题统计 (Whether to enable topic
                                                // statistics)
      node_graph_(new rclcpp::node_interfaces::NodeGraph(
          node_base_.get())),  // 创建节点图接口实例 (Create a NodeGraph interface instance)
      node_logging_(new rclcpp::node_interfaces::NodeLogging(
          node_base_.get())),  // 创建节点日志接口实例 (Create a NodeLogging interface instance)
      node_timers_(new rclcpp::node_interfaces::NodeTimers(
          node_base_.get())),  // 创建节点定时器接口实例 (Create a NodeTimers interface instance)
      node_topics_(new rclcpp::node_interfaces::NodeTopics(
          node_base_.get(),
          node_timers_.get())),  // 创建节点主题接口实例 (Create a NodeTopics interface instance)
      node_services_(new rclcpp::node_interfaces::NodeServices(
          node_base_.get())),  // 创建节点服务接口实例 (Create a NodeServices interface instance)
      node_clock_(new rclcpp::node_interfaces::NodeClock(  // 创建节点时钟接口实例 (Create a
                                                           // NodeClock interface instance)
          node_base_,
          node_topics_,
          node_graph_,
          node_services_,
          node_logging_,
          options.clock_type())),
      node_parameters_(new rclcpp::node_interfaces::NodeParameters(  // 创建节点参数接口实例 (Create
                                                                     // a NodeParameters interface
                                                                     // instance)
          node_base_,
          node_logging_,
          node_topics_,
          node_services_,
          node_clock_,
          options.parameter_overrides(),       // 参数覆盖 (Parameter overrides)
          options.start_parameter_services(),  // 是否启动参数服务 (Whether to start parameter
                                               // services)
          options.start_parameter_event_publisher(),  // 是否启动参数事件发布器 (Whether to start
                                                      // the parameter event publisher)
          // 获取参数事件的QoS配置 (Get QoS configuration for parameter events)
          get_parameter_events_qos(*node_base_, options),
          options.parameter_event_publisher_options(),
          options.allow_undeclared_parameters(),  // 是否允许未声明的参数 (Whether to allow
                                                  // undeclared parameters)
          options.automatically_declare_parameters_from_overrides())),  // 是否自动从覆盖中声明参数
                                                                        // (Whether to automatically
                                                                        // declare parameters from
                                                                        // overrides)
      node_time_source_(
          new rclcpp::node_interfaces::NodeTimeSource(  // 创建节点时间源接口实例 (Create a
                                                        // NodeTimeSource interface instance)
              node_base_,
              node_topics_,
              node_graph_,
              node_services_,
              node_logging_,
              node_clock_,
              node_parameters_,
              options.clock_qos(),           // 时钟QoS配置 (Clock QoS configuration)
              options.use_clock_thread())),  // 是否使用时钟线程 (Whether to use a clock thread)
      node_waitables_(new rclcpp::node_interfaces::NodeWaitables(
          node_base_.get())),  // 创建节点可等待接口实例 (Create a NodeWaitables interface instance)
      node_options_(options),
      sub_namespace_(""),
      effective_namespace_(create_effective_namespace(this->get_namespace(), sub_namespace_)) {
  // 声明QoS参数，使其可见 (Declare QoS parameters so they are visible)
  rclcpp::detail::declare_qos_parameters(
      rclcpp::QosOverridingOptions{
          QosPolicyKind::Depth,
          QosPolicyKind::Durability,
          QosPolicyKind::History,
          QosPolicyKind::Reliability,
      },
      node_parameters_, node_topics_->resolve_topic_name("/parameter_events"),
      options.parameter_event_qos(), rclcpp::detail::PublisherQosParametersTraits{});
}

/**
 * @brief 构造函数，用于从现有节点创建一个新的子节点 (Constructor for creating a new sub-node from
 * an existing node)
 *
 * @param other 现有节点 (Existing node)
 * @param sub_namespace 子命名空间 (Sub-namespace)
 */
Node::Node(const Node& other, const std::string& sub_namespace)
    : node_base_(other.node_base_),
      node_graph_(other.node_graph_),
      node_logging_(other.node_logging_),
      node_timers_(other.node_timers_),
      node_topics_(other.node_topics_),
      node_services_(other.node_services_),
      node_clock_(other.node_clock_),
      node_parameters_(other.node_parameters_),
      node_time_source_(other.node_time_source_),
      node_waitables_(other.node_waitables_),
      node_options_(other.node_options_),
      // 扩展子命名空间 (Extend the sub-namespace)
      sub_namespace_(extend_sub_namespace(other.get_sub_namespace(), sub_namespace)),
      // 创建有效命名空间 (Create effective namespace)
      effective_namespace_(create_effective_namespace(other.get_namespace(), sub_namespace_)) {
  // 验证新的有效命名空间 (Validate the new effective namespace)
  int validation_result;
  size_t invalid_index;
  rmw_ret_t rmw_ret =
      rmw_validate_namespace(effective_namespace_.c_str(), &validation_result, &invalid_index);

  // 检查验证结果 (Check the validation result)
  if (rmw_ret != RMW_RET_OK) {
    if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
      throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "failed to validate subnode namespace");
    }
    throw_from_rcl_error(RCL_RET_ERROR, "failed to validate subnode namespace");
  }

  // 检查命名空间有效性 (Check the namespace validity)
  if (validation_result != RMW_NAMESPACE_VALID) {
    throw rclcpp::exceptions::InvalidNamespaceError(
        effective_namespace_.c_str(), rmw_namespace_validation_result_string(validation_result),
        invalid_index);
  }
}

/**
 * @brief 节点析构函数 (Node destructor)
 */
Node::~Node() {
  // 按照允许在拆卸期间向 node_base 查询的顺序释放子接口 (Release sub-interfaces in an order that
  // allows them to consult with node_base during tear-down)
  node_waitables_.reset();
  node_time_source_.reset();
  node_parameters_.reset();
  node_clock_.reset();
  node_services_.reset();
  node_topics_.reset();
  node_timers_.reset();
  node_logging_.reset();
  node_graph_.reset();
}

/*!
 * \brief 获取节点的名称 (Get the name of the node)
 * \return 节点的名称 (const char pointer to the name of the node)
 */
const char* Node::get_name() const {
  // 返回 node_base_ 对象中存储的节点名称 (Return the node name stored in the node_base_ object)
  return node_base_->get_name();
}

/*!
 * \brief 获取节点的命名空间 (Get the namespace of the node)
 * \return 节点的命名空间 (const char pointer to the namespace of the node)
 */
const char* Node::get_namespace() const {
  // 返回 node_base_ 对象中存储的节点命名空间 (Return the node namespace stored in the node_base_
  // object)
  return node_base_->get_namespace();
}

/*!
 * \brief 获取节点的完全限定名称 (Get the fully qualified name of the node)
 * \return 节点的完全限定名称 (const char pointer to the fully qualified name of the node)
 */
const char* Node::get_fully_qualified_name() const {
  // 返回 node_base_ 对象中存储的节点的完全限定名称 (Return the fully qualified name of the node
  // stored in the node_base_ object)
  return node_base_->get_fully_qualified_name();
}

/*!
 * \brief 获取节点的日志记录器 (Get the logger of the node)
 * \return 节点的日志记录器 (rclcpp::Logger object for the node)
 */
rclcpp::Logger Node::get_logger() const {
  // 返回 node_logging_ 对象中的日志记录器 (Return the logger from the node_logging_ object)
  return node_logging_->get_logger();
}

/*!
 * \brief 创建回调组 (Create a callback group)
 * \param group_type 回调组类型 (rclcpp::CallbackGroupType for the group)
 * \param automatically_add_to_executor_with_node 是否自动添加到与节点关联的执行器中 (Whether to
 * automatically add to executor associated with node) \return 创建的回调组
 * (rclcpp::CallbackGroup::SharedPtr to the created callback group)
 */
rclcpp::CallbackGroup::SharedPtr Node::create_callback_group(
    rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node) {
  // 在 node_base_ 对象中创建回调组 (Create the callback group in the node_base_ object)
  return node_base_->create_callback_group(group_type, automatically_add_to_executor_with_node);
}

/*!
 * \brief 声明参数 (Declare a parameter)
 * \param name 参数名称 (The name of the parameter)
 * \param default_value 默认值 (The default value of the parameter)
 * \param parameter_descriptor 参数描述符 (The parameter descriptor)
 * \param ignore_override 是否忽略覆盖 (Whether to ignore override)
 * \return 声明的参数值 (const reference to the declared rclcpp::ParameterValue)
 */
const rclcpp::ParameterValue& Node::declare_parameter(
    const std::string& name,
    const rclcpp::ParameterValue& default_value,
    const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor,
    bool ignore_override) {
  // 在 node_parameters_ 对象中声明参数 (Declare the parameter in the node_parameters_ object)
  return this->node_parameters_->declare_parameter(
      name, default_value, parameter_descriptor, ignore_override);
}

/*!
 * \brief 声明参数 (Declare a parameter)
 * \param name 参数名称 (The name of the parameter)
 * \param type 参数类型 (The type of the parameter)
 * \param parameter_descriptor 参数描述符 (The parameter descriptor)
 * \param ignore_override 是否忽略覆盖 (Whether to ignore override)
 * \return 声明的参数值 (const reference to the declared rclcpp::ParameterValue)
 */
const rclcpp::ParameterValue& Node::declare_parameter(
    const std::string& name,
    rclcpp::ParameterType type,
    const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor,
    bool ignore_override) {
  // 在 node_parameters_ 对象中声明参数 (Declare the parameter in the node_parameters_ object)
  return this->node_parameters_->declare_parameter(
      name, type, parameter_descriptor, ignore_override);
}

/*!
 * \brief 取消声明参数 (Undeclare a parameter)
 * \param name 参数名称 (The name of the parameter)
 */
void Node::undeclare_parameter(const std::string& name) {
  // 在 node_parameters_ 对象中取消声明参数 (Undeclare the parameter in the node_parameters_ object)
  this->node_parameters_->undeclare_parameter(name);
}

/*!
 * \brief 检查节点是否有指定参数 (Check if the node has a specific parameter)
 * \param name 参数名称 (The name of the parameter)
 * \return 如果节点有该参数，则返回 true (Return true if the node has the parameter)
 */
bool Node::has_parameter(const std::string& name) const {
  // 在 node_parameters_ 对象中检查参数是否存在 (Check if the parameter exists in the
  // node_parameters_ object)
  return this->node_parameters_->has_parameter(name);
}

/*!
 * \brief 设置参数 (Set a parameter)
 * \param parameter 要设置的参数 (The rclcpp::Parameter to set)
 * \return 设置参数结果 (rcl_interfaces::msg::SetParametersResult object with the result of the
 * operation)
 */
rcl_interfaces::msg::SetParametersResult Node::set_parameter(const rclcpp::Parameter& parameter) {
  // 在 node_parameters_ 对象中原子性地设置参数 (Set the parameter atomically in the
  // node_parameters_ object)
  return node_parameters_->set_parameters_atomically({parameter});
}

/*!
 * \brief 设置一组参数 (Set a group of parameters)
 * \param parameters 要设置的参数组 (The vector of rclcpp::Parameter objects to set)
 * \return 设置参数结果的向量 (Vector of rcl_interfaces::msg::SetParametersResult objects with the
 * results of the operations)
 */
std::vector<rcl_interfaces::msg::SetParametersResult> Node::set_parameters(
    const std::vector<rclcpp::Parameter>& parameters) {
  // 在 node_parameters_ 对象中设置参数组 (Set the group of parameters in the node_parameters_
  // object)
  return node_parameters_->set_parameters(parameters);
}

/**
 * @brief 设置参数原子操作
 * @param parameters 要设置的参数列表
 * @return rcl_interfaces::msg::SetParametersResult 设置参数结果
 *
 * @brief Set parameters atomically
 * @param parameters The list of parameters to be set
 * @return rcl_interfaces::msg::SetParametersResult The result of setting the parameters
 */
rcl_interfaces::msg::SetParametersResult Node::set_parameters_atomically(
    const std::vector<rclcpp::Parameter>& parameters) {
  // 调用 node_parameters_ 对象的 set_parameters_atomically 方法进行参数设置
  // Call the set_parameters_atomically method of the node_parameters_ object to set the parameters
  return node_parameters_->set_parameters_atomically(parameters);
}

/**
 * @brief 获取指定名称的参数
 * @param name 参数名称
 * @return rclcpp::Parameter 返回获取到的参数
 *
 * @brief Get parameter with specified name
 * @param name Parameter name
 * @return rclcpp::Parameter The obtained parameter
 */
rclcpp::Parameter Node::get_parameter(const std::string& name) const {
  // 调用 node_parameters_ 对象的 get_parameter 方法获取指定名称的参数
  // Call the get_parameter method of the node_parameters_ object to get the parameter with the
  // specified name
  return node_parameters_->get_parameter(name);
}

/**
 * @brief 获取指定名称的参数
 * @param name 参数名称
 * @param parameter 输出参数，用于存储获取到的参数
 * @return bool 如果成功获取参数，则返回 true，否则返回 false
 *
 * @brief Get parameter with specified name
 * @param name Parameter name
 * @param parameter Output parameter, used to store the obtained parameter
 * @return bool Return true if the parameter is successfully obtained, otherwise return false
 */
bool Node::get_parameter(const std::string& name, rclcpp::Parameter& parameter) const {
  // 调用 node_parameters_ 对象的 get_parameter 方法获取指定名称的参数，并将结果存储在 parameter 中
  // Call the get_parameter method of the node_parameters_ object to get the parameter with the
  // specified name and store the result in parameter
  return node_parameters_->get_parameter(name, parameter);
}

/**
 * @brief 获取一组参数
 * @param names 参数名称列表
 * @return std::vector<rclcpp::Parameter> 返回获取到的参数列表
 *
 * @brief Get a group of parameters
 * @param names List of parameter names
 * @return std::vector<rclcpp::Parameter> The list of obtained parameters
 */
std::vector<rclcpp::Parameter> Node::get_parameters(const std::vector<std::string>& names) const {
  // 调用 node_parameters_ 对象的 get_parameters 方法获取一组参数
  // Call the get_parameters method of the node_parameters_ object to get a group of parameters
  return node_parameters_->get_parameters(names);
}

/**
 * @brief 描述指定名称的参数
 * @param name 参数名称
 * @return rcl_interfaces::msg::ParameterDescriptor 返回参数描述信息
 *
 * @brief Describe parameter with specified name
 * @param name Parameter name
 * @return rcl_interfaces::msg::ParameterDescriptor The parameter description information
 */
rcl_interfaces::msg::ParameterDescriptor Node::describe_parameter(const std::string& name) const {
  // 调用 node_parameters_ 对象的 describe_parameters 方法描述参数
  // Call the describe_parameters method of the node_parameters_ object to describe the parameter
  auto result = node_parameters_->describe_parameters({name});
  if (0 == result.size()) {
    // 如果结果为空，抛出参数未声明异常
    // If the result is empty, throw a ParameterNotDeclaredException
    throw rclcpp::exceptions::ParameterNotDeclaredException(name);
  }
  if (result.size() > 1) {
    // 如果结果数量大于1，抛出运行时错误
    // If the number of results is greater than 1, throw a runtime error
    throw std::runtime_error("number of described parameters unexpectedly more than one");
  }
  // 返回第一个参数描述
  // Return the first parameter description
  return result.front();
}

/**
 * @brief 描述一组参数
 * @param names 参数名称列表
 * @return std::vector<rcl_interfaces::msg::ParameterDescriptor> 返回参数描述信息列表
 *
 * @brief Describe a group of parameters
 * @param names List of parameter names
 * @return std::vector<rcl_interfaces::msg::ParameterDescriptor> The list of parameter description
 * information
 */
std::vector<rcl_interfaces::msg::ParameterDescriptor> Node::describe_parameters(
    const std::vector<std::string>& names) const {
  // 调用 node_parameters_ 对象的 describe_parameters 方法描述一组参数
  // Call the describe_parameters method of the node_parameters_ object to describe a group of
  // parameters
  return node_parameters_->describe_parameters(names);
}

/**
 * @brief 获取一组参数的类型
 * @param names 参数名称列表
 * @return std::vector<uint8_t> 返回参数类型列表
 *
 * @brief Get the types of a group of parameters
 * @param names List of parameter names
 * @return std::vector<uint8_t> The list of parameter types
 */
std::vector<uint8_t> Node::get_parameter_types(const std::vector<std::string>& names) const {
  // 调用 node_parameters_ 对象的 get_parameter_types 方法获取一组参数的类型
  // Call the get_parameter_types method of the node_parameters_ object to get the types of a group
  // of parameters
  return node_parameters_->get_parameter_types(names);
}

/**
 * @brief 列出参数
 * @param prefixes 参数名称前缀列表
 * @param depth 搜索深度
 * @return rcl_interfaces::msg::ListParametersResult 返回列出的参数结果
 *
 * @brief List parameters
 * @param prefixes List of parameter name prefixes
 * @param depth Search depth
 * @return rcl_interfaces::msg::ListParametersResult The result of listing the parameters
 */
rcl_interfaces::msg::ListParametersResult Node::list_parameters(
    const std::vector<std::string>& prefixes, uint64_t depth) const {
  // 调用 node_parameters_ 对象的 list_parameters 方法列出参数
  // Call the list_parameters method of the node_parameters_ object to list the parameters
  return node_parameters_->list_parameters(prefixes, depth);
}

/**
 * @brief 添加预设置参数回调
 * @param callback 预设置参数回调函数
 * @return rclcpp::Node::PreSetParametersCallbackHandle::SharedPtr 返回回调句柄的共享指针
 *
 * @brief Add pre-set parameters callback
 * @param callback Pre-set parameters callback function
 * @return rclcpp::Node::PreSetParametersCallbackHandle::SharedPtr Shared pointer to the callback
 * handle
 */
rclcpp::Node::PreSetParametersCallbackHandle::SharedPtr Node::add_pre_set_parameters_callback(
    PreSetParametersCallbackType callback) {
  // 调用 node_parameters_ 对象的 add_pre_set_parameters_callback 方法添加预设置参数回调
  // Call the add_pre_set_parameters_callback method of the node_parameters_ object to add a pre-set
  // parameters callback
  return node_parameters_->add_pre_set_parameters_callback(callback);
}

/**
 * @brief 添加设置参数回调函数。
 * @param callback 设置参数回调函数类型。
 * @return 返回设置参数回调句柄的共享指针。
 *
 * @brief Add a set parameters callback.
 * @param callback The set parameters callback type.
 * @return Returns a shared pointer to the set parameters callback handle.
 */
rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr Node::add_on_set_parameters_callback(
    OnSetParametersCallbackType callback) {
  // 调用 node_parameters_ 的 add_on_set_parameters_callback 方法，并返回结果。
  // Call the add_on_set_parameters_callback method of node_parameters_ and return the result.
  return node_parameters_->add_on_set_parameters_callback(callback);
}

/**
 * @brief 添加 post 设置参数回调函数。
 * @param callback post 设置参数回调函数类型。
 * @return 返回 post 设置参数回调句柄的共享指针。
 *
 * @brief Add a post set parameters callback.
 * @param callback The post set parameters callback type.
 * @return Returns a shared pointer to the post set parameters callback handle.
 */
rclcpp::Node::PostSetParametersCallbackHandle::SharedPtr Node::add_post_set_parameters_callback(
    PostSetParametersCallbackType callback) {
  // 调用 node_parameters_ 的 add_post_set_parameters_callback 方法，并返回结果。
  // Call the add_post_set_parameters_callback method of node_parameters_ and return the result.
  return node_parameters_->add_post_set_parameters_callback(callback);
}

/**
 * @brief 删除 pre 设置参数回调函数。
 * @param handler pre 设置参数回调句柄。
 *
 * @brief Remove a pre set parameters callback.
 * @param handler The pre set parameters callback handle.
 */
void Node::remove_pre_set_parameters_callback(const PreSetParametersCallbackHandle* const handler) {
  // 调用 node_parameters_ 的 remove_pre_set_parameters_callback 方法。
  // Call the remove_pre_set_parameters_callback method of node_parameters_.
  node_parameters_->remove_pre_set_parameters_callback(handler);
}

/**
 * @brief 删除设置参数回调函数。
 * @param handler 设置参数回调句柄。
 *
 * @brief Remove a set parameters callback.
 * @param handler The set parameters callback handle.
 */
void Node::remove_on_set_parameters_callback(const OnSetParametersCallbackHandle* const handler) {
  // 调用 node_parameters_ 的 remove_on_set_parameters_callback 方法。
  // Call the remove_on_set_parameters_callback method of node_parameters_.
  node_parameters_->remove_on_set_parameters_callback(handler);
}

/**
 * @brief 删除 post 设置参数回调函数。
 * @param handler post 设置参数回调句柄。
 *
 * @brief Remove a post set parameters callback.
 * @param handler The post set parameters callback handle.
 */
void Node::remove_post_set_parameters_callback(
    const PostSetParametersCallbackHandle* const handler) {
  // 调用 node_parameters_ 的 remove_post_set_parameters_callback 方法。
  // Call the remove_post_set_parameters_callback method of node_parameters_.
  node_parameters_->remove_post_set_parameters_callback(handler);
}

// 获取节点名称列表。
// Get the list of node names.
std::vector<std::string> Node::get_node_names() const { return node_graph_->get_node_names(); }

// 获取主题名称和类型的映射。
// Get the mapping of topic names and types.
std::map<std::string, std::vector<std::string>> Node::get_topic_names_and_types() const {
  return node_graph_->get_topic_names_and_types();
}

// 获取服务名称和类型的映射。
// Get the mapping of service names and types.
std::map<std::string, std::vector<std::string>> Node::get_service_names_and_types() const {
  return node_graph_->get_service_names_and_types();
}

// 根据节点名称和命名空间获取服务名称和类型的映射。
// Get the mapping of service names and types by node name and namespace.
std::map<std::string, std::vector<std::string>> Node::get_service_names_and_types_by_node(
    const std::string& node_name, const std::string& namespace_) const {
  return node_graph_->get_service_names_and_types_by_node(node_name, namespace_);
}

// 计算给定主题名称的发布者数量。
// Count the number of publishers for the given topic name.
size_t Node::count_publishers(const std::string& topic_name) const {
  return node_graph_->count_publishers(topic_name);
}

// 计算给定主题名称的订阅者数量。
// Count the number of subscribers for the given topic name.
size_t Node::count_subscribers(const std::string& topic_name) const {
  return node_graph_->count_subscribers(topic_name);
}

// 获取指定主题的发布者信息列表。
// Get the list of publisher information for the specified topic.
std::vector<rclcpp::TopicEndpointInfo> Node::get_publishers_info_by_topic(
    const std::string& topic_name, bool no_mangle) const {
  return node_graph_->get_publishers_info_by_topic(topic_name, no_mangle);
}

/**
 * @brief 获取订阅特定主题的节点信息 (Get the information of nodes that subscribe to a specific
 * topic)
 * @param topic_name 主题名称 (Topic name)
 * @param no_mangle 是否对主题名称进行修饰 (Whether to mangle the topic name or not)
 * @return 包含订阅特定主题节点信息的向量 (A vector containing the information of nodes that
 * subscribe to the specific topic)
 */
std::vector<rclcpp::TopicEndpointInfo> Node::get_subscriptions_info_by_topic(
    const std::string& topic_name, bool no_mangle) const {
  // 调用 node_graph_ 成员函数获取订阅特定主题的节点信息
  // Call the member function of node_graph_ to get the information of nodes that subscribe to a
  // specific topic
  return node_graph_->get_subscriptions_info_by_topic(topic_name, no_mangle);
}

/**
 * @brief 遍历回调组并执行指定函数 (Traverse callback groups and execute the specified function)
 * @param func 要执行的函数 (The function to be executed)
 */
void Node::for_each_callback_group(
    const node_interfaces::NodeBaseInterface::CallbackGroupFunction& func) {
  // 调用 node_base_ 成员函数遍历回调组并执行指定函数
  // Call the member function of node_base_ to traverse callback groups and execute the specified
  // function
  node_base_->for_each_callback_group(func);
}

/**
 * @brief 获取图形事件 (Get the graph event)
 * @return 图形事件共享指针 (Shared pointer of the graph event)
 */
rclcpp::Event::SharedPtr Node::get_graph_event() { return node_graph_->get_graph_event(); }

/**
 * @brief 等待图形更改 (Wait for graph change)
 * @param event 图形事件共享指针 (Shared pointer of the graph event)
 * @param timeout 超时时间 (Timeout duration)
 */
void Node::wait_for_graph_change(rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout) {
  // 调用 node_graph_ 成员函数等待图形更改
  // Call the member function of node_graph_ to wait for graph change
  node_graph_->wait_for_graph_change(event, timeout);
}

/**
 * @brief 获取节点时钟 (Get the node clock)
 * @return 节点时钟共享指针 (Shared pointer of the node clock)
 */
rclcpp::Clock::SharedPtr Node::get_clock() { return node_clock_->get_clock(); }

/**
 * @brief 获取节点时钟 (Get the node clock) - const 版本 (const version)
 * @return 节点时钟常量共享指针 (Const shared pointer of the node clock)
 */
rclcpp::Clock::ConstSharedPtr Node::get_clock() const { return node_clock_->get_clock(); }

/**
 * @brief 获取当前时间 (Get the current time)
 * @return 当前时间 (Current time)
 */
rclcpp::Time Node::now() const { return node_clock_->get_clock()->now(); }

/**
 * @brief 获取节点基础接口 (Get the node base interface)
 * @return 节点基础接口共享指针 (Shared pointer of the node base interface)
 */
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr Node::get_node_base_interface() {
  return node_base_;
}

/**
 * @brief 获取节点时钟接口 (Get the node clock interface)
 * @return 节点时钟接口共享指针 (Shared pointer of the node clock interface)
 */
rclcpp::node_interfaces::NodeClockInterface::SharedPtr Node::get_node_clock_interface() {
  return node_clock_;
}

/**
 * @brief 获取节点图形接口 (Get the node graph interface)
 * @return 节点图形接口共享指针 (Shared pointer of the node graph interface)
 */
rclcpp::node_interfaces::NodeGraphInterface::SharedPtr Node::get_node_graph_interface() {
  return node_graph_;
}

/**
 * @brief 获取节点日志接口 (Get the node logging interface)
 * @return 节点日志接口共享指针 (Shared pointer of the node logging interface)
 */
rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr Node::get_node_logging_interface() {
  return node_logging_;
}

/**
 * @brief 获取节点时间源接口 (Get the node time source interface)
 * @return 节点时间源接口共享指针 (Shared pointer of the node time source interface)
 */
rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr Node::get_node_time_source_interface() {
  return node_time_source_;
}

/**
 * @brief 获取节点的定时器接口 (Get the node's timers interface)
 *
 * @return 返回 rclcpp::node_interfaces::NodeTimersInterface 的共享指针 (Return a shared pointer of
 * rclcpp::node_interfaces::NodeTimersInterface)
 */
rclcpp::node_interfaces::NodeTimersInterface::SharedPtr Node::get_node_timers_interface() {
  // 返回节点定时器接口 (Return the node timers interface)
  return node_timers_;
}

/**
 * @brief 获取节点的话题接口 (Get the node's topics interface)
 *
 * @return 返回 rclcpp::node_interfaces::NodeTopicsInterface 的共享指针 (Return a shared pointer of
 * rclcpp::node_interfaces::NodeTopicsInterface)
 */
rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr Node::get_node_topics_interface() {
  // 返回节点话题接口 (Return the node topics interface)
  return node_topics_;
}

/**
 * @brief 获取节点的服务接口 (Get the node's services interface)
 *
 * @return 返回 rclcpp::node_interfaces::NodeServicesInterface 的共享指针 (Return a shared pointer
 * of rclcpp::node_interfaces::NodeServicesInterface)
 */
rclcpp::node_interfaces::NodeServicesInterface::SharedPtr Node::get_node_services_interface() {
  // 返回节点服务接口 (Return the node services interface)
  return node_services_;
}

/**
 * @brief 获取节点的参数接口 (Get the node's parameters interface)
 *
 * @return 返回 rclcpp::node_interfaces::NodeParametersInterface 的共享指针 (Return a shared pointer
 * of rclcpp::node_interfaces::NodeParametersInterface)
 */
rclcpp::node_interfaces::NodeParametersInterface::SharedPtr Node::get_node_parameters_interface() {
  // 返回节点参数接口 (Return the node parameters interface)
  return node_parameters_;
}

/**
 * @brief 获取节点的可等待对象接口 (Get the node's waitables interface)
 *
 * @return 返回 rclcpp::node_interfaces::NodeWaitablesInterface 的共享指针 (Return a shared pointer
 * of rclcpp::node_interfaces::NodeWaitablesInterface)
 */
rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr Node::get_node_waitables_interface() {
  // 返回节点可等待对象接口 (Return the node waitables interface)
  return node_waitables_;
}

// 获取子命名空间 (Get the sub namespace)
const std::string& Node::get_sub_namespace() const { return this->sub_namespace_; }

// 获取有效命名空间 (Get the effective namespace)
const std::string& Node::get_effective_namespace() const { return this->effective_namespace_; }

/**
 * @brief 创建一个子节点 (Create a sub node)
 *
 * @param sub_namespace 子命名空间 (The sub namespace)
 * @return 返回创建的子节点的共享指针 (Return a shared pointer of the created sub node)
 */
Node::SharedPtr Node::create_sub_node(const std::string& sub_namespace) {
  // 无法在此处使用 make_shared<Node>()，因为它要求构造函数为公共的，
  // 而这个构造函数是有意设置为受保护的 (Cannot use make_shared<Node>() here as it requires the
  // constructor to be public, and this constructor is intentionally protected instead)
  return std::shared_ptr<Node>(new Node(*this, sub_namespace));
}

// 获取节点选项 (Get the node options)
const NodeOptions& Node::get_node_options() const { return this->node_options_; }

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

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_node_interface_impl.hpp"  // implementation
#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
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
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/qos.hpp"

namespace rclcpp_lifecycle {

/**
 * @brief 构造函数，用于创建一个生命周期节点 (Constructor for creating a LifecycleNode)
 *
 * @param node_name 节点的名称 (Name of the node)
 * @param options 节点选项，包括参数、QoS等 (NodeOptions, including parameters, QoS, etc.)
 * @param enable_communication_interface 是否启用通信接口，默认为true
 */
LifecycleNode::LifecycleNode(
    const std::string &node_name,
    const rclcpp::NodeOptions &options,
    bool enable_communication_interface)
    // 调用另一个构造函数，传递给定的参数
    : LifecycleNode(node_name, "", options, enable_communication_interface) {
  // 这里没有其他操作，因为所有工作都由另一个构造函数完成
}

/**
 * @brief 构造一个 LifecycleNode 对象 (Constructs a LifecycleNode object)
 *
 * @param node_name 节点的名称 (The name of the node)
 * @param namespace_ 节点的命名空间 (The namespace of the node)
 * @param options 节点选项 (Node options)
 * @param enable_communication_interface 是否启用通信接口 (Whether to enable the communication
 * interface)
 */
LifecycleNode::LifecycleNode(
    const std::string &node_name,
    const std::string &namespace_,
    const rclcpp::NodeOptions &options,
    bool enable_communication_interface)
    :  // 初始化列表 (Initialization list)
       // 创建 NodeBase 接口对象 (Create NodeBase interface object)
      node_base_(new rclcpp::node_interfaces::NodeBase(
          node_name,
          namespace_,
          options.context(),
          *(options.get_rcl_node_options()),
          options.use_intra_process_comms(),
          options.enable_topic_statistics())),
      // 创建 NodeGraph 接口对象 (Create NodeGraph interface object)
      node_graph_(new rclcpp::node_interfaces::NodeGraph(node_base_.get())),
      // 创建 NodeLogging 接口对象 (Create NodeLogging interface object)
      node_logging_(new rclcpp::node_interfaces::NodeLogging(node_base_.get())),
      // 创建 NodeTimers 接口对象 (Create NodeTimers interface object)
      node_timers_(new rclcpp::node_interfaces::NodeTimers(node_base_.get())),
      // 创建 NodeTopics 接口对象 (Create NodeTopics interface object)
      node_topics_(new rclcpp::node_interfaces::NodeTopics(node_base_.get(), node_timers_.get())),
      // 创建 NodeServices 接口对象 (Create NodeServices interface object)
      node_services_(new rclcpp::node_interfaces::NodeServices(node_base_.get())),
      // 创建 NodeClock 接口对象 (Create NodeClock interface object)
      node_clock_(new rclcpp::node_interfaces::NodeClock(
          node_base_,
          node_topics_,
          node_graph_,
          node_services_,
          node_logging_,
          options.clock_type())),
      // 创建 NodeParameters 接口对象 (Create NodeParameters interface object)
      node_parameters_(new rclcpp::node_interfaces::NodeParameters(
          node_base_,
          node_logging_,
          node_topics_,
          node_services_,
          node_clock_,
          options.parameter_overrides(),
          options.start_parameter_services(),
          options.start_parameter_event_publisher(),
          options.parameter_event_qos(),
          options.parameter_event_publisher_options(),
          options.allow_undeclared_parameters(),
          options.automatically_declare_parameters_from_overrides())),
      // 创建 NodeTimeSource 接口对象 (Create NodeTimeSource interface object)
      node_time_source_(new rclcpp::node_interfaces::NodeTimeSource(
          node_base_,
          node_topics_,
          node_graph_,
          node_services_,
          node_logging_,
          node_clock_,
          node_parameters_,
          options.clock_qos(),
          options.use_clock_thread())),
      // 创建 NodeWaitables 接口对象 (Create NodeWaitables interface object)
      node_waitables_(new rclcpp::node_interfaces::NodeWaitables(node_base_.get())),
      // 存储节点选项 (Store node options)
      node_options_(options),
      // 创建 LifecycleNodeInterfaceImpl 对象 (Create LifecycleNodeInterfaceImpl object)
      impl_(new LifecycleNodeInterfaceImpl(node_base_, node_services_)) {
  // 初始化实现类 (Initialize the implementation class)
  impl_->init(enable_communication_interface);

  // clang-format off
  // 注册生命周期回调函数 (Register lifecycle callback functions)
  register_on_configure(std::bind(&LifecycleNodeInterface::on_configure, this, std::placeholders::_1));
  register_on_cleanup(std::bind(&LifecycleNodeInterface::on_cleanup, this, std::placeholders::_1));
  register_on_shutdown(std::bind(&LifecycleNodeInterface::on_shutdown, this, std::placeholders::_1));
  register_on_activate(std::bind(&LifecycleNodeInterface::on_activate, this, std::placeholders::_1));
  register_on_deactivate(std::bind(&LifecycleNodeInterface::on_deactivate, this, std::placeholders::_1));
  register_on_error(std::bind(&LifecycleNodeInterface::on_error, this, std::placeholders::_1));
  // clang-format on
}

/**
 * @brief 析构函数，释放LifecycleNode对象占用的资源
 *        Destructor, releases resources occupied by LifecycleNode object
 */
LifecycleNode::~LifecycleNode() {
  // 释放子接口，以便在拆除过程中与node_base进行协商
  // Release sub-interfaces in an order that allows them to consult with node_base during tear-down
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

/**
 * @brief 获取节点名称
 * @return 节点名称字符串
 */
const char *LifecycleNode::get_name() const { return node_base_->get_name(); }

/**
 * @brief 获取节点命名空间
 * @return 节点命名空间字符串
 */
const char *LifecycleNode::get_namespace() const { return node_base_->get_namespace(); }

/**
 * @brief 获取节点日志记录器
 * @return Logger对象
 */
rclcpp::Logger LifecycleNode::get_logger() const { return node_logging_->get_logger(); }

/**
 * @brief 创建回调组
 * @param group_type 回调组类型
 * @param automatically_add_to_executor_with_node 是否自动添加到节点执行器
 * @return 创建的回调组共享指针
 */
rclcpp::CallbackGroup::SharedPtr LifecycleNode::create_callback_group(
    rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node) {
  return node_base_->create_callback_group(group_type, automatically_add_to_executor_with_node);
}

/**
 *  @brief 声明参数 (Declare a parameter)
 *  @param name 参数名称 (Parameter name)
 *  @param default_value 参数的默认值 (Default value of the parameter)
 *  @param parameter_descriptor 参数描述符 (Parameter descriptor)
 *  @param ignore_override 是否忽略覆盖 (Whether to ignore override)
 *  @return 返回声明的参数值 (Return the declared parameter value)
 */
const rclcpp::ParameterValue &LifecycleNode::declare_parameter(
    const std::string &name,
    const rclcpp::ParameterValue &default_value,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
    bool ignore_override) {
  // 调用 node_parameters_ 的 declare_parameter 方法来声明参数
  return this->node_parameters_->declare_parameter(
      name, default_value, parameter_descriptor, ignore_override);
}

/**
 *  @brief 声明参数 (Declare a parameter)
 *  @param name 参数名称 (Parameter name)
 *  @param type 参数类型 (Parameter type)
 *  @param parameter_descriptor 参数描述符 (Parameter descriptor)
 *  @param ignore_override 是否忽略覆盖 (Whether to ignore override)
 *  @return 返回声明的参数值 (Return the declared parameter value)
 */
const rclcpp::ParameterValue &LifecycleNode::declare_parameter(
    const std::string &name,
    rclcpp::ParameterType type,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
    bool ignore_override) {
  // 调用 node_parameters_ 的 declare_parameter 方法来声明参数
  // (Call the declare_parameter method of node_parameters_ to declare the parameter)
  return this->node_parameters_->declare_parameter(
      name, type, parameter_descriptor, ignore_override);
}

/**
 *  @brief 取消声明参数 (Undeclare a parameter)
 *  @param name 参数名称 (Parameter name)
 */
void LifecycleNode::undeclare_parameter(const std::string &name) {
  // 调用 node_parameters_ 的 undeclare_parameter 方法来取消声明参数
  // (Call the undeclare_parameter method of node_parameters_ to undeclare the parameter)
  this->node_parameters_->undeclare_parameter(name);
}

/**
 *  @brief 检查是否有参数 (Check if there is a parameter)
 *  @param name 参数名称 (Parameter name)
 *  @return 如果存在参数，则返回 true，否则返回 false (Return true if the parameter exists,
 * otherwise return false)
 */
bool LifecycleNode::has_parameter(const std::string &name) const {
  // 调用 node_parameters_ 的 has_parameter 方法来检查参数是否存在 (Call the has_parameter method of
  // node_parameters_ to check if the parameter exists)
  return this->node_parameters_->has_parameter(name);
}

/**
 *  @brief 设置参数 (Set a parameter)
 *  @param parameter 要设置的参数值 (The parameter value to be set)
 *  @return 返回设置参数的结果 (Return the result of setting the parameter)
 */
rcl_interfaces::msg::SetParametersResult LifecycleNode::set_parameter(
    const rclcpp::Parameter &parameter) {
  // 调用 set_parameters_atomically 方法以原子方式设置参数 (Call the set_parameters_atomically
  // method to set the parameter atomically)
  return this->set_parameters_atomically({parameter});
}

/**
 * @brief 设置一组参数
 * @param parameters 参数向量
 * @return 返回一个包含设置结果的向量
 *
 * @brief Set a group of parameters
 * @param parameters Vector of parameters
 * @return Returns a vector containing the set results
 */
std::vector<rcl_interfaces::msg::SetParametersResult> LifecycleNode::set_parameters(
    const std::vector<rclcpp::Parameter> &parameters) {
  // 调用 node_parameters_ 的 set_parameters 方法来设置参数，并返回结果
  // Call the set_parameters method of node_parameters_ to set the parameters and return the result
  return node_parameters_->set_parameters(parameters);
}

/**
 * @brief 原子地设置一组参数
 * @param parameters 参数向量
 * @return 返回设置结果
 *
 * @brief Set a group of parameters atomically
 * @param parameters Vector of parameters
 * @return Returns the set result
 */
rcl_interfaces::msg::SetParametersResult LifecycleNode::set_parameters_atomically(
    const std::vector<rclcpp::Parameter> &parameters) {
  // 调用 node_parameters_ 的 set_parameters_atomically 方法以原子方式设置参数，并返回结果
  // Call the set_parameters_atomically method of node_parameters_ to set the parameters atomically
  // and return the result
  return node_parameters_->set_parameters_atomically(parameters);
}

/**
 * @brief 获取一组参数
 * @param names 参数名向量
 * @return 返回一个包含参数的向量
 *
 * @brief Get a group of parameters
 * @param names Vector of parameter names
 * @return Returns a vector containing the parameters
 */
std::vector<rclcpp::Parameter> LifecycleNode::get_parameters(
    const std::vector<std::string> &names) const {
  // 调用 node_parameters_ 的 get_parameters 方法来获取参数，并返回结果
  // Call the get_parameters method of node_parameters_ to get the parameters and return the result
  return node_parameters_->get_parameters(names);
}

/**
 * @brief 获取单个参数
 * @param name 参数名
 * @return 返回获取到的参数
 *
 * @brief Get a single parameter
 * @param name Parameter name
 * @return Returns the obtained parameter
 */
rclcpp::Parameter LifecycleNode::get_parameter(const std::string &name) const {
  // 调用 node_parameters_ 的 get_parameter 方法来获取参数，并返回结果
  // Call the get_parameter method of node_parameters_ to get the parameter and return the result
  return node_parameters_->get_parameter(name);
}

/**
 * @brief 获取单个参数并存储在给定的参数变量中
 * @param name 参数名
 * @param parameter 用于存储获取到的参数的引用
 * @return 返回一个布尔值表示是否成功获取参数
 *
 * @brief Get a single parameter and store it in the given parameter variable
 * @param name Parameter name
 * @param parameter Reference for storing the obtained parameter
 * @return Returns a boolean indicating whether the parameter was successfully obtained
 */
bool LifecycleNode::get_parameter(const std::string &name, rclcpp::Parameter &parameter) const {
  // 调用 node_parameters_ 的 get_parameter 方法来获取参数，并将结果存储在给定的参数变量中
  // Call the get_parameter method of node_parameters_ to get the parameter and store the result in
  // the given parameter variable
  return node_parameters_->get_parameter(name, parameter);
}

/**
 * @brief 描述单个参数
 * @param name 参数名
 * @return 返回参数描述信息
 *
 * @brief Describe a single parameter
 * @param name Parameter name
 * @return Returns the parameter description information
 */
rcl_interfaces::msg::ParameterDescriptor LifecycleNode::describe_parameter(
    const std::string &name) const {
  // 调用 node_parameters_ 的 describe_parameters 方法来获取参数描述，并将结果存储在 result 变量中
  // Call the describe_parameters method of node_parameters_ to get the parameter description and
  // store the result in the result variable
  auto result = node_parameters_->describe_parameters({name});

  // 如果结果为空，抛出一个参数未声明异常
  // If the result is empty, throw a ParameterNotDeclaredException
  if (0 == result.size()) {
    throw rclcpp::exceptions::ParameterNotDeclaredException(name);
  }

  // 如果结果的数量大于1，抛出一个运行时错误
  // If the number of results is greater than 1, throw a runtime error
  if (result.size() > 1) {
    throw std::runtime_error("number of described parameters unexpectedly more than one");
  }

  // 返回第一个参数描述
  // Return the first parameter description
  return result.front();
}

/**
 * @brief 描述参数 (Describe parameters)
 *
 * @param names 参数名列表 (List of parameter names)
 * @return std::vector<rcl_interfaces::msg::ParameterDescriptor> 参数描述符向量 (Vector of parameter
 * descriptors)
 */
std::vector<rcl_interfaces::msg::ParameterDescriptor> LifecycleNode::describe_parameters(
    const std::vector<std::string> &names) const {
  // 调用 node_parameters_ 的 describe_parameters 方法并返回结果 (Call the describe_parameters
  // method of node_parameters_ and return the result)
  return node_parameters_->describe_parameters(names);
}

/**
 * @brief 获取参数类型 (Get parameter types)
 *
 * @param names 参数名列表 (List of parameter names)
 * @return std::vector<uint8_t> 参数类型向量 (Vector of parameter types)
 */
std::vector<uint8_t> LifecycleNode::get_parameter_types(
    const std::vector<std::string> &names) const {
  // 调用 node_parameters_ 的 get_parameter_types 方法并返回结果 (Call the get_parameter_types
  // method of node_parameters_ and return the result)
  return node_parameters_->get_parameter_types(names);
}

/**
 * @brief 列出参数 (List parameters)
 *
 * @param prefixes 参数前缀列表 (List of parameter prefixes)
 * @param depth 搜索深度 (Search depth)
 * @return rcl_interfaces::msg::ListParametersResult 参数列表结果 (List parameters result)
 */
rcl_interfaces::msg::ListParametersResult LifecycleNode::list_parameters(
    const std::vector<std::string> &prefixes, uint64_t depth) const {
  // 调用 node_parameters_ 的 list_parameters 方法并返回结果
  // (Call the list_parameters method of node_parameters_ and return the result)
  return node_parameters_->list_parameters(prefixes, depth);
}

/**
 * @brief 添加预设参数回调 (Add pre-set parameters callback)
 *
 * @param callback 预设参数回调类型 (Pre-set parameters callback type)
 * @return rclcpp::Node::PreSetParametersCallbackHandle::SharedPtr 预设参数回调句柄共享指针 (Shared
 * pointer of pre-set parameters callback handle)
 */
rclcpp::Node::PreSetParametersCallbackHandle::SharedPtr
LifecycleNode::add_pre_set_parameters_callback(PreSetParametersCallbackType callback) {
  // 调用 node_parameters_ 的 add_pre_set_parameters_callback 方法并返回结果 (Call the
  // add_pre_set_parameters_callback method of node_parameters_ and return the result)
  return node_parameters_->add_pre_set_parameters_callback(callback);
}

/**
 * @brief 添加设置参数回调 (Add on-set parameters callback)
 *
 * @param callback 设置参数回调类型 (On-set parameters callback type)
 * @return rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr 设置参数回调句柄共享指针 (Shared
 * pointer of on-set parameters callback handle)
 */
rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
LifecycleNode::add_on_set_parameters_callback(OnSetParametersCallbackType callback) {
  // 调用 node_parameters_ 的 add_on_set_parameters_callback 方法并返回结果 (Call the
  // add_on_set_parameters_callback method of node_parameters_ and return the result)
  return node_parameters_->add_on_set_parameters_callback(callback);
}

/**
 * @brief 添加后置参数回调 (Add post-set parameters callback)
 *
 * @param callback 后置参数回调类型 (Post-set parameters callback type)
 * @return rclcpp::Node::PostSetParametersCallbackHandle::SharedPtr 后置参数回调句柄共享指针
 */
rclcpp::Node::PostSetParametersCallbackHandle::SharedPtr
LifecycleNode::add_post_set_parameters_callback(PostSetParametersCallbackType callback) {
  // 调用 node_parameters_ 的 add_post_set_parameters_callback 方法并返回结果
  return node_parameters_->add_post_set_parameters_callback(callback);
}

/**
 * @brief 移除预设参数回调 (Remove pre-set parameters callback)
 *
 * @param callback 预设参数回调句柄指针 (Pointer to pre-set parameters callback handle)
 */
void LifecycleNode::remove_pre_set_parameters_callback(
    const PreSetParametersCallbackHandle *const callback) {
  // 调用 node_parameters_ 的 remove_pre_set_parameters_callback 方法
  // (Call the remove_pre_set_parameters_callback method of node_parameters_)
  node_parameters_->remove_pre_set_parameters_callback(callback);
}

/**
 * @brief 移除设置参数回调 (Remove on-set parameters callback)
 *
 * @param callback 设置参数回调句柄指针 (Pointer to on-set parameters callback handle)
 */
void LifecycleNode::remove_on_set_parameters_callback(
    const OnSetParametersCallbackHandle *const callback) {
  // 调用 node_parameters_ 的 remove_on_set_parameters_callback 方法 (Call the
  // remove_on_set_parameters_callback method of node_parameters_)
  node_parameters_->remove_on_set_parameters_callback(callback);
}

/**
 * @brief 删除参数设置后的回调函数 (Remove the callback function after setting parameters)
 *
 * @param[in] callback 要删除的回调函数句柄 (The handle of the callback function to be removed)
 */
void LifecycleNode::remove_post_set_parameters_callback(
    const PostSetParametersCallbackHandle *const callback) {
  // 调用 node_parameters_ 的 remove_post_set_parameters_callback 方法删除回调函数
  // (Call the remove_post_set_parameters_callback method of node_parameters_ to remove the callback
  // function)
  node_parameters_->remove_post_set_parameters_callback(callback);
}

/**
 * @brief 获取节点名称列表 (Get a list of node names)
 *
 * @return std::vector<std::string> 节点名称列表 (List of node names)
 */
std::vector<std::string> LifecycleNode::get_node_names() const {
  // 调用 node_graph_ 的 get_node_names 方法获取节点名称列表
  // (Call the get_node_names method of node_graph_ to get the list of node names)
  return node_graph_->get_node_names();
}

/**
 * @brief 获取话题名称和类型 (Get topic names and types)
 * @param[in] no_demangle 是否对 C++ 类型进行解析 (Whether to parse C++ types)
 * @return std::map<std::string, std::vector<std::string>> 话题名称和类型映射 (Mapping of topic
 * names and types)
 */
std::map<std::string, std::vector<std::string>> LifecycleNode::get_topic_names_and_types(
    bool no_demangle) const {
  // 调用 node_graph_ 的 get_topic_names_and_types 方法获取话题名称和类型映射
  // (Call the get_topic_names_and_types method of node_graph_ to get the mapping of topic names and
  // types)
  return node_graph_->get_topic_names_and_types(no_demangle);
}

/**
 * @brief 获取服务名称和类型 (Get service names and types)
 * @return std::map<std::string, std::vector<std::string>> 服务名称和类型映射 (Mapping of service
 * names and types)
 */
std::map<std::string, std::vector<std::string>> LifecycleNode::get_service_names_and_types() const {
  // 调用 node_graph_ 的 get_service_names_and_types 方法获取服务名称和类型映射
  return node_graph_->get_service_names_and_types();
}

/**
 * @brief 根据节点名称和命名空间获取服务名称和类型
 *
 * @param[in] node_name 节点名称 (Node name)
 * @param[in] namespace_ 命名空间 (Namespace)
 * @return std::map<std::string, std::vector<std::string>> 服务名称和类型映射 (Mapping of service
 * names and types)
 */
std::map<std::string, std::vector<std::string>> LifecycleNode::get_service_names_and_types_by_node(
    const std::string &node_name, const std::string &namespace_) const {
  // 调用 node_graph_ 的 get_service_names_and_types_by_node 方法获取服务名称和类型映射
  return node_graph_->get_service_names_and_types_by_node(node_name, namespace_);
}

/**
 * @brief 统计发布者数量 (Count the number of publishers)
 *
 * @param[in] topic_name 话题名称 (Topic name)
 * @return size_t 发布者数量 (Number of publishers)
 */
size_t LifecycleNode::count_publishers(const std::string &topic_name) const {
  // 调用 node_graph_ 的 count_publishers 方法统计发布者数量
  // (Call the count_publishers method of node_graph_ to count the number of publishers)
  return node_graph_->count_publishers(topic_name);
}

/**
 * @brief 统计订阅者数量 (Count the number of subscribers)
 *
 * @param[in] topic_name 话题名称 (Topic name)
 * @return size_t 订阅者数量 (Number of subscribers)
 */
size_t LifecycleNode::count_subscribers(const std::string &topic_name) const {
  // 调用 node_graph_ 的 count_subscribers 方法统计订阅者数量
  return node_graph_->count_subscribers(topic_name);
}

/**
 * @brief 根据话题获取发布者信息 (Get publisher information by topic)
 *
 * @param[in] topic_name 话题名称 (Topic name)
 * @param[in] no_mangle 是否对 C++ 类型进行解析 (Whether to parse C++ types)
 * @return std::vector<rclcpp::TopicEndpointInfo> 发布者信息列表 (List of publisher information)
 */
std::vector<rclcpp::TopicEndpointInfo> LifecycleNode::get_publishers_info_by_topic(
    const std::string &topic_name, bool no_mangle) const {
  // 调用 node_graph_ 的 get_publishers_info_by_topic 方法获取发布者信息列表
  return node_graph_->get_publishers_info_by_topic(topic_name, no_mangle);
}

/**
 * @brief 根据话题获取订阅者信息 (Get subscriber information by topic)
 *
 * @param[in] topic_name 话题名称 (Topic name)
 * @param[in] no_mangle 是否对 C++ 类型进行解析 (Whether to parse C++ types)
 * @return std::vector<rclcpp::TopicEndpointInfo> 订阅者信息列表 (List of subscriber information)
 */
std::vector<rclcpp::TopicEndpointInfo> LifecycleNode::get_subscriptions_info_by_topic(
    const std::string &topic_name, bool no_mangle) const {
  // 调用 node_graph_ 的 get_subscriptions_info_by_topic 方法获取订阅者信息列表
  return node_graph_->get_subscriptions_info_by_topic(topic_name, no_mangle);
}

/**
 * @brief 对每个回调组执行给定的函数
 * @param func 要对每个回调组执行的函数
 *
 * @details This function applies the given function to each callback group.
 * @param func The function to be applied to each callback group.
 */
void LifecycleNode::for_each_callback_group(
    const rclcpp::node_interfaces::NodeBaseInterface::CallbackGroupFunction &func) {
  // 调用 node_base_ 的 for_each_callback_group 方法，将 func 应用于每个回调组
  // Call the for_each_callback_group method of node_base_, applying func to each callback group
  node_base_->for_each_callback_group(func);
}

/**
 * @brief 获取图形事件
 * @return 返回图形事件共享指针
 *
 * @details Get the graph event.
 * @return Returns a shared pointer to the graph event.
 */
rclcpp::Event::SharedPtr LifecycleNode::get_graph_event() { return node_graph_->get_graph_event(); }

/**
 * @brief 等待图形更改
 * @param event 图形事件共享指针
 * @param timeout 等待超时时间
 *
 * @details Wait for a graph change.
 * @param event A shared pointer to the graph event.
 * @param timeout The wait timeout duration.
 */
void LifecycleNode::wait_for_graph_change(
    rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout) {
  // 使用给定的事件和超时时间等待图形更改
  // Wait for a graph change with the given event and timeout duration
  node_graph_->wait_for_graph_change(event, timeout);
}

/**
 * @brief 获取时钟
 * @return 返回时钟共享指针
 *
 * @details Get the clock.
 * @return Returns a shared pointer to the clock.
 */
rclcpp::Clock::SharedPtr LifecycleNode::get_clock() { return node_clock_->get_clock(); }

/**
 * @brief 获取时钟（const版本）
 * @return 返回时钟常量共享指针
 *
 * @details Get the clock (const version).
 * @return Returns a const shared pointer to the clock.
 */
rclcpp::Clock::ConstSharedPtr LifecycleNode::get_clock() const { return node_clock_->get_clock(); }

/**
 * @brief 获取当前时间
 * @return 返回当前时间
 *
 * @details Get the current time.
 * @return Returns the current time.
 */
rclcpp::Time LifecycleNode::now() const { return node_clock_->get_clock()->now(); }

// 后续的函数都是获取不同接口的共享指针，注释类似，这里仅给出中文注释

/**
 * @brief 获取节点基本接口
 * @return 返回节点基本接口共享指针
 */
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr LifecycleNode::get_node_base_interface() {
  return node_base_;
}

/**
 * @brief 获取节点时钟接口
 * @return 返回节点时钟接口共享指针
 */
rclcpp::node_interfaces::NodeClockInterface::SharedPtr LifecycleNode::get_node_clock_interface() {
  return node_clock_;
}

/**
 * @brief 获取节点图形接口
 * @return 返回节点图形接口共享指针
 */
rclcpp::node_interfaces::NodeGraphInterface::SharedPtr LifecycleNode::get_node_graph_interface() {
  return node_graph_;
}

/**
 * @brief 获取节点日志接口
 * @return 返回节点日志接口共享指针
 */
rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
LifecycleNode::get_node_logging_interface() {
  return node_logging_;
}

/**
 * @brief 获取节点时间源接口
 * @return 返回节点时间源接口共享指针
 */
rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr
LifecycleNode::get_node_time_source_interface() {
  return node_time_source_;
}

/**
 * @brief 获取节点定时器接口
 * @return 返回节点定时器接口共享指针
 */
rclcpp::node_interfaces::NodeTimersInterface::SharedPtr LifecycleNode::get_node_timers_interface() {
  return node_timers_;
}

/**
 * @brief 获取节点的话题接口
 * @return 返回节点话题接口的共享指针
 *
 * @brief Get the node's topics interface
 * @return Returns a shared pointer to the node's topics interface
 */
rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr LifecycleNode::get_node_topics_interface() {
  // 返回节点话题接口的共享指针
  // Return a shared pointer to the node's topics interface
  return node_topics_;
}

/**
 * @brief 获取节点的服务接口
 * @return 返回节点服务接口的共享指针
 *
 * @brief Get the node's services interface
 * @return Returns a shared pointer to the node's services interface
 */
rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
LifecycleNode::get_node_services_interface() {
  // 返回节点服务接口的共享指针
  // Return a shared pointer to the node's services interface
  return node_services_;
}

/**
 * @brief 获取节点的参数接口
 * @return 返回节点参数接口的共享指针
 *
 * @brief Get the node's parameters interface
 * @return Returns a shared pointer to the node's parameters interface
 */
rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
LifecycleNode::get_node_parameters_interface() {
  // 返回节点参数接口的共享指针
  // Return a shared pointer to the node's parameters interface
  return node_parameters_;
}

/**
 * @brief 获取节点的等待接口
 * @return 返回节点等待接口的共享指针
 *
 * @brief Get the node's waitables interface
 * @return Returns a shared pointer to the node's waitables interface
 */
rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
LifecycleNode::get_node_waitables_interface() {
  // 返回节点等待接口的共享指针
  // Return a shared pointer to the node's waitables interface
  return node_waitables_;
}

/**
 * @brief 获取节点选项
 * @return 返回节点选项的引用
 *
 * @brief Get the node options
 * @return Returns a reference to the node options
 */
const rclcpp::NodeOptions &LifecycleNode::get_node_options() const { return node_options_; }

/* ========= =========
========= ========= */

/**
 * @brief 注册配置回调函数
 * @param fcn 配置回调函数
 * @return 返回注册结果，成功为 true，失败为 false
 *
 * @brief Register the configure callback function
 * @param fcn The configure callback function
 * @return Returns the registration result, true for success and false for failure
 */
bool LifecycleNode::register_on_configure(
    std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn) {
  // 注册配置回调函数
  // Register the configure callback function
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING, fcn);
}

/**
 * @brief 注册清理回调函数
 * @param fcn 清理回调函数
 * @return 返回注册结果，成功为 true，失败为 false
 *
 * @brief Register the cleanup callback function
 * @param fcn The cleanup callback function
 * @return Returns the registration result, true for success and false for failure
 */
bool LifecycleNode::register_on_cleanup(
    std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn) {
  // 注册清理回调函数
  // Register the cleanup callback function
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP, fcn);
}

/**
 * @brief 注册关闭回调函数
 * @param fcn 关闭回调函数
 * @return 返回注册结果，成功为 true，失败为 false
 *
 * @brief Register the shutdown callback function
 * @param fcn The shutdown callback function
 * @return Returns the registration result, true for success and false for failure
 */
bool LifecycleNode::register_on_shutdown(
    std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn) {
  // 注册关闭回调函数
  // Register the shutdown callback function
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN, fcn);
}

/**
 * @brief 注册激活回调函数
 * @param fcn 激活回调函数
 * @return 返回注册结果，成功为 true，失败为 false
 *
 * @brief Register the activate callback function
 * @param fcn The activate callback function
 * @return Returns the registration result, true for success and false for failure
 */
bool LifecycleNode::register_on_activate(
    std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn) {
  // 注册激活回调函数
  // Register the activate callback function
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING, fcn);
}

/**
 * @brief 注册在节点停用时要执行的回调函数 (Register a callback function to be executed when the
 * node is deactivated)
 *
 * @param fcn 要注册的回调函数 (The callback function to register)
 * @return 如果注册成功，返回true；否则返回false (Returns true if the registration is successful,
 * false otherwise)
 */
bool LifecycleNode::register_on_deactivate(
    std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn) {
  // 在停用状态时注册回调函数 (Register the callback function for the deactivating state)
  return impl_->register_callback(lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING, fcn);
}

/**
 * @brief 注册在节点出错时要执行的回调函数 (Register a callback function to be executed when the
 * node encounters an error)
 *
 * @param fcn 要注册的回调函数 (The callback function to register)
 * @return 如果注册成功，返回true；否则返回false (Returns true if the registration is successful,
 * false otherwise)
 */
bool LifecycleNode::register_on_error(
    std::function<LifecycleNodeInterface::CallbackReturn(const State &)> fcn) {
  // 在错误处理状态时注册回调函数 (Register the callback function for the error processing state)
  return impl_->register_callback(
      lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING, fcn);
}

/* ========= =========
========= ========= */

/**
 * @brief 获取当前节点的状态 (Get the current state of the node)
 * @return 当前节点的状态引用 (A reference to the current state of the node)
 */
const State &LifecycleNode::get_current_state() const { return impl_->get_current_state(); }

/**
 * @brief 获取可用状态列表 (Get the list of available states)
 *
 * @return 可用状态的向量 (A vector of available states)
 */
std::vector<State> LifecycleNode::get_available_states() const {
  return impl_->get_available_states();
}

/**
 * @brief 获取可用转换列表 (Get the list of available transitions)
 * @return 可用转换的向量 (A vector of available transitions)
 */
std::vector<Transition> LifecycleNode::get_available_transitions() const {
  return impl_->get_available_transitions();
}

/**
 * @brief 获取转换图 (Get the transition graph)
 * @return 转换图的向量 (A vector representing the transition graph)
 */
std::vector<Transition> LifecycleNode::get_transition_graph() const {
  return impl_->get_transition_graph();
}

/* ========= =========
========= ========= */

/**
 * @brief 触发给定转换 (Trigger the given transition)
 * @param transition 要触发的转换 (The transition to trigger)
 * @return 触发转换后的节点状态引用 (A reference to the node state after triggering the transition)
 */
const State &LifecycleNode::trigger_transition(const Transition &transition) {
  return trigger_transition(transition.id());
}

/**
 * @brief 触发给定转换，并获取回调函数的返回代码
 *    (Trigger the given transition and get the return code of the callback function)
 * @param transition 要触发的转换 (The transition to trigger)
 * @param cb_return_code 回调函数的返回代码 (The return code of the callback function)
 * @return 触发转换后的节点状态引用 (A reference to the node state after triggering the transition)
 */
const State &LifecycleNode::trigger_transition(
    const Transition &transition, LifecycleNodeInterface::CallbackReturn &cb_return_code) {
  return trigger_transition(transition.id(), cb_return_code);
}

/**
 * @brief 使用转换ID触发转换 (Trigger a transition using the transition ID)
 *
 * @param transition_id 要触发的转换ID (The transition ID to trigger)
 * @return 触发转换后的节点状态引用 (A reference to the node state after triggering the transition)
 */
const State &LifecycleNode::trigger_transition(uint8_t transition_id) {
  return impl_->trigger_transition(transition_id);
}

/**
 * @brief 触发生命周期节点的状态转换 (Trigger a state transition in the lifecycle node)
 *
 * @param transition_id 转换ID (Transition ID)
 * @param cb_return_code 回调返回码 (Callback return code)
 * @return State& 当前状态的引用 (Reference to the current state)
 */
const State &LifecycleNode::trigger_transition(
    uint8_t transition_id, LifecycleNodeInterface::CallbackReturn &cb_return_code) {
  // 调用内部实现来触发转换 (Invoke internal implementation to trigger the transition)
  return impl_->trigger_transition(transition_id, cb_return_code);
}

/**
 * @brief 配置生命周期节点 (Configure the lifecycle node)
 *
 * @return State& 当前状态的引用 (Reference to the current state)
 */
const State &LifecycleNode::configure() {
  // 触发配置转换 (Trigger the configure transition)
  return impl_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

/**
 * @brief 配置生命周期节点并返回回调代码 (Configure the lifecycle node and return callback code)
 *
 * @param cb_return_code 回调返回码 (Callback return code)
 * @return State& 当前状态的引用 (Reference to the current state)
 */
const State &LifecycleNode::configure(LifecycleNodeInterface::CallbackReturn &cb_return_code) {
  // 触发配置转换并返回回调代码 (Trigger the configure transition and return callback code)
  return impl_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, cb_return_code);
}

/**
 * @brief 清理生命周期节点 (Clean up the lifecycle node)
 *
 * @return State& 当前状态的引用 (Reference to the current state)
 */
const State &LifecycleNode::cleanup() {
  // 触发清理转换 (Trigger the cleanup transition)
  return impl_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
}

/**
 * @brief 清理生命周期节点并返回回调代码 (Clean up the lifecycle node and return callback code)
 *
 * @param cb_return_code 回调返回码 (Callback return code)
 * @return State& 当前状态的引用 (Reference to the current state)
 */
const State &LifecycleNode::cleanup(LifecycleNodeInterface::CallbackReturn &cb_return_code) {
  // 触发清理转换并返回回调代码 (Trigger the cleanup transition and return callback code)
  return impl_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, cb_return_code);
}

/**
 * @brief 激活生命周期节点 (Activate the lifecycle node)
 *
 * @return State& 当前状态的引用 (Reference to the current state)
 */
const State &LifecycleNode::activate() {
  // 触发激活转换 (Trigger the activate transition)
  return impl_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

/**
 * @brief 激活生命周期节点并返回回调代码 (Activate the lifecycle node and return callback code)
 *
 * @param cb_return_code 回调返回码 (Callback return code)
 * @return State& 当前状态的引用 (Reference to the current state)
 */
const State &LifecycleNode::activate(LifecycleNodeInterface::CallbackReturn &cb_return_code) {
  // 触发激活转换并返回回调代码 (Trigger the activate transition and return callback code)
  return impl_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, cb_return_code);
}

/**
 * @brief 去激活生命周期节点 (Deactivate the lifecycle node)
 *
 * @return State& 当前状态的引用 (Reference to the current state)
 */
const State &LifecycleNode::deactivate() {
  // 触发去激活转换 (Trigger the deactivate transition)
  return impl_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

/**
 * @brief 激活生命周期节点的状态 (Activate the state of the lifecycle node)
 * @param cb_return_code 回调返回代码 (Callback return code)
 * @return State& 当前节点的状态引用 (Reference to the current node's state)
 */
const State &LifecycleNode::deactivate(LifecycleNodeInterface::CallbackReturn &cb_return_code) {
  // 触发 TRANSITION_DEACTIVATE 转换，并返回当前节点的状态 (Trigger the TRANSITION_DEACTIVATE
  // transition and return the current node's state)
  return impl_->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, cb_return_code);
}

/**
 * @brief 关闭生命周期节点 (Shutdown the lifecycle node)
 * @return State& 当前节点的状态引用 (Reference to the current node's state)
 */
const State &LifecycleNode::shutdown() {
  // 触发 rcl_lifecycle_shutdown_label 转换 (Trigger the rcl_lifecycle_shutdown_label transition)
  return impl_->trigger_transition(rcl_lifecycle_shutdown_label);
}

/**
 * @brief 关闭生命周期节点，并返回回调代码 (Shutdown the lifecycle node and return callback code)
 * @param cb_return_code 回调返回代码 (Callback return code)
 * @return State& 当前节点的状态引用 (Reference to the current node's state)
 */
const State &LifecycleNode::shutdown(LifecycleNodeInterface::CallbackReturn &cb_return_code) {
  // 触发 rcl_lifecycle_shutdown_label 转换，并返回当前节点的状态 (Trigger the
  // rcl_lifecycle_shutdown_label transition and return the current node's state)
  return impl_->trigger_transition(rcl_lifecycle_shutdown_label, cb_return_code);
}

/**
 * @brief 在激活状态下执行的操作 (Operations to be performed in the activate state)
 * @param state 当前状态 (Current state)
 * @return node_interfaces::LifecycleNodeInterface::CallbackReturn 返回成功状态 (Return success
 * status)
 */
node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleNode::on_activate(const State &) {
  // 执行激活操作 (Perform activation operation)
  impl_->on_activate();
  // 返回成功状态 (Return success status)
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief 在停用状态下执行的操作 (Operations to be performed in the deactivate state)
 * @param state 当前状态 (Current state)
 * @return node_interfaces::LifecycleNodeInterface::CallbackReturn 返回成功状态 (Return success
 * status)
 */
node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleNode::on_deactivate(
    const State &) {
  // 执行停用操作 (Perform deactivation operation)
  impl_->on_deactivate();
  // 返回成功状态 (Return success status)
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/* ========= =========
========= ========= */

/**
 * @brief 添加托管实体 (Add managed entity)
 * @param managed_entity 要添加的托管实体 (The managed entity to be added)
 */
void LifecycleNode::add_managed_entity(
    std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface> managed_entity) {
  // 将托管实体添加到内部实现 (Add the managed entity to the internal implementation)
  impl_->add_managed_entity(managed_entity);
}

/**
 * @brief 添加定时器句柄 (Add timer handle)
 * @param timer 要添加的定时器 (The timer to be added)
 */
void LifecycleNode::add_timer_handle(std::shared_ptr<rclcpp::TimerBase> timer) {
  // 将定时器添加到内部实现 (Add the timer to the internal implementation)
  impl_->add_timer_handle(timer);
}

}  // namespace rclcpp_lifecycle

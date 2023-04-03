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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_NODE_IMPL_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_NODE_IMPL_HPP_

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/create_client.hpp"
#include "rclcpp/create_generic_publisher.hpp"
#include "rclcpp/create_generic_subscription.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_service.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/visibility_control.h"
#include "rmw/types.h"

namespace rclcpp_lifecycle {

/**
 * @brief 创建一个生命周期发布器 (Create a LifecyclePublisher)
 *
 * @tparam MessageT 消息类型 (Message type)
 * @tparam AllocatorT 分配器类型 (Allocator type)
 * @param topic_name 主题名称 (Topic name)
 * @param qos 服务质量配置 (Quality of Service configuration)
 * @param options 发布器选项 (Publisher options)
 * @return std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
 * 生命周期发布器的共享指针 (Shared pointer to the created LifecyclePublisher)
 */
template <typename MessageT, typename AllocatorT>
std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
LifecycleNode::create_publisher(
    const std::string& topic_name,
    const rclcpp::QoS& qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options) {
  // 使用指定的消息类型和分配器类型定义发布器类型 (Define publisher type using specified message and
  // allocator types)
  using PublisherT = rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>;

  // 创建一个生命周期发布器 (Create a LifecyclePublisher)
  auto pub =
      rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(*this, topic_name, qos, options);

  // 将创建的发布器添加到受管理实体列表中 (Add the created publisher to the list of managed
  // entities)
  this->add_managed_entity(pub);

  // 返回创建的生命周期发布器的共享指针 (Return shared pointer to the created LifecyclePublisher)
  return pub;
}

// TODO(karsten1987): Create LifecycleSubscriber

/**
 * @brief 创建一个订阅者 (Create a subscriber)
 *
 * @tparam MessageT 消息类型 (Message type)
 * @tparam CallbackT 回调函数类型 (Callback function type)
 * @tparam AllocatorT 分配器类型 (Allocator type)
 * @tparam SubscriptionT 订阅者类型 (Subscription type)
 * @tparam MessageMemoryStrategyT 消息内存策略类型 (Message memory strategy type)
 * @param topic_name 主题名称 (Topic name)
 * @param qos 服务质量配置 (Quality of Service configuration)
 * @param callback 收到消息时的回调函数 (Callback function when a message is received)
 * @param options 订阅选项 (Subscription options)
 * @param msg_mem_strat 消息内存策略共享指针 (Shared pointer to the message memory strategy)
 * @return std::shared_ptr<SubscriptionT> 创建的订阅者的共享指针 (Shared pointer to the created
 * subscriber)
 */
template <
    typename MessageT,
    typename CallbackT,
    typename AllocatorT,
    typename SubscriptionT,
    typename MessageMemoryStrategyT>
std::shared_ptr<SubscriptionT> LifecycleNode::create_subscription(
    const std::string& topic_name,
    const rclcpp::QoS& qos,
    CallbackT&& callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options,
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strat) {
  // 使用给定参数创建订阅者 (Create a subscriber using given arguments)
  return rclcpp::create_subscription<MessageT>(
      *this, topic_name, qos, std::forward<CallbackT>(callback), options, msg_mem_strat);
}

/**
 * @brief 创建一个墙时钟定时器 (Create a wall clock timer)
 *
 * @tparam DurationRepT 时间表示类型 (Time representation type)
 * @tparam DurationT 时间单位类型 (Time unit type)
 * @tparam CallbackT 回调函数类型 (Callback function type)
 * @param period 定时器周期 (Timer period)
 * @param callback 定时器回调函数 (Timer callback function)
 * @param group 回调组 (Callback group)
 * @return typename rclcpp::WallTimer<CallbackT>::SharedPtr 返回创建的墙时钟定时器智能指针 (Return
 * the created wall clock timer shared pointer)
 */
template <typename DurationRepT, typename DurationT, typename CallbackT>
typename rclcpp::WallTimer<CallbackT>::SharedPtr LifecycleNode::create_wall_timer(
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 调用 rclcpp 创建墙时钟定时器，并返回该定时器的智能指针
  // Call rclcpp to create a wall clock timer and return its smart pointer
  return rclcpp::create_wall_timer(
      period, std::move(callback), group, this->node_base_.get(), this->node_timers_.get());
}

/**
 * @brief 创建一个通用定时器 (Create a generic timer)
 *
 * @tparam DurationRepT 时间表示类型 (Time representation type)
 * @tparam DurationT 时间单位类型 (Time unit type)
 * @tparam CallbackT 回调函数类型 (Callback function type)
 * @param period 定时器周期 (Timer period)
 * @param callback 定时器回调函数 (Timer callback function)
 * @param group 回调组 (Callback group)
 * @return typename rclcpp::GenericTimer<CallbackT>::SharedPtr 返回创建的通用定时器智能指针 (Return
 * the created generic timer shared pointer)
 */
template <typename DurationRepT, typename DurationT, typename CallbackT>
typename rclcpp::GenericTimer<CallbackT>::SharedPtr LifecycleNode::create_timer(
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 调用 rclcpp 创建通用定时器，并返回该定时器的智能指针
  // Call rclcpp to create a generic timer and return its smart pointer
  return rclcpp::create_timer(
      this->get_clock(), period, std::move(callback), group, this->node_base_.get(),
      this->node_timers_.get());
}

/**
 * @brief 创建一个服务客户端 (Create a service client)
 *
 * @tparam ServiceT 服务类型 (Service type)
 * @param service_name 服务名称 (Service name)
 * @param qos_profile QoS 配置文件 (QoS profile)
 * @param group 回调组 (Callback group)
 * @return typename rclcpp::Client<ServiceT>::SharedPtr 返回创建的服务客户端智能指针 (Return the
 * created service client shared pointer)
 */
template <typename ServiceT>
typename rclcpp::Client<ServiceT>::SharedPtr LifecycleNode::create_client(
    const std::string& service_name,
    const rmw_qos_profile_t& qos_profile,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 调用 rclcpp 创建服务客户端，并返回该客户端的智能指针
  // Call rclcpp to create a service client and return its smart pointer
  return rclcpp::create_client<ServiceT>(
      node_base_, node_graph_, node_services_, service_name, qos_profile, group);
}

/**
 * @brief 创建一个服务客户端 (Create a service client)
 *
 * @tparam ServiceT 服务类型 (Service type)
 * @param service_name 服务名称 (Service name)
 * @param qos QoS 配置 (QoS configuration)
 * @param group 回调组 (Callback group)
 * @return typename rclcpp::Client<ServiceT>::SharedPtr 返回创建的服务客户端智能指针 (Return the
 * created service client shared pointer)
 */
template <typename ServiceT>
typename rclcpp::Client<ServiceT>::SharedPtr LifecycleNode::create_client(
    const std::string& service_name,
    const rclcpp::QoS& qos,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 调用 rclcpp 创建服务客户端，并返回该客户端的智能指针
  // Call rclcpp to create a service client and return its smart pointer
  return rclcpp::create_client<ServiceT>(
      node_base_, node_graph_, node_services_, service_name, qos, group);
}

/**
 * @brief 创建一个服务 (Create a service)
 *
 * @tparam ServiceT 服务类型 (Service type)
 * @tparam CallbackT 回调函数类型 (Callback function type)
 * @param[in] service_name 服务名称 (Service name)
 * @param[in] callback 服务回调函数 (Service callback function)
 * @param[in] qos_profile QoS配置文件 (QoS profile)
 * @param[in] group 回调组 (Callback group)
 * @return rclcpp::Service<ServiceT>::SharedPtr 创建的服务共享指针 (Shared pointer of the created
 * service)
 */
template <typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr LifecycleNode::create_service(
    const std::string& service_name,
    CallbackT&& callback,
    const rmw_qos_profile_t& qos_profile,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 调用rclcpp::create_service创建服务并返回 (Call rclcpp::create_service to create a service and
  // return it)
  return rclcpp::create_service<ServiceT, CallbackT>(
      node_base_, node_services_, service_name, std::forward<CallbackT>(callback), qos_profile,
      group);
}

/**
 * @brief 创建一个服务 (Create a service)
 *
 * @tparam ServiceT 服务类型 (Service type)
 * @tparam CallbackT 回调函数类型 (Callback function type)
 * @param[in] service_name 服务名称 (Service name)
 * @param[in] callback 服务回调函数 (Service callback function)
 * @param[in] qos QoS设置 (QoS settings)
 * @param[in] group 回调组 (Callback group)
 * @return rclcpp::Service<ServiceT>::SharedPtr 创建的服务共享指针 (Shared pointer of the created
 * service)
 */
template <typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr LifecycleNode::create_service(
    const std::string& service_name,
    CallbackT&& callback,
    const rclcpp::QoS& qos,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 调用rclcpp::create_service创建服务并返回 (Call rclcpp::create_service to create a service and
  // return it)
  return rclcpp::create_service<ServiceT, CallbackT>(
      node_base_, node_services_, service_name, std::forward<CallbackT>(callback), qos, group);
}

/**
 * @brief 创建一个通用发布器 (Create a generic publisher)
 *
 * @tparam AllocatorT 分配器类型 (Allocator type)
 * @param[in] topic_name 主题名称 (Topic name)
 * @param[in] topic_type 主题类型 (Topic type)
 * @param[in] qos QoS设置 (QoS settings)
 * @param[in] options 发布器选项 (Publisher options)
 * @return std::shared_ptr<rclcpp::GenericPublisher> 创建的通用发布器共享指针 (Shared pointer of the
 * created generic publisher)
 */
template <typename AllocatorT>
std::shared_ptr<rclcpp::GenericPublisher> LifecycleNode::create_generic_publisher(
    const std::string& topic_name,
    const std::string& topic_type,
    const rclcpp::QoS& qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options) {
  // 调用rclcpp::create_generic_publisher创建通用发布器并返回 (Call rclcpp::create_generic_publisher
  // to create a generic publisher and return it)
  return rclcpp::create_generic_publisher(
      node_topics_,
      // TODO(karsten1987): LifecycleNode目前不支持子命名空间 (LifecycleNode is currently not
      // supporting subnamespaces) 参见 https://github.com/ros2/rclcpp/issues/1614 (see
      // https://github.com/ros2/rclcpp/issues/1614)
      topic_name, topic_type, qos, options);
}

/**
 * @brief 创建一个通用订阅器 (Create a generic subscription)
 *
 * @tparam AllocatorT 分配器类型 (Allocator type)
 * @param[in] topic_name 主题名称 (Topic name)
 * @param[in] topic_type 主题类型 (Topic type)
 * @param[in] qos QoS设置 (QoS settings)
 * @param[in] callback 订阅回调函数 (Subscription callback function)
 * @param[in] options 订阅选项 (Subscription options)
 * @return std::shared_ptr<rclcpp::GenericSubscription> 创建的通用订阅器共享指针 (Shared pointer of
 * the created generic subscription)
 */
template <typename AllocatorT>
std::shared_ptr<rclcpp::GenericSubscription> LifecycleNode::create_generic_subscription(
    const std::string& topic_name,
    const std::string& topic_type,
    const rclcpp::QoS& qos,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options) {
  // 调用rclcpp::create_generic_subscription创建通用订阅器并返回 (Call
  // rclcpp::create_generic_subscription to create a generic subscription and return it)
  return rclcpp::create_generic_subscription(
      node_topics_,
      // TODO(karsten1987): LifecycleNode目前不支持子命名空间 (LifecycleNode is currently not
      // supporting subnamespaces) 参见 https://github.com/ros2/rclcpp/issues/1614 (see
      // https://github.com/ros2/rclcpp/issues/1614)
      topic_name, topic_type, qos, std::move(callback), options);
}

/**
 * @brief 声明一个参数 (Declare a parameter)
 *
 * @tparam ParameterT 参数类型 (Parameter type)
 * @param name 参数名称 (Parameter name)
 * @param default_value 参数默认值 (Default value for the parameter)
 * @param parameter_descriptor 参数描述符 (Parameter descriptor)
 * @param ignore_override 是否忽略覆盖 (Whether to ignore override)
 * @return 返回声明的参数值 (Return the declared parameter value)
 */
template <typename ParameterT>
auto LifecycleNode::declare_parameter(
    const std::string& name,
    const ParameterT& default_value,
    const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor,
    bool ignore_override) {
  // 使用 declare_parameter 函数声明参数并获取参数值 (Declare parameter using declare_parameter
  // function and get the parameter value)
  return this
      ->declare_parameter(
          name, rclcpp::ParameterValue(default_value), parameter_descriptor, ignore_override)
      .get<ParameterT>();
}

/**
 * @brief 声明一个参数，不提供默认值 (Declare a parameter without providing a default value)
 *
 * @tparam ParameterT 参数类型 (Parameter type)
 * @param name 参数名称 (Parameter name)
 * @param parameter_descriptor 参数描述符 (Parameter descriptor)
 * @param ignore_override 是否忽略覆盖 (Whether to ignore override)
 * @return 返回声明的参数值 (Return the declared parameter value)
 */
template <typename ParameterT>
auto LifecycleNode::declare_parameter(
    const std::string& name,
    const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor,
    bool ignore_override) {
  // 使用模板魔法从 ParameterT 获取正确的 rclcpp::ParameterType (Use template magic to get the
  // correct rclcpp::ParameterType from ParameterT)
  rclcpp::ParameterValue value{ParameterT{}};

  // 使用 declare_parameter 函数声明参数并获取参数值 (Declare parameter using declare_parameter
  // function and get the parameter value)
  return this->declare_parameter(name, value.get_type(), parameter_descriptor, ignore_override)
      .get<ParameterT>();
}

/**
 * @brief 声明一组参数 (Declare a set of parameters)
 *
 * @tparam ParameterT 参数类型 (Parameter type)
 * @param namespace_ 命名空间 (Namespace)
 * @param parameters 参数映射 (Parameters map)
 * @return 返回声明的参数值向量 (Return the declared parameter values vector)
 */
template <typename ParameterT>
std::vector<ParameterT> LifecycleNode::declare_parameters(
    const std::string& namespace_, const std::map<std::string, ParameterT>& parameters) {
  std::vector<ParameterT> result;
  // 标准化命名空间 (Normalize the namespace)
  std::string normalized_namespace = namespace_.empty() ? "" : (namespace_ + ".");

  // 遍历参数映射，并使用 declare_parameter 函数声明每个参数 (Iterate through the parameters map and
  // declare each parameter using declare_parameter function)
  std::transform(
      parameters.begin(), parameters.end(), std::back_inserter(result),
      [this, &normalized_namespace](auto element) {
        return this->declare_parameter(normalized_namespace + element.first, element.second);
      });

  return result;
}

/**
 * @brief 声明参数 (Declare parameters)
 *
 * @tparam ParameterT 参数类型 (Parameter type)
 * @param namespace_ 命名空间 (Namespace)
 * @param parameters 一个映射，包含参数名称和值及其描述符的对 (A map containing pairs of parameter
 * names and values along with their descriptors)
 * @return std::vector<ParameterT> 返回一组声明的参数 (Returns a vector of declared parameters)
 */
template <typename ParameterT>
std::vector<ParameterT> LifecycleNode::declare_parameters(
    const std::string& namespace_,
    const std::map<std::string, std::pair<ParameterT, rcl_interfaces::msg::ParameterDescriptor>>&
        parameters) {
  // 初始化结果向量 (Initialize result vector)
  std::vector<ParameterT> result;

  // 标准化命名空间 (Normalize the namespace)
  std::string normalized_namespace = namespace_.empty() ? "" : (namespace_ + ".");

  // 使用 std::transform 将参数插入结果向量中 (Use std::transform to insert parameters into the
  // result vector)
  std::transform(
      parameters.begin(), parameters.end(), std::back_inserter(result),
      [this, &normalized_namespace](auto element) {
        // 声明参数并将其转换为 ParameterT 类型 (Declare the parameter and cast it to ParameterT
        // type)
        return static_cast<ParameterT>(this->declare_parameter(
            normalized_namespace + element.first, element.second.first, element.second.second));
      });

  // 返回结果向量 (Return the result vector)
  return result;
}

/**
 * @brief 获取指定名称的参数 (Get the parameter with the specified name)
 *
 * @tparam ParameterT 参数类型 (Parameter type)
 * @param name 参数名称 (Parameter name)
 * @param parameter 用于存储获取到的参数值的引用 (A reference to store the retrieved parameter
 * value)
 * @return bool 是否成功获取参数 (Whether the parameter was successfully retrieved or not)
 */
template <typename ParameterT>
bool LifecycleNode::get_parameter(const std::string& name, ParameterT& parameter) const {
  // 创建 rclcpp::Parameter 类型的参数对象 (Create an rclcpp::Parameter object for the parameter)
  rclcpp::Parameter param(name, parameter);

  // 获取指定名称的参数 (Get the parameter with the specified name)
  bool result = get_parameter(name, param);

  // 将获取到的参数值存储在 parameter 变量中 (Store the retrieved parameter value in the parameter
  // variable)
  parameter = param.get_value<ParameterT>();

  // 返回获取结果 (Return the retrieval result)
  return result;
}

/**
 * @brief 获取具有指定前缀的一组参数 (Get a set of parameters with the specified prefix)
 *
 * @tparam MapValueT 映射值类型 (Map value type)
 * @param prefix 参数名称前缀 (Parameter name prefix)
 * @param values 一个映射，用于存储获取到的参数名称及其值 (A map to store the retrieved parameter
 * names and their values)
 * @return bool 是否成功获取参数 (Whether the parameters were successfully retrieved or not)
 */
// 这是上面 get_parameter 的部分特化版本，
// 其中 ParameterT 的具体类型为 std::map，但待确定的类型是映射中的值。
// (This is a partially-specialized version of get_parameter above,
// where our concrete type for ParameterT is std::map, but the to-be-determined
// type is the value in the map.)
template <typename MapValueT>
bool LifecycleNode::get_parameters(
    const std::string& prefix, std::map<std::string, MapValueT>& values) const {
  // 初始化一个映射，用于存储 rclcpp::Parameter 类型的参数 (Initialize a map to store
  // rclcpp::Parameter type parameters)
  std::map<std::string, rclcpp::Parameter> params;

  // 获取具有指定前缀的一组参数 (Get a set of parameters with the specified prefix)
  bool result = node_parameters_->get_parameters_by_prefix(prefix, params);

  // 如果成功获取参数 (If the parameters were successfully retrieved)
  if (result) {
    // 遍历参数映射，并将其值存储在 values 变量中 (Iterate through the parameter map and store their
    // values in the values variable)
    for (const auto& param : params) {
      values[param.first] = param.second.get_value<MapValueT>();
    }
  }

  // 返回获取结果 (Return the retrieval result)
  return result;
}

/**
 * @brief 获取指定参数，如果没有找到，则返回提供的替代值 (Get the specified parameter, if not found,
 * return the provided alternative value)
 *
 * @tparam ParameterT 参数类型 (Parameter type)
 * @param name 参数名 (Parameter name)
 * @param[out] value 存储获取到的参数值，或者在未找到时存储替代值的引用 (Stores the obtained
 * parameter value, or a reference to the alternative value when not found)
 * @param alternative_value 未找到参数时返回的替代值 (Alternative value returned when the parameter
 * is not found)
 * @return 如果获取到了参数，则返回 true，否则返回 false (If the parameter is obtained, return true,
 * otherwise return false)
 */
template <typename ParameterT>
bool LifecycleNode::get_parameter_or(
    const std::string& name, ParameterT& value, const ParameterT& alternative_value) const {
  // 尝试获取指定名称的参数，如果成功获取，则 got_parameter 为 true (Try to get the parameter with
  // the specified name, if successfully obtained, got_parameter is true)
  bool got_parameter = get_parameter(name, value);

  // 如果没有获取到参数 (If the parameter is not obtained)
  if (!got_parameter) {
    // 将 value 设置为提供的替代值 (Set value to the provided alternative value)
    value = alternative_value;
  }

  // 返回是否成功获取参数的布尔值 (Return the boolean value of whether the parameter was
  // successfully obtained)
  return got_parameter;
}

}  // namespace rclcpp_lifecycle
#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_NODE_IMPL_HPP_

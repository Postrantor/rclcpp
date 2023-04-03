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

#ifndef RCLCPP__NODE_IMPL_HPP_
#define RCLCPP__NODE_IMPL_HPP_

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcl/publisher.h"
#include "rcl/subscription.h"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/create_client.hpp"
#include "rclcpp/create_generic_publisher.hpp"
#include "rclcpp/create_generic_subscription.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_service.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/detail/resolve_enable_topic_statistics.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

#ifndef RCLCPP__NODE_HPP_
#include "node.hpp"
#endif

namespace rclcpp {

RCLCPP_LOCAL
inline std::string extend_name_with_sub_namespace(
    const std::string &name, const std::string &sub_namespace) {
  // 创建一个新的字符串变量，将传入的名称赋给它
  std::string name_with_sub_namespace(name);

  // 如果子命名空间不为空，且名称的第一个字符不是'/'或'~'
  // '~'
  if (sub_namespace != "" && name.front() != '/' && name.front() != '~') {
    // 将子命名空间和名称组合成一个新的字符串
    name_with_sub_namespace = sub_namespace + "/" + name;
  }

  // 返回带有子命名空间的名称
  return name_with_sub_namespace;
}

template <typename MessageT, typename AllocatorT, typename PublisherT>
std::shared_ptr<PublisherT> Node::create_publisher(
    const std::string &topic_name,
    const rclcpp::QoS &qos,
    const PublisherOptionsWithAllocator<AllocatorT> &options) {
  // 使用指定的参数创建一个发布者，并返回其智能指针
  return rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(
      *this, extend_name_with_sub_namespace(topic_name, this->get_sub_namespace()), qos, options);
}

template <
    typename MessageT,
    typename CallbackT,
    typename AllocatorT,
    typename SubscriptionT,
    typename MessageMemoryStrategyT>
std::shared_ptr<SubscriptionT> Node::create_subscription(
    const std::string &topic_name,
    const rclcpp::QoS &qos,
    CallbackT &&callback,
    const SubscriptionOptionsWithAllocator<AllocatorT> &options,
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strat) {
  // 使用指定的参数创建一个订阅者，并返回其智能指针
  return rclcpp::create_subscription<MessageT>(
      *this, extend_name_with_sub_namespace(topic_name, this->get_sub_namespace()), qos,
      std::forward<CallbackT>(callback), options, msg_mem_strat);
}

/**
 * @brief 创建一个墙时钟定时器 (Create a wall-clock timer)
 *
 * @tparam DurationRepT 周期类型 (Period type)
 * @tparam DurationT 周期单位 (Period unit)
 * @tparam CallbackT 回调函数类型 (Callback function type)
 * @param period 定时器周期 (Timer period)
 * @param callback 定时器回调函数 (Timer callback function)
 * @param group 回调组 (Callback group)
 * @return typename rclcpp::WallTimer<CallbackT>::SharedPtr 返回创建的墙时钟定时器 (Return the
 * created wall-clock timer)
 */
template <typename DurationRepT, typename DurationT, typename CallbackT>
typename rclcpp::WallTimer<CallbackT>::SharedPtr Node::create_wall_timer(
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 调用 rclcpp::create_wall_timer 创建墙时钟定时器 (Call rclcpp::create_wall_timer to create a
  // wall-clock timer)
  return rclcpp::create_wall_timer(
      period, std::move(callback), group, this->node_base_.get(), this->node_timers_.get());
}

/**
 * @brief 创建一个通用定时器 (Create a generic timer)
 *
 * @tparam DurationRepT 周期类型 (Period type)
 * @tparam DurationT 周期单位 (Period unit)
 * @tparam CallbackT 回调函数类型 (Callback function type)
 * @param period 定时器周期 (Timer period)
 * @param callback 定时器回调函数 (Timer callback function)
 * @param group 回调组 (Callback group)
 * @return typename rclcpp::GenericTimer<CallbackT>::SharedPtr 返回创建的通用定时器 (Return the
 * created generic timer)
 */
template <typename DurationRepT, typename DurationT, typename CallbackT>
typename rclcpp::GenericTimer<CallbackT>::SharedPtr Node::create_timer(
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 调用 rclcpp::create_timer 创建通用定时器 (Call rclcpp::create_timer to create a generic timer)
  return rclcpp::create_timer(
      this->get_clock(), period, std::move(callback), group, this->node_base_.get(),
      this->node_timers_.get());
}

/**
 * @brief 创建一个服务客户端 (Create a service client)
 *
 * @tparam ServiceT 服务类型 (Service type)
 * @param service_name 服务名 (Service name)
 * @param qos 服务质量配置 (Quality of Service configuration)
 * @param group 回调组 (Callback group)
 * @return typename Client<ServiceT>::SharedPtr 返回创建的服务客户端 (Return the created service
 * client)
 */
template <typename ServiceT>
typename Client<ServiceT>::SharedPtr Node::create_client(
    const std::string &service_name,
    const rclcpp::QoS &qos,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 调用 rclcpp::create_client 创建服务客户端 (Call rclcpp::create_client to create a service
  // client)
  return rclcpp::create_client<ServiceT>(
      node_base_, node_graph_, node_services_,
      extend_name_with_sub_namespace(service_name, this->get_sub_namespace()), qos, group);
}

/**
 * @brief 创建一个服务客户端 (Create a service client)
 *
 * @tparam ServiceT 服务类型 (Service type)
 * @param service_name 服务名 (Service name)
 * @param qos_profile 服务质量配置 (Quality of Service configuration)
 * @param group 回调组 (Callback group)
 * @return typename Client<ServiceT>::SharedPtr 返回创建的服务客户端 (Return the created service
 * client)
 */
template <typename ServiceT>
typename Client<ServiceT>::SharedPtr Node::create_client(
    const std::string &service_name,
    const rmw_qos_profile_t &qos_profile,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 调用 rclcpp::create_client 创建服务客户端 (Call rclcpp::create_client to create a service
  // client)
  return rclcpp::create_client<ServiceT>(
      node_base_, node_graph_, node_services_,
      extend_name_with_sub_namespace(service_name, this->get_sub_namespace()), qos_profile, group);
}

/**
 * @brief 创建一个服务对象 (Create a service object)
 *
 * @tparam ServiceT 服务类型 (Service type)
 * @tparam CallbackT 回调函数类型 (Callback function type)
 * @param[in] service_name 服务名称 (Service name)
 * @param[in] callback 服务回调函数 (Service callback function)
 * @param[in] qos Quality of Service 设置 (Quality of Service settings)
 * @param[in] group 回调组 (Callback group)
 * @return 返回创建的服务对象的智能指针 (Return the smart pointer of the created service object)
 */
template <typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr Node::create_service(
    const std::string &service_name,
    CallbackT &&callback,
    const rclcpp::QoS &qos,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 使用给定的参数创建服务对象，并返回该对象的智能指针
  // (Create a service object with the given parameters and return its smart pointer)
  return rclcpp::create_service<ServiceT, CallbackT>(
      node_base_, node_services_,
      extend_name_with_sub_namespace(service_name, this->get_sub_namespace()),
      std::forward<CallbackT>(callback), qos, group);
}

/**
 * @brief 创建一个服务对象 (Create a service object)
 *
 * @tparam ServiceT 服务类型 (Service type)
 * @tparam CallbackT 回调函数类型 (Callback function type)
 * @param[in] service_name 服务名称 (Service name)
 * @param[in] callback 服务回调函数 (Service callback function)
 * @param[in] qos_profile Quality of Service 配置 (Quality of Service profile)
 * @param[in] group 回调组 (Callback group)
 * @return 返回创建的服务对象的智能指针 (Return the smart pointer of the created service object)
 */
template <typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr Node::create_service(
    const std::string &service_name,
    CallbackT &&callback,
    const rmw_qos_profile_t &qos_profile,
    rclcpp::CallbackGroup::SharedPtr group) {
  // 使用给定的参数创建服务对象，并返回该对象的智能指针
  // (Create a service object with the given parameters and return its smart pointer)
  return rclcpp::create_service<ServiceT, CallbackT>(
      node_base_, node_services_,
      extend_name_with_sub_namespace(service_name, this->get_sub_namespace()),
      std::forward<CallbackT>(callback), qos_profile, group);
}

/**
 * @brief 创建一个通用发布器对象 (Create a generic publisher object)
 *
 * @tparam AllocatorT 分配器类型 (Allocator type)
 * @param[in] topic_name 主题名称 (Topic name)
 * @param[in] topic_type 主题类型 (Topic type)
 * @param[in] qos Quality of Service 设置 (Quality of Service settings)
 * @param[in] options 发布器选项 (Publisher options)
 * @return 返回创建的通用发布器对象的智能指针 (Return the smart pointer of the created generic
 * publisher object)
 */
template <typename AllocatorT>
std::shared_ptr<rclcpp::GenericPublisher> Node::create_generic_publisher(
    const std::string &topic_name,
    const std::string &topic_type,
    const rclcpp::QoS &qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options) {
  // 使用给定的参数创建通用发布器对象，并返回该对象的智能指针
  return rclcpp::create_generic_publisher(
      node_topics_, extend_name_with_sub_namespace(topic_name, this->get_sub_namespace()),
      topic_type, qos, options);
}

/**
 * @brief 创建一个通用订阅者 (Create a generic subscription)
 *
 * @tparam AllocatorT 分配器类型 (Allocator type)
 * @param topic_name 订阅的主题名称 (Topic name to subscribe)
 * @param topic_type 订阅的主题类型 (Topic type to subscribe)
 * @param qos 服务质量配置 (Quality of Service configuration)
 * @param callback 当收到消息时调用的回调函数 (Callback function to be called when a message is
 * received)
 * @param options 订阅选项 (Subscription options)
 * @return std::shared_ptr<rclcpp::GenericSubscription> 通用订阅者指针 (Pointer to the created
 * generic subscription)
 */
template <typename AllocatorT>
std::shared_ptr<rclcpp::GenericSubscription> Node::create_generic_subscription(
    const std::string &topic_name,
    const std::string &topic_type,
    const rclcpp::QoS &qos,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options) {
  // 使用给定参数创建通用订阅者 (Create a generic subscription with the given arguments)
  return rclcpp::create_generic_subscription(
      node_topics_, extend_name_with_sub_namespace(topic_name, this->get_sub_namespace()),
      topic_type, qos, std::move(callback), options);
}

/**
 * @brief 声明一个参数并设置默认值 (Declare a parameter and set its default value)
 *
 * @tparam ParameterT 参数类型 (Parameter type)
 * @param name 参数名称 (Parameter name)
 * @param default_value 默认值 (Default value)
 * @param parameter_descriptor 参数描述符 (Parameter descriptor)
 * @param ignore_override 是否忽略重写 (Whether to ignore overrides)
 * @return ParameterT 声明的参数值 (Declared parameter value)
 */
template <typename ParameterT>
auto Node::declare_parameter(
    const std::string &name,
    const ParameterT &default_value,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
    bool ignore_override) {
  try {
    // 使用给定的默认值和描述符声明参数
    return this
        ->declare_parameter(
            name, rclcpp::ParameterValue(default_value), parameter_descriptor, ignore_override)
        .get<ParameterT>();
  } catch (const ParameterTypeException &ex) {
    // 参数类型异常处理 (Handle parameter type exception)
    throw exceptions::InvalidParameterTypeException(name, ex.what());
  }
}

/**
 * @brief 声明一个参数，不设置默认值 (Declare a parameter without setting its default value)
 *
 * @tparam ParameterT 参数类型 (Parameter type)
 * @param name 参数名称 (Parameter name)
 * @param parameter_descriptor 参数描述符 (Parameter descriptor)
 * @param ignore_override 是否忽略重写 (Whether to ignore overrides)
 * @return ParameterT 声明的参数值 (Declared parameter value)
 */
template <typename ParameterT>
auto Node::declare_parameter(
    const std::string &name,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
    bool ignore_override) {
  // 利用模板魔术从 ParameterT 获取正确的 rclcpp::ParameterType
  rclcpp::ParameterValue value{ParameterT{}};
  try {
    // 使用给定的描述符声明参数 (Declare the parameter with the given descriptor)
    return this->declare_parameter(name, value.get_type(), parameter_descriptor, ignore_override)
        .get<ParameterT>();
  } catch (const ParameterTypeException &) {
    // 参数类型异常处理 (Handle parameter type exception)
    throw exceptions::UninitializedStaticallyTypedParameterException(name);
  }
}

/**
 * @brief 声明参数列表（Declare a list of parameters）
 *
 * @tparam ParameterT 参数类型（Parameter type）
 * @param namespace_ 参数命名空间（Namespace for the parameters）
 * @param parameters 一个字符串到ParameterT类型的映射，包含要声明的参数名称和默认值（A map from
 * string to ParameterT, containing the names and default values of the parameters to declare）
 * @param ignore_overrides 是否忽略覆盖（Whether to ignore overrides or not）
 * @return std::vector<ParameterT> 返回一组已声明的参数（Returns a vector of declared parameters）
 */
template <typename ParameterT>
std::vector<ParameterT> Node::declare_parameters(
    const std::string &namespace_,
    const std::map<std::string, ParameterT> &parameters,
    bool ignore_overrides) {
  // 初始化结果向量（Initialize result vector）
  std::vector<ParameterT> result;

  // 标准化命名空间（Normalize namespace）
  std::string normalized_namespace = namespace_.empty() ? "" : (namespace_ + ".");

  // 使用std::transform将参数映射转换为结果向量（Use std::transform to convert the parameter map
  // into the result vector）
  std::transform(
      parameters.begin(), parameters.end(), std::back_inserter(result),
      [this, &normalized_namespace, ignore_overrides](auto element) {
        // 声明每个参数并添加到结果向量中（Declare each parameter and add it to the result vector）
        return this->declare_parameter(
            normalized_namespace + element.first, element.second,
            rcl_interfaces::msg::ParameterDescriptor(), ignore_overrides);
      });

  // 返回结果向量（Return the result vector）
  return result;
}

/**
 * @brief 声明参数列表（Declare a list of parameters）
 *
 * @tparam ParameterT 参数类型（Parameter type）
 * @param namespace_ 参数命名空间（Namespace for the parameters）
 * @param parameters
 * 一个字符串到ParameterT类型和描述符的映射，包含要声明的参数名称、默认值和描述符（A map from string
 * to pair of ParameterT and descriptor, containing the names, default values and descriptors of the
 * parameters to declare）
 * @param ignore_overrides 是否忽略覆盖（Whether to ignore overrides or not）
 * @return std::vector<ParameterT> 返回一组已声明的参数（Returns a vector of declared parameters）
 */
template <typename ParameterT>
std::vector<ParameterT> Node::declare_parameters(
    const std::string &namespace_,
    const std::map<std::string, std::pair<ParameterT, rcl_interfaces::msg::ParameterDescriptor>>
        &parameters,
    bool ignore_overrides) {
  // 初始化结果向量（Initialize result vector）
  std::vector<ParameterT> result;

  // 标准化命名空间（Normalize namespace）
  std::string normalized_namespace = namespace_.empty() ? "" : (namespace_ + ".");

  // 使用std::transform将参数映射转换为结果向量
  std::transform(
      parameters.begin(), parameters.end(), std::back_inserter(result),
      [this, &normalized_namespace, ignore_overrides](auto element) {
        // 声明每个参数并添加到结果向量中（Declare each parameter and add it to the result vector）
        return static_cast<ParameterT>(this->declare_parameter(
            normalized_namespace + element.first, element.second.first, element.second.second,
            ignore_overrides));
      });

  // 返回结果向量（Return the result vector）
  return result;
}

/**
 * @brief 获取指定名称的参数值
 *
 * @tparam ParameterT 参数值类型
 * @param[in] name 参数名称
 * @param[out] parameter 参数值的输出引用
 * @return 如果获取成功，则返回 true，否则返回 false。
 */
template <typename ParameterT>
bool Node::get_parameter(const std::string &name, ParameterT &parameter) const {
  // 扩展参数名，包含子命名空间
  std::string sub_name = extend_name_with_sub_namespace(name, this->get_sub_namespace());

  rclcpp::Parameter parameter_variant;

  // 获取参数变量
  bool result = get_parameter(sub_name, parameter_variant);
  if (result) {
    // 将参数值转换为指定类型
    parameter = static_cast<ParameterT>(parameter_variant.get_value<ParameterT>());
  }

  return result;
}

/**
 * @brief 获取指定名称的参数值，如果不存在，则使用备选值
 *
 * @tparam ParameterT 参数值类型
 * @param[in] name 参数名称
 * @param[out] parameter 参数值的输出引用
 * @param[in] alternative_value 备选值
 * @return 如果获取成功，则返回 true，否则返回 false。
 */
template <typename ParameterT>
bool Node::get_parameter_or(
    const std::string &name, ParameterT &parameter, const ParameterT &alternative_value) const {
  // 扩展参数名，包含子命名空间
  std::string sub_name = extend_name_with_sub_namespace(name, this->get_sub_namespace());

  // 获取参数值
  bool got_parameter = get_parameter(sub_name, parameter);
  if (!got_parameter) {
    // 如果参数不存在，使用备选值
    parameter = alternative_value;
  }
  return got_parameter;
}

/**
 * @brief 获取指定名称的参数值，如果不存在，则使用备选值
 *
 * @tparam ParameterT 参数值类型
 * @param[in] name 参数名称
 * @param[in] alternative_value 备选值
 * @return 返回获取到的参数值或备选值
 */
template <typename ParameterT>
ParameterT Node::get_parameter_or(
    const std::string &name, const ParameterT &alternative_value) const {
  ParameterT parameter;
  get_parameter_or(name, parameter, alternative_value);
  return parameter;
}

/**
 * @brief 获取具有指定前缀的一组参数值
 *
 * @tparam ParameterT 参数值类型
 * @param[in] prefix 参数名称前缀
 * @param[out] values 输出参数值的映射
 * @return 如果获取成功，则返回 true，否则返回 false。
 */
template <typename ParameterT>
bool Node::get_parameters(
    const std::string &prefix, std::map<std::string, ParameterT> &values) const {
  std::map<std::string, rclcpp::Parameter> params;
  // 获取具有指定前缀的参数集
  bool result = node_parameters_->get_parameters_by_prefix(prefix, params);
  if (result) {
    for (const auto &param : params) {
      // 将参数值转换为指定类型并存储到输出映射中
      values[param.first] = static_cast<ParameterT>(param.second.get_value<ParameterT>());
    }
  }

  return result;
}

}  // namespace rclcpp

#endif  // RCLCPP__NODE_IMPL_HPP_

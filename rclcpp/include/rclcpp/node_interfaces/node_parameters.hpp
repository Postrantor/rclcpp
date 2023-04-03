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

#ifndef RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_HPP_

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcutils/macros.h"

namespace rclcpp {
namespace node_interfaces {

/**
 * @brief 内部结构体，用于存储有关参数的有用信息 (Internal struct for holding useful info about
 * parameters)
 */
struct ParameterInfo {
  /// 当前参数的值 (Current value of the parameter)
  rclcpp::ParameterValue value;

  /// 参数描述 (A description of the parameter)
  rcl_interfaces::msg::ParameterDescriptor descriptor;
};

/**
 * @brief 内部RAII风格的守卫，用于处理参数变更递归 (Internal RAII-style guard for mutation
 * recursion)
 */
class ParameterMutationRecursionGuard {
public:
  /**
   * @brief 构造函数 (Constructor)
   *
   * @param allow_mod 是否允许修改参数 (Whether to allow modification of the parameter)
   */
  explicit ParameterMutationRecursionGuard(bool& allow_mod) : allow_modification_(allow_mod) {
    // 如果不允许修改参数，则抛出异常 (If parameter modification is not allowed, throw an exception)
    if (!allow_modification_) {
      throw rclcpp::exceptions::ParameterModifiedInCallbackException(
          "cannot set or declare a parameter, or change the callback from within set callback");
    }

    // 设置为不允许修改参数 (Set to not allow parameter modification)
    allow_modification_ = false;
  }

  /// 析构函数，在对象销毁时将参数修改标志设置为允许 (Destructor, sets the parameter modification
  /// flag to allowed when the object is destroyed)
  ~ParameterMutationRecursionGuard() { allow_modification_ = true; }

private:
  /// 参数修改标志的引用 (Reference to the parameter modification flag)
  bool& allow_modification_;
};

/// 实现 Node API 的 NodeParameters 部分。 (Implementation of the NodeParameters part of the Node
/// API.)
class NodeParameters : public NodeParametersInterface {
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeParameters)

  /// 构造函数。 (Constructor.)
  /**
   * 如果使用 automatically_declare_parameters_from_overrides，那么
   * get_parameter_overrides()、has_parameter() 和 declare_parameter() 的重写将不会被尊重。 (If
   * using automatically_declare_parameters_from_overrides, overrides of get_parameter_overrides(),
   * has_parameter(), declare_parameter() will not be respected.) 如果这是一个问题，请为
   * automatically_declare_parameters_from_overrides 传递 false，并在构造完成后手动调用
   * perform_automatically_declare_parameters_from_overrides()。 (If this is an issue, pass false
   * for automatically_declare_parameters_from_overrides and invoke
   * perform_automatically_declare_parameters_from_overrides() manually after
   * construction.)
   */
  RCLCPP_PUBLIC
  NodeParameters(
      const node_interfaces::NodeBaseInterface::SharedPtr
          node_base,  ///< 节点基本接口的共享指针。 (Shared pointer to the node base interface.)
      const node_interfaces::NodeLoggingInterface::SharedPtr
          node_logging,  ///< 节点日志接口的共享指针。 (Shared pointer to the node logging
                         ///< interface.)
      const node_interfaces::NodeTopicsInterface::SharedPtr
          node_topics,  ///< 节点主题接口的共享指针。 (Shared pointer to the node topics interface.)
      const node_interfaces::NodeServicesInterface::SharedPtr
          node_services,  ///< 节点服务接口的共享指针。 (Shared pointer to the node services
                          ///< interface.)
      const node_interfaces::NodeClockInterface::SharedPtr
          node_clock,  ///< 节点时钟接口的共享指针。 (Shared pointer to the node clock interface.)
      const std::vector<Parameter>&
          parameter_overrides,  ///< 参数覆盖向量。 (Vector of parameter overrides.)
      bool start_parameter_services,  ///< 是否启动参数服务。 (Whether to start parameter services
                                      ///< or not.)
      bool start_parameter_event_publisher,  ///< 是否启动参数事件发布器。 (Whether to start
                                             ///< parameter event publisher or not.)
      const rclcpp::QoS&
          parameter_event_qos,  ///< 参数事件的 QoS 设置。 (QoS settings for parameter events.)
      const rclcpp::PublisherOptionsBase&
          parameter_event_publisher_options,  ///< 参数事件发布器选项。 (Parameter event publisher
                                              ///< options.)
      bool allow_undeclared_parameters,  ///< 是否允许未声明的参数。 (Whether to allow undeclared
                                         ///< parameters or not.)
      bool
          automatically_declare_parameters_from_overrides);  ///< 是否从覆盖中自动声明参数。
                                                             ///< (Whether to automatically declare
                                                             ///< parameters from overrides or not.)

  /**
   * @brief 析构函数 (Destructor)
   */
  RCLCPP_PUBLIC
  virtual ~NodeParameters();

  /**
   * @brief 声明参数 (Declare a parameter)
   *
   * @param name 参数名 (Parameter name)
   * @param default_value 默认值 (Default value)
   * @param parameter_descriptor 参数描述符 (Parameter descriptor)
   * @param ignore_override 是否忽略覆盖 (Whether to ignore override)
   * @return 返回声明的参数值 (Return the declared parameter value)
   */
  RCLCPP_PUBLIC
  const rclcpp::ParameterValue& declare_parameter(
      const std::string& name,
      const rclcpp::ParameterValue& default_value,
      const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
          rcl_interfaces::msg::ParameterDescriptor{},
      bool ignore_override = false) override;

  /**
   * @brief 声明参数 (Declare a parameter)
   *
   * @param name 参数名 (Parameter name)
   * @param type 参数类型 (Parameter type)
   * @param parameter_descriptor 参数描述符 (Parameter descriptor)
   * @param ignore_override 是否忽略覆盖 (Whether to ignore override)
   * @return 返回声明的参数值 (Return the declared parameter value)
   */
  RCLCPP_PUBLIC
  const rclcpp::ParameterValue& declare_parameter(
      const std::string& name,
      rclcpp::ParameterType type,
      const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
          rcl_interfaces::msg::ParameterDescriptor(),
      bool ignore_override = false) override;

  /**
   * @brief 取消声明参数 (Undeclare a parameter)
   *
   * @param name 参数名 (Parameter name)
   */
  RCLCPP_PUBLIC
  void undeclare_parameter(const std::string& name) override;

  /**
   * @brief 检查是否存在参数 (Check if a parameter exists)
   *
   * @param name 参数名 (Parameter name)
   * @return 返回是否存在参数 (Return whether the parameter exists)
   */
  RCLCPP_PUBLIC
  bool has_parameter(const std::string& name) const override;

  /**
   * @brief 设置一组参数 (Set a group of parameters)
   *
   * @param parameters 参数向量 (Vector of parameters)
   * @return 返回设置参数结果的向量 (Return a vector of set parameter results)
   */
  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
      const std::vector<rclcpp::Parameter>& parameters) override;

  /**
   * @brief 原子性地设置一组参数 (Set a group of parameters atomically)
   *
   * @param parameters 参数向量 (Vector of parameters)
   * @return 返回设置参数结果 (Return the set parameters result)
   */
  RCLCPP_PUBLIC
  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
      const std::vector<rclcpp::Parameter>& parameters) override;

  /**
   * @brief 获取一组参数 (Get a group of parameters)
   *
   * @param names 参数名向量 (Vector of parameter names)
   * @return 返回参数向量 (Return a vector of parameters)
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::Parameter> get_parameters(
      const std::vector<std::string>& names) const override;

  /**
   * @brief 获取一个参数 (Get a parameter)
   *
   * @param name 参数名 (Parameter name)
   * @return 返回参数 (Return the parameter)
   */
  RCLCPP_PUBLIC
  rclcpp::Parameter get_parameter(const std::string& name) const override;

  /**
   * @brief 获取一个参数 (Get a parameter)
   *
   * @param name 参数名 (Parameter name)
   * @param[out] parameter 输出参数 (Output parameter)
   * @return 返回是否成功获取参数 (Return whether the parameter was successfully obtained)
   */
  RCLCPP_PUBLIC
  bool get_parameter(const std::string& name, rclcpp::Parameter& parameter) const override;

  /**
   * @brief 获取带有指定前缀的参数 (Get parameters with the specified prefix)
   *
   * @param[in] prefix 参数前缀 (Parameter prefix)
   * @param[out] parameters 存储获取到的参数的映射 (A map to store the obtained parameters)
   * @return 是否成功获取参数 (Whether the parameters were successfully obtained)
   */
  RCLCPP_PUBLIC
  bool get_parameters_by_prefix(
      const std::string& prefix,
      std::map<std::string, rclcpp::Parameter>& parameters) const override;

  /**
   * @brief 描述指定名称的参数 (Describe parameters with specified names)
   *
   * @param[in] names 要描述的参数名称列表 (List of parameter names to describe)
   * @return 参数描述符向量 (Vector of parameter descriptors)
   */
  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
      const std::vector<std::string>& names) const override;

  /**
   * @brief 获取指定名称参数的类型 (Get types of parameters with specified names)
   *
   * @param[in] names 要获取类型的参数名称列表 (List of parameter names to get types for)
   * @return 参数类型向量 (Vector of parameter types)
   */
  RCLCPP_PUBLIC
  std::vector<uint8_t> get_parameter_types(const std::vector<std::string>& names) const override;

  /**
   * @brief 列出具有指定前缀的参数 (List parameters with specified prefixes)
   *
   * @param[in] prefixes 参数前缀列表 (List of parameter prefixes)
   * @param[in] depth 搜索深度 (Search depth)
   * @return 包含参数信息的结果对象 (Result object containing parameter information)
   */
  RCLCPP_PUBLIC
  rcl_interfaces::msg::ListParametersResult list_parameters(
      const std::vector<std::string>& prefixes, uint64_t depth) const override;

  /**
   * @brief 添加预设参数回调 (Add pre-set parameters callback)
   *
   * @param[in] callback 预设参数回调类型 (Pre-set parameters callback type)
   * @return 预设参数回调句柄的共享指针 (Shared pointer to the pre-set parameters callback handle)
   */
  RCLCPP_PUBLIC
  RCUTILS_WARN_UNUSED
  PreSetParametersCallbackHandle::SharedPtr add_pre_set_parameters_callback(
      PreSetParametersCallbackType callback) override;

  /**
   * @brief 添加设置参数回调 (Add on-set parameters callback)
   *
   * @param[in] callback 设置参数回调类型 (On-set parameters callback type)
   * @return 设置参数回调句柄的共享指针 (Shared pointer to the on-set parameters callback handle)
   */
  RCLCPP_PUBLIC
  RCUTILS_WARN_UNUSED
  OnSetParametersCallbackHandle::SharedPtr add_on_set_parameters_callback(
      OnSetParametersCallbackType callback) override;

  /**
   * @brief 添加后置参数回调 (Add post-set parameters callback)
   *
   * @param[in] callback 后置参数回调类型 (Post-set parameters callback type)
   * @return 后置参数回调句柄的共享指针 (Shared pointer to the post-set parameters callback handle)
   */
  RCLCPP_PUBLIC
  RCUTILS_WARN_UNUSED
  PostSetParametersCallbackHandle::SharedPtr add_post_set_parameters_callback(
      PostSetParametersCallbackType callback) override;

  /**
   * @brief 删除设置参数回调函数 (Remove the on_set_parameters callback function)
   *
   * @param handler 指向 OnSetParametersCallbackHandle 类型的指针 (Pointer to the
   * OnSetParametersCallbackHandle type)
   */
  RCLCPP_PUBLIC
  void remove_on_set_parameters_callback(
      const OnSetParametersCallbackHandle* const handler) override;

  /**
   * @brief 删除 post_set_parameters 回调函数 (Remove the post_set_parameters callback function)
   *
   * @param handler 指向 PostSetParametersCallbackHandle 类型的指针 (Pointer to the
   * PostSetParametersCallbackHandle type)
   */
  RCLCPP_PUBLIC
  void remove_post_set_parameters_callback(
      const PostSetParametersCallbackHandle* const handler) override;

  /**
   * @brief 删除 pre_set_parameters 回调函数 (Remove the pre_set_parameters callback function)
   *
   * @param handler 指向 PreSetParametersCallbackHandle 类型的指针 (Pointer to the
   * PreSetParametersCallbackHandle type)
   */
  RCLCPP_PUBLIC
  void remove_pre_set_parameters_callback(
      const PreSetParametersCallbackHandle* const handler) override;

  /**
   * @brief 获取参数覆盖值映射 (Get the parameter overrides map)
   *
   * @return const std::map<std::string, rclcpp::ParameterValue>& 参数覆盖值映射的引用 (Reference to
   * the parameter overrides map)
   */
  RCLCPP_PUBLIC
  const std::map<std::string, rclcpp::ParameterValue>& get_parameter_overrides() const override;

  // 定义 PreSetCallbacksHandleContainer 类型为 PreSetParametersCallbackHandle::WeakPtr 的列表
  // (Define PreSetCallbacksHandleContainer type as a list of
  // PreSetParametersCallbackHandle::WeakPtr)
  using PreSetCallbacksHandleContainer = std::list<PreSetParametersCallbackHandle::WeakPtr>;

  // 定义 OnSetCallbacksHandleContainer 类型为 OnSetParametersCallbackHandle::WeakPtr 的列表
  // (Define OnSetCallbacksHandleContainer type as a list of OnSetParametersCallbackHandle::WeakPtr)
  using OnSetCallbacksHandleContainer = std::list<OnSetParametersCallbackHandle::WeakPtr>;

  // 定义 PostSetCallbacksHandleContainer 类型为 PostSetParametersCallbackHandle::WeakPtr 的列表
  // (Define PostSetCallbacksHandleContainer type as a list of
  // PostSetParametersCallbackHandle::WeakPtr)
  using PostSetCallbacksHandleContainer = std::list<PostSetParametersCallbackHandle::WeakPtr>;

  // 已弃用，使用 OnSetCallbacksHandleContainer 代替
  // (Deprecated, use OnSetCallbacksHandleContainer instead)
  using CallbacksContainerType [[deprecated("use OnSetCallbacksHandleContainer instead")]] =
      OnSetCallbacksHandleContainer;

protected:
  /**
   * @brief 自动从参数覆盖中声明参数 (Automatically declare parameters from overrides)
   *
   * 这个函数会自动从参数覆盖列表中声明所有参数，而无需手动逐一声明。
   * (This function will automatically declare all parameters from the parameter override list,
   * without manually declaring them one by one.)
   */
  RCLCPP_PUBLIC
  void perform_automatically_declare_parameters_from_overrides() {
    // 获取参数覆盖列表 (Get the parameter override list)
    // 参数覆盖列表是一个字典，其中包含了需要声明的参数名称及其对应的值。
    // (The parameter override list is a dictionary containing the names of the parameters to be
    // declared and their corresponding values.)
    auto parameter_overrides = get_node_options().parameter_overrides();

    // 遍历参数覆盖列表 (Iterate through the parameter override list)
    // 对于列表中的每个参数，都将其声明为一个新的参数。
    // (For each parameter in the list, declare it as a new parameter.)
    for (const auto& parameter_override : parameter_overrides) {
      // 获取参数名称 (Get the parameter name)
      // 参数名称是一个字符串，用于唯一标识参数。
      // (The parameter name is a string that uniquely identifies the parameter.)
      const std::string& parameter_name = parameter_override.get_name();

      // 获取参数值 (Get the parameter value)
      // 参数值可以是不同类型的数据，例如整数、浮点数、布尔值等。
      // (The parameter value can be data of different types, such as integers, floating point
      // numbers, boolean values, etc.)
      const rclcpp::ParameterValue& parameter_value = parameter_override.get_parameter_value();

      // 声明参数 (Declare the parameter)
      // 将参数名称和参数值作为输入，声明一个新的参数。
      // (Declare a new parameter with the parameter name and parameter value as input.)
      this->declare_parameter(parameter_name, parameter_value);
    }
  }

private:
  /**
   * @class NodeParameters
   * @brief 禁用 NodeParameters 类的复制功能 (Disable copy for NodeParameters class)
   */
  RCLCPP_DISABLE_COPY(NodeParameters)

  // 定义一个递归互斥锁，以保护参数的访问和修改 (Define a recursive mutex to protect access and
  // modification of parameters)
  mutable std::recursive_mutex mutex_;

  // 有时我们不希望允许对参数进行修改 (There are times when we don't want to allow modifications to
  // parameters) 尤其是在 set_parameter 回调尝试调用 set_parameter、declare_parameter 等情况下
  // (particularly when a set_parameter callback tries to call set_parameter, declare_parameter,
  // etc) 在这些情况下，这个值将被设置为 false (In those cases, this will be set to false)
  bool parameter_modification_enabled_{true};

  // 预设回调句柄容器 (Pre-set callbacks handle container)
  PreSetCallbacksHandleContainer pre_set_parameters_callback_container_;

  // 设置回调句柄容器 (On-set callbacks handle container)
  OnSetCallbacksHandleContainer on_set_parameters_callback_container_;

  // 后置回调句柄容器 (Post-set callbacks handle container)
  PostSetCallbacksHandleContainer post_set_parameters_callback_container_;

  // 存储参数信息的映射 (A map to store parameter information)
  std::map<std::string, ParameterInfo> parameters_;

  // 存储参数覆盖值的映射 (A map to store parameter override values)
  std::map<std::string, rclcpp::ParameterValue> parameter_overrides_;

  // 是否允许未声明的参数 (Whether to allow undeclared parameters)
  bool allow_undeclared_ = false;

  // 事件发布器 (Event publisher)
  Publisher<rcl_interfaces::msg::ParameterEvent>::SharedPtr events_publisher_;

  // 参数服务对象的智能指针 (Shared pointer to the parameter service object)
  std::shared_ptr<ParameterService> parameter_service_;

  // 组合名称 (Combined name)
  std::string combined_name_;

  // 节点日志接口智能指针 (Shared pointer to the node logging interface)
  node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  // 节点时钟接口智能指针 (Shared pointer to the node clock interface)
  node_interfaces::NodeClockInterface::SharedPtr node_clock_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_HPP_

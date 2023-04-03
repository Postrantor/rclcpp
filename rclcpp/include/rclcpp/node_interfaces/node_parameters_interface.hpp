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

#ifndef RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_INTERFACE_HPP_

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/detail/node_interfaces_helpers.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/**
 * @brief 预设参数回调处理结构体 (PreSetParametersCallbackHandle structure)
 */
struct PreSetParametersCallbackHandle {
  // 定义智能指针类型的宏 (Define smart pointer type macros)
  RCLCPP_SMART_PTR_DEFINITIONS(PreSetParametersCallbackHandle)

  // 定义预设参数回调函数类型 (Define the pre-set parameters callback function type)
  using PreSetParametersCallbackType = std::function<void(std::vector<rclcpp::Parameter> &)>;

  // 预设参数回调函数 (Pre-set parameters callback function)
  PreSetParametersCallbackType callback;
};

/**
 * @brief 设置参数回调处理结构体 (OnSetParametersCallbackHandle structure)
 */
struct OnSetParametersCallbackHandle {
  // 定义智能指针类型的宏 (Define smart pointer type macros)
  RCLCPP_SMART_PTR_DEFINITIONS(OnSetParametersCallbackHandle)

  // 定义设置参数回调函数类型 (Define the on-set parameters callback function type)
  using OnSetParametersCallbackType = std::function<rcl_interfaces::msg::SetParametersResult(
      const std::vector<rclcpp::Parameter> &)>;

  // 定义已弃用的回调函数类型 (Define deprecated callback function type)
  using OnParametersSetCallbackType [[deprecated("use OnSetParametersCallbackType instead")]] =
      OnSetParametersCallbackType;

  // 设置参数回调函数 (On-set parameters callback function)
  OnSetParametersCallbackType callback;
};

/**
 * @brief 后设参数回调处理结构体 (PostSetParametersCallbackHandle structure)
 */
struct PostSetParametersCallbackHandle {
  // 定义智能指针类型的宏 (Define smart pointer type macros)
  RCLCPP_SMART_PTR_DEFINITIONS(PostSetParametersCallbackHandle)

  // 定义后设参数回调函数类型 (Define the post-set parameters callback function type)
  using PostSetParametersCallbackType = std::function<void(const std::vector<rclcpp::Parameter> &)>;

  // 后设参数回调函数 (Post-set parameters callback function)
  PostSetParametersCallbackType callback;
};

/**
 * @brief 纯虚拟接口类，用于 Node API 的 NodeParameters 部分。
 *        Pure virtual interface class for the NodeParameters part of the Node API.
 */
class NodeParametersInterface {
public:
  /// 定义智能指针别名，仅用于 NodeParametersInterface 类型。
  /// Define smart pointer aliases, only for NodeParametersInterface type.
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeParametersInterface)

  /// @brief 虚析构函数 (Virtual destructor)
  /// @details 释放 NodeParametersInterface 对象时调用此虚析构函数 (Called when a
  /// NodeParametersInterface object is released)
  RCLCPP_PUBLIC
  virtual ~NodeParametersInterface() = default;

  /// @brief 声明并初始化一个参数 (Declare and initialize a parameter)
  ///
  /// @param[in] name 参数名 (Parameter name)
  /// @param[in] default_value 参数默认值 (Default value of the parameter)
  /// @param[in] parameter_descriptor 参数描述符，包含参数的元数据 (Parameter descriptor containing
  /// metadata about the parameter, optional)
  /// @param[in] ignore_override 是否忽略覆盖，默认为 false (Whether to ignore overrides, defaults
  /// to false, optional)
  ///
  /// @return 返回声明的参数值的引用 (Returns a reference to the declared parameter value)
  ///
  /// \sa rclcpp::Node::declare_parameter
  RCLCPP_PUBLIC
  virtual const rclcpp::ParameterValue &declare_parameter(
      const std::string &name,
      const rclcpp::ParameterValue &default_value,
      const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
          rcl_interfaces::msg::ParameterDescriptor(),
      bool ignore_override = false) = 0;

  /// @brief 声明一个参数 (Declare a parameter)
  ///
  /// @param[in] name 参数名 (Parameter name)
  /// @param[in] type 参数类型 (Parameter type)
  /// @param[in] parameter_descriptor 参数描述符，包含参数的元数据 (Parameter descriptor containing
  /// metadata about the parameter, optional)
  /// @param[in] ignore_override 是否忽略覆盖，默认为 false (Whether to ignore overrides, defaults
  /// to false, optional)
  ///
  /// @return 返回声明的参数值的引用 (Returns a reference to the declared parameter value)
  ///
  /// \sa rclcpp::Node::declare_parameter
  RCLCPP_PUBLIC
  virtual const rclcpp::ParameterValue &declare_parameter(
      const std::string &name,
      rclcpp::ParameterType type,
      const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
          rcl_interfaces::msg::ParameterDescriptor(),
      bool ignore_override = false) = 0;

  /// 取消声明参数 (Undeclare a parameter)
  /**
   * \param[in] name 参数名称 (The name of the parameter)
   * \sa rclcpp::Node::undeclare_parameter
   */
  RCLCPP_PUBLIC
  virtual void undeclare_parameter(const std::string &name) = 0;

  /// 如果参数已声明，则返回 true，否则返回 false (Return true if the parameter has been declared,
  /// otherwise false)
  /**
   * \param[in] name 参数名称 (The name of the parameter)
   * \return 返回布尔值，表示参数是否已声明 (Returns a boolean indicating whether the parameter has
   * been declared) \sa rclcpp::Node::has_parameter
   */
  RCLCPP_PUBLIC
  virtual bool has_parameter(const std::string &name) const = 0;

  /// 一次设置一个或多个参数 (Set one or more parameters, one at a time)
  /**
   * \param[in] parameters 包含参数的向量 (A vector containing the parameters)
   * \return 返回一个包含 SetParametersResult 的向量，表示每个参数的设置结果 (Returns a vector of
   * SetParametersResult representing the result of setting each parameter) \sa
   * rclcpp::Node::set_parameters
   */
  RCLCPP_PUBLIC
  virtual std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
      const std::vector<rclcpp::Parameter> &parameters) = 0;

  /// 一次性设置一个或多个参数 (Set one or more parameters, all at once)
  /**
   * \param[in] parameters 包含参数的向量 (A vector containing the parameters)
   * \return 返回一个 SetParametersResult，表示所有参数的设置结果 (Returns a SetParametersResult
   * representing the result of setting all parameters) \sa rclcpp::Node::set_parameters_atomically
   */
  RCLCPP_PUBLIC
  virtual rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
      const std::vector<rclcpp::Parameter> &parameters) = 0;

  /// 获取给定名称的参数描述。
  /// Get descriptions of parameters given their names.
  /*
   * \param[in] names 要检查的参数名称列表。
   * \param[in] names a list of parameter names to check.
   * \return 找到的参数列表。
   * \return the list of parameters that were found.
   * 未找到的参数将从返回的列表中省略。
   * Any parameter not found is omitted from the returned list.
   */
  RCLCPP_PUBLIC
  virtual std::vector<rclcpp::Parameter> get_parameters(
      const std::vector<std::string> &names) const = 0;

  /// 根据名称获取一个参数的描述。
  /// Get the description of one parameter given a name.
  /*
   * \param[in] name 要查找的参数的名称。
   * \param[in] name the name of the parameter to look for.
   * \return 如果节点上存在该参数，则返回该参数。
   * \return the parameter if it exists on the node.
   * \throws std::out_of_range 如果节点上不存在该参数。
   * \throws std::out_of_range if the parameter does not exist on the node.
   */
  RCLCPP_PUBLIC
  virtual rclcpp::Parameter get_parameter(const std::string &name) const = 0;

  /// 根据名称获取一个参数的描述。
  /// Get the description of one parameter given a name.
  /*
   * \param[in] name 要查找的参数的名称。
   * \param[in] name the name of the parameter to look for.
   * \param[out] parameter 如果节点上存在参数，则为描述。
   * \param[out] parameter the description if parameter exists on the node.
   * \return 如果节点上存在该参数，则返回 true，
   * \return true if the parameter exists on the node, or
   * \return 如果节点上不存在该参数，则返回 false。
   * \return false if the parameter does not exist.
   */
  RCLCPP_PUBLIC
  virtual bool get_parameter(const std::string &name, rclcpp::Parameter &parameter) const = 0;

  /// 将具有指定前缀的所有参数获取到参数映射中。
  /// Get all parameters that have the specified prefix into the parameters map.
  /*
   * \param[in] prefix 要查找的前缀的名称。
   * \param[in] prefix the name of the prefix to look for.
   * \param[out] parameters 匹配前缀的参数映射。
   * \param[out] parameters a map of parameters that matched the prefix.
   * \return 如果节点上存在任何带有前缀的参数，则返回 true，
   * \return true if any parameters with the prefix exists on the node, or
   * \return 否则返回 false。
   * \return false otherwise.
   */
  RCLCPP_PUBLIC
  virtual bool get_parameters_by_prefix(
      const std::string &prefix, std::map<std::string, rclcpp::Parameter> &parameters) const = 0;

  /**
   * @brief 描述参数的虚拟函数 (Virtual function for describing parameters)
   * @param names 参数名字列表 (List of parameter names)
   * @return std::vector<rcl_interfaces::msg::ParameterDescriptor> 参数描述符向量 (Vector of
   * parameter descriptors)
   */
  RCLCPP_PUBLIC
  virtual std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
      const std::vector<std::string> &names) const = 0;

  /**
   * @brief 获取参数类型的虚拟函数 (Virtual function for getting parameter types)
   * @param names 参数名字列表 (List of parameter names)
   * @return std::vector<uint8_t> 参数类型向量 (Vector of parameter types)
   */
  RCLCPP_PUBLIC
  virtual std::vector<uint8_t> get_parameter_types(const std::vector<std::string> &names) const = 0;

  /**
   * @brief 列出参数的虚拟函数 (Virtual function for listing parameters)
   * @param prefixes 参数前缀列表 (List of parameter prefixes)
   * @param depth 搜索深度 (Search depth)
   * @return rcl_interfaces::msg::ListParametersResult 列出参数结果 (List parameters result)
   */
  RCLCPP_PUBLIC
  virtual rcl_interfaces::msg::ListParametersResult list_parameters(
      const std::vector<std::string> &prefixes, uint64_t depth) const = 0;

  // 定义 OnSetParametersCallbackType 类型 (Define the OnSetParametersCallbackType type)
  using OnSetParametersCallbackType = OnSetParametersCallbackHandle::OnSetParametersCallbackType;

  // 定义 PostSetParametersCallbackType 类型 (Define the PostSetParametersCallbackType type)
  using PostSetParametersCallbackType =
      PostSetParametersCallbackHandle::PostSetParametersCallbackType;

  // 定义 PreSetParametersCallbackType 类型 (Define the PreSetParametersCallbackType type)
  using PreSetParametersCallbackType = PreSetParametersCallbackHandle::PreSetParametersCallbackType;

  /// 添加一个在参数验证之前触发的回调函数。
  /// Add a callback that gets triggered before parameters are validated.
  /**
   * \param[in] callback 预设参数回调类型的回调函数。
   * \param[in] callback Callback function of type PreSetParametersCallbackType.
   * \sa rclcpp::Node::add_pre_set_parameters_callback
   */
  RCLCPP_PUBLIC
  virtual PreSetParametersCallbackHandle::SharedPtr add_pre_set_parameters_callback(
      PreSetParametersCallbackType callback) = 0;

  /// 在设置参数之前添加一个验证参数的回调函数。
  /// Add a callback to validate parameters before they are set.
  /**
   * \param[in] callback 在设置参数回调类型的回调函数。
   * \param[in] callback Callback function of type OnSetParametersCallbackType.
   * \sa rclcpp::Node::add_on_set_parameters_callback
   */
  RCLCPP_PUBLIC
  virtual OnSetParametersCallbackHandle::SharedPtr add_on_set_parameters_callback(
      OnSetParametersCallbackType callback) = 0;

  /// 添加一个在成功设置参数后触发的回调函数。
  /// Add a callback that gets triggered after parameters are set successfully.
  /**
   * \param[in] callback 后置参数回调类型的回调函数。
   * \param[in] callback Callback function of type PostSetParametersCallbackType.
   * \sa rclcpp::Node::add_post_set_parameters_callback
   */
  RCLCPP_PUBLIC
  virtual PostSetParametersCallbackHandle::SharedPtr add_post_set_parameters_callback(
      PostSetParametersCallbackType callback) = 0;

  /// 删除使用 `add_pre_set_parameters_callback` 注册的回调函数。
  /// Remove a callback registered with `add_pre_set_parameters_callback`.
  /**
   * \param[in] handler 预设参数回调句柄指针。
   * \param[in] handler Pointer to the PreSetParametersCallbackHandle.
   * \sa rclcpp::Node::remove_pre_set_parameters_callback
   */
  RCLCPP_PUBLIC
  virtual void remove_pre_set_parameters_callback(
      const PreSetParametersCallbackHandle *const handler) = 0;

  /// 删除使用 `add_on_set_parameters_callback` 注册的回调函数。
  /// Remove a callback registered with `add_on_set_parameters_callback`.
  /**
   * \param[in] handler 在设置参数回调句柄指针。
   * \param[in] handler Pointer to the OnSetParametersCallbackHandle.
   * \sa rclcpp::Node::remove_on_set_parameters_callback
   */
  RCLCPP_PUBLIC
  virtual void remove_on_set_parameters_callback(
      const OnSetParametersCallbackHandle *const handler) = 0;

  /// 删除使用 `add_post_set_parameters_callback` 注册的回调函数。
  /// Remove a callback registered with `add_post_set_parameters_callback`.
  /**
   * \sa rclcpp::Node::remove_post_set_parameters_callback
   */
  RCLCPP_PUBLIC
  virtual void
  // 参数列表：handler - 一个指向 PostSetParametersCallbackHandle
  // 类型的常量指针，用于存储要删除的回调函数句柄。 Parameter list: handler - A constant pointer to
  // the PostSetParametersCallbackHandle type, used to store the handle of the callback to be
  // removed.
  remove_post_set_parameters_callback(const PostSetParametersCallbackHandle *const handler) = 0;

  /// 返回 NodeParameters 使用的初始参数值，以覆盖默认值。
  /// Return the initial parameter values used by the NodeParameters to override default values.
  RCLCPP_PUBLIC
  virtual const std::map<std::string, rclcpp::ParameterValue> &
  // 获取参数覆盖值的函数，返回一个包含参数名称和参数值的映射表的引用。
  // Function to get parameter overrides, returns a reference to a map containing parameter names
  // and parameter values.
  get_parameter_overrides() const = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeParametersInterface, parameters)

#endif  // RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_INTERFACE_HPP_

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

#ifndef RCLCPP__PARAMETER_CLIENT_HPP_
#define RCLCPP__PARAMETER_CLIENT_HPP_

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/srv/get_parameter_types.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_yaml_param_parser/parser.h"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"

namespace rclcpp {

class AsyncParametersClient {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(AsyncParametersClient)

  /// 创建一个异步参数客户端（Create an async parameters client）
  /**
   * \param[in] node_base_interface 对应节点的节点基本接口（The node base interface of the
   * corresponding node） \param[in] node_topics_interface 节点话题基本接口（Node topic base
   * interface） \param[in] node_graph_interface 对应节点的节点图接口（The node graph interface of
   * the corresponding node） \param[in] node_services_interface 节点服务接口（Node service
   * interface） \param[in] remote_node_name 远程节点名称（Name of the remote node） \param[in]
   * qos_profile 用于订阅的 rmw QoS 配置文件（The rmw qos profile to use to subscribe） \param[in]
   * group (可选) 异步参数客户端将添加到此回调组（The async parameter client will be added to this
   * callback group; optional） \deprecated 使用 rclcpp::QoS 代替 rmw_qos_profile_t（use rclcpp::QoS
   * instead of rmw_qos_profile_t）
   */
  [[deprecated("use rclcpp::QoS instead of rmw_qos_profile_t")]] RCLCPP_PUBLIC
  AsyncParametersClient(
      const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
      const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
      const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
      const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
      const std::string& remote_node_name,
      const rmw_qos_profile_t& qos_profile,
      rclcpp::CallbackGroup::SharedPtr group = nullptr)
      : AsyncParametersClient(
            node_base_interface,
            node_topics_interface,
            node_graph_interface,
            node_services_interface,
            remote_node_name,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile)),
            group) {}

  /// 创建一个异步参数客户端。 (Create an async parameters client.)
  /**
   * \param[in] node_base_interface 对应节点的节点基础接口。 (The node base interface of the
   * corresponding node.) \param[in] node_topics_interface 节点主题基础接口。 (Node topic base
   * interface.) \param[in] node_graph_interface 对应节点的节点图接口。 (The node graph interface of
   * the corresponding node.) \param[in] node_services_interface 节点服务接口。 (Node service
   * interface.) \param[in] remote_node_name (可选) 远程节点的名称。 (name of the remote node,
   * optional) \param[in] qos_profile (可选) 用于订阅的qos配置文件。 (The qos profile to use to
   * subscribe, optional) \param[in] group (可选) 异步参数客户端将添加到此回调组中。 (The async
   * parameter client will be added to this callback group, optional)
   */
  RCLCPP_PUBLIC
  AsyncParametersClient(
      const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
      const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
      const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
      const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
      const std::string& remote_node_name = "",
      const rclcpp::QoS& qos_profile = rclcpp::ParametersQoS(),
      rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// 构造函数 (Constructor)
  /**
   * \param[in] node 异步参数客户端将添加到此节点。 (The async parameters client will be added to
   * this node.) \param[in] remote_node_name 远程节点的名称。 (name of the remote node) \param[in]
   * qos_profile 用于订阅的rmw qos配置文件。 (The rmw qos profile to use to subscribe) \param[in]
   * group (可选) 异步参数客户端将添加到此回调组中。 (The async parameter client will be added to
   * this callback group, optional) \deprecated 使用rclcpp::QoS代替rmw_qos_profile_t。 (use
   * rclcpp::QoS instead of rmw_qos_profile_t)
   */
  template <typename NodeT>
  [[deprecated("use rclcpp::QoS instead of rmw_qos_profile_t")]] AsyncParametersClient(
      const std::shared_ptr<NodeT> node,
      const std::string& remote_node_name,
      const rmw_qos_profile_t& qos_profile,
      rclcpp::CallbackGroup::SharedPtr group = nullptr)
      : AsyncParametersClient(
            node->get_node_base_interface(),
            node->get_node_topics_interface(),
            node->get_node_graph_interface(),
            node->get_node_services_interface(),
            remote_node_name,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile)),
            group) {}

  /**
   * \brief 构造函数
   * \param[in] node 异步参数客户端将添加到此节点。
   * \param[in] remote_node_name (可选) 远程节点的名称
   * \param[in] qos_profile (可选) 用于订阅的服务质量配置文件
   * \param[in] group (可选) 异步参数客户端将添加到此回调组。
   *
   * \brief Constructor
   * \param[in] node The async parameters client will be added to this node.
   * \param[in] remote_node_name (optional) name of the remote node
   * \param[in] qos_profile (optional) The qos profile to use to subscribe
   * \param[in] group (optional) The async parameter client will be added to this callback group.
   */
  template <typename NodeT>
  explicit AsyncParametersClient(
      const std::shared_ptr<NodeT> node,  ///< [in] 要添加异步参数客户端的节点 / The node to add the
                                          ///< async parameters client to
      const std::string& remote_node_name =
          "",  ///< [in] (可选) 远程节点的名称 / (optional) Name of the remote node
      const rclcpp::QoS& qos_profile =
          rclcpp::ParametersQoS(),  ///< [in] (可选) 用于订阅的服务质量配置文件 / (optional) The qos
                                    ///< profile to use to subscribe
      rclcpp::CallbackGroup::SharedPtr group =
          nullptr)  ///< [in] (可选) 异步参数客户端将添加到此回调组 / (optional) The async parameter
                    ///< client will be added to this callback group
      : AsyncParametersClient(
            node->get_node_base_interface(),
            node->get_node_topics_interface(),
            node->get_node_graph_interface(),
            node->get_node_services_interface(),
            remote_node_name,
            qos_profile,
            group) {}

  /**
   * \brief 构造函数
   * \param[in] node 异步参数客户端将添加到此节点。
   * \param[in] remote_node_name 远程节点的名称
   * \param[in] qos_profile 用于订阅的 rmw 服务质量配置文件
   * \param[in] group (可选) 异步参数客户端将添加到此回调组。
   * \deprecated 使用 rclcpp::QoS 代替 rmw_qos_profile_t
   *
   * \brief Constructor
   * \param[in] node The async parameters client will be added to this node.
   * \param[in] remote_node_name Name of the remote node
   * \param[in] qos_profile The rmw qos profile to use to subscribe
   * \param[in] group (optional) The async parameter client will be added to this callback group.
   * \deprecated use rclcpp::QoS instead of rmw_qos_profile_t
   */
  template <typename NodeT>
  [[deprecated("use rclcpp::QoS instead of rmw_qos_profile_t")]] AsyncParametersClient(
      NodeT* node,  ///< [in] 要添加异步参数客户端的节点 / The node to add the async parameters
                    ///< client to
      const std::string& remote_node_name,  ///< [in] 远程节点的名称 / Name of the remote node
      const rmw_qos_profile_t& qos_profile,  ///< [in] 用于订阅的 rmw 服务质量配置文件 / The rmw qos
                                             ///< profile to use to subscribe
      rclcpp::CallbackGroup::SharedPtr group =
          nullptr)  ///< [in] (可选) 异步参数客户端将添加到此回调组 / (optional) The async parameter
                    ///< client will be added to this callback group
      : AsyncParametersClient(
            node->get_node_base_interface(),
            node->get_node_topics_interface(),
            node->get_node_graph_interface(),
            node->get_node_services_interface(),
            remote_node_name,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile)),
            group) {}

  /**
   * \brief 构造函数，用于创建一个异步参数客户端 (Constructor for creating an AsyncParametersClient)
   *
   * \param[in] node 异步参数客户端将添加到此节点 (The async parameters client will be added to this
   * node) \param[in] remote_node_name (可选) 远程节点的名称 (optional) name of the remote node
   * \param[in] qos_profile (可选) 用于订阅的服务质量配置文件 (optional) The qos profile to use to
   * subscribe \param[in] group (可选) 异步参数客户端将添加到此回调组 (optional) The async parameter
   * client will be added to this callback group
   */
  template <typename NodeT>
  explicit AsyncParametersClient(
      NodeT* node,  // 节点指针，将在此节点上添加异步参数客户端 (Node pointer, the async parameters
                    // client will be added to this node)
      const std::string& remote_node_name =
          "",  // 远程节点的名称，默认为空字符串 (Name of the remote node, default is empty string)
      const rclcpp::QoS& qos_profile =
          rclcpp::ParametersQoS(),  // 服务质量配置文件，默认为 rclcpp::ParametersQoS() (QoS
                                    // profile, default is rclcpp::ParametersQoS())
      rclcpp::CallbackGroup::SharedPtr group =
          nullptr)  // 回调组共享指针，默认为空指针 (Callback group shared pointer, default is
                    // nullptr)
      : AsyncParametersClient(  // 调用另一个构造函数 (Calling another constructor)
            node->get_node_base_interface(),  // 获取节点基本接口 (Get the node base interface)
            node->get_node_topics_interface(),  // 获取节点主题接口 (Get the node topics interface)
            node->get_node_graph_interface(),  // 获取节点图形接口 (Get the node graph interface)
            node->get_node_services_interface(),  // 获取节点服务接口 (Get the node services
                                                  // interface)
            remote_node_name,                     // 远程节点的名称 (Name of the remote node)
            qos_profile,                          // 服务质量配置文件 (QoS profile)
            group)                                // 回调组 (Callback group)
  {}

  /// 获取参数值
  /// Get parameter values
  /**
   * \param[in] names 参数名列表
   * \param[in] callback 用于处理结果的回调函数，默认为 nullptr
   * \return 返回一个 std::shared_future 对象，包含一组 rclcpp::Parameter 对象
   *
   * \param[in] names A list of parameter names
   * \param[in] callback A callback function to handle the result, default is nullptr
   * \return Returns a std::shared_future object containing a vector of rclcpp::Parameter objects
   */
  RCLCPP_PUBLIC
  std::shared_future<std::vector<rclcpp::Parameter>> get_parameters(
      const std::vector<std::string>& names,
      std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback = nullptr);

  /// 描述参数
  /// Describe parameters
  /**
   * \param[in] names 参数名列表
   * \param[in] callback 用于处理结果的回调函数，默认为 nullptr
   * \return 返回一个 std::shared_future 对象，包含一组 rcl_interfaces::msg::ParameterDescriptor
   * 对象
   *
   * \param[in] names A list of parameter names
   * \param[in] callback A callback function to handle the result, default is nullptr
   * \return Returns a std::shared_future object containing a vector of
   * rcl_interfaces::msg::ParameterDescriptor objects
   */
  RCLCPP_PUBLIC
  std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> describe_parameters(
      const std::vector<std::string>& names,
      std::function<void(std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>>)>
          callback = nullptr);

  /// 获取参数类型
  /// Get parameter types
  /**
   * \param[in] names 参数名列表
   * \param[in] callback 用于处理结果的回调函数，默认为 nullptr
   * \return 返回一个 std::shared_future 对象，包含一组 rclcpp::ParameterType 对象
   *
   * \param[in] names A list of parameter names
   * \param[in] callback A callback function to handle the result, default is nullptr
   * \return Returns a std::shared_future object containing a vector of rclcpp::ParameterType
   * objects
   */
  RCLCPP_PUBLIC
  std::shared_future<std::vector<rclcpp::ParameterType>> get_parameter_types(
      const std::vector<std::string>& names,
      std::function<void(std::shared_future<std::vector<rclcpp::ParameterType>>)> callback =
          nullptr);

  /// 设置参数值
  /// Set parameter values
  /**
   * \param[in] parameters 一个包含要设置的参数的列表
   * \param[in] callback 用于处理结果的回调函数，默认为 nullptr
   * \return 返回一个 std::shared_future 对象，包含一组 rcl_interfaces::msg::SetParametersResult
   * 对象
   *
   * \param[in] parameters A list of parameters to be set
   * \param[in] callback A callback function to handle the result, default is nullptr
   * \return Returns a std::shared_future object containing a vector of
   * rcl_interfaces::msg::SetParametersResult objects
   */
  RCLCPP_PUBLIC
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> set_parameters(
      const std::vector<rclcpp::Parameter>& parameters,
      std::function<void(std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>)>
          callback = nullptr);

  /// 原子性地设置参数值
  /// Set parameter values atomically
  /**
   * \param[in] parameters 一个包含要设置的参数的列表
   * \param[in] callback 用于处理结果的回调函数，默认为 nullptr
   * \return 返回一个 std::shared_future 对象，包含一个 rcl_interfaces::msg::SetParametersResult
   * 对象
   *
   * \param[in] parameters A list of parameters to be set
   * \param[in] callback A callback function to handle the result, default is nullptr
   * \return Returns a std::shared_future object containing a single
   * rcl_interfaces::msg::SetParametersResult object
   */
  RCLCPP_PUBLIC
  std::shared_future<rcl_interfaces::msg::SetParametersResult> set_parameters_atomically(
      const std::vector<rclcpp::Parameter>& parameters,
      std::function<void(std::shared_future<rcl_interfaces::msg::SetParametersResult>)> callback =
          nullptr);

  /// 一次删除多个参数。
  /// Delete several parameters at once.
  /**
   * 此函数的行为类似于命令行工具 `ros2 param delete`。
   * This function behaves like command-line tool `ros2 param delete` would.
   *
   * \param parameters_names 参数名的向量
   * \param parameters_names vector of parameters names
   * \return 用于删除参数的 set_parameter 服务的 future 对象
   * \return the future of the set_parameter service used to delete the parameters
   */
  RCLCPP_PUBLIC
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> delete_parameters(
      const std::vector<std::string>& parameters_names);

  /// 从 yaml 文件加载参数。
  /// Load parameters from yaml file.
  /**
   * 此函数的行为类似于命令行工具 `ros2 param load`。
   * This function behaves like command-line tool `ros2 param load` would.
   *
   * \param yaml_filename yaml 文件的完整名称
   * \param yaml_filename the full name of the yaml file
   * \return 用于加载参数的 set_parameter 服务的 future 对象
   * \return the future of the set_parameter service used to load the parameters
   */
  RCLCPP_PUBLIC
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> load_parameters(
      const std::string& yaml_filename);

  /// 从参数映射中加载参数。
  /// Load parameters from parameter map.
  /**
   * 此函数根据节点名称过滤要设置的参数。
   * This function filters the parameters to be set based on the node name.
   *
   * 如果一个 FQN 中的节点名称存在两个重复的键，则不能保证哪一个会被设置。
   * If two duplicate keys exist in node names belongs to one FQN, there is no guarantee
   * which one could be set.
   *
   * \param parameter_map 要加载的命名参数
   * \param parameter_map named parameters to be loaded
   * \return 用于加载参数的 set_parameter 服务的 future 对象
   * \return the future of the set_parameter service used to load the parameters
   * \throw InvalidParametersException 如果没有要设置的参数
   * \throw InvalidParametersException if there is no parameter to set
   */
  RCLCPP_PUBLIC
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> load_parameters(
      const rclcpp::ParameterMap& parameter_map);

  /**
   * @brief 列出参数
   * @param prefixes 参数前缀列表
   * @param depth 搜索深度
   * @param callback 回调函数，默认为空
   * @return std::shared_future<rcl_interfaces::msg::ListParametersResult>
   * 包含参数列表的共享future对象
   *
   * @brief List parameters
   * @param prefixes List of parameter prefixes
   * @param depth Search depth
   * @param callback Callback function, default is nullptr
   * @return std::shared_future<rcl_interfaces::msg::ListParametersResult> Shared future object
   * containing the list of parameters
   */
  RCLCPP_PUBLIC
  std::shared_future<rcl_interfaces::msg::ListParametersResult> list_parameters(
      const std::vector<std::string>& prefixes,
      uint64_t depth,
      std::function<void(std::shared_future<rcl_interfaces::msg::ListParametersResult>)> callback =
          nullptr);

  /**
   * @brief 订阅参数事件
   * @tparam CallbackT 回调类型
   * @tparam AllocatorT 分配器类型，默认为std::allocator<void>
   * @param callback 回调函数
   * @param qos 服务质量配置，默认为rclcpp::ParameterEventsQoS()
   * @param options 订阅选项，默认为rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()
   * @return rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
   * 指向订阅对象的共享指针
   *
   * @brief Subscribe to parameter events
   * @tparam CallbackT Callback type
   * @tparam AllocatorT Allocator type, default is std::allocator<void>
   * @param callback Callback function
   * @param qos Quality of service configuration, default is rclcpp::ParameterEventsQoS()
   * @param options Subscription options, default is
   * rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()
   * @return rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr Shared pointer to
   * the subscription object
   */
  template <typename CallbackT, typename AllocatorT = std::allocator<void>>
  typename rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr on_parameter_event(
      CallbackT&& callback,
      const rclcpp::QoS& qos = rclcpp::ParameterEventsQoS(),
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options =
          (rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>())) {
    return this->on_parameter_event(this->node_topics_interface_, callback, qos, options);
  }

  /**
   * @brief NodeT类型只需要有一个名为get_node_topics_interface()的方法，
   *        它返回一个指向NodeTopicsInterface的shared_ptr，或者本身就是一个NodeTopicsInterface指针。
   *
   * @brief The NodeT type only needs to have a method called get_node_topics_interface(),
   *        which returns a shared_ptr to a NodeTopicsInterface, or be a
   *        NodeTopicsInterface pointer itself.
   */
  template <typename CallbackT, typename NodeT, typename AllocatorT = std::allocator<void>>
  static typename rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
  on_parameter_event(
      NodeT&& node,
      CallbackT&& callback,
      const rclcpp::QoS& qos = rclcpp::ParameterEventsQoS(),
      const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options =
          (rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>())) {
    // 使用给定的节点、参数事件主题、服务质量配置、回调函数和订阅选项创建订阅
    // Create a subscription with the given node, parameter events topic, QoS configuration,
    // callback function, and subscription options
    return rclcpp::create_subscription<rcl_interfaces::msg::ParameterEvent>(
        node, "/parameter_events", qos, std::forward<CallbackT>(callback), options);
  }

  /// 返回参数服务是否准备好。 (Return if the parameter services are ready.)
  /**
   * 此方法检查以下服务： (This method checks the following services:)
   *  - 获取参数 (get parameter)
   *  - 获取参数 (get parameter)
   *  - 设置参数 (set parameters)
   *  - 列出参数 (list parameters)
   *  - 描述参数 (describe parameters)
   *
   * \return 如果服务准备好，则返回 `true`，否则返回 `false` (`true` if the service is ready,
   * `false` otherwise)
   */
  RCLCPP_PUBLIC
  bool service_is_ready() const;

  /// 等待服务准备好。 (Wait for the services to be ready.)
  /**
   * \param[in] timeout 最长等待时间 (maximum time to wait)
   * \return 如果服务准备好且未超时，则返回 `true`，否则返回 `false` (`true` if the services are
   * ready and the timeout is not over, `false` otherwise)
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
      // 超时时间，默认值为 -1，表示无限期等待 (Timeout duration, default value is -1 which means
      // waiting indefinitely)
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    // 将超时时间转换为纳秒，并调用 wait_for_service_nanoseconds 函数 (Convert the timeout duration
    // to nanoseconds and call the wait_for_service_nanoseconds function)
    return wait_for_service_nanoseconds(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

protected:
  /**
   * @brief 等待服务的响应，直到超时。Wait for the service response until timeout.
   *
   * @param[in] timeout 超时时间（以纳秒为单位）。Timeout duration in nanoseconds.
   * @return 如果在超时之前服务可用，则返回 true；否则返回 false。Returns true if the service is
   * available before the timeout, otherwise returns false.
   */
  RCLCPP_PUBLIC
  bool wait_for_service_nanoseconds(std::chrono::nanoseconds timeout);

private:
  /*!
   * \brief 参数列表说明 (Parameter list description)
   * \param node_topics_interface_ 节点主题接口共享指针 (Shared pointer of node topics interface)
   * \param get_parameters_client_ 获取参数服务客户端共享指针 (Shared pointer of get parameters
   * service client) \param get_parameter_types_client_ 获取参数类型服务客户端共享指针 (Shared
   * pointer of get parameter types service client) \param set_parameters_client_
   * 设置参数服务客户端共享指针 (Shared pointer of set parameters service client) \param
   * set_parameters_atomically_client_ 原子性设置参数服务客户端共享指针 (Shared pointer of set
   * parameters atomically service client) \param list_parameters_client_ 列出参数服务客户端共享指针
   * (Shared pointer of list parameters service client) \param describe_parameters_client_
   * 描述参数服务客户端共享指针 (Shared pointer of describe parameters service client) \param
   * remote_node_name_ 远程节点名称字符串 (String of remote node name)
   */
  // 节点主题接口共享指针 (Shared pointer of node topics interface)
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface_;
  // 获取参数服务客户端共享指针 (Shared pointer of get parameters service client)
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
  // 获取参数类型服务客户端共享指针 (Shared pointer of get parameter types service client)
  rclcpp::Client<rcl_interfaces::srv::GetParameterTypes>::SharedPtr get_parameter_types_client_;
  // 设置参数服务客户端共享指针 (Shared pointer of set parameters service client)
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_client_;
  // 原子性设置参数服务客户端共享指针 (Shared pointer of set parameters atomically service client)
  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr
      set_parameters_atomically_client_;
  // 列出参数服务客户端共享指针 (Shared pointer of list parameters service client)
  rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_client_;
  // 描述参数服务客户端共享指针 (Shared pointer of describe parameters service client)
  rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_client_;
  // 远程节点名称字符串 (String of remote node name)
  std::string remote_node_name_;
};

class SyncParametersClient {
public:
  /**
   * @brief RCLCPP_SMART_PTR_DEFINITIONS(SyncParametersClient) 定义智能指针类型
   *        (Defines smart pointer types for SyncParametersClient)
   */
  RCLCPP_SMART_PTR_DEFINITIONS(SyncParametersClient)

  /**
   * @brief 构造函数，已弃用，请使用 rclcpp::QoS 替代 rmw_qos_profile_t
   *        (Constructor, deprecated, use rclcpp::QoS instead of rmw_qos_profile_t)
   *
   * @tparam NodeT 节点类型 (Node type)
   * @param node 共享指针类型的节点 (Shared pointer type node)
   * @param remote_node_name 远程节点名称 (Remote node name)
   * @param qos_profile QoS 配置 (QoS profile configuration)
   */
  template <typename NodeT>
  [[deprecated("use rclcpp::QoS instead of rmw_qos_profile_t")]] SyncParametersClient(
      std::shared_ptr<NodeT> node,
      const std::string& remote_node_name,
      const rmw_qos_profile_t& qos_profile)
      : SyncParametersClient(
            std::make_shared<rclcpp::executors::SingleThreadedExecutor>(),
            node,
            remote_node_name,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile))) {}

  /**
   * @brief 显式构造函数 (Explicit constructor)
   *
   * @tparam NodeT 节点类型 (Node type)
   * @param node 共享指针类型的节点 (Shared pointer type node)
   * @param remote_node_name 远程节点名称，默认为空字符串 (Remote node name, default to empty
   * string)
   * @param qos_profile QoS 配置，默认为 rclcpp::ParametersQoS() (QoS profile configuration, default
   * to rclcpp::ParametersQoS())
   */
  template <typename NodeT>
  explicit SyncParametersClient(
      std::shared_ptr<NodeT> node,
      const std::string& remote_node_name = "",
      const rclcpp::QoS& qos_profile = rclcpp::ParametersQoS())
      : SyncParametersClient(
            std::make_shared<rclcpp::executors::SingleThreadedExecutor>(),
            node,
            remote_node_name,
            qos_profile) {}

  /**
   * @brief 构造函数，已弃用，请使用 rclcpp::QoS 替代 rmw_qos_profile_t
   *        (Constructor, deprecated, use rclcpp::QoS instead of rmw_qos_profile_t)
   *
   * @tparam NodeT 节点类型 (Node type)
   * @param executor 执行器的共享指针 (Shared pointer of the executor)
   * @param node 共享指针类型的节点 (Shared pointer type node)
   * @param remote_node_name 远程节点名称 (Remote node name)
   * @param qos_profile QoS 配置 (QoS profile configuration)
   */
  template <typename NodeT>
  [[deprecated("use rclcpp::QoS instead of rmw_qos_profile_t")]] SyncParametersClient(
      rclcpp::Executor::SharedPtr executor,
      std::shared_ptr<NodeT> node,
      const std::string& remote_node_name,
      const rmw_qos_profile_t& qos_profile)
      : SyncParametersClient(
            executor,
            node->get_node_base_interface(),
            node->get_node_topics_interface(),
            node->get_node_graph_interface(),
            node->get_node_services_interface(),
            remote_node_name,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile))) {}

  /**
   * \brief 同步参数客户端构造函数（SyncParametersClient constructor）
   * \tparam NodeT 节点类型（Node type）
   * \param[in] executor 执行器共享指针（Shared pointer to the executor）
   * \param[in] node 节点共享指针（Shared pointer to the node）
   * \param[in] remote_node_name 远程节点名称，默认为空字符串（Remote node name, default is an empty
   * string） \param[in] qos_profile QoS配置，默认为rclcpp::ParametersQoS()（QoS profile, default is
   * rclcpp::ParametersQoS()）
   */
  template <typename NodeT>
  SyncParametersClient(
      rclcpp::Executor::SharedPtr executor,
      std::shared_ptr<NodeT> node,
      const std::string& remote_node_name = "",
      const rclcpp::QoS& qos_profile = rclcpp::ParametersQoS())
      : SyncParametersClient(  // 使用构造函数委托（Use constructor delegation）
            executor,
            node->get_node_base_interface(),
            node->get_node_topics_interface(),
            node->get_node_graph_interface(),
            node->get_node_services_interface(),
            remote_node_name,
            qos_profile) {}

  /**
   * \brief [[deprecated]] 同步参数客户端构造函数（SyncParametersClient constructor）
   * \tparam NodeT 节点类型（Node type）
   * \param[in] node 节点指针（Pointer to the node）
   * \param[in] remote_node_name 远程节点名称（Remote node name）
   * \param[in] qos_profile rmw_qos_profile_t 类型的QoS配置（QoS profile of type rmw_qos_profile_t）
   */
  template <typename NodeT>
  [[deprecated("use rclcpp::QoS instead of rmw_qos_profile_t")]] SyncParametersClient(
      NodeT* node, const std::string& remote_node_name, const rmw_qos_profile_t& qos_profile)
      : SyncParametersClient(  // 使用构造函数委托（Use constructor delegation）
            std::make_shared<rclcpp::executors::SingleThreadedExecutor>(),
            node,
            remote_node_name,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile))) {}

  /**
   * \brief 显式同步参数客户端构造函数（Explicit SyncParametersClient constructor）
   * \tparam NodeT 节点类型（Node type）
   * \param[in] node 节点指针（Pointer to the node）
   * \param[in] remote_node_name 远程节点名称，默认为空字符串（Remote node name, default is an empty
   * string） \param[in] qos_profile QoS配置，默认为rclcpp::ParametersQoS()（QoS profile, default is
   * rclcpp::ParametersQoS()）
   */
  template <typename NodeT>
  explicit SyncParametersClient(
      NodeT* node,
      const std::string& remote_node_name = "",
      const rclcpp::QoS& qos_profile = rclcpp::ParametersQoS())
      : SyncParametersClient(  // 使用构造函数委托（Use constructor delegation）
            std::make_shared<rclcpp::executors::SingleThreadedExecutor>(),
            node,
            remote_node_name,
            qos_profile) {}

  /**
   * @brief 一个同步参数客户端的构造函数，使用 `rmw_qos_profile_t` 类型的 QoS 配置。 (A constructor
   * for a SyncParametersClient using `rmw_qos_profile_t` type of QoS configuration.)
   *
   * @tparam NodeT 节点类型 (Node type)
   * @param[in] executor 执行器共享指针 (Executor shared pointer)
   * @param[in] node 节点指针 (Node pointer)
   * @param[in] remote_node_name 远程节点名称 (Remote node name)
   * @param[in] qos_profile QoS 配置 (QoS configuration)
   *
   * @deprecated 使用 rclcpp::QoS 代替 rmw_qos_profile_t (Use rclcpp::QoS instead of
   * rmw_qos_profile_t)
   */
  template <typename NodeT>
  [[deprecated("use rclcpp::QoS instead of rmw_qos_profile_t")]] SyncParametersClient(
      rclcpp::Executor::SharedPtr executor,
      NodeT* node,
      const std::string& remote_node_name,
      const rmw_qos_profile_t& qos_profile)
      : SyncParametersClient(
            executor,
            node->get_node_base_interface(),
            node->get_node_topics_interface(),
            node->get_node_graph_interface(),
            node->get_node_services_interface(),
            remote_node_name,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile))) {}

  /**
   * @brief 一个同步参数客户端的构造函数。 (A constructor for a SyncParametersClient.)
   *
   * @tparam NodeT 节点类型 (Node type)
   * @param[in] executor 执行器共享指针 (Executor shared pointer)
   * @param[in] node 节点指针 (Node pointer)
   * @param[in] remote_node_name 远程节点名称 (Remote node name)
   * @param[in] qos_profile QoS 配置 (QoS configuration)
   */
  template <typename NodeT>
  SyncParametersClient(
      rclcpp::Executor::SharedPtr executor,
      NodeT* node,
      const std::string& remote_node_name = "",
      const rclcpp::QoS& qos_profile = rclcpp::ParametersQoS())
      : SyncParametersClient(
            executor,
            node->get_node_base_interface(),
            node->get_node_topics_interface(),
            node->get_node_graph_interface(),
            node->get_node_services_interface(),
            remote_node_name,
            qos_profile) {}

  /**
   * @brief 一个同步参数客户端的构造函数，使用 `rmw_qos_profile_t` 类型的 QoS 配置。 (A constructor
   * for a SyncParametersClient using `rmw_qos_profile_t` type of QoS configuration.)
   *
   * @param[in] executor 执行器共享指针 (Executor shared pointer)
   * @param[in] node_base_interface 节点基础接口共享指针 (Node base interface shared pointer)
   * @param[in] node_topics_interface 节点主题接口共享指针 (Node topics interface shared pointer)
   * @param[in] node_graph_interface 节点图形接口共享指针 (Node graph interface shared pointer)
   * @param[in] node_services_interface 节点服务接口共享指针 (Node services interface shared
   * pointer)
   * @param[in] remote_node_name 远程节点名称 (Remote node name)
   * @param[in] qos_profile QoS 配置 (QoS configuration)
   *
   * @deprecated 使用 rclcpp::QoS 代替 rmw_qos_profile_t (Use rclcpp::QoS instead of
   * rmw_qos_profile_t)
   */
  [[deprecated("use rclcpp::QoS instead of rmw_qos_profile_t")]] RCLCPP_PUBLIC SyncParametersClient(
      rclcpp::Executor::SharedPtr executor,
      const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
      const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
      const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
      const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
      const std::string& remote_node_name,
      const rmw_qos_profile_t& qos_profile)
      : executor_(executor), node_base_interface_(node_base_interface) {
    async_parameters_client_ = std::make_shared<AsyncParametersClient>(
        node_base_interface, node_topics_interface, node_graph_interface, node_services_interface,
        remote_node_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile)));
  }

  /**
   * @brief 一个同步参数客户端的构造函数。 (A constructor for a SyncParametersClient.)
   *
   * @param[in] executor 执行器共享指针 (Executor shared pointer)
   * @param[in] node_base_interface 节点基础接口共享指针 (Node base interface shared pointer)
   * @param[in] node_topics_interface 节点主题接口共享指针 (Node topics interface shared pointer)
   * @param[in] node_graph_interface 节点图形接口共享指针 (Node graph interface shared pointer)
   * @param[in] node_services_interface 节点服务接口共享指针 (Node services interface shared
   * pointer)
   * @param[in] remote_node_name 远程节点名称 (Remote node name)
   * @param[in] qos_profile QoS 配置 (QoS configuration)
   */
  RCLCPP_PUBLIC
  SyncParametersClient(
      rclcpp::Executor::SharedPtr executor,
      const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
      const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
      const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
      const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
      const std::string& remote_node_name = "",
      const rclcpp::QoS& qos_profile = rclcpp::ParametersQoS())
      : executor_(executor), node_base_interface_(node_base_interface) {
    async_parameters_client_ = std::make_shared<AsyncParametersClient>(
        node_base_interface, node_topics_interface, node_graph_interface, node_services_interface,
        remote_node_name, qos_profile);
  }

  /**
   * @brief 获取一组参数的值
   *        Get the values of a group of parameters.
   *
   * @tparam RepT 时间表示类型，默认为 int64_t
   *              Time representation type, default to int64_t.
   * @tparam RatioT 时间单位，默认为 std::milli
   *                Time unit, default to std::milli.
   * @param parameter_names 参数名称列表
   *                        List of parameter names.
   * @param timeout 超时时间，默认为 -1，表示无限等待
   *                Timeout, default to -1, which means waiting indefinitely.
   * @return std::vector<rclcpp::Parameter> 参数对象列表
   *                                         List of parameter objects.
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  std::vector<rclcpp::Parameter> get_parameters(
      const std::vector<std::string>& parameter_names,
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    // 将超时时间转换为纳秒，并调用另一个 get_parameters 函数
    // Convert the timeout to nanoseconds and call another get_parameters function.
    return get_parameters(
        parameter_names, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  // 判断是否存在指定名称的参数
  // Check if a parameter with the specified name exists.
  RCLCPP_PUBLIC
  bool has_parameter(const std::string& parameter_name);

  /**
   * @brief 获取指定名称的参数值，若不存在，则调用传入的处理函数
   *        Get the value of the parameter with the specified name,
   *        if it does not exist, call the passed-in handler function.
   *
   * @tparam T 参数值类型
   *           Parameter value type.
   * @param parameter_name 参数名称
   *                       Parameter name.
   * @param parameter_not_found_handler 未找到参数时调用的处理函数
   *                                    Handler function to call when the parameter is not found.
   * @return T 参数值
   *           Parameter value.
   */
  template <typename T>
  T get_parameter_impl(
      const std::string& parameter_name, std::function<T()> parameter_not_found_handler) {
    // 创建一个字符串向量，将参数名称添加进去
    // Create a vector of strings and add the parameter name to it.
    std::vector<std::string> names;
    names.push_back(parameter_name);

    // 获取参数列表
    // Get the list of parameters.
    auto vars = get_parameters(names);

    // 如果参数列表为空，或者参数类型为 PARAMETER_NOT_SET，则调用处理函数
    // If the parameter list is empty or the parameter type is PARAMETER_NOT_SET, call the handler
    // function.
    if ((vars.size() != 1) || (vars[0].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)) {
      return parameter_not_found_handler();
    } else {
      // 否则返回参数值
      // Otherwise, return the parameter value.
      return static_cast<T>(vars[0].get_value<T>());
    }
  }

  /**
   * @brief 获取指定名称的参数值，若不存在，则返回默认值
   *        Get the value of the parameter with the specified name,
   *        if it does not exist, return the default value.
   *
   * @tparam T 参数值类型
   *           Parameter value type.
   * @param parameter_name 参数名称
   *                       Parameter name.
   * @param default_value 默认值
   *                      Default value.
   * @return T 参数值
   *           Parameter value.
   */
  template <typename T>
  T get_parameter(const std::string& parameter_name, const T& default_value) {
    // 调用 get_parameter_impl 函数，并传入 lambda 表达式作为处理函数
    // Call the get_parameter_impl function and pass in a lambda expression as the handler function.
    return get_parameter_impl(
        parameter_name, std::function<T()>([&default_value]() -> T { return default_value; }));
  }

  /**
   * @brief 获取指定名称的参数值，若不存在，则抛出异常
   *        Get the value of the parameter with the specified name,
   *        if it does not exist, throw an exception.
   *
   * @tparam T 参数值类型
   *           Parameter value type.
   * @param parameter_name 参数名称
   *                       Parameter name.
   * @return T 参数值
   *           Parameter value.
   */
  template <typename T>
  T get_parameter(const std::string& parameter_name) {
    // 调用 get_parameter_impl 函数，并传入 lambda 表达式作为处理函数
    // Call the get_parameter_impl function and pass in a lambda expression as the handler function.
    return get_parameter_impl(
        parameter_name, std::function<T()>([&parameter_name]() -> T {
          throw std::runtime_error("Parameter '" + parameter_name + "' is not set");
        }));
  }

  /**
   * @brief 描述一组参数
   *        Describe a group of parameters.
   *
   * @tparam RepT 时间表示类型，默认为 int64_t
   *              Time representation type, default to int64_t.
   * @tparam RatioT 时间单位，默认为 std::milli
   *                Time unit, default to std::milli.
   * @param parameter_names 参数名称列表
   *                        List of parameter names.
   * @param timeout 超时时间，默认为 -1，表示无限等待
   *                Timeout, default to -1, which means waiting indefinitely.
   * @return std::vector<rcl_interfaces::msg::ParameterDescriptor> 参数描述对象列表
   *                                                               List of parameter descriptor
   * objects.
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
      const std::vector<std::string>& parameter_names,
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    // 将超时时间转换为纳秒，并调用另一个 describe_parameters 函数
    // Convert the timeout to nanoseconds and call another describe_parameters function.
    return describe_parameters(
        parameter_names, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  /**
   * @brief 获取一组参数的类型
   *        Get the types of a group of parameters.
   *
   * @tparam RepT 时间表示类型，默认为 int64_t
   *              Time representation type, default to int64_t.
   * @tparam RatioT 时间单位，默认为 std::milli
   *                Time unit, default to std::milli.
   * @param parameter_names 参数名称列表
   *                        List of parameter names.
   * @param timeout 超时时间，默认为 -1，表示无限等待
   *                Timeout, default to -1, which means waiting indefinitely.
   * @return std::vector<rclcpp::ParameterType> 参数类型列表
   *                                             List of parameter types.
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  std::vector<rclcpp::ParameterType> get_parameter_types(
      const std::vector<std::string>& parameter_names,
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    // 将超时时间转换为纳秒，并调用另一个 get_parameter_types 函数
    // Convert the timeout to nanoseconds and call another get_parameter_types function.
    return get_parameter_types(
        parameter_names, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  /**
   *  @brief 设置一组参数 (Set a group of parameters)
   *  @tparam RepT 默认为 int64_t，表示时间的数量类型 (Default is int64_t, represents the quantity
   * type of time)
   *  @tparam RatioT 默认为 std::milli，表示时间单位 (Default is std::milli, represents the time
   * unit)
   *  @param parameters 参数向量 (Vector of parameters)
   *  @param timeout 超时时间，默认值为 -1，表示无限等待 (Timeout duration, default value is -1,
   * which means waiting indefinitely)
   *  @return 返回一个包含 SetParametersResult 的向量 (Returns a vector containing
   * SetParametersResult)
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
      const std::vector<rclcpp::Parameter>& parameters,
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    // 调用 set_parameters 函数，并将 timeout 转换为纳秒 (Call the set_parameters function and
    // convert timeout to nanoseconds)
    return set_parameters(
        parameters, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  /**
   *  @brief 原子地设置一组参数 (Set a group of parameters atomically)
   *  @tparam RepT 默认为 int64_t，表示时间的数量类型 (Default is int64_t, represents the quantity
   * type of time)
   *  @tparam RatioT 默认为 std::milli，表示时间单位 (Default is std::milli, represents the time
   * unit)
   *  @param parameters 参数向量 (Vector of parameters)
   *  @param timeout 超时时间，默认值为 -1，表示无限等待 (Timeout duration, default value is -1,
   * which means waiting indefinitely)
   *  @return 返回一个 SetParametersResult 对象 (Returns a SetParametersResult object)
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
      const std::vector<rclcpp::Parameter>& parameters,
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    // 调用 set_parameters_atomically 函数，并将 timeout 转换为纳秒 (Call the
    // set_parameters_atomically function and convert timeout to nanoseconds)
    return set_parameters_atomically(
        parameters, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  /**
   *  @brief 一次性删除多个参数 (Delete several parameters at once)
   *  @tparam RepT 默认为 int64_t，表示时间的数量类型 (Default is int64_t, represents the quantity
   * type of time)
   *  @tparam RatioT 默认为 std::milli，表示时间单位 (Default is std::milli, represents the time
   * unit)
   *  @param parameters_names 参数名称向量 (Vector of parameter names)
   *  @param timeout 超时时间，默认值为 -1，表示无限等待 (Timeout duration, default value is -1,
   * which means waiting indefinitely)
   *  @return 返回一个包含 SetParametersResult 的向量 (Returns a vector containing
   * SetParametersResult)
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  std::vector<rcl_interfaces::msg::SetParametersResult> delete_parameters(
      const std::vector<std::string>& parameters_names,
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    // 调用 delete_parameters 函数，并将 timeout 转换为纳秒 (Call the delete_parameters function and
    // convert timeout to nanoseconds)
    return delete_parameters(
        parameters_names, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  /**
   *  @brief 从 yaml 文件加载参数 (Load parameters from yaml file)
   *  @tparam RepT 默认为 int64_t，表示时间的数量类型 (Default is int64_t, represents the quantity
   * type of time)
   *  @tparam RatioT 默认为 std::milli，表示时间单位 (Default is std::milli, represents the time
   * unit)
   *  @param yaml_filename yaml 文件的完整名称 (The full name of the yaml file)
   *  @param timeout 超时时间，默认值为 -1，表示无限等待 (Timeout duration, default value is -1,
   * which means waiting indefinitely)
   *  @return 返回一个包含 SetParametersResult 的向量 (Returns a vector containing
   * SetParametersResult)
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  std::vector<rcl_interfaces::msg::SetParametersResult> load_parameters(
      const std::string& yaml_filename,
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    // 调用 load_parameters 函数，并将 timeout 转换为纳秒 (Call the load_parameters function and
    // convert timeout to nanoseconds)
    return load_parameters(
        yaml_filename, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  /**
   *  @brief 列出参数 (List parameters)
   *  @tparam RepT 默认为 int64_t，表示时间的数量类型 (Default is int64_t, represents the quantity
   * type of time)
   *  @tparam RatioT 默认为 std::milli，表示时间单位 (Default is std::milli, represents the time
   * unit)
   *  @param parameter_prefixes 参数前缀向量 (Vector of parameter prefixes)
   *  @param depth 搜索深度 (Search depth)
   *  @param timeout 超时时间，默认值为 -1，表示无限等待 (Timeout duration, default value is -1,
   * which means waiting indefinitely)
   *  @return 返回一个 ListParametersResult 对象 (Returns a ListParametersResult object)
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  rcl_interfaces::msg::ListParametersResult list_parameters(
      const std::vector<std::string>& parameter_prefixes,
      uint64_t depth,
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    // 调用 list_parameters 函数，并将 timeout 转换为纳秒 (Call the list_parameters function and
    // convert timeout to nanoseconds)
    return list_parameters(
        parameter_prefixes, depth, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  // 使用 doxygen 风格的参数列表说明
  /**
   * @brief 订阅参数事件 (Subscribe to parameter events)
   * @tparam CallbackT 回调函数类型 (Callback function type)
   * @param callback 当参数事件发生时要调用的回调函数 (The callback function to be called when a
   * parameter event occurs)
   * @return 返回一个参数事件订阅共享指针 (Returns a shared pointer to the parameter event
   * subscription)
   */
  template <typename CallbackT>
  typename rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr on_parameter_event(
      CallbackT&& callback) {
    // 将回调函数转发给异步参数客户端进行参数事件订阅 (Forward the callback to the async parameters
    // client for parameter event subscription)
    return async_parameters_client_->on_parameter_event(std::forward<CallbackT>(callback));
  }

  /**
   * @brief 订阅节点的参数事件 (Subscribe to parameter events of a node)
   * @tparam CallbackT 回调函数类型 (Callback function type)
   * @tparam NodeT 节点类型，只需要有一个名为 get_node_topics_interface() 的方法 (Node type, only
   * needs to have a method named get_node_topics_interface())
   * @param node 要订阅参数事件的节点 (The node to subscribe to parameter events)
   * @param callback 当参数事件发生时要调用的回调函数 (The callback function to be called when a
   * parameter event occurs)
   * @return 返回一个参数事件订阅共享指针 (Returns a shared pointer to the parameter event
   * subscription)
   */
  template <typename CallbackT, typename NodeT>
  static typename rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
  on_parameter_event(NodeT&& node, CallbackT&& callback) {
    // 将节点和回调函数转发给 AsyncParametersClient 进行参数事件订阅 (Forward the node and callback
    // to AsyncParametersClient for parameter event subscription)
    return AsyncParametersClient::on_parameter_event(node, std::forward<CallbackT>(callback));
  }

  // 检查服务是否准备好 (Check if the service is ready)
  RCLCPP_PUBLIC
  bool service_is_ready() const { return async_parameters_client_->service_is_ready(); }

  /**
   * @brief 等待服务准备就绪 (Wait for the service to be ready)
   * @tparam RepT 时间表示类型，默认为 int64_t (Time representation type, default is int64_t)
   * @tparam RatioT 时间单位，默认为毫秒 (Time unit, default is milliseconds)
   * @param timeout 等待超时时间，默认为 -1 表示无限等待 (Waiting timeout, default is -1 which means
   * waiting indefinitely)
   * @return 如果服务准备就绪，返回 true，否则返回 false (Returns true if the service is ready,
   * otherwise returns false)
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    // 使用异步参数客户端等待服务准备就绪 (Wait for the service to be ready using the async
    // parameters client)
    return async_parameters_client_->wait_for_service(timeout);
  }

protected:
  /**
   * @brief 获取参数列表
   * @param parameter_names 参数名列表
   * @param timeout 超时时间
   * @return std::vector<rclcpp::Parameter> 参数列表
   *
   * @brief Get the list of parameters
   * @param parameter_names List of parameter names
   * @param timeout Timeout duration
   * @return std::vector<rclcpp::Parameter> List of parameters
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::Parameter> get_parameters(
      const std::vector<std::string>& parameter_names, std::chrono::nanoseconds timeout);

  /**
   * @brief 描述参数列表
   * @param parameter_names 参数名列表
   * @param timeout 超时时间
   * @return std::vector<rcl_interfaces::msg::ParameterDescriptor> 参数描述符列表
   *
   * @brief Describe the list of parameters
   * @param parameter_names List of parameter names
   * @param timeout Timeout duration
   * @return std::vector<rcl_interfaces::msg::ParameterDescriptor> List of parameter descriptors
   */
  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
      const std::vector<std::string>& parameter_names, std::chrono::nanoseconds timeout);

  /**
   * @brief 获取参数类型列表
   * @param parameter_names 参数名列表
   * @param timeout 超时时间
   * @return std::vector<rclcpp::ParameterType> 参数类型列表
   *
   * @brief Get the list of parameter types
   * @param parameter_names List of parameter names
   * @param timeout Timeout duration
   * @return std::vector<rclcpp::ParameterType> List of parameter types
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::ParameterType> get_parameter_types(
      const std::vector<std::string>& parameter_names, std::chrono::nanoseconds timeout);

  /**
   * @brief 设置参数列表
   * @param parameters 参数列表
   * @param timeout 超时时间
   * @return std::vector<rcl_interfaces::msg::SetParametersResult> 设置参数结果列表
   *
   * @brief Set the list of parameters
   * @param parameters List of parameters
   * @param timeout Timeout duration
   * @return std::vector<rcl_interfaces::msg::SetParametersResult> List of set parameter results
   */
  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
      const std::vector<rclcpp::Parameter>& parameters, std::chrono::nanoseconds timeout);

  /**
   * @brief 删除参数列表
   * @param parameters_names 参数名列表
   * @param timeout 超时时间
   * @return std::vector<rcl_interfaces::msg::SetParametersResult> 删除参数结果列表
   *
   * @brief Delete the list of parameters
   * @param parameters_names List of parameter names
   * @param timeout Timeout duration
   * @return std::vector<rcl_interfaces::msg::SetParametersResult> List of delete parameter results
   */
  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult> delete_parameters(
      const std::vector<std::string>& parameters_names, std::chrono::nanoseconds timeout);

  /**
   * @brief 加载参数文件
   * @param yaml_filename YAML文件名
   * @param timeout 超时时间
   * @return std::vector<rcl_interfaces::msg::SetParametersResult> 加载参数结果列表
   *
   * @brief Load parameters from a file
   * @param yaml_filename YAML filename
   * @param timeout Timeout duration
   * @return std::vector<rcl_interfaces::msg::SetParametersResult> List of load parameter results
   */
  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult> load_parameters(
      const std::string& yaml_filename, std::chrono::nanoseconds timeout);

  /**
   * @brief 原子性地设置参数列表
   * @param parameters 参数列表
   * @param timeout 超时时间
   * @return rcl_interfaces::msg::SetParametersResult 设置参数结果
   *
   * @brief Set the list of parameters atomically
   * @param parameters List of parameters
   * @param timeout Timeout duration
   * @return rcl_interfaces::msg::SetParametersResult Set parameter result
   */
  RCLCPP_PUBLIC
  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
      const std::vector<rclcpp::Parameter>& parameters, std::chrono::nanoseconds timeout);

  /**
   * @brief 列出参数
   * @param parameter_prefixes 参数前缀列表
   * @param depth 深度
   * @param timeout 超时时间
   * @return rcl_interfaces::msg::ListParametersResult 列出参数结果
   *
   * @brief List parameters
   * @param parameter_prefixes List of parameter prefixes
   * @param depth Depth
   * @param timeout Timeout duration
   * @return rcl_interfaces::msg::ListParametersResult List parameters result
   */
  RCLCPP_PUBLIC
  rcl_interfaces::msg::ListParametersResult list_parameters(
      const std::vector<std::string>& parameter_prefixes,
      uint64_t depth,
      std::chrono::nanoseconds timeout);

private:
  /**
   * @brief 一个基于 rclcpp 的 ROS2 示例代码，包含 executor、node_base_interface 和
   * async_parameters_client。
   * @brief An example code based on rclcpp for ROS2, including executor, node_base_interface, and
   * async_parameters_client.
   */

  // 声明一个共享指针类型的 executor 变量
  // Declare a shared pointer type executor variable
  rclcpp::Executor::SharedPtr executor_;

  // 声明一个共享指针类型的 node_base_interface 变量，用于与节点的底层接口进行交互
  // Declare a shared pointer type node_base_interface variable for interacting with the underlying
  // interface of the node
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;

  // 声明一个共享指针类型的 async_parameters_client 变量，用于异步参数客户端操作
  // Declare a shared pointer type async_parameters_client variable for asynchronous parameter
  // client operations
  AsyncParametersClient::SharedPtr async_parameters_client_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_CLIENT_HPP_

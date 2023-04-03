// Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_GRAPH_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_GRAPH_INTERFACE_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rcl/graph.h"
#include "rcl/guard_condition.h"
#include "rclcpp/event.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/detail/node_interfaces_helpers.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/**
 * @brief Endpoint 类型枚举 (Endpoint type enumeration)
 *
 * @details 用于区分发布者、订阅者以及无效端点类型的枚举类 (An enumeration class to distinguish
 * between publisher, subscriber and invalid endpoint types)
 */
enum class EndpointType {
  Invalid = RMW_ENDPOINT_INVALID,           ///< 无效端点类型 (Invalid endpoint type)
  Publisher = RMW_ENDPOINT_PUBLISHER,       ///< 发布者端点类型 (Publisher endpoint type)
  Subscription = RMW_ENDPOINT_SUBSCRIPTION  ///< 订阅者端点类型 (Subscriber endpoint type)
};

/**
 * @brief 结构体，包含主题端点信息，如关联的节点名称、节点命名空间、主题类型、端点类型、端点 GID 和
 * QoS。 (Struct that contains topic endpoint information like the associated node name, node
 * namespace, topic type, endpoint type, endpoint GID, and its QoS.)
 */
class TopicEndpointInfo {
public:
  /**
   * @brief 从 rcl_topic_endpoint_info_t 构造一个 TopicEndpointInfo。
   *        (Construct a TopicEndpointInfo from a rcl_topic_endpoint_info_t.)
   * @param info rcl_topic_endpoint_info_t 结构体实例。(An instance of rcl_topic_endpoint_info_t
   * structure.)
   */
  RCLCPP_PUBLIC
  explicit TopicEndpointInfo(const rcl_topic_endpoint_info_t& info)
      : node_name_(info.node_name),            ///< 设置节点名称。(Set the node name.)
        node_namespace_(info.node_namespace),  ///< 设置节点命名空间。(Set the node namespace.)
        topic_type_(info.topic_type),          ///< 设置主题类型。(Set the topic type.)
        endpoint_type_(static_cast<rclcpp::EndpointType>(
            info.endpoint_type)),              ///< 设置端点类型。(Set the endpoint type.)
        qos_profile_(
            {info.qos_profile.history, info.qos_profile.depth},
            info.qos_profile)  ///< 设置 QoS 配置。(Set the QoS profile.)
  {
    // 复制端点 GID。(Copy the endpoint GID.)
    std::copy(info.endpoint_gid, info.endpoint_gid + RMW_GID_STORAGE_SIZE, endpoint_gid_.begin());
  }

  /**
   * \brief 获取节点名称的可变引用 (Get a mutable reference to the node name)
   * \return 节点名称的可变引用 (A mutable reference to the node name)
   */
  RCLCPP_PUBLIC
  std::string& node_name();

  /**
   * \brief 获取节点名称的常量引用 (Get a const reference to the node name)
   * \return 节点名称的常量引用 (A const reference to the node name)
   */
  RCLCPP_PUBLIC
  const std::string& node_name() const;

  /**
   * \brief 获取节点命名空间的可变引用 (Get a mutable reference to the node namespace)
   * \return 节点命名空间的可变引用 (A mutable reference to the node namespace)
   */
  RCLCPP_PUBLIC
  std::string& node_namespace();

  /**
   * \brief 获取节点命名空间的常量引用 (Get a const reference to the node namespace)
   * \return 节点命名空间的常量引用 (A const reference to the node namespace)
   */
  RCLCPP_PUBLIC
  const std::string& node_namespace() const;

  /**
   * \brief 获取主题类型字符串的可变引用 (Get a mutable reference to the topic type string)
   * \return 主题类型字符串的可变引用 (A mutable reference to the topic type string)
   */
  RCLCPP_PUBLIC
  std::string& topic_type();

  /**
   * \brief 获取主题类型字符串的常量引用 (Get a const reference to the topic type string)
   * \return 主题类型字符串的常量引用 (A const reference to the topic type string)
   */
  RCLCPP_PUBLIC
  const std::string& topic_type() const;

  /// 获取主题端点类型的可变引用 (Get a mutable reference to the topic endpoint type)
  /// 返回主题端点类型的可变引用 (Returns a mutable reference to the topic endpoint type)
  RCLCPP_PUBLIC
  rclcpp::EndpointType& endpoint_type();

  /// 获取主题端点类型的常量引用 (Get a const reference to the topic endpoint type)
  /// 返回主题端点类型的常量引用 (Returns a const reference to the topic endpoint type)
  RCLCPP_PUBLIC
  const rclcpp::EndpointType& endpoint_type() const;

  /// 获取主题端点 GID 的可变引用 (Get a mutable reference to the GID of the topic endpoint)
  /// 返回主题端点 GID 的可变引用 (Returns a mutable reference to the GID of the topic endpoint)
  RCLCPP_PUBLIC
  std::array<uint8_t, RMW_GID_STORAGE_SIZE>& endpoint_gid();

  /// 获取主题端点 GID 的常量引用 (Get a const reference to the GID of the topic endpoint)
  /// 返回主题端点 GID 的常量引用 (Returns a const reference to the GID of the topic endpoint)
  RCLCPP_PUBLIC
  const std::array<uint8_t, RMW_GID_STORAGE_SIZE>& endpoint_gid() const;

  /// 获取主题端点 QoS 配置文件的可变引用 (Get a mutable reference to the QoS profile of the topic
  /// endpoint) 返回主题端点 QoS 配置文件的可变引用 (Returns a mutable reference to the QoS profile
  /// of the topic endpoint)
  RCLCPP_PUBLIC
  rclcpp::QoS& qos_profile();

  /// 获取主题端点 QoS 配置文件的常量引用 (Get a const reference to the QoS profile of the topic
  /// endpoint) 返回主题端点 QoS 配置文件的常量引用 (Returns a const reference to the QoS profile of
  /// the topic endpoint)
  RCLCPP_PUBLIC
  const rclcpp::QoS& qos_profile() const;

private:
  // 节点名称 (Node name)
  std::string node_name_;
  // 节点命名空间 (Node namespace)
  std::string node_namespace_;
  // 话题类型 (Topic type)
  std::string topic_type_;
  // 端点类型，例如发布者或订阅者 (Endpoint type, e.g., publisher or subscriber)
  rclcpp::EndpointType endpoint_type_;
  // 端点全局唯一标识符（GID）存储，用于唯一标识节点中的实体 (Endpoint GID storage, used to uniquely
  // identify entities within a node)
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> endpoint_gid_;
  // 服务质量（QoS）配置文件，包含了与消息传递相关的各种策略 (Quality of Service (QoS) profile,
  // contains various policies related to message delivery)
  rclcpp::QoS qos_profile_;
};

namespace node_interfaces {

/// 纯虚接口类，用于实现节点图部分的节点 API。
/// Pure virtual interface class for the NodeGraph part of the Node API.
class NodeGraphInterface {
public:
  // 使用智能指针别名，仅限于 NodeGraphInterface 类型。
  // Use smart pointer aliases, only for NodeGraphInterface type.
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeGraphInterface)

  // 声明为公共成员函数
  // Declare as a public member function
  RCLCPP_PUBLIC
  // 虚析构函数，允许通过基类指针删除派生类对象。默认实现。
  // Virtual destructor, allows deletion of derived objects through a base class pointer. Default
  // implementation.
  virtual ~NodeGraphInterface() = default;

  /// 返回一个映射，其中包含现有主题名称到主题类型列表的映射。(Return a map of existing topic names
  /// to list of topic types.)
  /**
   * 当至少存在一个发布者或订阅者时，主题被认为是存在的，无论它们是本地的还是远程的。 (A topic is
   * considered to exist when at least one publisher or subscriber exists for it, whether they be
   * local or remote to this process.) 返回的名称是主题的实际名称，可能是由其他节点或此节点发布的。
   * (The returned names are the actual names of the topics, either announced by another nodes or by
   * this one.)
   * 使用此函数返回的名称创建发布者或订阅者可能不会导致使用所需的主题名称，具体取决于正在使用的重映射规则。
   * (Attempting to create publishers or subscribers using names returned by this function may not
   * result in the desired topic name being used depending on the remap rules in use.)
   *
   * \param[in] no_demangle 如果为 true，则不对主题名称和类型进行解析 (if true, topic names and
   * types are not demangled)
   */
  RCLCPP_PUBLIC
  virtual std::map<std::string, std::vector<std::string>> get_topic_names_and_types(
      bool no_demangle = false) const = 0;

  /// 返回一个映射，其中包含现有服务名称到服务类型列表的映射。(Return a map of existing service
  /// names to list of service types.)
  /**
   * 当至少存在一个服务服务器或服务客户端时，服务被认为是存在的，无论它们是本地的还是远程的。 (A
   * service is considered to exist when at least one service server or service client exists for
   * it, whether they be local or remote to this process.)
   * 返回的名称是服务的实际名称，可能是由其他节点或此节点发布的。 (The returned names are the actual
   * names of the services, either announced by another nodes or by this one.)
   * 使用此函数返回的名称创建客户端或服务可能不会导致使用所需的服务名称，具体取决于正在使用的重映射规则。
   * (Attempting to create clients or services using names returned by this function may not result
   * in the desired service name being used depending on the remap rules in use.)
   */
  RCLCPP_PUBLIC
  virtual std::map<std::string, std::vector<std::string>> get_service_names_and_types() const = 0;

  /// 返回一个特定节点的现有服务名称到服务类型列表的映射。
  /// Return a map of existing service names to list of service types for a specific node.
  /**
   * 此函数仅考虑服务 - 不考虑客户端。
   * This function only considers services - not clients.
   * 返回的名称是应用了重映射规则后的实际名称。
   * The returned names are the actual names after remap rules applied.
   * 尝试使用此功能返回的名称创建服务客户端可能不会导致使用所需的服务名称，具体取决于正在使用的重映射规则。
   * Attempting to create service clients using names returned by this function may not
   * result in the desired service name being used depending on the remap rules in use.
   *
   * \param[in] node_name 节点的名称
   * \param[in] node_name name of the node
   * \param[in] namespace_ 节点的命名空间
   * \param[in] namespace_ namespace of the node
   */
  RCLCPP_PUBLIC
  virtual std::map<std::string, std::vector<std::string>> get_service_names_and_types_by_node(
      const std::string& node_name, const std::string& namespace_) const = 0;

  /// 返回一个特定节点的现有服务名称和类型的映射。
  /// Return a map of existing service names and types with a specific node.
  /**
   * 此函数仅考虑客户端 - 不考虑服务服务器。
   * This function only considers clients - not service servers.
   * 返回的名称是应用了重映射规则后的实际名称。
   * The returned names are the actual names after remap rules applied.
   * 尝试使用此功能返回的名称创建服务服务器可能不会导致使用所需的服务名称，具体取决于正在使用的重映射规则。
   * Attempting to create service servers using names returned by this function may not
   * result in the desired service name being used depending on the remap rules in use.
   *
   * \param[in] node_name 节点的名称
   * \param[in] node_name name of the node
   * \param[in] namespace_ 节点的命名空间
   * \param[in] namespace_ namespace of the node
   */
  RCLCPP_PUBLIC
  virtual std::map<std::string, std::vector<std::string>> get_client_names_and_types_by_node(
      const std::string& node_name, const std::string& namespace_) const = 0;

  /// 返回一个特定节点的现有主题名称到主题类型列表的映射。
  /// Return a map of existing topic names to list of topic types for a specific node.
  /**
   * 此函数仅考虑发布者 - 不考虑订阅者。
   * This function only considers publishers - not subscribers.
   * 返回的名称是应用了重映射规则后的实际名称。
   * The returned names are the actual names after remap rules applied.
   * 尝试使用此功能返回的名称创建发布者或订阅者可能不会导致使用所需的主题名称，具体取决于正在使用的重映射规则。
   * Attempting to create publishers or subscribers using names returned by this function may not
   * result in the desired topic name being used depending on the remap rules in use.
   *
   * \param[in] node_name 节点的名称
   * \param[in] node_name name of the node
   * \param[in] namespace_ 节点的命名空间
   * \param[in] namespace_ namespace of the node
   * \param[in] no_demangle 如果为 true，则不对主题名称和类型进行解扰
   * \param[in] no_demangle if true, topic names and types are not demangled
   */
  RCLCPP_PUBLIC
  virtual std::map<std::string, std::vector<std::string>> get_publisher_names_and_types_by_node(
      const std::string& node_name,
      const std::string& namespace_,
      bool no_demangle = false) const = 0;

  /// 返回一个特定节点的现有主题名称到主题类型列表的映射。(Return a map of existing topic names to
  /// list of topic types for a specific node.)
  /**
   * 此函数仅考虑订阅者 - 不考虑发布者。(This function only considers subscribers - not publishers.)
   * 返回的名称是应用了重映射规则后的实际名称。(The returned names are the actual names after remap
   * rules applied.)
   * 尝试使用此函数返回的名称创建发布者或订阅者可能不会导致使用所需的主题名称，具体取决于正在使用的重映射规则。(Attempting
   * to create publishers or subscribers using names returned by this function may not result in the
   * desired topic name being used depending on the remap rules in use.)
   *
   * \param[in] node_name 节点的名称 (name of the node)
   * \param[in] namespace_ 节点的命名空间 (namespace of the node)
   * \param[in] no_demangle 如果为 true，则不对主题名称和类型进行解扰 (if true, topic names and
   * types are not demangled)
   */
  RCLCPP_PUBLIC
  virtual std::map<std::string, std::vector<std::string>> get_subscriber_names_and_types_by_node(
      const std::string& node_name,
      const std::string& namespace_,
      bool no_demangle = false) const = 0;

  /// 返回现有节点名称（字符串）的向量。(Return a vector of existing node names (string).)
  /*
   * 返回的名称是应用了重映射规则后的实际名称。(The returned names are the actual names after remap
   * rules applied.)
   */
  RCLCPP_PUBLIC
  virtual std::vector<std::string> get_node_names() const = 0;

  /// 返回一个包含现有节点名称、命名空间和安全领域（字符串元组）的向量。
  /// Return a vector of existing node names, namespaces and enclaves (tuple of string).
  /*
   * 返回的名称是应用重映射规则后的实际名称。
   * The returned names are the actual names after remap rules applied.
   * 安全领域包含运行时安全工件，可用于建立安全网络。
   * The enclaves contain the runtime security artifacts, those can be
   * used to establish secured network.
   * 请参阅 https://design.ros2.org/articles/ros2_security_enclaves.html
   * See https://design.ros2.org/articles/ros2_security_enclaves.html
   */
  RCLCPP_PUBLIC
  virtual std::vector<std::tuple<std::string, std::string, std::string>>
  get_node_names_with_enclaves() const = 0;

  /// 返回一个包含现有节点名称和命名空间（字符串对）的向量。
  /// Return a vector of existing node names and namespaces (pair of string).
  /*
   * 返回的名称是应用重映射规则后的实际名称。
   * The returned names are the actual names after remap rules applied.
   */
  RCLCPP_PUBLIC
  virtual std::vector<std::pair<std::string, std::string>> get_node_names_and_namespaces()
      const = 0;

  /// 返回给定主题上广告的发布者数量。
  /// Return the number of publishers that are advertised on a given topic.
  /*
   * \param[in] topic_name 实际使用的主题名称；它不会自动重新映射。
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   */
  RCLCPP_PUBLIC
  virtual size_t count_publishers(const std::string& topic_name) const = 0;

  /// 返回为给定主题创建订阅的订阅者数量。
  /// Return the number of subscribers who have created a subscription for a given topic.
  /*
   * \param[in] topic_name 实际使用的主题名称；它不会自动重新映射。
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   */
  RCLCPP_PUBLIC
  virtual size_t count_subscribers(const std::string& topic_name) const = 0;

  /// 返回在 ROS 图发生更改时触发的 rcl 保护条件。
  /// Return the rcl guard condition which is triggered when the ROS graph changes.
  RCLCPP_PUBLIC
  virtual const rcl_guard_condition_t* get_graph_guard_condition() const = 0;

  /// 通知等待图形更改的线程。
  /// Notify threads waiting on graph changes.
  /**
   * 影响等待通知保护条件的线程，参见：
   * Affects threads waiting on the notify guard condition, see:
   * get_notify_guard_condition(),
   * 以及使用图形事件等待图形更改的线程，参见：wait_for_graph_change()。
   * get_notify_guard_condition(), as well as the threads waiting on graph
   * changes using a graph Event, see: wait_for_graph_change().
   *
   * 通常仅由 rclcpp::graph_listener::GraphListener 使用。
   * This is typically only used by the rclcpp::graph_listener::GraphListener.
   *
   * 当发生 rcl 错误时，抛出 RCLBaseError（该异常的子类）
   * \throws RCLBaseError (a child of that exception) when an rcl error occurs
   */
  RCLCPP_PUBLIC
  virtual void notify_graph_change() = 0;

  /// 通知所有阻塞节点操作，已经发生了关闭。
  /// Notify any and all blocking node actions that shutdown has occurred.
  RCLCPP_PUBLIC
  virtual void notify_shutdown() = 0;

  /// 返回一个图形事件，当发生图形更改时将被设置。
  /// Return a graph event, which will be set anytime a graph change occurs.
  /**
   * 图形事件对象是一个必须返回的贷款。
   * The graph Event object is a loan which must be returned.
   * 事件对象是有作用域的，因此要返回贷款只需让它超出范围即可。
   * The Event object is scoped and therefore to return the load just let it go
   * out of scope.
   */
  RCLCPP_PUBLIC
  virtual rclcpp::Event::SharedPtr get_graph_event() = 0;

  /// 通过等待事件变为已设置来等待图形事件发生。
  /// Wait for a graph event to occur by waiting on an Event to become set.
  /**
   * 给定的事件必须通过get_graph_event()方法获取。
   * The given Event must be acquired through the get_graph_event() method.
   *
   * \throws InvalidEventError 如果给定的事件是空指针
   * \throws InvalidEventError if the given event is nullptr
   * \throws EventNotRegisteredError 如果给定的事件没有使用
   *   get_graph_event()获得。
   * \throws EventNotRegisteredError if the given event was not acquired with
   *   get_graph_event().
   */
  RCLCPP_PUBLIC
  virtual void wait_for_graph_change(
      rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout) = 0;

  /// 返回在贷款图形事件上的数量，参见get_graph_event()。
  /// Return the number of on loan graph events, see get_graph_event().
  /**
   * 这通常只由rclcpp::graph_listener::GraphListener使用。
   * This is typically only used by the rclcpp::graph_listener::GraphListener.
   */
  RCLCPP_PUBLIC
  virtual size_t count_graph_users() const = 0;

  /// 返回给定主题上发布者的主题端点信息。
  /// Return the topic endpoint information about publishers on a given topic.
  /**
   * \param[in] topic_name 实际使用的主题名称；它不会被自动重映射。
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   * \param[in] no_mangle 如果为 `true`，则 `topic_name` 需要是有效的中间件主题名称，
   *   否则应该是一个有效的 ROS 主题名称。
   * \param[in] no_mangle if `true`, `topic_name` needs to be a valid middleware topic name,
   *   otherwise it should be a valid ROS topic name.
   * \sa rclcpp::Node::get_publishers_info_by_topic
   */
  RCLCPP_PUBLIC
  virtual std::vector<rclcpp::TopicEndpointInfo> get_publishers_info_by_topic(
      const std::string& topic_name, bool no_mangle = false) const = 0;

  /// 返回给定主题上订阅者的主题端点信息。
  /// Return the topic endpoint information about subscriptions on a given topic.
  /**
   * \param[in] topic_name 实际使用的主题名称；它不会被自动重映射。
   * \param[in] topic_name the actual topic name used; it will not be automatically remapped.
   * \param[in] no_mangle 如果为 `true`，则 `topic_name` 需要是有效的中间件主题名称，
   *   否则应该是一个有效的 ROS 主题名称。
   * \param[in] no_mangle if `true`, `topic_name` needs to be a valid middleware topic name,
   *   otherwise it should be a valid ROS topic name.
   * \sa rclcpp::Node::get_subscriptions_info_by_topic
   */
  RCLCPP_PUBLIC
  virtual std::vector<rclcpp::TopicEndpointInfo> get_subscriptions_info_by_topic(
      const std::string& topic_name, bool no_mangle = false) const = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeGraphInterface, graph)

#endif  // RCLCPP__NODE_INTERFACES__NODE_GRAPH_INTERFACE_HPP_

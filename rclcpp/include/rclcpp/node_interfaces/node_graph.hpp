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

#ifndef RCLCPP__NODE_INTERFACES__NODE_GRAPH_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_GRAPH_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rcl/guard_condition.h"
#include "rclcpp/event.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/topic_endpoint_info_array.h"

namespace rclcpp {

namespace graph_listener {
class GraphListener;
}  // namespace graph_listener

namespace node_interfaces {

/// \class NodeGraph
/// \brief NodeGraph 类，实现 NodeGraphInterface。 (NodeGraph class, implementing
/// NodeGraphInterface.)
class NodeGraph : public NodeGraphInterface {
public:
  /// \typedef NodeGraph_SMART_PTR_ALIASES_ONLY
  /// \brief 使用智能指针别名仅限于 NodeGraph 类。 (Use smart pointer aliases only for the NodeGraph
  /// class.)
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeGraph)

  /**
   * @brief 构造函数，创建一个 NodeGraph 对象 (Constructor, creates a NodeGraph object)
   *
   * @param node_base 指向 rclcpp::node_interfaces::NodeBaseInterface 类型的指针 (A pointer to an
   * object of type rclcpp::node_interfaces::NodeBaseInterface)
   */
  RCLCPP_PUBLIC
  explicit NodeGraph(rclcpp::node_interfaces::NodeBaseInterface* node_base);

  /**
   * @brief 析构函数，释放 NodeGraph 对象资源 (Destructor, releases the resources of the NodeGraph
   * object)
   */
  RCLCPP_PUBLIC
  virtual ~NodeGraph();

  /**
   * @brief 获取主题名称和类型 (Get topic names and types)
   *
   * @param no_demangle 是否取消对主题名称的解析，默认为 false (Whether to demangle the topic names
   * or not, default is false)
   * @return 返回一个 map，键值是主题名称，值是主题类型的 vector (Returns a map where the key is the
   * topic name and the value is a vector of topic types)
   */
  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>> get_topic_names_and_types(
      bool no_demangle = false) const override;

  /**
   * @brief 获取服务名称和类型 (Get service names and types)
   *
   * @return 返回一个 map，键值是服务名称，值是服务类型的 vector (Returns a map where the key is the
   * service name and the value is a vector of service types)
   */
  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>> get_service_names_and_types() const override;

  /**
   * @brief 根据节点获取服务名称和类型 (Get service names and types by node)
   *
   * @param node_name 节点名称 (Node name)
   * @param namespace_ 命名空间 (Namespace)
   * @return 返回一个 map，键值是服务名称，值是服务类型的 vector (Returns a map where the key is the
   * service name and the value is a vector of service types)
   */
  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>> get_service_names_and_types_by_node(
      const std::string& node_name, const std::string& namespace_) const override;

  /**
   * @brief 根据节点获取客户端名称和类型 (Get client names and types by node)
   *
   * @param node_name 节点名称 (Node name)
   * @param namespace_ 命名空间 (Namespace)
   * @return 返回一个 map，键值是客户端名称，值是客户端类型的 vector (Returns a map where the key is
   * the client name and the value is a vector of client types)
   */
  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>> get_client_names_and_types_by_node(
      const std::string& node_name, const std::string& namespace_) const override;

  /**
   * @brief 获取指定节点的发布者名称和类型
   *        Get publisher names and types for a specific node.
   *
   * @param[in] node_name 节点名称 (Node name)
   * @param[in] namespace_ 命名空间 (Namespace)
   * @param[in] no_demangle 是否取消名称修饰 (Whether to demangle the name or not)
   * @return 返回一个映射，其中键是发布者名称，值是类型向量 (Return a map where the key is the
   * publisher name and the value is a vector of types)
   */
  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>> get_publisher_names_and_types_by_node(
      const std::string& node_name,
      const std::string& namespace_,
      bool no_demangle = false) const override;

  /**
   * @brief 获取指定节点的订阅者名称和类型
   *        Get subscriber names and types for a specific node.
   *
   * @param[in] node_name 节点名称 (Node name)
   * @param[in] namespace_ 命名空间 (Namespace)
   * @param[in] no_demangle 是否取消名称修饰 (Whether to demangle the name or not)
   * @return 返回一个映射，其中键是订阅者名称，值是类型向量 (Return a map where the key is the
   * subscriber name and the value is a vector of types)
   */
  RCLCPP_PUBLIC
  std::map<std::string, std::vector<std::string>> get_subscriber_names_and_types_by_node(
      const std::string& node_name,
      const std::string& namespace_,
      bool no_demangle = false) const override;

  /**
   * @brief 获取所有节点名称
   *        Get all node names.
   *
   * @return 返回一个字符串向量，包含所有节点名称 (Return a vector of strings containing all node
   * names)
   */
  RCLCPP_PUBLIC
  std::vector<std::string> get_node_names() const override;

  /**
   * @brief 获取带有围墙的节点名称
   *        Get node names with enclaves.
   *
   * @return 返回一个元组向量，其中每个元素包含节点名称、命名空间和围墙名称 (Return a vector of
   * tuples where each element contains the node name, namespace, and enclave name)
   */
  RCLCPP_PUBLIC
  std::vector<std::tuple<std::string, std::string, std::string>> get_node_names_with_enclaves()
      const override;

  /**
   * @brief 获取节点名称和命名空间
   *        Get node names and namespaces.
   *
   * @return 返回一个 pair 向量，其中每个元素包含节点名称和命名空间 (Return a vector of pairs where
   * each element contains the node name and namespace)
   */
  RCLCPP_PUBLIC
  std::vector<std::pair<std::string, std::string>> get_node_names_and_namespaces() const override;

  /**
   * @brief 计算指定主题的发布者数量
   *        Count publishers for a specific topic.
   *
   * @param[in] topic_name 主题名称 (Topic name)
   * @return 返回发布者数量 (Return the number of publishers)
   */
  RCLCPP_PUBLIC
  size_t count_publishers(const std::string& topic_name) const override;

  /// 计算订阅给定主题名称的节点数量 (Count the number of nodes subscribing to the given topic name)
  /**
   * \param[in] topic_name 要查询的主题名称 (The topic name to query)
   * \return 订阅此主题的节点数量 (The number of nodes subscribed to this topic)
   */
  RCLCPP_PUBLIC
  size_t count_subscribers(const std::string& topic_name) const override;

  /// 获取图形保护条件 (Get the graph guard condition)
  /**
   * \return rcl_guard_condition_t 指针 (Pointer to an rcl_guard_condition_t)
   */
  RCLCPP_PUBLIC
  const rcl_guard_condition_t* get_graph_guard_condition() const override;

  /// 通知图形发生更改 (Notify that the graph has changed)
  RCLCPP_PUBLIC
  void notify_graph_change() override;

  /// 通知关闭 (Notify shutdown)
  RCLCPP_PUBLIC
  void notify_shutdown() override;

  /// 获取图形事件 (Get the graph event)
  /**
   * \return SharedPtr指向rclcpp::Event对象 (SharedPtr pointing to an rclcpp::Event object)
   */
  RCLCPP_PUBLIC
  rclcpp::Event::SharedPtr get_graph_event() override;

  /// 等待图形更改 (Wait for graph changes)
  /**
   * \param[in] event 用于等待的事件 (Event to wait on)
   * \param[in] timeout 等待超时时间 (Timeout duration for the wait)
   */
  RCLCPP_PUBLIC
  void wait_for_graph_change(rclcpp::Event::SharedPtr event,
                             std::chrono::nanoseconds timeout) override;

  /// 计算图形用户的数量 (Count the number of graph users)
  /**
   * \return 图形中的用户数量 (Number of users in the graph)
   */
  RCLCPP_PUBLIC
  size_t count_graph_users() const override;

  /// 根据主题获取发布者信息 (Get publisher information by topic)
  /**
   * \param[in] topic_name 要查询的主题名称 (The topic name to query)
   * \param[in] no_mangle 是否取消混淆主题名称 (Whether to demangle the topic name or not)
   * \return 包含rclcpp::TopicEndpointInfo的向量 (Vector containing rclcpp::TopicEndpointInfo)
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::TopicEndpointInfo> get_publishers_info_by_topic(
      const std::string& topic_name, bool no_mangle = false) const override;

  /// 根据主题获取订阅者信息 (Get subscriber information by topic)
  /**
   * \param[in] topic_name 要查询的主题名称 (The topic name to query)
   * \param[in] no_mangle 是否取消混淆主题名称 (Whether to demangle the topic name or not)
   * \return 包含rclcpp::TopicEndpointInfo的向量 (Vector containing rclcpp::TopicEndpointInfo)
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::TopicEndpointInfo> get_subscriptions_info_by_topic(
      const std::string& topic_name, bool no_mangle = false) const override;

private:
  RCLCPP_DISABLE_COPY(NodeGraph)

  /// @brief 构造函数中给定的 NodeBaseInterface 句柄 (Handle to the NodeBaseInterface given in the
  /// constructor)
  rclcpp::node_interfaces::NodeBaseInterface* node_base_;

  /// @brief 在节点之间共享的图形监听器，用于等待节点的图形更改 (Graph Listener which waits on graph
  /// changes for the node and is shared across nodes)
  std::shared_ptr<rclcpp::graph_listener::GraphListener> graph_listener_;
  /// @brief 此节点是否需要添加到图形监听器中 (Whether or not this node needs to be added to the
  /// graph listener)
  std::atomic_bool should_add_to_graph_listener_;

  /// @brief 保护与图形事件相关的数据结构的互斥锁 (Mutex to guard the graph event related data
  /// structures)
  mutable std::mutex graph_mutex_;
  /// @brief 用于在图形更改时通知等待线程的条件变量 (For notifying waiting threads
  /// (wait_for_graph_change()) on changes (notify_graph_change()))
  std::condition_variable graph_cv_;
  /// @brief 被借出的图形事件的弱引用 (Weak references to graph events out on loan)
  std::vector<rclcpp::Event::WeakPtr> graph_events_;
  /// @brief 外部借出的图形事件数量，用于确定是否应监视图形 (Number of graph events out on loan,
  /// used to determine if the graph should be monitored)
  /** graph_users_count_ 是原子操作，因此可以在不获取 graph_mutex_ 的情况下访问它
   * (graph_users_count_ is atomic so that it can be accessed without acquiring the graph_mutex_) */
  std::atomic_size_t graph_users_count_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_GRAPH_HPP_

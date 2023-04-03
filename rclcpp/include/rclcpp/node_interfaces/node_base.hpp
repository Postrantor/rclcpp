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

#ifndef RCLCPP__NODE_INTERFACES__NODE_BASE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_BASE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rcl/node.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/// 实现 Node API 的 NodeBase 部分。 (Implementation of the NodeBase part of the Node API.)
class NodeBase : public NodeBaseInterface, public std::enable_shared_from_this<NodeBase> {
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeBase)

  /// 构造函数。 (Constructor.)
  /**
   * 如果为 default_callback_group 提供 nullptr（默认），构造函数将使用 create_callback_group()
   * 方法创建一个， 但不会发生虚拟分派，因此不会使用该方法的重写。 (If nullptr (default) is given
   * for the default_callback_group, one will be created by the constructor using the
   * create_callback_group() method, but virtual dispatch will not occur so overrides of that method
   * will not be used.)
   *
   * @param[in] node_name 节点名称 (Node name)
   * @param[in] namespace_ 命名空间 (Namespace)
   * @param[in] context 共享指针 rclcpp::Context (Shared pointer to rclcpp::Context)
   * @param[in] rcl_node_options rcl_node_options_t 类型的节点选项 (Node options of type
   * rcl_node_options_t)
   * @param[in] use_intra_process_default 是否使用默认的进程内通信 (Whether to use default
   * intra-process communication)
   * @param[in] enable_topic_statistics_default 是否启用默认的主题统计 (Whether to enable default
   * topic statistics)
   * @param[in] default_callback_group 默认回调组的共享指针 (Shared pointer to the default callback
   * group), 默认为 nullptr (default is nullptr)
   */
  RCLCPP_PUBLIC
  NodeBase(
      const std::string& node_name,
      const std::string& namespace_,
      rclcpp::Context::SharedPtr context,
      const rcl_node_options_t& rcl_node_options,
      bool use_intra_process_default,
      bool enable_topic_statistics_default,
      rclcpp::CallbackGroup::SharedPtr default_callback_group = nullptr);

  /**
   * @brief 节点基类的虚析构函数 (Virtual destructor of the NodeBase class)
   */
  RCLCPP_PUBLIC
  virtual ~NodeBase();

  /**
   * @brief 获取节点名称 (Get the node name)
   * @return 节点名称 (Node name)
   */
  RCLCPP_PUBLIC
  const char* get_name() const override;

  /**
   * @brief 获取节点命名空间 (Get the node namespace)
   * @return 节点命名空间 (Node namespace)
   */
  RCLCPP_PUBLIC
  const char* get_namespace() const override;

  /**
   * @brief 获取节点的完全限定名称 (Get the fully qualified name of the node)
   * @return 完全限定名称 (Fully qualified name)
   */
  RCLCPP_PUBLIC
  const char* get_fully_qualified_name() const override;

  /**
   * @brief 获取节点上下文 (Get the node context)
   * @return 节点上下文共享指针 (Shared pointer to the node context)
   */
  RCLCPP_PUBLIC
  rclcpp::Context::SharedPtr get_context() override;

  /**
   * @brief 获取 RCL 节点句柄 (Get the RCL node handle)
   * @return RCL 节点句柄指针 (Pointer to the RCL node handle)
   */
  RCLCPP_PUBLIC
  rcl_node_t* get_rcl_node_handle() override;

  /**
   * @brief 获取常量 RCL 节点句柄 (Get the constant RCL node handle)
   * @return 常量 RCL 节点句柄指针 (Constant pointer to the RCL node handle)
   */
  RCLCPP_PUBLIC
  const rcl_node_t* get_rcl_node_handle() const override;

  /**
   * @brief 获取共享的 RCL 节点句柄 (Get the shared RCL node handle)
   * @return 共享指针到 RCL 节点句柄 (Shared pointer to the RCL node handle)
   */
  RCLCPP_PUBLIC
  std::shared_ptr<rcl_node_t> get_shared_rcl_node_handle() override;

  /**
   * @brief 获取共享的常量 RCL 节点句柄 (Get the shared constant RCL node handle)
   * @return 共享指针到常量 RCL 节点句柄 (Shared pointer to the constant RCL node handle)
   */
  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_node_t> get_shared_rcl_node_handle() const override;

  /**
   * @brief 创建回调组 (Create a callback group)
   * @param group_type 回调组类型 (Callback group type)
   * @param automatically_add_to_executor_with_node 是否自动添加到与节点关联的执行器中 (Whether to
   * automatically add to the executor associated with the node)
   * @return 新创建的回调组共享指针 (Shared pointer to the newly created callback group)
   */
  RCLCPP_PUBLIC
  rclcpp::CallbackGroup::SharedPtr create_callback_group(
      rclcpp::CallbackGroupType group_type,
      bool automatically_add_to_executor_with_node = true) override;

  /**
   * @brief 获取默认回调组 (Get the default callback group)
   * @return 默认回调组共享指针 (Shared pointer to the default callback group)
   */
  RCLCPP_PUBLIC
  rclcpp::CallbackGroup::SharedPtr get_default_callback_group() override;

  /**
   * @brief 检查回调组是否在节点中 (Check if the callback group is in the node)
   * @param group 要检查的回调组共享指针 (Shared pointer to the callback group to check)
   * @return 如果回调组在节点中，则为 true，否则为 false (True if the callback group is in the node,
   * false otherwise)
   */
  RCLCPP_PUBLIC
  bool callback_group_in_node(rclcpp::CallbackGroup::SharedPtr group) override;

  /// 遍历存储的回调组，对每个有效的回调组调用给定的函数。
  /// Iterate over the stored callback groups, calling the given function on each valid one.
  /**
   * 以线程安全的方式调用此方法，并确保仅对仍然有效的项调用给定的函数。
   * This method is called in a thread-safe way, and also makes sure to only call the given
   * function on those items that are still valid.
   *
   * \param[in] func 每个有效回调组上要调用的回调函数。
   * \param[in] func The callback function to call on each valid callback group.
   */
  RCLCPP_PUBLIC
  void for_each_callback_group(const CallbackGroupFunction& func) override;

  /// 获取与执行器关联的原子变量引用
  /// Get the reference of the atomic variable associated with the executor
  RCLCPP_PUBLIC
  std::atomic_bool& get_associated_with_executor_atomic() override;

  /// 获取通知保护条件的引用
  /// Get the reference of the notify guard condition
  RCLCPP_PUBLIC
  rclcpp::GuardCondition& get_notify_guard_condition() override;

  /// 获取是否使用内部进程通信的默认值
  /// Get the default value for using intra-process communication
  RCLCPP_PUBLIC
  bool get_use_intra_process_default() const override;

  /// 获取是否启用主题统计信息的默认值
  /// Get the default value for enabling topic statistics
  bool get_enable_topic_statistics_default() const override;

  /// 解析主题或服务名称（考虑命名空间和扩展）
  /// Resolve the topic or service name (considering namespace and expansion)
  /**
   * \param[in] name 要解析的主题或服务名称
   * \param[in] name The topic or service name to resolve
   * \param[in] is_service 是否是服务名称，如果为主题则为 false
   * \param[in] is_service Whether it's a service name, false if it's a topic
   * \param[in] only_expand 仅扩展命名空间，默认为 false
   * \param[in] only_expand Only expand the namespace, default is false
   *
   * \return 解析后的主题或服务名称
   * \return The resolved topic or service name
   */
  std::string resolve_topic_or_service_name(
      const std::string& name, bool is_service, bool only_expand = false) const override;

private:
  /**
   * @brief 禁用拷贝构造函数和赋值操作符 (Disable copy constructor and assignment operator)
   */
  RCLCPP_DISABLE_COPY(NodeBase)

  /// ROS2 上下文共享指针 (Shared pointer to ROS2 context)
  rclcpp::Context::SharedPtr context_;

  /// 默认使用进程内通信 (Default use of intra-process communication)
  bool use_intra_process_default_;

  /// 默认启用主题统计 (Default enable topic statistics)
  bool enable_topic_statistics_default_;

  /// 节点句柄共享指针 (Shared pointer to node handle)
  std::shared_ptr<rcl_node_t> node_handle_;

  /// 默认回调组共享指针 (Shared pointer to default callback group)
  rclcpp::CallbackGroup::SharedPtr default_callback_group_;

  /// 互斥锁，保护回调组 (Mutex to protect callback groups)
  std::mutex callback_groups_mutex_;

  /// 弱指针向量，存储回调组 (Vector of weak pointers to store callback groups)
  std::vector<rclcpp::CallbackGroup::WeakPtr> callback_groups_;

  /// 原子布尔值，表示节点是否与执行器关联 (Atomic boolean representing if the node is associated
  /// with an executor)
  std::atomic_bool associated_with_executor_;

  /**
   * @brief 通知执行器节点变化的守卫条件 (Guard condition for notifying the Executor of changes to
   * this node)
   */
  mutable std::recursive_mutex notify_guard_condition_mutex_;
  rclcpp::GuardCondition notify_guard_condition_;
  bool notify_guard_condition_is_valid_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_BASE_HPP_

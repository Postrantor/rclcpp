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

#ifndef RCLCPP__NODE_INTERFACES__NODE_BASE_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_BASE_INTERFACE_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include "rcl/node.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/detail/node_interfaces_helpers.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace node_interfaces {

/// 纯虚拟接口类，用于 Node API 的 NodeBase 部分。
class NodeBaseInterface {
public:
  // 智能指针别名，仅用于 NodeBaseInterface 类型
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeBaseInterface)

  // 公共成员函数
  RCLCPP_PUBLIC
  // 虚析构函数，允许通过基类指针删除派生类对象
  virtual ~NodeBaseInterface() = default;

  /** 返回节点的名称。
   * \return 节点的名称。
   */
  RCLCPP_PUBLIC
  virtual const char* get_name() const = 0;

  /** 返回节点的命名空间。
   * \return 节点的命名空间。
   */
  RCLCPP_PUBLIC
  virtual const char* get_namespace() const = 0;

  /** 返回节点的完全限定名称。
   * \return 节点的完全限定名称。
   */
  RCLCPP_PUBLIC
  virtual const char* get_fully_qualified_name() const = 0;

  /** 返回节点的上下文。
   * \return 指向节点上下文的SharedPtr。
   */
  RCLCPP_PUBLIC
  virtual rclcpp::Context::SharedPtr get_context() = 0;

  /// 返回 rcl_node_t 节点句柄（非const版本）。
  RCLCPP_PUBLIC
  virtual rcl_node_t* get_rcl_node_handle() = 0;

  /// 返回 rcl_node_t 节点句柄（const版本）。
  RCLCPP_PUBLIC
  virtual const rcl_node_t* get_rcl_node_handle() const = 0;

  /** 返回一个 std::shared_ptr 中的 rcl_node_t 节点句柄。
   * 当节点被销毁后，此句柄仍然有效。
   * 实际的 rcl 节点在所有范围内都不会被最终确定。
   */
  RCLCPP_PUBLIC
  virtual std::shared_ptr<rcl_node_t> get_shared_rcl_node_handle() = 0;

  /** 返回一个 std::shared_ptr 中的 rcl_node_t 节点句柄。
   * 当节点被销毁后，此句柄仍然有效。
   * 实际的 rcl 节点在所有范围内都不会被最终确定。
   */
  RCLCPP_PUBLIC
  virtual std::shared_ptr<const rcl_node_t> get_shared_rcl_node_handle() const = 0;

  /// 创建并返回一个回调组。
  RCLCPP_PUBLIC
  virtual rclcpp::CallbackGroup::SharedPtr create_callback_group(
      rclcpp::CallbackGroupType group_type,
      bool automatically_add_to_executor_with_node = true) = 0;

  /// 返回默认的回调组。
  RCLCPP_PUBLIC
  virtual rclcpp::CallbackGroup::SharedPtr get_default_callback_group() = 0;

  /// 如果给定的回调组与此节点关联，则返回 true。
  RCLCPP_PUBLIC
  virtual bool callback_group_in_node(rclcpp::CallbackGroup::SharedPtr group) = 0;

  // 定义 CallbackGroupFunction 类型，该类型为一个函数，接受 rclcpp::CallbackGroup::SharedPtr 参数。
  using CallbackGroupFunction = std::function<void(rclcpp::CallbackGroup::SharedPtr)>;

  /** 遍历存储的回调组，对每个有效的回调组调用给定的函数。
   * 以线程安全的方式调用此方法，并确保只对仍然有效的项目调用给定的函数。
   * \param[in] func 要在每个有效回调组上调用的回调函数。
   */
  RCLCPP_PUBLIC
  virtual void for_each_callback_group(const CallbackGroupFunction& func) = 0;

  /// 返回用于确保只使用一个执行器的原子布尔值。
  RCLCPP_PUBLIC
  virtual std::atomic_bool& get_associated_with_executor_atomic() = 0;

  /// 返回一个守护条件，当内部节点状态发生变化时应通知该条件。
  /**
   * 例如，在添加或删除发布者时应进行通知。
   *
   * \return 若 GuardCondition 有效，则返回之；否则抛出运行时错误。
   */
  RCLCPP_PUBLIC
  virtual rclcpp::GuardCondition& get_notify_guard_condition() = 0;

  /// 返回使用进程内通信的默认偏好设置。
  RCLCPP_PUBLIC
  virtual bool get_use_intra_process_default() const = 0;

  /// 返回启用主题统计信息收集的默认偏好设置。
  RCLCPP_PUBLIC
  virtual bool get_enable_topic_statistics_default() const = 0;

  /// 扩展并重映射给定的主题或服务名称。
  RCLCPP_PUBLIC
  virtual std::string resolve_topic_or_service_name(
      /// 主题或服务名称
      const std::string& name,
      /// 是否为服务
      bool is_service,
      /// 是否仅扩展，默认为 false
      bool only_expand = false) const = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeBaseInterface, base)

#endif  // RCLCPP__NODE_INTERFACES__NODE_BASE_INTERFACE_HPP_

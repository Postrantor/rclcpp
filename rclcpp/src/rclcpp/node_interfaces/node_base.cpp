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

#include "rclcpp/node_interfaces/node_base.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "../logging_mutex.hpp"
#include "rcl/arguments.h"
#include "rclcpp/exceptions.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

using rclcpp::exceptions::throw_from_rcl_error;

using rclcpp::node_interfaces::NodeBase;

/**
 * @brief 构造函数，用于创建一个 NodeBase 实例 (Constructor for creating a NodeBase instance)
 *
 * @param node_name 节点名称 (Node name)
 * @param namespace_ 命名空间 (Namespace)
 * @param context 共享指针，指向 rclcpp::Context 实例 (Shared pointer to an rclcpp::Context
 * instance)
 * @param rcl_node_options RCL 节点选项 (RCL node options)
 * @param use_intra_process_default 是否使用默认的进程内通信 (Whether to use the default
 * intra-process communication)
 * @param enable_topic_statistics_default 是否启用默认主题统计 (Whether to enable default topic
 * statistics)
 * @param default_callback_group 默认回调组的共享指针 (Shared pointer to the default callback group)
 */
NodeBase::NodeBase(
    const std::string& node_name,
    const std::string& namespace_,
    rclcpp::Context::SharedPtr context,
    const rcl_node_options_t& rcl_node_options,
    bool use_intra_process_default,
    bool enable_topic_statistics_default,
    rclcpp::CallbackGroup::SharedPtr default_callback_group)
    : context_(context),
      use_intra_process_default_(use_intra_process_default),
      enable_topic_statistics_default_(enable_topic_statistics_default),
      node_handle_(nullptr),
      default_callback_group_(default_callback_group),
      associated_with_executor_(false),
      notify_guard_condition_(context),
      notify_guard_condition_is_valid_(false) {
  // 创建 rcl 节点并将其存储在具有自定义析构函数的 shared_ptr 中
  std::unique_ptr<rcl_node_t> rcl_node(new rcl_node_t(rcl_get_zero_initialized_node()));

  // 获取全局日志互斥锁 (Get the global logging mutex)
  std::shared_ptr<std::recursive_mutex> logging_mutex = get_global_logging_mutex();

  rcl_ret_t ret;
  {
    std::lock_guard<std::recursive_mutex> guard(*logging_mutex);
    // TODO(ivanpauno): /rosout Qos should be reconfigurable.
    // TODO(ivanpauno): Instead of mutually excluding rcl_node_init with the global logger mutex,
    // rcl_logging_rosout_init_publisher_for_node could be decoupled from there and be called
    // here directly.
    // 初始化 rcl 节点 (Initialize the rcl node)
    ret = rcl_node_init(
        rcl_node.get(), node_name.c_str(), namespace_.c_str(), context_->get_rcl_context().get(),
        &rcl_node_options);
  }
  if (ret != RCL_RET_OK) {
    // 处理节点名称无效的情况 (Handle the case when the node name is invalid)
    if (ret == RCL_RET_NODE_INVALID_NAME) {
      rcl_reset_error();  // 丢弃 rcl_node_init 错误 (Discard the rcl_node_init error)
      int validation_result;
      size_t invalid_index;
      // 验证节点名称 (Validate the node name)
      rmw_ret_t rmw_ret =
          rmw_validate_node_name(node_name.c_str(), &validation_result, &invalid_index);
      if (rmw_ret != RMW_RET_OK) {
        if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
          throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "failed to validate node name");
        }
        throw_from_rcl_error(RCL_RET_ERROR, "failed to validate node name");
      }

      if (validation_result != RMW_NODE_NAME_VALID) {
        throw rclcpp::exceptions::InvalidNodeNameError(
            node_name.c_str(), rmw_node_name_validation_result_string(validation_result),
            invalid_index);
      } else {
        throw std::runtime_error("valid rmw node name but invalid rcl node name");
      }
    }

    // 处理命名空间无效的情况 (Handle the case when the namespace is invalid)
    if (ret == RCL_RET_NODE_INVALID_NAMESPACE) {
      rcl_reset_error();  // 丢弃 rcl_node_init 错误 (Discard the rcl_node_init error)
      int validation_result;
      size_t invalid_index;
      // 验证命名空间 (Validate the namespace)
      rmw_ret_t rmw_ret =
          rmw_validate_namespace(namespace_.c_str(), &validation_result, &invalid_index);
      if (rmw_ret != RMW_RET_OK) {
        if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
          throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "failed to validate namespace");
        }
        throw_from_rcl_error(RCL_RET_ERROR, "failed to validate namespace");
      }

      if (validation_result != RMW_NAMESPACE_VALID) {
        throw rclcpp::exceptions::InvalidNamespaceError(
            namespace_.c_str(), rmw_namespace_validation_result_string(validation_result),
            invalid_index);
      } else {
        throw std::runtime_error("valid rmw node namespace but invalid rcl node namespace");
      }
    }
    // 抛出 rcl 节点初始化失败的异常 (Throw an exception for the failed rcl node initialization)
    throw_from_rcl_error(ret, "failed to initialize rcl node");
  }

  // 使用自定义析构函数将 rcl 节点存储在 node_handle_ 中 (Store the rcl node in node_handle_ with a
  // custom destructor)
  node_handle_.reset(rcl_node.release(), [logging_mutex](rcl_node_t* node) -> void {
    std::lock_guard<std::recursive_mutex> guard(*logging_mutex);
    // TODO(ivanpauno): Instead of mutually excluding rcl_node_fini with the global logger mutex,
    // rcl_logging_rosout_fini_publisher_for_node could be decoupled from there and be called
    // here directly.
    if (rcl_node_fini(node) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
          "rclcpp", "Error in destruction of rcl node handle: %s", rcl_get_error_string().str);
    }
    delete node;
  });

  // 如果需要，创建默认回调组 (Create the default callback group, if needed)
  if (nullptr == default_callback_group_) {
    using rclcpp::CallbackGroupType;
    default_callback_group_ = NodeBase::create_callback_group(CallbackGroupType::MutuallyExclusive);
  }

  // 表示 notify_guard_condition 现在有效 (Indicate the notify_guard_condition is now valid)
  notify_guard_condition_is_valid_ = true;
}

/**
 * @brief 析构函数，用于释放 NodeBase 资源 (Destructor for releasing NodeBase resources)
 */
NodeBase::~NodeBase() {
  // 在从图形监听器中移除自身后，完成中断保护条件的最终操作 (Finalize the interrupt guard condition
  // after removing self from graph listener)
  {
    std::lock_guard<std::recursive_mutex> notify_condition_lock(notify_guard_condition_mutex_);
    // 将通知保护条件标记为无效 (Mark the notify guard condition as invalid)
    notify_guard_condition_is_valid_ = false;
  }
}

/**
 * @brief 获取节点名称 (Get the node name)
 * @return 节点名称 (The node name)
 */
const char* NodeBase::get_name() const { return rcl_node_get_name(node_handle_.get()); }

/**
 * @brief 获取节点命名空间 (Get the node namespace)
 * @return 节点命名空间 (The node namespace)
 */
const char* NodeBase::get_namespace() const { return rcl_node_get_namespace(node_handle_.get()); }

/**
 * @brief 获取节点完全限定名称 (Get the node fully qualified name)
 * @return 节点完全限定名称 (The node fully qualified name)
 */
const char* NodeBase::get_fully_qualified_name() const {
  return rcl_node_get_fully_qualified_name(node_handle_.get());
}

/**
 * @brief 获取节点上下文 (Get the node context)
 * @return 节点上下文共享指针 (A shared pointer to the node context)
 */
rclcpp::Context::SharedPtr NodeBase::get_context() { return context_; }

/**
 * @brief 获取 rcl_node_t 句柄 (Get the rcl_node_t handle)
 * @return rcl_node_t 句柄指针 (A pointer to the rcl_node_t handle)
 */
rcl_node_t* NodeBase::get_rcl_node_handle() { return node_handle_.get(); }

/**
 * @brief 获取 rcl_node_t 句柄 (Get the rcl_node_t handle)
 * @return rcl_node_t 句柄常量指针 (A constant pointer to the rcl_node_t handle)
 */
const rcl_node_t* NodeBase::get_rcl_node_handle() const { return node_handle_.get(); }

/**
 * @brief 获取共享的 rcl_node_t 句柄 (Get the shared rcl_node_t handle)
 * @return rcl_node_t 共享指针 (A shared pointer to the rcl_node_t handle)
 */
std::shared_ptr<rcl_node_t> NodeBase::get_shared_rcl_node_handle() {
  return std::shared_ptr<rcl_node_t>(shared_from_this(), node_handle_.get());
}

/**
 * @brief 获取共享的 rcl_node_t 句柄 (Get the shared rcl_node_t handle)
 * @return rcl_node_t 共享常量指针 (A shared constant pointer to the rcl_node_t handle)
 */
std::shared_ptr<const rcl_node_t> NodeBase::get_shared_rcl_node_handle() const {
  return std::shared_ptr<const rcl_node_t>(shared_from_this(), node_handle_.get());
}

/**
 * @brief 创建回调组 (Create a callback group)
 * @param group_type 回调组类型 (The callback group type)
 * @param automatically_add_to_executor_with_node 是否自动添加到与节点关联的执行器中 (Whether to
 * automatically add to the executor associated with the node)
 * @return 回调组共享指针 (A shared pointer to the callback group)
 */
rclcpp::CallbackGroup::SharedPtr NodeBase::create_callback_group(
    rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node) {
  // 创建回调组对象 (Create a callback group object)
  auto group =
      std::make_shared<rclcpp::CallbackGroup>(group_type, automatically_add_to_executor_with_node);
  // 锁定回调组互斥锁，以确保线程安全 (Lock the callback groups mutex to ensure thread safety)
  std::lock_guard<std::mutex> lock(callback_groups_mutex_);
  // 将新创建的回调组添加到回调组列表中 (Add the newly created callback group to the list of
  // callback groups)
  callback_groups_.push_back(group);
  // 返回回调组共享指针 (Return the shared pointer to the callback group)
  return group;
}

/**
 * @brief 获取默认的回调组 (Get the default callback group)
 *
 * @return 返回一个共享指针，指向默认的回调组 (Return a shared pointer to the default callback
 * group)
 */
rclcpp::CallbackGroup::SharedPtr NodeBase::get_default_callback_group() {
  // 返回默认回调组 (Return the default callback group)
  return default_callback_group_;
}

/**
 * @brief 检查给定的回调组是否在节点中
 *
 * @param group 需要检查的回调组的共享指针
 * @return 如果回调组在节点中，则返回 true，否则返回 false
 */
bool NodeBase::callback_group_in_node(rclcpp::CallbackGroup::SharedPtr group) {
  // 锁定互斥体以保护回调组列表
  std::lock_guard<std::mutex> lock(callback_groups_mutex_);
  // 遍历回调组列表 (Iterate over the callback groups list)
  for (auto& weak_group : this->callback_groups_) {
    // 尝试获取当前回调组的共享指针
    auto cur_group = weak_group.lock();
    // 如果当前回调组存在且与给定的回调组相同，则返回 true
    if (cur_group && (cur_group == group)) {
      return true;
    }
  }
  // 如果没有找到给定的回调组，则返回 false
  return false;
}

/**
 * @brief 对每个回调组应用指定的函数 (Apply the specified function to each callback group)
 *
 * @param func 要应用于每个回调组的函数 (The function to apply to each callback group)
 */
void NodeBase::for_each_callback_group(const CallbackGroupFunction& func) {
  // 锁定互斥体以保护回调组列表
  std::lock_guard<std::mutex> lock(callback_groups_mutex_);
  // 遍历回调组列表 (Iterate over the callback groups list)
  for (rclcpp::CallbackGroup::WeakPtr& weak_group : this->callback_groups_) {
    // 尝试获取当前回调组的共享指针
    rclcpp::CallbackGroup::SharedPtr group = weak_group.lock();
    // 如果当前回调组存在，则对其应用指定的函数 (If the current callback group exists, apply the
    // specified function to it)
    if (group) {
      func(group);
    }
  }
}

/**
 * @brief 获取与执行器关联的原子布尔值 (Get the atomic boolean associated with the executor)
 *
 * @return 返回与执行器关联的原子布尔值的引用 (Return a reference to the atomic boolean associated
 * with the executor)
 */
std::atomic_bool& NodeBase::get_associated_with_executor_atomic() {
  // 返回与执行器关联的原子布尔值 (Return the atomic boolean associated with the executor)
  return associated_with_executor_;
}

/**
 * @brief 获取通知保护条件 (Get the notify guard condition)
 *
 * @return 返回通知保护条件的引用 (Return a reference to the notify guard condition)
 */
rclcpp::GuardCondition& NodeBase::get_notify_guard_condition() {
  // 锁定递归互斥体以保护通知保护条件 (Lock the recursive mutex to protect the notify guard
  // condition)
  std::lock_guard<std::recursive_mutex> notify_condition_lock(notify_guard_condition_mutex_);
  // 如果通知保护条件无效，则抛出异常 (Throw an exception if the notify guard condition is invalid)
  if (!notify_guard_condition_is_valid_) {
    throw std::runtime_error("failed to get notify guard condition because it is invalid");
  }
  // 返回通知保护条件 (Return the notify guard condition)
  return notify_guard_condition_;
}

/**
 * @brief 获取默认的内部进程通信设置 (Get the default intra-process communication setting)
 *
 * @return 返回默认的内部进程通信设置 (Return the default intra-process communication setting)
 */
bool NodeBase::get_use_intra_process_default() const { return use_intra_process_default_; }

/**
 * @brief 获取默认的主题统计设置 (Get the default topic statistics setting)
 *
 * @return 返回默认的主题统计设置 (Return the default topic statistics setting)
 */
bool NodeBase::get_enable_topic_statistics_default() const {
  return enable_topic_statistics_default_;
}

/**
 * @brief 解析主题或服务名称 (Resolve the topic or service name)
 *
 * @param name 要解析的名称 (The name to resolve)
 * @param is_service 如果为 true，则表示解析服务名称；否则解析主题名称 (If true, resolve a service
 * name; otherwise, resolve a topic name)
 * @param only_expand 如果为 true，则仅扩展名称，不进行解析 (If true, only expand the name without
 * resolving it)
 * @return 返回解析后的名称 (Return the resolved name)
 */
std::string NodeBase::resolve_topic_or_service_name(
    const std::string& name, bool is_service, bool only_expand) const {
  char* output_cstr = NULL;
  // 获取默认分配器 (Get the default allocator)
  auto allocator = rcl_get_default_allocator();
  // 调用 rcl_node_resolve_name 函数来解析名称 (Call the rcl_node_resolve_name function to resolve
  // the name)
  rcl_ret_t ret = rcl_node_resolve_name(
      node_handle_.get(), name.c_str(), allocator, is_service, only_expand, &output_cstr);
  // 如果解析失败，则抛出异常 (Throw an exception if the resolution fails)
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to resolve name", rcl_get_error_state());
  }
  // 将输出的 C 字符串转换为 std::string (Convert the output C string to std::string)
  std::string output{output_cstr};
  // 释放输出的 C 字符串 (Deallocate the output C string)
  allocator.deallocate(output_cstr, allocator.state);
  // 返回解析后的名称 (Return the resolved name)
  return output;
}

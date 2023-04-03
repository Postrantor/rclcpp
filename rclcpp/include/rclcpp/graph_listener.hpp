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

#ifndef RCLCPP__GRAPH_LISTENER_HPP_
#define RCLCPP__GRAPH_LISTENER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rcl/guard_condition.h"
#include "rcl/wait.h"
#include "rclcpp/context.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

namespace graph_listener {

/// Thrown when a function is called on a GraphListener that is already shutdown.
/// 当在已关闭的GraphListener上调用函数时抛出
class GraphListenerShutdownError : public std::runtime_error {
public:
  /// Constructor with a default error message.
  /// 构造函数，带有默认错误消息
  GraphListenerShutdownError() : std::runtime_error("GraphListener already shutdown") {}
};

/// Thrown when a node has already been added to the GraphListener.
/// 当节点已经添加到GraphListener时抛出
class NodeAlreadyAddedError : public std::runtime_error {
public:
  /// Constructor with a default error message.
  /// 构造函数，带有默认错误消息
  NodeAlreadyAddedError() : std::runtime_error("node already added") {}
};

/// Thrown when the given node is not in the GraphListener.
/// 当给定节点不在GraphListener中时抛出
class NodeNotFoundError : public std::runtime_error {
public:
  /// Constructor with a default error message.
  /// 构造函数，带有默认错误消息
  NodeNotFoundError() : std::runtime_error("node not found") {}
};

/// Notifies many nodes of graph changes by listening in a thread.
/// 通过在线程中监听来通知许多节点图形更改
class GraphListener : public std::enable_shared_from_this<GraphListener> {
public:
  /// Constructor taking a shared pointer to the parent context.
  /// 接受指向父上下文的共享指针的构造函数
  RCLCPP_PUBLIC
  explicit GraphListener(const rclcpp::Context::SharedPtr &parent_context);

  /// Destructor.
  /// 析构函数
  RCLCPP_PUBLIC
  virtual ~GraphListener();

  /// Start the graph listener's listen thread if it hasn't been started.
  /**
   * This function is thread-safe.
   * 此函数是线程安全的
   *
   * \throws GraphListenerShutdownError if the GraphListener is shutdown
   * 如果GraphListener已关闭，则抛出GraphListenerShutdownError
   * \throws std::runtime if the parent context was destroyed
   * 如果父上下文被销毁，抛出std::runtime
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   * 抛出rclcpp::exceptions::throw_from_rcl_error可能抛出的任何内容
   */
  RCLCPP_PUBLIC
  virtual void start_if_not_started();

  /// Add a node to the graph listener's list of nodes.
  /**
   * \throws GraphListenerShutdownError if the GraphListener is shutdown
   * 如果GraphListener已关闭，则抛出GraphListenerShutdownError
   * \throws NodeAlreadyAddedError if the given node is already in the list
   * 如果给定节点已在列表中，则抛出NodeAlreadyAddedError
   * \throws std::invalid_argument if node is nullptr
   * 如果节点为空指针，则抛出std::invalid_argument
   * \throws std::system_error anything std::mutex::lock() throws
   * 抛出std::mutex::lock()可能抛出的任何std::system_error
   */
  RCLCPP_PUBLIC
  virtual void add_node(rclcpp::node_interfaces::NodeGraphInterface *node_graph);

  /// Return true if the given node is in the graph listener's list of nodes.
  /**
   * Also return false if given nullptr.
   * 如果给定空指针，也返回false
   *
   * \throws std::system_error anything std::mutex::lock() throws
   * 抛出std::mutex::lock()可能抛出的任何std::system_error
   */
  RCLCPP_PUBLIC
  virtual bool has_node(rclcpp::node_interfaces::NodeGraphInterface *node_graph);

  /// Remove a node from the graph listener's list of nodes.
  /**
   *
   * \throws NodeNotFoundError if the given node is not in the list
   * 如果给定节点不在列表中，则抛出NodeNotFoundError
   * \throws std::invalid_argument if node is nullptr
   * 如果节点为空指针，则抛出std::invalid_argument
   * \throws std::system_error anything std::mutex::lock() throws
   * 抛出std::mutex::lock()可能抛出的任何std::system_error
   */
  RCLCPP_PUBLIC
  virtual void remove_node(rclcpp::node_interfaces::NodeGraphInterface *node_graph);

  /// Stop the listening thread.
  /**
   * The thread cannot be restarted, and the class is defunct after calling.
   * 调用后，线程无法重新启动，类已失效
   * This function is called by the ~GraphListener() and does nothing if
   * shutdown() was already called.
   * 如果已经调用了shutdown()，则此函数由~GraphListener()调用，并且不执行任何操作
   * This function exists separately from the ~GraphListener() so that it can
   * be called before and exceptions can be caught.
   * 此函数与~GraphListener()分开存在，以便可以在异常捕获之前调用它
   *
   * If start_if_not_started() was never called, this function still succeeds,
   * but start_if_not_started() still cannot be called after this function.
   * 如果从未调用start_if_not_started()，此函数仍然成功，
   * 但在此函数之后仍无法调用start_if_not_started()
   *
   * Note that if you override this method, but leave shutdown to be called in
   * the destruction of this base class, it will not call the overridden
   * version from your base class.
   * 因此，如果您重写此方法，但是将shutdown留在此基类的析构中调用，
   * 它将不会从您的基类中调用被重写的版本
   * So you need to ensure you call your class's shutdown() in its destructor.
   * 因此，您需要确保在其析构函数中调用类的shutdown()
   *
   * \throws rclcpp::execptions::RCLError from rcl_guard_condition_fini()
   * 来自rcl_guard_condition_fini()的rclcpp::execptions::RCLError
   * \throws rclcpp::execptions::RCLError from rcl_wait_set_fini()
   * 来自rcl_wait_set_fini()的rclcpp::execptions::RCLError
   * \throws std::system_error anything std::mutex::lock() throws
   * 抛出std::mutex::lock()可能抛出的任何std::system_error
   */
  RCLCPP_PUBLIC
  virtual void shutdown();

  /// Nothrow version of shutdown(), logs to RCLCPP_ERROR instead.
  /// shutdown()的无抛出版本，改为记录到RCLCPP_ERROR
  RCLCPP_PUBLIC
  virtual void shutdown(const std::nothrow_t &) noexcept;

  /// Return true if shutdown() has been called, else false.
  /// 如果已调用shutdown()，则返回true；否则返回false
  RCLCPP_PUBLIC
  virtual bool is_shutdown();

protected:
  /// Main function for the listening thread.
  /// 监听线程的主要功能
  RCLCPP_PUBLIC
  virtual void run();

  /// The loop that runs inside the listening thread.
  /// 在监听线程内部运行的循环
  RCLCPP_PUBLIC
  virtual void run_loop();

  /// Initialize the wait set.
  /// 初始化等待集
  RCLCPP_PUBLIC
  void init_wait_set();

  /// Clean up the wait set.
  /// 清理等待集
  RCLCPP_PUBLIC
  void cleanup_wait_set();

private:
  /// Private method to handle the actual shutdown logic.
  /// 处理实际关闭逻辑的私有方法
  void __shutdown();

  // Member variables
  std::weak_ptr<rclcpp::Context> weak_parent_context_;
  std::shared_ptr<rcl_context_t> rcl_parent_context_;

  std::thread listener_thread_;
  bool is_started_;
  std::atomic_bool is_shutdown_;
  mutable std::mutex shutdown_mutex_;

  mutable std::mutex node_graph_interfaces_barrier_mutex_;
  mutable std::mutex node_graph_interfaces_mutex_;
  std::vector<rclcpp::node_interfaces::NodeGraphInterface *> node_graph_interfaces_;

  rclcpp::GuardCondition interrupt_guard_condition_;
  rcl_wait_set_t wait_set_ = rcl_get_zero_initialized_wait_set();
};

}  // namespace graph_listener
}  // namespace rclcpp

#endif  // RCLCPP__GRAPH_LISTENER_HPP_

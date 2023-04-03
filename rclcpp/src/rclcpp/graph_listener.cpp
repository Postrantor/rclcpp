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

#include "rclcpp/graph_listener.hpp"

#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "rcl/error_handling.h"
#include "rcl/types.h"
#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/impl/cpp/demangle.hpp"

using rclcpp::exceptions::throw_from_rcl_error;

namespace rclcpp {
namespace graph_listener {

/**
 * @brief 构造函数，初始化 GraphListener 对象 (Constructor, initializes the GraphListener object)
 * @param parent_context 父上下文的共享指针 (Shared pointer to the parent context)
 */
GraphListener::GraphListener(const std::shared_ptr<Context> &parent_context)
    : weak_parent_context_(parent_context),
      rcl_parent_context_(parent_context->get_rcl_context()),
      is_started_(false),
      is_shutdown_(false),
      interrupt_guard_condition_(parent_context) {}

/**
 * @brief 析构函数，关闭 GraphListener 对象 (Destructor, shuts down the GraphListener object)
 */
GraphListener::~GraphListener() { GraphListener::shutdown(std::nothrow); }

/**
 * @brief 初始化 wait set (Initialize the wait set)
 */
void GraphListener::init_wait_set() {
  rcl_ret_t ret = rcl_wait_set_init(
      &wait_set_,
      0,  // number_of_subscriptions 订阅数量 (Number of subscriptions)
      2,  // number_of_guard_conditions 守卫条件数量 (Number of guard conditions)
      0,  // number_of_timers 定时器数量 (Number of timers)
      0,  // number_of_clients 客户端数量 (Number of clients)
      0,  // number_of_services 服务数量 (Number of services)
      0,  // number_of_events 事件数量 (Number of events)
      rcl_parent_context_.get(), rcl_get_default_allocator());
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to initialize wait set");
  }
}

/**
 * @brief 如果尚未启动，则启动 GraphListener (Starts the GraphListener if not started)
 */
void GraphListener::start_if_not_started() {
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  if (is_shutdown_.load()) {
    throw GraphListenerShutdownError();
  }
  auto parent_context = weak_parent_context_.lock();
  if (!is_started_ && parent_context) {
    // 注册一个在关闭时的钩子，用于关闭图形监听器 (Register an on_shutdown hook to shutdown the
    // graph listener) 这对于确保在静态对象销毁之前完成 wait set 的终止非常重要 (This is important
    // to ensure that the wait set is finalized before destruction of static objects occurs)
    std::weak_ptr<GraphListener> weak_this = shared_from_this();
    parent_context->on_shutdown([weak_this]() {
      auto shared_this = weak_this.lock();
      if (shared_this) {
        // 如果可以避免，不应从 on_shutdown 中抛出异常 (Should not throw from on_shutdown if it can
        // be avoided)
        shared_this->shutdown(std::nothrow);
      }
    });
    // 在启动之前初始化 wait set (Initialize the wait set before starting)
    init_wait_set();
    // 启动监听器线程 (Start the listener thread)
    listener_thread_ = std::thread(&GraphListener::run, this);
    is_started_ = true;
  }
}

/**
 * @brief 运行 GraphListener 线程，捕获异常并记录错误日志。
 * @details Run the GraphListener thread, catch exceptions and log error messages.
 */
void GraphListener::run() {
  try {
    run_loop();
  } catch (const std::exception &exc) {
    // 捕获异常，并记录详细的错误日志。
    // Catch the exception and log a detailed error message.
    RCUTILS_LOG_ERROR_NAMED(
        "rclcpp", "caught %s exception in GraphListener thread: %s",
        rmw::impl::cpp::demangle(exc).c_str(), exc.what());
    std::rethrow_exception(std::current_exception());
  } catch (...) {
    // 捕获未知异常，并记录错误日志。
    // Catch unknown exceptions and log an error message.
    RCUTILS_LOG_ERROR_NAMED("rclcpp", "unknown error in GraphListener thread");
    std::rethrow_exception(std::current_exception());
  }
}

/**
 * @brief GraphListener 的主循环，监视节点图状态变化并通知相关节点。
 * @details The main loop of GraphListener, monitoring node graph state changes and notifying
 * relevant nodes.
 */
void GraphListener::run_loop() {
  while (true) {
    // 如果调用了 shutdown()，则退出循环。
    // If shutdown() was called, exit the loop.
    if (is_shutdown_.load()) {
      return;
    }
    rcl_ret_t ret;
    {
      // 此 "barrier" 锁确保其他函数在唤醒 rcl_wait 后可以获取 node_graph_interfaces_mutex_。
      // This "barrier" lock ensures that other functions can acquire the
      // node_graph_interfaces_mutex_ after waking up rcl_wait.
      std::lock_guard<std::mutex> nodes_barrier_lock(node_graph_interfaces_barrier_mutex_);
      // 这个锁在下一行传递给 nodes_lock。
      // This lock is passed to nodes_lock in the next line.
      node_graph_interfaces_mutex_.lock();
    }
    // 当循环继续或退出时，此锁将被释放。
    // This lock is released when the loop continues or exits.
    std::lock_guard<std::mutex> nodes_lock(node_graph_interfaces_mutex_, std::adopt_lock);
    // 如果需要，调整 wait_set 的大小。
    // Resize the wait set if necessary.
    const size_t node_graph_interfaces_size = node_graph_interfaces_.size();
    // 为中断和关闭守卫条件添加 2
    // Add 2 for the interrupt and shutdown guard conditions
    if (wait_set_.size_of_guard_conditions < (node_graph_interfaces_size + 2)) {
      ret = rcl_wait_set_resize(&wait_set_, 0, node_graph_interfaces_size + 2, 0, 0, 0, 0);
      if (RCL_RET_OK != ret) {
        throw_from_rcl_error(ret, "failed to resize wait set");
      }
    }
    // 清空 wait_set。
    // Clear the wait set.
    ret = rcl_wait_set_clear(&wait_set_);
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to clear wait set");
    }
    // 将中断守卫条件放入 wait_set。
    // Put the interrupt guard condition in the wait set.
    detail::add_guard_condition_to_rcl_wait_set(wait_set_, interrupt_guard_condition_);

    // 将每个节点的图守卫条件放入 wait_set。
    // Put graph guard conditions for each node into the wait set.
    std::vector<size_t> graph_gc_indexes(node_graph_interfaces_size, 0u);
    for (size_t i = 0u; i < node_graph_interfaces_size; ++i) {
      auto node_ptr = node_graph_interfaces_[i];
      // 只有当节点的某个用户正在观察时，才等待图变化。
      // Only wait on graph changes if some user of the node is watching.
      if (node_ptr->count_graph_users() == 0) {
        continue;
      }
      // 将节点的图守卫条件添加到 wait_set。
      // Add the graph guard condition for the node to the wait set.
      auto graph_gc = node_ptr->get_graph_guard_condition();
      if (!graph_gc) {
        throw_from_rcl_error(RCL_RET_ERROR, "failed to get graph guard condition");
      }
      ret = rcl_wait_set_add_guard_condition(&wait_set_, graph_gc, &graph_gc_indexes[i]);
      if (RCL_RET_OK != ret) {
        throw_from_rcl_error(ret, "failed to add graph guard condition to wait set");
      }
    }

    // 等待：图变化、中断或关闭/SIGINT
    // Wait for: graph changes, interrupt, or shutdown/SIGINT
    ret = rcl_wait(&wait_set_, -1);  // block forever until a guard condition is triggered
    if (RCL_RET_TIMEOUT == ret) {
      throw std::runtime_error("rcl_wait unexpectedly timed out");
    }
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to wait on wait set");
    }

    // 通知守卫条件被触发（设置）的节点。
    // Notify nodes whose guard conditions are set (triggered).
    for (size_t i = 0u; i < node_graph_interfaces_size; ++i) {
      const auto node_ptr = node_graph_interfaces_[i];
      auto graph_gc = node_ptr->get_graph_guard_condition();
      if (!graph_gc) {
        throw_from_rcl_error(RCL_RET_ERROR, "failed to get graph guard condition");
      }
      if (graph_gc == wait_set_.guard_conditions[graph_gc_indexes[i]]) {
        node_ptr->notify_graph_change();
      }
      if (is_shutdown_) {
        // 如果关闭，则还要通知该节点。
        // If shutdown, also notify the node of this.
        node_ptr->notify_shutdown();
      }
    }
  }  // while (true)
}

/**
 * @brief 中断函数 (Interrupt function)
 *
 * @param[in] interrupt_guard_condition 指向 GuardCondition 的指针，用于触发中断 (Pointer to
 * GuardCondition to trigger the interrupt)
 */
static void interrupt_(GuardCondition *interrupt_guard_condition) {
  // 触发中断守卫条件 (Trigger the interrupt guard condition)
  interrupt_guard_condition->trigger();
}

/**
 * @brief 获取节点锁 (Acquire nodes lock)
 *
 * @param[in] node_graph_interfaces_barrier_mutex 节点图接口屏障互斥锁的指针 (Pointer to the node
 * graph interfaces barrier mutex)
 * @param[in] node_graph_interfaces_mutex 节点图接口互斥锁的指针 (Pointer to the node graph
 * interfaces mutex)
 * @param[in] interrupt_guard_condition 指向 GuardCondition 的指针，用于触发中断 (Pointer to
 * GuardCondition to trigger the interrupt)
 */
static void acquire_nodes_lock_(
    std::mutex *node_graph_interfaces_barrier_mutex,
    std::mutex *node_graph_interfaces_mutex,
    GuardCondition *interrupt_guard_condition) {
  {
    // 获取此锁以防止唤醒后的运行循环重新锁定 nodes_mutext_
    // (Acquire this lock to prevent the run loop from re-locking the nodes_mutext_ after being
    // woken up)
    std::lock_guard<std::mutex> nodes_barrier_lock(*node_graph_interfaces_barrier_mutex);
    // 触发中断守卫条件以唤醒 rcl_wait (Trigger the interrupt guard condition to wake up rcl_wait)
    interrupt_(interrupt_guard_condition);
    node_graph_interfaces_mutex->lock();
  }
}

/**
 * @brief 检查节点是否存在 (Check if node exists)
 *
 * @param[in] node_graph_interfaces 节点图接口的指针向量 (Pointer vector of node graph interfaces)
 * @param[in] node_graph 要检查的节点图接口指针 (Pointer to the node graph interface to check for)
 * @return 如果节点存在，则返回 true；否则，返回 false (Returns true if the node exists, otherwise
 * returns false)
 */
static bool has_node_(
    std::vector<rclcpp::node_interfaces::NodeGraphInterface *> *node_graph_interfaces,
    rclcpp::node_interfaces::NodeGraphInterface *node_graph) {
  for (const auto node_ptr : (*node_graph_interfaces)) {
    if (node_graph == node_ptr) {
      return true;
    }
  }
  return false;
}

bool GraphListener::has_node(rclcpp::node_interfaces::NodeGraphInterface *node_graph) {
  if (!node_graph) {
    return false;
  }
  // 使用屏障获取节点互斥锁，以防止运行循环在被中断后重新锁定节点互斥锁
  // (Acquire the nodes mutex using the barrier to prevent the run loop from re-locking the nodes
  // mutex after being interrupted)
  acquire_nodes_lock_(
      &node_graph_interfaces_barrier_mutex_, &node_graph_interfaces_mutex_,
      &interrupt_guard_condition_);
  // 将现在获取的 node_graph_interfaces_mutex_ 存储在 adopt_lock 中的范围锁中
  // (Store the now acquired node_graph_interfaces_mutex_ in the scoped lock using adopt_lock)
  std::lock_guard<std::mutex> nodes_lock(node_graph_interfaces_mutex_, std::adopt_lock);
  return has_node_(&node_graph_interfaces_, node_graph);
}

void GraphListener::add_node(rclcpp::node_interfaces::NodeGraphInterface *node_graph) {
  if (!node_graph) {
    throw std::invalid_argument("node is nullptr");
  }
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  if (is_shutdown_.load()) {
    throw GraphListenerShutdownError();
  }

  // 使用屏障获取节点互斥锁，以防止运行循环在被中断后重新锁定节点互斥锁
  // (Acquire the nodes mutex using the barrier to prevent the run loop from re-locking the nodes
  // mutex after being interrupted)
  acquire_nodes_lock_(
      &node_graph_interfaces_barrier_mutex_, &node_graph_interfaces_mutex_,
      &interrupt_guard_condition_);
  // 将现在获取的 node_graph_interfaces_mutex_ 存储在 adopt_lock 中的范围锁中
  // (Store the now acquired node_graph_interfaces_mutex_ in the scoped lock using adopt_lock)
  std::lock_guard<std::mutex> nodes_lock(node_graph_interfaces_mutex_, std::adopt_lock);
  if (has_node_(&node_graph_interfaces_, node_graph)) {
    throw NodeAlreadyAddedError();
  }
  node_graph_interfaces_.push_back(node_graph);
  // 运行循环已经被 acquire_nodes_lock_() 中断，并在 nodes_lock 释放 node_graph_interfaces_mutex_
  // 时评估新节点 (The run loop has already been interrupted by acquire_nodes_lock_() and will
  // evaluate the new node when nodes_lock releases the node_graph_interfaces_mutex_)
}

/*!
 * \brief 静态函数 remove_node_ 从 node_graph_interfaces 中移除指定的节点。
 *        Static function remove_node_ removes the specified node from node_graph_interfaces.
 * \param[in,out] node_graph_interfaces 节点图接口的向量，将从中删除指定的节点。
 *                  A vector of NodeGraphInterface pointers, from which the specified node will be
 * removed. \param[in] node_graph 需要从向量中删除的节点图接口。 The NodeGraphInterface to be
 * removed from the vector.
 */
static void remove_node_(
    std::vector<rclcpp::node_interfaces::NodeGraphInterface *> *node_graph_interfaces,
    rclcpp::node_interfaces::NodeGraphInterface *node_graph) {
  // 如果找到节点，则删除它。Remove the node if it is found.
  for (auto it = node_graph_interfaces->begin(); it != node_graph_interfaces->end(); ++it) {
    if (node_graph == *it) {
      // 找到节点，删除它。Found the node, remove it.
      node_graph_interfaces->erase(it);
      // 现在触发中断保护条件以确保 Now trigger the interrupt guard condition to make sure
      return;
    }
  }
  // 在循环中未找到。Not found in the loop.
  throw NodeNotFoundError();
}

/*!
 * \brief GraphListener 类的 remove_node 成员函数，用于从 node_graph_interfaces 中移除指定的节点。
 *        Member function remove_node of GraphListener class, used to remove the specified node from
 * node_graph_interfaces. \param[in] node_graph 需要从向量中删除的节点图接口。 The
 * NodeGraphInterface to be removed from the vector.
 */
void GraphListener::remove_node(rclcpp::node_interfaces::NodeGraphInterface *node_graph) {
  if (!node_graph) {
    throw std::invalid_argument("node is nullptr");
  }
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  if (is_shutdown()) {
    // 如果关闭，则运行循环已连接，因此我们可以直接删除它们。If shutdown, then the run loop has been
    // joined, so we can remove them directly.
    return remove_node_(&node_graph_interfaces_, node_graph);
  }
  // 否则，首先中断并锁定运行循环以安全地删除节点。Otherwise, first interrupt and lock against the
  // run loop to safely remove the node.
  // 使用障碍物获取节点互斥锁，以防止运行循环在被中断后重新锁定节点互斥锁。Acquire the nodes mutex
  // using the barrier to prevent the run loop from re-locking the nodes mutex after being
  // interrupted.
  acquire_nodes_lock_(
      &node_graph_interfaces_barrier_mutex_, &node_graph_interfaces_mutex_,
      &interrupt_guard_condition_);
  // 将现在获得的 node_graph_interfaces_mutex_ 存储在范围锁中，使用 adopt_lock。Store the now
  // acquired node_graph_interfaces_mutex_ in the scoped lock using adopt_lock.
  std::lock_guard<std::mutex> nodes_lock(node_graph_interfaces_mutex_, std::adopt_lock);
  remove_node_(&node_graph_interfaces_, node_graph);
}

/*!
 * \brief GraphListener 类的 cleanup_wait_set 成员函数，用于清理等待集。
 *        Member function cleanup_wait_set of GraphListener class, used to clean up the wait set.
 */
void GraphListener::cleanup_wait_set() {
  // 调用 rcl_wait_set_fini 函数来完成对 wait_set_ 的清理工作
  // Call rcl_wait_set_fini function to clean up the wait_set_
  rcl_ret_t ret = rcl_wait_set_fini(&wait_set_);

  // 检查 rcl_wait_set_fini 函数调用是否成功
  // Check if the rcl_wait_set_fini function call is successful
  if (RCL_RET_OK != ret) {
    // 如果失败，则抛出异常并附带错误信息
    // If failed, throw an exception with the error message
    throw_from_rcl_error(ret, "failed to finalize wait set");
  }
}

/*!
 * \brief GraphListener 类的 __shutdown 成员函数，用于关闭图形监听器。
 *        Member function __shutdown of GraphListener class, used to shut down the graph listener.
 */
void GraphListener::__shutdown() {
  // 使用 std::lock_guard 对 shutdown_mutex_ 进行加锁，防止多线程问题
  // Use std::lock_guard to lock shutdown_mutex_ to prevent multithreading issues
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);

  // 检查 is_shutdown_ 是否为 false，如果是则将其设置为 true 并执行后续操作
  // Check if is_shutdown_ is false, and if so, set it to true and perform subsequent operations
  if (!is_shutdown_.exchange(true)) {
    // 如果 is_started_ 为 true，则中断监听线程并等待其结束
    // If is_started_ is true, interrupt the listener thread and wait for it to finish
    if (is_started_) {
      interrupt_(&interrupt_guard_condition_);
      listener_thread_.join();
    }

    // 如果 is_started_ 为 true，则调用 cleanup_wait_set 函数清理等待集
    // If is_started_ is true, call the cleanup_wait_set function to clean up the wait set
    if (is_started_) {
      cleanup_wait_set();
    }
  }
}

/*!
 * \brief GraphListener 类的 shutdown 成员函数，用于关闭图形监听器。
 *        Member function shutdown of GraphListener class, used to shut down the graph listener.
 */
void GraphListener::shutdown() { this->__shutdown(); }

/*!
 * \brief GraphListener 类的 noexcept shutdown 成员函数，用于以 noexcept 方式关闭图形监听器。
 *        Member function noexcept shutdown of GraphListener class, used to shut down the graph
 * listener in a noexcept way. \param[in] std::nothrow_t & 无异常抛出对象。No exception thrown
 * object.
 */
void GraphListener::shutdown(const std::nothrow_t &) noexcept {
  // 使用 try-catch 语句捕获异常，确保不会向上层抛出异常
  // Use try-catch statement to catch exceptions, ensuring that no exceptions are thrown to the
  // upper level
  try {
    // 调用私有方法 __shutdown() 来关闭图形监听器
    // Call the private method __shutdown() to shut down the graph listener
    this->__shutdown();
  } catch (const std::exception &exc) {
    // 当捕获到 std::exception 类型的异常时，记录错误信息
    // When a std::exception type exception is caught, log the error message
    RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "caught %s exception when shutting down GraphListener: %s",
        rmw::impl::cpp::demangle(exc).c_str(), exc.what());
  } catch (...) {
    // 当捕获到未知类型的异常时，记录错误信息
    // When an unknown type exception is caught, log the error message
    RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "caught unknown exception when shutting down GraphListener");
  }
}

/*!
 * \brief GraphListener 类的 is_shutdown 成员函数，用于检查图形监听器是否已关闭。
 *        Member function is_shutdown of GraphListener class, used to check if the graph listener is
 * shut down. \return 图形监听器是否已关闭。Whether the graph listener is shut down.
 */
bool GraphListener::is_shutdown() { return is_shutdown_.load(); }

}  // namespace graph_listener
}  // namespace rclcpp

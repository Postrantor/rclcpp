// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTOR_HPP_
#define RCLCPP__EXECUTOR_HPP_

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rcl/guard_condition.h"
#include "rcl/wait.h"
#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rcpputils/scope_exit.hpp"

namespace rclcpp {

/**
 * @brief 定义一个类型别名，用于存储弱指针的回调组和节点之间的映射关系。
 * Define a type alias for storing the mapping relationship between weak pointer callback groups and
 * nodes.
 *
 * @tparam rclcpp::CallbackGroup::WeakPtr 回调组的弱指针类型。
 * The weak pointer type of the callback group.
 * @tparam rclcpp::node_interfaces::NodeBaseInterface::WeakPtr 节点基础接口的弱指针类型。
 * The weak pointer type of the node base interface.
 * @tparam std::owner_less<rclcpp::CallbackGroup::WeakPtr> 用于比较回调组弱指针的比较器。
 * The comparator for comparing callback group weak pointers.
 */
typedef std::map<
    rclcpp::CallbackGroup::WeakPtr,
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
    std::owner_less<rclcpp::CallbackGroup::WeakPtr>>
    WeakCallbackGroupsToNodesMap;

// 前向声明，用于便捷方法签名中。
// Forward declaration, used in convenience method signature.
class Node;

/// 协调可用通信任务的顺序和时序。
/// Coordinate the order and timing of available communication tasks.
/**
 * Executor 提供 spin 函数（包括 spin_node_once 和 spin_some）。
 * 它通过查找可用的工作并根据子类实现提供的线程或并发方案来完成，从而协调节点和回调组。
 * 可用工作的示例包括执行订阅回调或计时器回调。
 * 执行器结构允许通信图和执行模型之间的解耦。
 * 有关执行范例，请参见 SingleThreadedExecutor 和 MultiThreadedExecutor。
 *
 * Executor provides spin functions (including spin_node_once and spin_some).
 * It coordinates the nodes and callback groups by looking for available work and completing it,
 * based on the threading or concurrency scheme provided by the subclass implementation.
 * An example of available work is executing a subscription callback, or a timer callback.
 * The executor structure allows for a decoupling of the communication graph and the execution
 * model.
 * See SingleThreadedExecutor and MultiThreadedExecutor for examples of execution paradigms.
 */
class Executor {
public:
  // 不可复制的智能指针定义
  // Smart pointer definitions not copyable
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Executor)

  /// 默认构造函数。
  /// Default constructor.
  /**
   * \param[in] options 用于配置执行器的选项。
   * \param[in] options Options used to configure the executor.
   */
  RCLCPP_PUBLIC
  explicit Executor(const rclcpp::ExecutorOptions &options = rclcpp::ExecutorOptions());

  /// 默认析构函数。
  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~Executor();

  /// 当工作可用时，周期性地执行工作。阻塞调用，可能无限期阻塞。
  /// Do work periodically as it becomes available to us. Blocking call, may block indefinitely.
  // 执行器实现需要实现 spin 方法。
  // It is up to the implementation of Executor to implement spin.
  virtual void spin() = 0;

  /// 将回调组添加到执行器。
  /// Add a callback group to an executor.
  /**
   * 执行器可以有零个或多个回调组，在 `spin` 函数中提供工作。
   * An executor can have zero or more callback groups which provide work during `spin` functions.
   * 当执行器试图添加回调组时，执行器检查它是否已经与另一个执行器关联，如果已经关联，则抛出异常。
   * When an executor attempts to add a callback group, the executor checks to see if it is already
   * associated with another executor, and if it has been, then an exception is thrown.
   * 否则，回调组将被添加到执行器。
   * Otherwise, the callback group is added to the executor.
   *
   * 使用此方法添加回调组不会以任何方式将其节点与此执行器关联
   * Adding a callback group with this method does not associate its node with this executor
   * in any way
   *
   * \param[in] group_ptr 指向回调组的共享指针
   * \param[in] group_ptr a shared ptr that points to a callback group
   * \param[in] node_ptr 指向节点基本接口的共享指针
   * \param[in] node_ptr a shared pointer that points to a node base interface
   * \param[in] notify 如果在此函数期间触发中断保护条件，则为 True。如果执行器在 rmw
   * 层被阻塞，等待工作，并且被通知添加了新的回调组，它将唤醒。 \param[in] notify True to trigger
   * the interrupt guard condition during this function. If the executor is blocked at the rmw layer
   * while waiting for work and it is notified that a new callback group was added, it will wake up.
   * \throw std::runtime_error 如果回调组已关联到执行器
   * \throw std::runtime_error if the callback group is associated to an executor
   */
  RCLCPP_PUBLIC
  virtual void add_callback_group(
      rclcpp::CallbackGroup::SharedPtr group_ptr,
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
      bool notify = true);

  /// 获取属于执行器的回调组。
  /// Get callback groups that belong to executor.
  /**
   * 此函数返回指向与执行器关联的回调组的弱指针向量。
   * This function returns a vector of weak pointers that point to callback groups that were
   * associated with the executor.
   * 与此执行器关联的回调组可能已通过 `add_callback_group` 添加，或在将节点添加到执行器时使用
   * `add_node` 添加， 或者在已与此执行器关联的节点创建它时自动添加，并且
   * automatically_add_to_executor_with_node 参数为 true。 The callback groups associated with this
   * executor may have been added with `add_callback_group`, or added when a node was added to the
   * executor with `add_node`, or automatically added when it created by a node already associated
   * with this executor and the automatically_add_to_executor_with_node parameter was true.
   *
   * \return 指向与执行器关联的回调组的弱指针向量
   * \return a vector of weak pointers that point to callback groups that are associated with
   * the executor
   */
  RCLCPP_PUBLIC
  virtual std::vector<rclcpp::CallbackGroup::WeakPtr> get_all_callback_groups();

  /// 获取属于执行器的回调组。
  /// Get callback groups that belong to executor.
  /**
   * 此函数返回指向与执行器关联的回调组的弱指针向量。
   * This function returns a vector of weak pointers that point to callback groups that were
   * associated with the executor.
   * 使用 `add_callback_group` 添加了与此执行器关联的回调组。
   * The callback groups associated with this executor have been added with
   * `add_callback_group`.
   *
   * \return 指向与执行器关联的回调组的弱指针向量
   * \return a vector of weak pointers that point to callback groups that are associated with
   * the executor
   */
  RCLCPP_PUBLIC
  virtual std::vector<rclcpp::CallbackGroup::WeakPtr> get_manually_added_callback_groups();

  /// 获取属于执行器的回调组。
  /// Get callback groups that belong to executor.
  /**
   * 此函数返回指向从与执行器关联的节点添加的回调组的弱指针向量。
   * This function returns a vector of weak pointers that point to callback groups that were
   * added from a node that is associated with the executor.
   * 当使用 `add_node` 将节点添加到执行器时，回调组被添加，或者如果它们在将来由该节点创建并将
   * automatically_add_to_executor_with_node 参数设置为 true，则自动添加。
   * The callback groups are added when a node is added to the executor with `add_node`, or
   * automatically if they are created in the future by that node and have the
   * automatically_add_to_executor_with_node argument set to true.
   *
   * \return 指向从与执行器关联的节点的回调组的弱指针向量
   * \return a vector of weak pointers that point to callback groups from a node associated with
   * the executor
   */
  RCLCPP_PUBLIC
  virtual std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_automatically_added_callback_groups_from_nodes();

  /// 移除执行器中的回调组。
  /// Remove a callback group from the executor.
  /**
   * 回调组从执行器中移除并与执行器解除关联。
   * If the callback group removed was the last callback group from the node
   * that is associated with the executor, the interrupt guard condition
   * is triggered and node's guard condition is removed from the executor.
   *
   * 该函数仅移除通过 rclcpp::Executor::add_callback_group 手动添加的回调组。
   * This function only removes a callback group that was manually added with
   * rclcpp::Executor::add_callback_group.
   * 若要移除使用 rclcpp::Executor::add_node 添加的节点的回调组，请改用
   * rclcpp::Executor::remove_node。 To remove callback groups that were added from a node using
   * rclcpp::Executor::add_node, use rclcpp::Executor::remove_node instead.
   *
   * \param[in] group_ptr 要添加的回调组的共享指针。
   * \param[in] group_ptr Shared pointer to the callback group to be added.
   * \param[in] notify 如果在此函数期间触发中断保护条件，则为 True。如果执行器在等待工作时被阻塞在
   * rmw 层，并且收到回调组已删除的通知，它将唤醒。 \param[in] notify True to trigger the interrupt
   * guard condition during this function. If the executor is blocked at the rmw layer while waiting
   * for work and it is notified that a callback group was removed, it will wake up. \throw
   * std::runtime_error 如果节点在回调组之前被删除 \throw std::runtime_error if node is deleted
   * before callback group \throw std::runtime_error 如果回调组未与执行器关联 \throw
   * std::runtime_error if the callback group is not associated with the executor
   */
  RCLCPP_PUBLIC
  virtual void remove_callback_group(
      rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify = true);

  /// 向执行器添加节点。
  /// Add a node to the executor.
  /**
   * 节点具有关联的回调组，此方法将这些回调组中的任何一个添加到此执行器，这些回调组具有自动_add_to_executor_with_node
   * 参数为 true。 Nodes have associated callback groups, and this method adds any of those callback
   * groups to this executor which have their automatically_add_to_executor_with_node parameter
   * true. 节点还与执行器关联，以便在节点上创建具有自动_add_to_executor_with_node 参数设置为 true
   * 的未来回调组时，它们也会自动与此执行器关联。 The node is also associated with the executor so
   * that future callback groups which are created on the node with the
   * automatically_add_to_executor_with_node parameter set to true are also automatically associated
   * with this executor.
   *
   * 自动_add_to_executor_with_node 参数设置为 false 的回调组必须使用
   * rclcpp::Executor::add_callback_group 方法手动添加到执行器。 Callback groups with the
   * automatically_add_to_executor_with_node parameter set to false must be manually added to an
   * executor using the rclcpp::Executor::add_callback_group method.
   *
   * 如果节点已经与执行器关联，此方法将抛出异常。
   * If a node is already associated with an executor, this method throws an exception.
   *
   * \param[in] node_ptr 要添加的节点的共享指针。
   * \param[in] node_ptr Shared pointer to the node to be added.
   * \param[in] notify 如果在此函数期间触发中断保护条件，则为 True。如果执行器在等待工作时被阻塞在
   * rmw 层，并且收到新节点已添加的通知，它将唤醒。 \param[in] notify True to trigger the interrupt
   * guard condition during this function. If the executor is blocked at the rmw layer while waiting
   * for work and it is notified that a new node was added, it will wake up. \throw
   * std::runtime_error 如果节点已经关联到一个执行器 \throw std::runtime_error if a node is already
   * associated to an executor
   */
  RCLCPP_PUBLIC
  virtual void add_node(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// 方便的函数，接受 Node 并转发 NodeBaseInterface。
  /// Convenience function which takes Node and forwards NodeBaseInterface.
  /**
   * \see rclcpp::Executor::add_node
   */
  RCLCPP_PUBLIC
  virtual void add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true);

  /// 从执行器中移除一个节点。
  /// Remove a node from the executor.
  /**
   * 当使用 rclcpp::Executor::add_node 添加此节点时自动添加的任何回调组都会自动删除，
   * 并且该节点不再与此执行器关联。
   * Any callback groups automatically added when this node was added with
   * rclcpp::Executor::add_node are automatically removed, and the node is no longer associated
   * with this executor.
   *
   * 这也意味着给定节点创建的未来回调组不再自动添加到此执行器。
   * This also means that future callback groups created by the given node are no longer
   * automatically added to this executor.
   *
   * \param[in] node_ptr 要删除的节点的共享指针。
   * \param[in] node_ptr Shared pointer to the node to remove.
   * \param[in] notify True 表示触发中断保护条件并唤醒执行器。
   * \param[in] notify True to trigger the interrupt guard condition and wake up the executor.
   * 当执行器在另一个线程中等待工作时，如果从执行器中删除了最后一个节点，这是有用的，
   * 因为否则执行器将永远不会被通知。
   * This is useful if the last node was removed from the executor while the executor was blocked
   * waiting for work in another thread, because otherwise the executor would never be notified.
   * \throw std::runtime_error 如果节点没有与执行器关联。
   * \throw std::runtime_error if the node is not associated with an executor.
   * \throw std::runtime_error 如果节点没有与此执行器关联。
   * \throw std::runtime_error if the node is not associated with this executor.
   */
  RCLCPP_PUBLIC
  virtual void remove_node(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// 方便的函数，接受 Node 并转发 NodeBaseInterface。
  /// Convenience function which takes Node and forwards NodeBaseInterface.
  /**
   * \see rclcpp::Executor::remove_node
   */
  RCLCPP_PUBLIC
  virtual void remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true);

  /// 添加一个节点到执行器，执行下一个可用的工作单元，并移除该节点。 (Add a node to executor,
  /// execute the next available unit of work, and remove the node.)
  /**
   * \param[in] node 要添加的节点的共享指针。(Shared pointer to the node to add.)
   * \param[in] timeout 等待工作变得可用的时间。负值会导致 spin_node_once
   * 无限期阻塞（默认行为）。超时为0会使此函数非阻塞。(How long to wait for work to become
   * available. Negative values cause spin_node_once to block indefinitely (the default behavior). A
   * timeout of 0 causes this function to be non-blocking.)
   */
  template <typename RepT = int64_t, typename T = std::milli>
  void spin_node_once(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
      std::chrono::duration<RepT, T> timeout = std::chrono::duration<RepT, T>(-1)) {
    // 将超时时间转换为纳秒并调用 spin_node_once_nanoseconds 函数。(Convert the timeout duration to
    // nanoseconds and call spin_node_once_nanoseconds function.)
    return spin_node_once_nanoseconds(
        node, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  /// 方便函数，接受 Node 并转发 NodeBaseInterface。 (Convenience function which takes Node and
  /// forwards NodeBaseInterface.)
  template <typename NodeT = rclcpp::Node, typename RepT = int64_t, typename T = std::milli>
  void spin_node_once(
      std::shared_ptr<NodeT> node,
      std::chrono::duration<RepT, T> timeout = std::chrono::duration<RepT, T>(-1)) {
    // 获取节点基本接口并将超时时间转换为纳秒，然后调用 spin_node_once_nanoseconds 函数。 (Get the
    // node base interface and convert the timeout duration to nanoseconds, then call
    // spin_node_once_nanoseconds function.)
    return spin_node_once_nanoseconds(
        node->get_node_base_interface(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  /// 添加一个节点，完成所有立即可用的工作，并移除该节点。 (Add a node, complete all immediately
  /// available work, and remove the node.)
  /**
   * \param[in] node 要添加的节点的共享指针。(Shared pointer to the node to add.)
   */
  RCLCPP_PUBLIC
  void spin_node_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);

  /// 方便函数，接受 Node 并转发 NodeBaseInterface。 (Convenience function which takes Node and
  /// forwards NodeBaseInterface.)
  RCLCPP_PUBLIC
  void spin_node_some(std::shared_ptr<rclcpp::Node> node);

  /// 收集一次工作并执行所有可用的工作，可以在一段时间内进行。
  /// Collect work once and execute all available work, optionally within a duration.
  /**
   * 这个函数可以被重写。默认实现适用于单线程执行模型。
   * This function can be overridden. The default implementation is suitable for a
   * single-threaded model of execution.
   * 添加具有阻塞回调的订阅、计时器、服务等将导致此函数阻塞（可能产生意外后果）。
   * Adding subscriptions, timers, services, etc. with blocking callbacks will cause this function
   * to block (which may have unintended consequences).
   *
   * \param[in] max_duration 最大执行时间，或者为0表示无限制。
   * \param[in] max_duration The maximum amount of time to spend executing work, or 0 for no limit.
   * 注意，spin_some() 可能比这个时间长，因为它只有在超过 max_duration 时才返回。
   * Note that spin_some() may take longer than this time as it only returns once max_duration has
   * been exceeded.
   */
  RCLCPP_PUBLIC
  virtual void spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0));

  /// 在一段时间内或直到没有更多工作可用时，反复收集和执行工作。
  /// Collect and execute work repeatedly within a duration or until no more work is available.
  /**
   * 这个函数可以被重写。默认实现适用于单线程执行模型。
   * This function can be overridden. The default implementation is suitable for a
   * single-threaded model of execution.
   * 添加具有阻塞回调的订阅、计时器、服务等将导致此函数阻塞（可能产生意外后果）。
   * Adding subscriptions, timers, services, etc. with blocking callbacks will cause this function
   * to block (which may have unintended consequences).
   * 如果 waitables 的执行时间比新 waitables 准备好的周期长，此方法将在 `max_duration`
   * 用完前一直执行工作。 If the time that waitables take to be executed is longer than the period
   * on which new waitables become ready, this method will execute work repeatedly until
   * `max_duration` has elapsed.
   *
   * \param[in] max_duration 最大执行时间，必须 >= 0。`0` 可能会一直阻塞，直到没有更多的工作可用。
   * \param[in] max_duration The maximum amount of time to spend executing work, must be >= 0.
   *   `0` is potentially block forever until no more work is available.
   * \throw std::invalid_argument 如果 max_duration 小于 0。
   * \throw std::invalid_argument if max_duration is less than 0.
   * 注意，spin_all() 可能比这个时间长，因为它只有在超过 max_duration 时才返回。
   * Note that spin_all() may take longer than this time as it only returns once max_duration has
   * been exceeded.
   */
  RCLCPP_PUBLIC
  virtual void spin_all(std::chrono::nanoseconds max_duration);

  /// 执行一次工作。
  /// Execute work once.
  /**
   * \param[in] timeout 超时时间，默认为 -1。
   * \param[in] timeout Timeout duration, default is std::chrono::nanoseconds(-1).
   */
  RCLCPP_PUBLIC
  virtual void spin_once(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// Spin (blocking) until the future is complete, it times out waiting, or rclcpp is interrupted.
  /**
   * \param[in] future The future to wait on. If this function returns SUCCESS, the future can be
   *   accessed without blocking (though it may still throw an exception).
   * \param[in] timeout Optional timeout parameter, which gets passed to Executor::spin_node_once.
   *   `-1` is block forever, `0` is non-blocking.
   *   If the time spent inside the blocking loop exceeds this timeout, return a TIMEOUT return
   *   code.
   * \return The return code, one of `SUCCESS`, `INTERRUPTED`, or `TIMEOUT`.
   */
  // 旋转（阻塞）直到future完成，超时等待或rclcpp被中断。
  /**
   * \param[in] future
   * 要等待的future。如果此函数返回SUCCESS，则可以在不阻塞的情况下访问future（尽管它仍可能引发异常）。
   * \param[in] timeout 可选的超时参数，将传递给Executor::spin_node_once。
   *   `-1`表示永久阻塞，`0`表示非阻塞。
   *   如果在阻塞循环内花费的时间超过此超时，则返回一个TIMEOUT返回代码。
   * \return 返回代码，为`SUCCESS`、`INTERRUPTED`或`TIMEOUT`之一。
   */
  template <typename FutureT, typename TimeRepT = int64_t, typename TimeT = std::milli>
  FutureReturnCode spin_until_future_complete(
      const FutureT &future,
      std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1)) {
    // TODO(wjwwood): does not work recursively; can't call spin_node_until_future_complete
    // inside a callback executed by an executor.
    // TODO（wjwwood）：不支持递归；不能在执行器执行的回调内部调用spin_node_until_future_complete。

    // Check the future before entering the while loop.
    // If the future is already complete, don't try to spin.
    // 在进入while循环之前检查future。
    // 如果future已经完成，不要尝试旋转。
    std::future_status status = future.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      return FutureReturnCode::SUCCESS;
    }

    auto end_time = std::chrono::steady_clock::now();  // 记录当前时间（Record the current time）
    std::chrono::nanoseconds timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        timeout);  // 将超时时间转换为纳秒（Convert the timeout duration to nanoseconds）

    // 如果超时时间大于0，计算结束时间
    // （If the timeout duration is greater than 0, calculate the end time）
    if (timeout_ns > std::chrono::nanoseconds::zero()) {
      end_time += timeout_ns;
    }
    // 初始化剩余超时时间（Initialize the remaining timeout duration）
    std::chrono::nanoseconds timeout_left = timeout_ns;

    // 如果已经在旋转，则抛出异常（Throw an exception if already spinning）
    if (spinning.exchange(true)) {
      throw std::runtime_error("spin_until_future_complete() called while already spinning");
    }
    RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););
    while (rclcpp::ok(this->context_) && spinning.load()) {
      // 当上下文有效且正在旋转时执行循环
      // （Execute the loop while the context is valid and spinning） Do one item of work.
      spin_once_impl(timeout_left);  // 执行一次旋转实现（Perform a single spin implementation）

      // Check if the future is set, return SUCCESS if it is.
      status = future.wait_for(
          std::chrono::seconds(0));  // 检查未来是否已设置（Check if the future is set）
      // 如果未来已准备好，返回成功（Return success if the future is ready）
      if (status == std::future_status::ready) {
        return FutureReturnCode::SUCCESS;
      }
      // If the original timeout is < 0, then this is blocking, never TIMEOUT.
      if (timeout_ns < std::chrono::nanoseconds::zero()) {
        // 如果原始超时时间小于0，则为阻塞，永不超时
        // （If the original timeout duration is less than 0, this is blocking and will never
        // timeout）
        continue;
      }
      // Otherwise check if we still have time to wait, return TIMEOUT if not.
      auto now = std::chrono::steady_clock::now();  // 获取当前时间（Get the current time）
      // 如果当前时间大于等于结束时间
      // （If the current time is greater than or equal to the end time）
      if (now >= end_time) {
        return FutureReturnCode::TIMEOUT;  // 返回超时（Return timeout）
      }
      // Subtract the elapsed time from the original timeout.
      // 计算剩余超时时间（Calculate the remaining timeout duration）
      timeout_left = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - now);
    }

    // The future did not complete before ok() returned false, return INTERRUPTED.
    // 在ok()返回false之前，未来未完成，返回中断
    return FutureReturnCode::INTERRUPTED;
  }

  /// 取消任何正在运行的 spin* 函数，使其返回。
  /// Cancel any running spin* function, causing it to return.
  /**
   * 该函数可以从任何线程异步调用。
   * This function can be called asynchronously from any thread.
   * \throws std::runtime_error 如果触发保护条件时出现问题
   * \throws std::runtime_error if there is an issue triggering the guard condition
   */
  RCLCPP_PUBLIC
  void cancel();

  /// 支持动态切换内存策略。
  /// Support dynamic switching of the memory strategy.
  /**
   * 在另一个线程中执行器旋转时切换内存策略可能会产生意外后果。
   * Switching the memory strategy while the executor is spinning in another threading could have
   * unintended consequences. \param[in] memory_strategy 要设置的内存策略的共享指针。 \param[in]
   * memory_strategy Shared pointer to the memory strategy to set. \throws std::runtime_error 如果
   * memory_strategy 为空 \throws std::runtime_error if memory_strategy is null
   */
  RCLCPP_PUBLIC
  void set_memory_strategy(memory_strategy::MemoryStrategy::SharedPtr memory_strategy);

  /// 如果执行器当前正在旋转，则返回 true。
  /// Returns true if the executor is currently spinning.
  /**
   * 该函数可以从任何线程异步调用。
   * This function can be called asynchronously from any thread.
   * \return 如果执行器当前正在旋转，则为 True。
   * \return True if the executor is currently spinning.
   */
  RCLCPP_PUBLIC
  bool is_spinning();

protected:
  // RCLCPP_PUBLIC
  // 以纳秒为单位执行一次节点的旋转（Spin the node once with a nanosecond timeout）
  // 对给定节点执行一次 spin，超时时间为纳秒级别（Perform a single spin for the given node with a
  // nanosecond timeout）
  void spin_node_once_nanoseconds(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node, std::chrono::nanoseconds timeout);

  // RCLCPP_PUBLIC
  // 实现部分旋转，最大持续时间为纳秒级别，可选择是否详尽（Implement partial spinning with a maximum
  // duration in nanoseconds and an option to be exhaustive）
  void spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive);

  /// 查找下一个可用的可执行项，并执行与之相关的工作（Find the next available executable and do the
  /// work associated with it）
  /**
   * \param[in] any_exec 可以容纳任何可执行类型（计时器、订阅、服务、客户端）的联合结构（Union
   * structure that can hold any executable type (timer, subscription, service, client)） \throws
   * std::runtime_error 如果触发 guard condition 有问题，则抛出异常（Throws an exception if there is
   * an issue triggering the guard condition）
   */
  RCLCPP_PUBLIC
  void execute_any_executable(AnyExecutable &any_exec);

  // RCLCPP_PUBLIC
  // 执行订阅（Execute the subscription）
  static void execute_subscription(rclcpp::SubscriptionBase::SharedPtr subscription);

  // RCLCPP_PUBLIC
  // 执行计时器（Execute the timer）
  static void execute_timer(rclcpp::TimerBase::SharedPtr timer);

  // RCLCPP_PUBLIC
  // 执行服务（Execute the service）
  static void execute_service(rclcpp::ServiceBase::SharedPtr service);

  // RCLCPP_PUBLIC
  // 执行客户端（Execute the client）
  static void execute_client(rclcpp::ClientBase::SharedPtr client);

  /**
   * \throws std::runtime_error 如果 wait set 无法清除，则抛出异常（Throws an exception if the wait
   * set cannot be cleared）
   */
  RCLCPP_PUBLIC
  void wait_for_work(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  // RCLCPP_PUBLIC
  // 通过组获取节点（Get the node by group）
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_by_group(
      const WeakCallbackGroupsToNodesMap &weak_groups_to_nodes,
      rclcpp::CallbackGroup::SharedPtr group);

  /// 返回节点是否已添加到此执行器。
  /// Return true if the node has been added to this executor.
  /**
   * \param[in] node_ptr 指向节点基本接口的共享指针
   * \param[in] node_ptr a shared pointer that points to a node base interface
   * \param[in] weak_groups_to_nodes 用于查找节点的映射
   * \param[in] weak_groups_to_nodes map to nodes to lookup
   * \return 如果节点与执行器关联，则返回true，否则返回false
   * \return true if the node is associated with the executor, otherwise false
   */
  RCLCPP_PUBLIC
  bool has_node(
      const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
      const WeakCallbackGroupsToNodesMap &weak_groups_to_nodes) const;

  RCLCPP_PUBLIC
  rclcpp::CallbackGroup::SharedPtr get_group_by_timer(rclcpp::TimerBase::SharedPtr timer);

  /// 将回调组添加到执行器中
  /// Add a callback group to an executor
  /**
   * \see rclcpp::Executor::add_callback_group
   */
  RCLCPP_PUBLIC
  virtual void add_callback_group_to_map(
      rclcpp::CallbackGroup::SharedPtr group_ptr,
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
      WeakCallbackGroupsToNodesMap &weak_groups_to_nodes,
      bool notify = true) RCPPUTILS_TSA_REQUIRES(mutex_);

  /// 从执行器中移除回调组。
  /// Remove a callback group from the executor.
  /**
   * \see rclcpp::Executor::remove_callback_group
   */
  RCLCPP_PUBLIC
  virtual void remove_callback_group_from_map(
      rclcpp::CallbackGroup::SharedPtr group_ptr,
      WeakCallbackGroupsToNodesMap &weak_groups_to_nodes,
      bool notify = true) RCPPUTILS_TSA_REQUIRES(mutex_);

  /// \brief 获取下一个可执行的实体（Get the next ready executable entity）
  /// \param[out] any_executable 用于存储找到的可执行实体（The found executable entity will be
  /// stored in this parameter） \return 如果找到了可执行实体，则返回 true，否则返回 false（Return
  /// true if an executable entity is found, otherwise return false）
  RCLCPP_PUBLIC
  bool get_next_ready_executable(AnyExecutable &any_executable);

  /// \brief 从给定的映射中获取下一个可执行的实体（Get the next ready executable entity from the
  /// given map） \param[out] any_executable 用于存储找到的可执行实体（The found executable entity
  /// will be stored in this parameter） \param[in] weak_groups_to_nodes
  /// 一个包含弱回调组和节点映射的容器（A container with weak callback groups to nodes mapping）
  /// \return 如果找到了可执行实体，则返回 true，否则返回 false（Return true if an executable entity
  /// is found, otherwise return false）
  RCLCPP_PUBLIC
  bool get_next_ready_executable_from_map(
      AnyExecutable &any_executable, const WeakCallbackGroupsToNodesMap &weak_groups_to_nodes);

  /// \brief 获取下一个可执行实体，如果没有立即可用的实体，则等待一段时间（Get the next executable
  /// entity, wait for a period of time if no entity is immediately available） \param[out]
  /// any_executable 用于存储找到的可执行实体（The found executable entity will be stored in this
  /// parameter） \param[in] timeout 等待可执行实体的超时时间，默认为 -1 纳秒，表示无限等待（The
  /// timeout for waiting for an executable entity, default is -1 nanoseconds which means wait
  /// indefinitely） \return 如果找到了可执行实体，则返回 true，否则返回 false（Return true if an
  /// executable entity is found, otherwise return false）
  RCLCPP_PUBLIC
  bool get_next_executable(
      AnyExecutable &any_executable,
      std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// \brief 添加所有可以从关联节点自动添加的回调组（Add all callback groups that can be
  /// automatically added from associated nodes）
  /**
   * 在收集实体之前，执行器会检查与执行器关联的节点中的任何回调组是否可以自动添加到此执行器。
   * 这样可以处理已添加到节点但未显式添加到执行器的任何回调组。
   * 需要注意的是，为了让回调组通过此函数自动添加到执行器，需要通过 `add_node`
   * 方法添加回调组的节点。
   */
  RCLCPP_PUBLIC
  virtual void add_callback_groups_from_nodes_associated_to_executor()
      RCPPUTILS_TSA_REQUIRES(mutex_);

  /// 旋转状态，用于防止对 spin 的多线程调用以及取消阻塞 spins（Spinning state, used to prevent
  /// multi threaded calls to spin and to cancel blocking spins）
  std::atomic_bool spinning;

  /// 用于向 rmw 层发出信号以唤醒特殊事件的保护条件（Guard condition for signaling the rmw layer to
  /// wake up for special events）
  rclcpp::GuardCondition interrupt_guard_condition_;

  // 用于管理关闭的智能指针 (Smart pointer for managing shutdown)
  std::shared_ptr<rclcpp::GuardCondition> shutdown_guard_condition_;

  /// 等待集，用于管理 rmw 层等待的实体 (Wait set for managing entities that the rmw layer waits on)
  rcl_wait_set_t wait_set_ = rcl_get_zero_initialized_wait_set();

  // 保护后续 memory_strategy_ 的互斥锁 (Mutex to protect the subsequent memory_strategy_)
  mutable std::mutex mutex_;

  /// 内存策略：用于处理用户定义的内存分配策略的接口 (The memory strategy: an interface for handling
  /// user-defined memory allocation strategies)
  memory_strategy::MemoryStrategy::SharedPtr memory_strategy_ RCPPUTILS_TSA_PT_GUARDED_BY(mutex_);

  /// 与此执行器关联的上下文 (The context associated with this executor)
  std::shared_ptr<rclcpp::Context> context_;

  // 禁用复制 (Disable copy)
  RCLCPP_DISABLE_COPY(Executor)

  // 公共方法 (Public method)
  RCLCPP_PUBLIC
  virtual void spin_once_impl(std::chrono::nanoseconds timeout);

  // 定义回调组到保护条件的映射类型 (Define the mapping type from callback groups to guard
  // conditions)
  typedef std::map<
      rclcpp::CallbackGroup::WeakPtr,
      const rclcpp::GuardCondition *,
      std::owner_less<rclcpp::CallbackGroup::WeakPtr>>
      WeakCallbackGroupsToGuardConditionsMap;

  /// 将回调组映射到保护条件 (Maps callback groups to guard conditions)
  WeakCallbackGroupsToGuardConditionsMap weak_groups_to_guard_conditions_
      RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// 将回调组映射到与执行器关联的节点 (Maps callback groups associated to nodes)
  WeakCallbackGroupsToNodesMap weak_groups_associated_with_executor_to_nodes_
      RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// 将回调组映射到与执行器关联的节点 (Maps callback groups to nodes associated with executor)
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes_associated_with_executor_
      RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// 将所有回调组映射到节点 (Maps all callback groups to nodes)
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// 与执行器关联的节点 (Nodes that are associated with the executor)
  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_
      RCPPUTILS_TSA_GUARDED_BY(mutex_);

  /// 注册到上下文的关闭回调句柄 (Shutdown callback handle registered to Context)
  rclcpp::OnShutdownCallbackHandle shutdown_callback_handle_;
};

}  // namespace rclcpp

#endif  // RCLCPP__EXECUTOR_HPP_

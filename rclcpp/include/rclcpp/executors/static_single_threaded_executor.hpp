// Copyright 2019 Nobleo Technology
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

#ifndef RCLCPP__EXECUTORS__STATIC_SINGLE_THREADED_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__STATIC_SINGLE_THREADED_EXECUTOR_HPP_

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/executor.hpp"
#include "rclcpp/executors/static_executor_entities_collector.hpp"
#include "rclcpp/experimental/executable_list.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"

namespace rclcpp {
namespace executors {

/// 静态执行器实现 (Static executor implementation)
/**
 * 这个执行器是原始单线程执行器的静态版本。 (This executor is a static version of the original
 * single threaded executor.) 它之所以是静态的，是因为它不会在每次迭代时重构可执行列表。 (It's
 * static because it doesn't reconstruct the executable list for every iteration.) 在调用 spin()
 * 之前创建所有节点、回调组、计时器、订阅等，并仅在将实体添加/删除到/从节点时进行修改。 (All nodes,
 * callbackgroups, timers, subscriptions etc. are created before spin() is called, and modified only
 * when an entity is added/removed to/from a node.)
 *
 * 若要运行此执行器而不是 SingleThreadedExecutor，请替换以下内容：(To run this executor instead of
 * SingleThreadedExecutor replace:) rclcpp::executors::SingleThreadedExecutor exec; 由 (by)
 * rclcpp::executors::StaticSingleThreadedExecutor exec;
 * 在源代码中并以以下方式旋转节点：(in your source code and spin node(s) in the following way:)
 * exec.add_node(node);
 * exec.spin();
 * exec.remove_node(node);
 */
class StaticSingleThreadedExecutor : public rclcpp::Executor {
public:
  // 智能指针定义 (Smart pointer definitions)
  RCLCPP_SMART_PTR_DEFINITIONS(StaticSingleThreadedExecutor)

  /// 默认构造函数。参见 Executor 的默认构造函数。
  /// Default constructor. See the default constructor for Executor.
  RCLCPP_PUBLIC
  explicit StaticSingleThreadedExecutor(
      const rclcpp::ExecutorOptions& options = rclcpp::ExecutorOptions());

  /// 默认析构函数。
  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~StaticSingleThreadedExecutor();

  /// spin 的静态执行器实现。
  /// Static executor implementation of spin.
  /**
   * 此函数将阻塞，直到有工作进入并执行，然后继续阻塞。
   * 只有通过 CTRL-C（由全局信号处理程序管理）才能中断它。
   * 当 spin() 被调用时，如果已经在 spinning，则抛出 std::runtime_error。
   * This function will block until work comes in, execute it, and keep blocking.
   * It will only be interrupted by a CTRL-C (managed by the global signal handler).
   * \throws std::runtime_error when spin() called while already spinning
   */
  RCLCPP_PUBLIC
  void spin() override;

  /// spin_some 的静态执行器实现。
  /// Static executor implementation of spin some.
  /**
   * 此非阻塞功能将执行此 API 调用时就绪的实体，直到超时或没有更多工作可用。
   * 在执行工作时变为就绪状态的实体，在这里不会被考虑。
   * This non-blocking function will execute entities that
   * were ready when this API was called, until timeout or no
   * more work available. Entities that got ready while
   * executing work, won't be taken into account here.
   *
   * 示例：
   * Example:
   *   while(condition) {
   *     spin_some();
   *     sleep(); // 用户应该有一些同步工作或
   *              // 睡眠以避免 100% 的 CPU 使用率。
   *              // User should have some sync work or
   *              // sleep to avoid a 100% CPU usage.
   *   }
   */
  RCLCPP_PUBLIC
  void spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) override;

  /// spin_all 的静态执行器实现。
  /// Static executor implementation of spin all.
  /**
   * 此非阻塞功能将执行实体，直到超时（必须 >= 0）或没有更多工作可用。
   * 如果超时为 `0`，则可能会永远阻塞，直到没有更多工作可用。
   * 如果在执行可用工作时新的实体变为就绪状态，只要超时尚未过期，它们将被执行。
   * This non-blocking function will execute entities until timeout (must be >= 0)
   * or no more work available.
   * If timeout is `0`, potentially it blocks forever until no more work is available.
   * If new entities get ready while executing work available, they will be executed
   * as long as the timeout hasn't expired.
   *
   * 示例：
   * Example:
   *   while(condition) {
   *     spin_all();
   *     sleep(); // 用户应该有一些同步工作或
   *              // 睡眠以避免 100% 的 CPU 使用率。
   *              // User should have some sync work or
   *              // sleep to avoid a 100% CPU usage.
   *   }
   */
  RCLCPP_PUBLIC
  void spin_all(std::chrono::nanoseconds max_duration) override;

  /// 将回调组添加到执行器。
  /// Add a callback group to an executor.
  /**
   * \sa rclcpp::Executor::add_callback_group
   */
  RCLCPP_PUBLIC
  void add_callback_group(
      rclcpp::CallbackGroup::SharedPtr group_ptr,
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
      bool notify = true) override;

  /// 从执行器中移除回调组。
  /// Remove callback group from the executor.
  /**
   * \sa rclcpp::Executor::remove_callback_group
   */
  RCLCPP_PUBLIC
  void remove_callback_group(
      rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify = true) override;

  /// 将节点添加到执行器。
  /// Add a node to the executor.
  /**
   * \sa rclcpp::Executor::add_node
   */
  RCLCPP_PUBLIC
  void add_node(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true) override;

  /// 接受 Node 类型参数并转发 NodeBaseInterface 的便捷函数。
  /// Convenience function which takes Node and forwards NodeBaseInterface.
  /**
   * \sa rclcpp::StaticSingleThreadedExecutor::add_node
   */
  RCLCPP_PUBLIC
  void add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override;

  /// 从执行器中移除一个节点。
  /// Remove a node from the executor.
  /**
   * \sa rclcpp::Executor::remove_node
   */
  RCLCPP_PUBLIC
  void remove_node(
      // 要移除的节点指针。
      // The pointer of the node to be removed.
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
      // 是否通知标志，默认为 true。
      // Whether to notify, default is true.
      bool notify = true) override;

  /// 方便函数，接受 Node 并转发 NodeBaseInterface。
  /// Convenience function which takes Node and forwards NodeBaseInterface.
  /**
   * \sa rclcpp::Executor::remove_node
   */
  RCLCPP_PUBLIC
  void remove_node(
      // 要移除的节点指针。
      // The pointer of the node to be removed.
      std::shared_ptr<rclcpp::Node> node_ptr,
      // 是否通知标志，默认为 true。
      // Whether to notify, default is true.
      bool notify = true) override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr>
  // 获取所有回调组。
  // Get all callback groups.
  get_all_callback_groups() override;

  /// 获取属于执行器的回调组。
  /// Get callback groups that belong to executor.
  /**
   * \sa rclcpp::Executor::get_manually_added_callback_groups()
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_manually_added_callback_groups() override;

  /// 获取属于执行器的回调组。
  /// Get callback groups that belong to executor.
  /**
   * \sa rclcpp::Executor::get_automatically_added_callback_groups_from_nodes()
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_automatically_added_callback_groups_from_nodes()
      override;

protected:
  /**
   * @brief 从等待集中执行准备好的可执行文件。
   * @brief Executes ready executables from wait set.
   * @param spin_once 如果为 true，则仅执行第一个准备好的可执行文件。
   * @param spin_once if true executes only the first ready executable.
   * @return 如果有任何可执行文件已准备好，则返回 true。
   * @return true if any executable was ready.
   */
  RCLCPP_PUBLIC
  bool execute_ready_executables(bool spin_once = false);

  RCLCPP_PUBLIC
  void spin_some_impl(
      // 最大持续时间。
      // The maximum duration.
      std::chrono::nanoseconds max_duration,
      // 是否详尽标志。
      // Whether exhaustive flag.
      bool exhaustive);

  RCLCPP_PUBLIC
  void spin_once_impl(
      // 超时时间。
      // The timeout duration.
      std::chrono::nanoseconds timeout) override;

private:
  // 禁用 StaticSingleThreadedExecutor 的拷贝。
  // Disable copy of StaticSingleThreadedExecutor.
  RCLCPP_DISABLE_COPY(StaticSingleThreadedExecutor)

  // 实体收集器的共享指针。
  // Shared pointer of the entities collector.
  StaticExecutorEntitiesCollector::SharedPtr entities_collector_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__STATIC_SINGLE_THREADED_EXECUTOR_HPP_

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

#ifndef RCLCPP__EXECUTORS_HPP_
#define RCLCPP__EXECUTORS_HPP_

#include <future>
#include <memory>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/executors/static_single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/// 创建一个默认的单线程执行器并执行任何立即可用的工作。
/// Create a default single-threaded executor and execute any immediately available work.
/**
 * \param[in] node_ptr 要旋转的节点的共享指针。
 * \param[in] node_ptr Shared pointer to the node to spin.
 */
RCLCPP_PUBLIC
void spin_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

// 重载函数，参数类型为 rclcpp::Node::SharedPtr
// Overloaded function with parameter type rclcpp::Node::SharedPtr
RCLCPP_PUBLIC
void spin_some(rclcpp::Node::SharedPtr node_ptr);

/// 创建一个默认的单线程执行器并旋转指定的节点。
/// Create a default single-threaded executor and spin the specified node.
/**
 * \param[in] node_ptr 要旋转的节点的共享指针。
 * \param[in] node_ptr Shared pointer to the node to spin.
 */
RCLCPP_PUBLIC
void spin(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

// 重载函数，参数类型为 rclcpp::Node::SharedPtr
// Overloaded function with parameter type rclcpp::Node::SharedPtr
RCLCPP_PUBLIC
void spin(rclcpp::Node::SharedPtr node_ptr);

namespace executors {

// 使用 rclcpp::executors::MultiThreadedExecutor 和 rclcpp::executors::SingleThreadedExecutor
// 命名空间 Use the rclcpp::executors::MultiThreadedExecutor and
// rclcpp::executors::SingleThreadedExecutor namespaces
using rclcpp::executors::MultiThreadedExecutor;
using rclcpp::executors::SingleThreadedExecutor;

/// Spin (blocking) until the future is complete, it times out waiting, or rclcpp is interrupted.
/**
 * \param[in] executor 执行器，用于 spin 节点。
 * \param[in] node_ptr 要 spin 的节点。
 * \param[in] future 等待的 future。如果返回 `SUCCESS`，则在此函数之后可以安全地访问 future。
 * \param[in] timeout 可选的超时参数，传递给 Executor::spin_node_once。
 *   `-1` 是无限阻塞，`0` 是非阻塞。
 *   如果在阻塞循环内花费的时间超过此超时，则返回 `TIMEOUT` 返回代码。
 * \return 返回代码，为 `SUCCESS`、`INTERRUPTED` 或 `TIMEOUT` 之一。
 *
 * \param[in] executor The executor which will spin the node.
 * \param[in] node_ptr The node to spin.
 * \param[in] future The future to wait on. If `SUCCESS`, the future is safe to
 *   access after this function
 * \param[in] timeout Optional timeout parameter, which gets passed to
 *   Executor::spin_node_once.
 *   `-1` is block forever, `0` is non-blocking.
 *   If the time spent inside the blocking loop exceeds this timeout, return a `TIMEOUT` return
 * code. \return The return code, one of `SUCCESS`, `INTERRUPTED`, or `TIMEOUT`.
 */
template <typename FutureT, typename TimeRepT = int64_t, typename TimeT = std::milli>
rclcpp::FutureReturnCode spin_node_until_future_complete(
    rclcpp::Executor& executor,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    const FutureT& future,
    std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1)) {
  // TODO(wjwwood): 不支持递归；不能在执行器执行的回调中调用 spin_node_until_future_complete。
  // TODO(wjwwood): does not work recursively; can't call spin_node_until_future_complete
  // inside a callback executed by an executor.
  executor.add_node(node_ptr);                               // 将节点添加到执行器中
                                                             // Add the node to the executor
  auto retcode =
      executor.spin_until_future_complete(future, timeout);  // 执行器等待 future 完成或超时
  // Executor waits for the future to complete or times out
  executor.remove_node(node_ptr);  // 从执行器中移除节点
                                   // Remove the node from the executor
  return retcode;                  // 返回结果代码
                                   // Return the result code
}

template <
    typename NodeT = rclcpp::Node,
    typename FutureT,
    typename TimeRepT = int64_t,
    typename TimeT = std::milli>
rclcpp::FutureReturnCode spin_node_until_future_complete(
    rclcpp::Executor& executor,
    std::shared_ptr<NodeT> node_ptr,
    const FutureT& future,
    std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1)) {
  // 调用 rclcpp::executors::spin_node_until_future_complete，传递节点基本接口、future 和超时参数
  // Call rclcpp::executors::spin_node_until_future_complete, passing the node base interface,
  // future, and timeout parameters
  return rclcpp::executors::spin_node_until_future_complete(
      executor, node_ptr->get_node_base_interface(), future, timeout);
}

}  // namespace executors

/**
 * @brief 以给定的超时时间等待 future 完成 (Wait for a future to complete with a given timeout)
 *
 * @tparam FutureT        期望的 future 类型 (The expected future type)
 * @tparam TimeRepT       时间表示类型，默认为 int64_t (Time representation type, default is
 * int64_t)
 * @tparam TimeT          时间单位，默认为 std::milli (Time unit, default is std::milli)
 * @param node_ptr        节点的共享指针 (Shared pointer of the node)
 * @param future          需要等待完成的 future (The future to wait for completion)
 * @param timeout         超时时间，默认为 -1，表示无限等待 (Timeout duration, default is -1 which
 * means infinite waiting)
 * @return rclcpp::FutureReturnCode 返回 future 完成状态的代码 (Return code of the future completion
 * status)
 */
template <typename FutureT, typename TimeRepT = int64_t, typename TimeT = std::milli>
rclcpp::FutureReturnCode spin_until_future_complete(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    const FutureT& future,
    std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1)) {
  // 创建一个单线程执行器 (Create a single-threaded executor)
  rclcpp::executors::SingleThreadedExecutor executor;
  // 使用执行器、节点、future 和超时时间，等待 future 完成 (Wait for the future to complete using
  // the executor, node, future, and timeout)
  return executors::spin_node_until_future_complete<FutureT>(executor, node_ptr, future, timeout);
}

/**
 * @brief 以给定的超时时间等待 future 完成 (Wait for a future to complete with a given timeout)
 *
 * @tparam NodeT          节点类型，默认为 rclcpp::Node (Node type, default is rclcpp::Node)
 * @tparam FutureT        期望的 future 类型 (The expected future type)
 * @tparam TimeRepT       时间表示类型，默认为 int64_t (Time representation type, default is
 * int64_t)
 * @tparam TimeT          时间单位，默认为 std::milli (Time unit, default is std::milli)
 * @param node_ptr        节点的共享指针 (Shared pointer of the node)
 * @param future          需要等待完成的 future (The future to wait for completion)
 * @param timeout         超时时间，默认为 -1，表示无限等待 (Timeout duration, default is -1 which
 * means infinite waiting)
 * @return rclcpp::FutureReturnCode 返回 future 完成状态的代码 (Return code of the future completion
 * status)
 */
template <
    typename NodeT = rclcpp::Node,
    typename FutureT,
    typename TimeRepT = int64_t,
    typename TimeT = std::milli>
rclcpp::FutureReturnCode spin_until_future_complete(
    std::shared_ptr<NodeT> node_ptr,
    const FutureT& future,
    std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1)) {
  // 使用节点的基础接口、future 和超时时间，等待 future 完成 (Wait for the future to complete using
  // the node's base interface, future, and timeout)
  return rclcpp::spin_until_future_complete(node_ptr->get_node_base_interface(), future, timeout);
}

}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS_HPP_

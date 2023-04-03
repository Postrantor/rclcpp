// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__WAIT_FOR_MESSAGE_HPP_
#define RCLCPP__WAIT_FOR_MESSAGE_HPP_

#include <memory>
#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_set.hpp"
#include "rcpputils/scope_exit.hpp"

namespace rclcpp {
/// 等待下一条传入的消息。
/**
 * 给定一个已经初始化的订阅，
 * 在指定的超时之前等待下一条传入的消息到达。
 *
 * \param[out] out 当新消息到来时要填充的消息。
 * \param[in] subscription 指向先前初始化订阅的共享指针。
 * \param[in] context 指向要监视SIGINT请求的上下文的共享指针。
 * \param[in] time_to_wait 指定在返回之前的超时参数。
 * \return 如果成功接收到消息，则为true，如果无法获取消息或在上下文中异步触发关闭，则为false。
 */
// Wait for the next incoming message.
/**
 * Given an already initialized subscription,
 * wait for the next incoming message to arrive before the specified timeout.
 *
 * \param[out] out is the message to be filled when a new message is arriving.
 * \param[in] subscription shared pointer to a previously initialized subscription.
 * \param[in] context shared pointer to a context to watch for SIGINT requests.
 * \param[in] time_to_wait parameter specifying the timeout before returning.
 * \return true if a message was successfully received, false if message could not
 * be obtained or shutdown was triggered asynchronously on the context.
 */
template <class MsgT, class Rep = int64_t, class Period = std::milli>
bool wait_for_message(
    MsgT& out,
    std::shared_ptr<rclcpp::Subscription<MsgT>> subscription,
    std::shared_ptr<rclcpp::Context> context,
    std::chrono::duration<Rep, Period> time_to_wait = std::chrono::duration<Rep, Period>(-1)) {
  // 创建一个 GuardCondition 对象，用于监视上下文中的关闭请求。
  auto gc = std::make_shared<rclcpp::GuardCondition>(context);
  // 添加一个关闭回调，当上下文关闭时触发 GuardCondition。
  auto shutdown_callback_handle =
      context->add_on_shutdown_callback([weak_gc = std::weak_ptr<rclcpp::GuardCondition>{gc}]() {
        auto strong_gc = weak_gc.lock();
        if (strong_gc) {
          strong_gc->trigger();
        }
      });

  // 初始化一个 WaitSet 对象。
  rclcpp::WaitSet wait_set;
  // 将订阅添加到 WaitSet。
  wait_set.add_subscription(subscription);
  // 当离开作用域时，从 WaitSet 中删除订阅。
  RCPPUTILS_SCOPE_EXIT(wait_set.remove_subscription(subscription););
  // 将 GuardCondition 添加到 WaitSet。
  wait_set.add_guard_condition(gc);
  // 等待指定的超时或者有事件发生。
  auto ret = wait_set.wait(time_to_wait);
  // 如果 WaitSet 没有准备好，返回 false。
  if (ret.kind() != rclcpp::WaitResultKind::Ready) {
    return false;
  }

  // 如果 GuardCondition 被触发，返回 false。
  if (wait_set.get_rcl_wait_set().guard_conditions[0]) {
    return false;
  }

  // 定义一个 MessageInfo 对象。
  rclcpp::MessageInfo info;
  // 尝试从订阅中获取消息，如果失败则返回 false。
  if (!subscription->take(out, info)) {
    return false;
  }

  // 成功获取消息，返回 true。
  return true;
}

/// 等待下一条传入的消息。
/**
 * 在指定的主题上等待下一条传入的消息在指定的超时之前到达。
 *
 * \param[out] out 当新消息到来时要填充的消息。
 * \param[in] node 用于初始化订阅的节点指针。
 * \param[in] topic 要等待消息的主题。
 * \param[in] time_to_wait 指定在返回之前的超时参数。
 * \return 如果成功接收到消息，则为true，如果无法获取消息或在上下文中异步触发关闭，则为false。
 */
// Wait for the next incoming message.
/**
 * Wait for the next incoming message to arrive on a specified topic before the specified timeout.
 *
 * \param[out] out is the message to be filled when a new message is arriving.
 * \param[in] node the node pointer to initialize the subscription on.
 * \param[in] topic the topic to wait for messages.
 * \param[in] time_to_wait parameter specifying the timeout before returning.
 * \return true if a message was successfully received, false if message could not
 * be obtained or shutdown was triggered asynchronously on the context.
 */
template <class MsgT, class Rep = int64_t, class Period = std::milli>
bool wait_for_message(
    MsgT& out,
    rclcpp::Node::SharedPtr node,
    const std::string& topic,
    std::chrono::duration<Rep, Period> time_to_wait = std::chrono::duration<Rep, Period>(-1)) {
  // 在节点上创建一个订阅，订阅指定的主题，设置回调函数为空。
  auto sub = node->create_subscription<MsgT>(topic, 1, [](const std::shared_ptr<const MsgT>) {});
  // 使用先前创建的订阅等待消息。
  return wait_for_message<MsgT, Rep, Period>(
      out, sub, node->get_node_options().context(), time_to_wait);
}

}  // namespace rclcpp

#endif  // RCLCPP__WAIT_FOR_MESSAGE_HPP_

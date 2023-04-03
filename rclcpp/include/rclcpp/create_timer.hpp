// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CREATE_TIMER_HPP_
#define RCLCPP__CREATE_TIMER_HPP_

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/duration.hpp"
#include "rclcpp/node_interfaces/get_node_base_interface.hpp"
#include "rclcpp/node_interfaces/get_node_clock_interface.hpp"
#include "rclcpp/node_interfaces/get_node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"

namespace rclcpp {
namespace detail {
/// 执行一个安全的转换到纳秒计时周期 (Perform a safe cast to a timer period in nanoseconds)
/**
 *
 * \tparam DurationRepT 时间表示类型 (Duration representation type)
 * \tparam DurationT 时间单位类型 (Duration unit type)
 * \param period 回调执行周期。此持续时间必须满足 0 <= period < nanoseconds::max() (Period to
 * execute callback. This duration must be 0 <= period < nanoseconds::max()) \return 以
 * chrono::duration::nanoseconds 表示的周期 (Period, expressed as chrono::duration::nanoseconds)
 * \throws std::invalid_argument 如果 period 为负数或过大 (If period is negative or too large)
 */
template <typename DurationRepT, typename DurationT>
std::chrono::nanoseconds safe_cast_to_period_in_ns(
    std::chrono::duration<DurationRepT, DurationT> period) {
  // 如果 period 小于零，抛出异常 (If period is less than zero, throw an exception)
  if (period < std::chrono::duration<DurationRepT, DurationT>::zero()) {
    throw std::invalid_argument{"timer period cannot be negative"};
  }

  // 转换为双精度表示可能会丢失精度，并允许下面的检查成功 (Casting to a double representation might
  // lose precision and allow the check below to succeed)
  // 但实际上转换为纳秒可能会失败。使用比最大值少 1 DurationT 的纳秒 (Using 1 DurationT worth of
  // nanoseconds less than max)
  constexpr auto maximum_safe_cast_ns =
      std::chrono::nanoseconds::max() - std::chrono::duration<DurationRepT, DurationT>(1);

  // 如果 period 大于 nanoseconds::max()，则转换为纳秒的 duration_cast
  // 将溢出有符号整数，这是未定义行为 (If period is greater than nanoseconds::max(), the
  // duration_cast to nanoseconds will overflow a signed integer, which is undefined behavior)
  // 检查任何 std::chrono::duration 是否大于 nanoseconds::max() 是一个困难的一般问题 (Checking
  // whether any std::chrono::duration is greater than nanoseconds::max() is a difficult general
  // problem) 这是 Howard Hinnant（<chrono> 的作者）在此处的回答的更保守版本 (This is a more
  // conservative version of Howard Hinnant's (the <chrono> guy>) response here)
  // https://stackoverflow.com/a/44637334/2089061
  // 然而，这并没有解决所有可能的 period 持续时间类型的问题 (However, this doesn't solve the issue
  // for all possible duration types of period) 后续问题：https://github.com/ros2/rclcpp/issues/1177
  // (Follow-up issue: https://github.com/ros2/rclcpp/issues/1177)
  constexpr auto ns_max_as_double =
      std::chrono::duration_cast<std::chrono::duration<double, std::chrono::nanoseconds::period>>(
          maximum_safe_cast_ns);
  if (period > ns_max_as_double) {
    throw std::invalid_argument{"timer period must be less than std::chrono::nanoseconds::max()"};
  }

  // 将 period 转换为纳秒 (Cast period to nanoseconds)
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period);
  // 如果转换后的 period_ns 小于零，抛出异常 (If the converted period_ns is less than zero, throw an
  // exception)
  if (period_ns < std::chrono::nanoseconds::zero()) {
    throw std::runtime_error{"Casting timer period to nanoseconds resulted in integer overflow."};
  }

  // 返回以纳秒表示的周期 (Return the period expressed in nanoseconds)
  return period_ns;
}
}  // namespace detail

/// \brief 使用给定的时钟创建一个计时器 (Create a timer with a given clock)
/// \internal
/// \tparam CallbackT 计时器回调函数类型 (Type of the timer callback function)
/// \param[in] node_base 与节点相关的基本接口的共享指针 (Shared pointer to the node-related base
/// interface) \param[in] node_timers 与节点相关的计时器接口的共享指针 (Shared pointer to the
/// node-related timers interface) \param[in] clock 用于计时器的时钟的共享指针 (Shared pointer to
/// the clock used for the timer) \param[in] period 计时器周期 (Timer period) \param[in] callback
/// 计时器回调函数 (Timer callback function) \param[in] group 可选的回调组共享指针，默认为 nullptr
/// (Optional shared pointer to the callback group, default is nullptr) \return
/// 创建的计时器的共享指针 (Shared pointer to the created timer)
template <typename CallbackT>
typename rclcpp::TimerBase::SharedPtr create_timer(
    std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
    std::shared_ptr<node_interfaces::NodeTimersInterface> node_timers,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Duration period,
    CallbackT&& callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr) {
  // 调用另一个重载版本的 create_timer 函数，将参数转换为适当的类型 (Call another overloaded version
  // of create_timer function, converting the arguments to the appropriate types)
  return create_timer(
      clock, period.to_chrono<std::chrono::nanoseconds>(), std::forward<CallbackT>(callback), group,
      node_base.get(), node_timers.get());
}

/**
 * @brief 创建一个具有给定时钟的计时器 (Create a timer with a given clock)
 *
 * @tparam NodeT 节点类型 (Node type)
 * @tparam CallbackT 回调函数类型 (Callback function type)
 * @param node 传入的节点对象 (The input node object)
 * @param clock 计时器使用的时钟 (The clock used by the timer)
 * @param period 计时器周期 (Timer period)
 * @param callback 计时器到期时执行的回调函数 (Callback function to be executed when the timer
 * expires)
 * @param group 回调组 (Optional, default is nullptr) (Callback group)
 * @return rclcpp::TimerBase::SharedPtr 返回创建的计时器共享指针 (Return the created timer shared
 * pointer)
 */
template <typename NodeT, typename CallbackT>
typename rclcpp::TimerBase::SharedPtr create_timer(
    NodeT node,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Duration period,
    CallbackT&& callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr) {
  // 使用给定的参数创建计时器，并返回其共享指针
  // (Create the timer with the given parameters and return its shared pointer)
  return create_timer(
      clock, period.to_chrono<std::chrono::nanoseconds>(), std::forward<CallbackT>(callback), group,
      rclcpp::node_interfaces::get_node_base_interface(node).get(),
      rclcpp::node_interfaces::get_node_timers_interface(node).get());
}

/// 方便的创建一个具有节点资源的通用定时器的方法。
/// Convenience method to create a general timer with node resources.
/**
 *
 * \tparam DurationRepT 时间段表示类型
 * \tparam DurationT 时间段单位类型
 * \tparam CallbackT 回调函数类型
 * \param clock 要使用的时钟
 * \param period 执行回调的时间段。此持续时间必须为 0 <= period < nanoseconds::max()
 * \param callback 定时器周期内执行的回调
 * \param group 回调组
 * \param node_base 节点基本接口
 * \param node_timers 节点计时器接口
 * \return 通用定时器的共享指针
 * \throws std::invalid_argument 如果 clock、node_base 或 node_timers 为空，或者 period 为负数或过大
 *
 * \tparam DurationRepT Duration representation type
 * \tparam DurationT Duration unit type
 * \tparam CallbackT Callback function type
 * \param clock Clock to be used
 * \param period Period to execute callback. This duration must be 0 <= period < nanoseconds::max()
 * \param callback Callback to execute via the timer period
 * \param group Callback group
 * \param node_base Node base interface
 * \param node_timers Node timer interface
 * \return Shared pointer to a generic timer
 * \throws std::invalid_argument If either clock, node_base or node_timers are nullptr, or period is
 * negative or too large
 */
template <typename DurationRepT, typename DurationT, typename CallbackT>
typename rclcpp::GenericTimer<CallbackT>::SharedPtr create_timer(
    rclcpp::Clock::SharedPtr clock,
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group,
    node_interfaces::NodeBaseInterface* node_base,
    node_interfaces::NodeTimersInterface* node_timers) {
  // 检查时钟是否为空
  // Check if the clock is nullptr
  if (clock == nullptr) {
    throw std::invalid_argument{"clock cannot be null"};
  }
  // 检查节点基本接口是否为空
  // Check if the node base interface is nullptr
  if (node_base == nullptr) {
    throw std::invalid_argument{"input node_base cannot be null"};
  }
  // 检查节点计时器接口是否为空
  // Check if the node timers interface is nullptr
  if (node_timers == nullptr) {
    throw std::invalid_argument{"input node_timers cannot be null"};
  }

  // 将持续时间安全转换为纳秒
  // Safely cast the duration to nanoseconds
  const std::chrono::nanoseconds period_ns = detail::safe_cast_to_period_in_ns(period);

  // 添加一个新的通用定时器。
  // Add a new generic timer.
  auto timer = rclcpp::GenericTimer<CallbackT>::make_shared(
      std::move(clock), period_ns, std::move(callback), node_base->get_context());
  // 将定时器添加到节点计时器接口和回调组中
  // Add the timer to the node timers interface and callback group
  node_timers->add_timer(timer, group);
  // 返回通用定时器的共享指针
  // Return the shared pointer to the generic timer
  return timer;
}

/// 方便的方法来创建一个使用节点资源的墙壁定时器。
/// Convenience method to create a wall timer with node resources.
/**
 *
 * \tparam DurationRepT 时间间隔表示类型
 * \tparam DurationT 时间间隔单位类型
 * \tparam CallbackT 回调函数类型
 * \param period 执行回调的时间间隔。此持续时间必须满足 0 <= period < nanoseconds::max()
 * \param callback 定时器周期内执行的回调函数
 * \param group 回调组
 * \param node_base 节点基本接口
 * \param node_timers 节点定时器接口
 * \return 指向墙壁定时器的共享指针
 * \throws std::invalid_argument 如果 node_base 或 node_timers
 * 为空，或者 period 为负数或过大
 */
template <typename DurationRepT, typename DurationT, typename CallbackT>
typename rclcpp::WallTimer<CallbackT>::SharedPtr create_wall_timer(
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group,
    node_interfaces::NodeBaseInterface* node_base,
    node_interfaces::NodeTimersInterface* node_timers) {
  // 检查 node_base 是否为空
  // Check if node_base is null
  if (node_base == nullptr) {
    throw std::invalid_argument{"input node_base cannot be null"};
  }

  // 检查 node_timers 是否为空
  // Check if node_timers is null
  if (node_timers == nullptr) {
    throw std::invalid_argument{"input node_timers cannot be null"};
  }

  // 将 period 转换为纳秒，确保安全转换
  // Convert period to nanoseconds, ensuring safe conversion
  const std::chrono::nanoseconds period_ns = detail::safe_cast_to_period_in_ns(period);

  // 添加一个新的墙壁定时器
  // Add a new wall timer.
  auto timer = rclcpp::WallTimer<CallbackT>::make_shared(
      period_ns, std::move(callback), node_base->get_context());
  // 将定时器添加到节点定时器接口中，并关联回调组
  // Add the timer to the node timers interface and associate it with the callback group.
  node_timers->add_timer(timer, group);
  // 返回创建的墙壁定时器
  // Return the created wall timer
  return timer;
}
}  // namespace rclcpp

#endif  // RCLCPP__CREATE_TIMER_HPP_

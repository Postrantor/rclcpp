// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__QOS_OVERRIDING_OPTIONS_HPP_
#define RCLCPP__QOS_OVERRIDING_OPTIONS_HPP_

#include <functional>
#include <initializer_list>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/qos_policy_kind.h"

namespace rclcpp {

//! 枚举类，表示 QoS 策略类型。
enum class RCLCPP_PUBLIC_TYPE QosPolicyKind {
  //! 避免 ROS 命名空间约定策略。
  AvoidRosNamespaceConventions = RMW_QOS_POLICY_AVOID_ROS_NAMESPACE_CONVENTIONS,
  Deadline = RMW_QOS_POLICY_DEADLINE,
  Depth = RMW_QOS_POLICY_DEPTH,
  Durability = RMW_QOS_POLICY_DURABILITY,
  History = RMW_QOS_POLICY_HISTORY,
  Lifespan = RMW_QOS_POLICY_LIFESPAN,
  Liveliness = RMW_QOS_POLICY_LIVELINESS,
  LivelinessLeaseDuration = RMW_QOS_POLICY_LIVELINESS_LEASE_DURATION,
  Reliability = RMW_QOS_POLICY_RELIABILITY,
  Invalid = RMW_QOS_POLICY_INVALID,
};

//! 将 QosPolicyKind 转换为字符串。
/*!
 * \param[in] qpk QosPolicyKind 枚举值。
 * \return QosPolicyKind 对应的字符串表示。
 */
RCLCPP_PUBLIC
const char *qos_policy_kind_to_cstr(const QosPolicyKind &qpk);

//! 重载 << 操作符，用于输出 QosPolicyKind。
/*!
 * \param[in] os 输出流。
 * \param[in] qpk QosPolicyKind 枚举值。
 * \return 输出流。
 */
RCLCPP_PUBLIC
std::ostream &operator<<(std::ostream &os, const QosPolicyKind &qpk);

using QosCallbackResult = rcl_interfaces::msg::SetParametersResult;
using QosCallback = std::function<QosCallbackResult(const rclcpp::QoS &)>;

namespace detail {
template <typename T>  // 前向声明
class QosParameters;
}  // namespace detail

//! 用于订阅/发布构造函数中指定 QoS 可配置性的选项类。
/*!
 * This options struct allows configuring:
 * - Which policy kinds will have declared parameters.
 * - An optional callback, that will be called to validate the final qos profile.
 * - An optional id. In the case that different qos are desired for two publishers/subscriptions in
 *   the same topic, this id will allow disambiguating them.
 *
 * Example parameter file:
 *
 * ```yaml
 * my_node_name:
 *  ros__parameters:
 *    qos_overrides:
 *      /my/topic/name:
 *        publisher:  # publisher without provided id
 *          reliability: reliable
 *          depth: 100
 *        publisher_my_id:  # publisher with `id="my_id"
 *          reliability: reliable
 *          depth: 10
 * ```
 */
class QosOverridingOptions {
public:
  //! 默认构造函数，不允许覆盖。
  RCLCPP_PUBLIC
  QosOverridingOptions() = default;

  //! 构造函数，传入一组 QoS 策略和验证回调。
  /*!
   * Constructor taking a list of QoS policies and a verification callback.
   *
   * This constructor is implicit, e.g.:
   * ```cpp
   * node->create_publisher(
   *   "topic_name",
   *   default_qos_profile,
   *   {
   *     {QosPolicyKind::Reliability},
   *     [] (auto && qos) {return check_qos_validity(qos)},
   *     "my_id"
   *   });
   * ```
   * \param[in] policy_kinds 允许重新配置的策略类型列表。
   * \param[in] validation_callback 用于验证用户设置的 qos 配置文件有效性的回调。
   * \param[in] id 实体的 ID。
   */
  RCLCPP_PUBLIC
  QosOverridingOptions(
      std::initializer_list<QosPolicyKind> policy_kinds,
      QosCallback validation_callback = nullptr,
      std::string id = {});

  /*!
   * 获取实体 ID。Get the entity ID.
   * \return 实体 ID。
   */
  RCLCPP_PUBLIC
  const std::string &get_id() const;

  /*!
   * 获取策略类型列表。Get the list of policy kinds.
   * \return 策略类型列表。
   */
  RCLCPP_PUBLIC
  const std::vector<QosPolicyKind> &get_policy_kinds() const;

  /*!
   * 获取验证回调。Get the validation callback.
   * \return 验证回调。
   */
  RCLCPP_PUBLIC
  const QosCallback &get_validation_callback() const;

  //! 构造函数，传入一组 QoS 策略和验证回调。
  /*!
   * Constructor taking a list of QoS policies and a verification callback.
   * Same as `QosOverridingOptions` constructor, but only declares the default policies:
   * History, Depth, Reliability.
   */
  RCLCPP_PUBLIC
  static QosOverridingOptions with_default_policies(
      QosCallback validation_callback = nullptr, std::string id = {});

private:
  //! 请求创建参数的实体的 ID。
  std::string id_;
  //! 允许重新配置的策略类型。
  std::vector<QosPolicyKind> policy_kinds_;
  //! 用于验证配置文件的验证回调。
  QosCallback validation_callback_;
};

}  // namespace rclcpp

#endif  // RCLCPP__QOS_OVERRIDING_OPTIONS_HPP_

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

#include "rclcpp/qos_overriding_options.hpp"

#include <initializer_list>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rmw/qos_policy_kind.h"
#include "rmw/qos_string_conversions.h"

namespace rclcpp {

/*!
 * \brief 将 QosPolicyKind 转换为字符串的表示形式
 * \param[in] qpk QosPolicyKind 枚举值
 * \return 对应的字符串表示形式
 * \throws std::invalid_argument 如果给定的 QosPolicyKind 未知
 */
const char* qos_policy_kind_to_cstr(const QosPolicyKind& qpk) {
  // 调用 rmw_qos_policy_kind_to_str 函数将 QosPolicyKind 转换为字符串
  const char* ret = rmw_qos_policy_kind_to_str(static_cast<rmw_qos_policy_kind_t>(qpk));
  if (!ret) {
    // 如果转换失败，抛出异常
    throw std::invalid_argument{"unknown QoS policy kind"};
  }
  return ret;
}

// 重载 << 运算符以便于输出 QosPolicyKind
std::ostream& operator<<(std::ostream& oss, const QosPolicyKind& qpk) {
  return oss << qos_policy_kind_to_cstr(qpk);
}

// 定义默认策略列表
static std::initializer_list<QosPolicyKind> kDefaultPolicies = {
    QosPolicyKind::History, QosPolicyKind::Depth, QosPolicyKind::Reliability};

/*!
 * \brief QosOverridingOptions 构造函数
 * \param[in] policy_kinds 要覆盖的 QoS 策略种类列表
 * \param[in] validation_callback 用于验证 QoS 配置的回调函数
 * \param[in] id 用于标识 QosOverridingOptions 的字符串
 */
QosOverridingOptions::QosOverridingOptions(
    std::initializer_list<QosPolicyKind> policy_kinds,
    QosCallback validation_callback,
    std::string id)
    : id_{std::move(id)},
      policy_kinds_{policy_kinds},
      validation_callback_{std::move(validation_callback)} {}

/*!
 * \brief 使用默认策略创建 QosOverridingOptions 对象
 * \param[in] validation_callback 用于验证 QoS 配置的回调函数
 * \param[in] id 用于标识 QosOverridingOptions 的字符串
 * \return 创建的 QosOverridingOptions 对象
 */
QosOverridingOptions QosOverridingOptions::with_default_policies(
    QosCallback validation_callback, std::string id) {
  return QosOverridingOptions{kDefaultPolicies, validation_callback, id};
}

// 获取 QosOverridingOptions 的 ID (Get the ID of QosOverridingOptions)
const std::string& QosOverridingOptions::get_id() const { return id_; }

// 获取 QosOverridingOptions 的策略种类列表
const std::vector<QosPolicyKind>& QosOverridingOptions::get_policy_kinds() const {
  return policy_kinds_;
}

// 获取 QosOverridingOptions 的验证回调函数 (Get the validation callback of QosOverridingOptions)
/*
> [!NOTE]
> 这里是定义的时候就是给出的函数指针，这里返回的就是为了这样用！
> using QosCallbackResult = rcl_interfaces::msg::SetParametersResult;
> using QosCallback = std::function<QosCallbackResult(const rclcpp::QoS &)>;
*/
const QosCallback& QosOverridingOptions::get_validation_callback() const {
  return validation_callback_;
}

}  // namespace rclcpp

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

#ifndef RCLCPP__QOS_HPP_
#define RCLCPP__QOS_HPP_

#include <string>

#include "rcl/logging_rosout.h"
#include "rclcpp/duration.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/incompatible_qos_events_statuses.h"
#include "rmw/qos_profiles.h"
#include "rmw/types.h"

namespace rclcpp {

/**
 * @brief 从 QoS 策略类型中获取策略名称 (Get policy name from QoS policy kind)
 *
 * @param policy_kind QoS 策略类型 (QoS policy kind)
 * @return std::string 策略名称 (Policy name)
 */
RCLCPP_PUBLIC
std::string qos_policy_name_from_kind(rmw_qos_policy_kind_t policy_kind);

// 历史策略枚举类 (History policy enumeration class)
enum class HistoryPolicy {
  KeepLast = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  KeepAll = RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  SystemDefault = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
  Unknown = RMW_QOS_POLICY_HISTORY_UNKNOWN,
};

// 可靠性策略枚举类 (Reliability policy enumeration class)
enum class ReliabilityPolicy {
  BestEffort = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  Reliable = RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  SystemDefault = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
  BestAvailable = RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE,
  Unknown = RMW_QOS_POLICY_RELIABILITY_UNKNOWN,
};

// 持久性策略枚举类 (Durability policy enumeration class)
enum class DurabilityPolicy {
  Volatile = RMW_QOS_POLICY_DURABILITY_VOLATILE,
  TransientLocal = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  SystemDefault = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
  BestAvailable = RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE,
  Unknown = RMW_QOS_POLICY_DURABILITY_UNKNOWN,
};

// 活跃度策略枚举类 (Liveliness policy enumeration class)
enum class LivelinessPolicy {
  Automatic = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
  ManualByTopic = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
  SystemDefault = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  BestAvailable = RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE,
  Unknown = RMW_QOS_POLICY_LIVELINESS_UNKNOWN,
};

// QoS 兼容性枚举类 (QoS compatibility enumeration class)
enum class QoSCompatibility {
  Ok = RMW_QOS_COMPATIBILITY_OK,            // 兼容 (Compatible)
  Warning = RMW_QOS_COMPATIBILITY_WARNING,  // 警告 (Warning)
  Error = RMW_QOS_COMPATIBILITY_ERROR,      // 错误 (Error)
};

/// QoS 初始化值，不能直接创建，请使用 KeepAll 或 KeepLast 代替。
struct RCLCPP_PUBLIC QoSInitialization {
  rmw_qos_history_policy_t history_policy;
  size_t depth;

  /// 构造函数，接收历史策略和深度（即使它不会被使用）。
  QoSInitialization(
      rmw_qos_history_policy_t history_policy_arg,
      size_t depth_arg,
      bool print_depth_warning = true);

  /// 从现有的 rmw_qos_profile_t 创建一个 QoSInitialization，使用其历史和深度。
  static QoSInitialization from_rmw(const rmw_qos_profile_t& rmw_qos);
};

/// 用于将 QoS 初始化为 keep_all 历史设置。
struct RCLCPP_PUBLIC KeepAll : public rclcpp::QoSInitialization {
  KeepAll();
};

/// 用于将 QoS 初始化为 keep_last 历史设置和给定深度。
struct RCLCPP_PUBLIC KeepLast : public rclcpp::QoSInitialization {
  explicit KeepLast(size_t depth, bool print_depth_warning = true);
};

/// 封装质量服务设置，返回 `rmw_qos_profile_t` 的结构。
/**
 * 质量服务设置控制发布者、订阅者和其他实体的行为，包括如何发送或重新发送数据、
 * 在发布和订阅方如何缓冲数据等。
 * 参见： (See:) <a
 * href="https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html">
 *     https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
 *   </a>
 */
class RCLCPP_PUBLIC QoS {
public:
  /// 通过指定历史策略和历史深度创建一个 QoS。
  /**
   * 使用默认初始配置文件时，默认值将包括：
   *
   *   - \link rclcpp::ReliabilityPolicy::Reliable ReliabilityPolicy::Reliable\endlink
   *   - \link rclcpp::DurabilityPolicy::Volatile DurabilityPolicy::Volatile\endlink
   *
   * 有关默认设置的完整列表，请参阅 rmw_qos_profile_default。
   * 如果将其他 rmw_qos_profile_t 传递给 initial_profile，则默认值将从该配置文件派生。
   *
   * \param[in] qos_initialization 指定历史策略和历史深度。
   * \param[in] initial_profile 用于基于默认设置的 rmw_qos_profile_t 实例。
   */
  explicit QoS(
      const QoSInitialization& qos_initialization,
      const rmw_qos_profile_t& initial_profile = rmw_qos_profile_default);

  /// 转换构造函数，方便在仅指定深度的常见情况下进行构造。
  /**
   * 这是一个便利构造函数，调用 QoS(KeepLast(history_depth))。
   * \param[in] history_depth 发布时可以排队的消息数，或者在被订阅替换之前可以排队的消息数。
   */
  // cppcheck-suppress noExplicitConstructor
  QoS(size_t history_depth);  // NOLINT(runtime/explicit): conversion constructor
  rmw_qos_profile_t& get_rmw_qos_profile();
  const rmw_qos_profile_t& get_rmw_qos_profile() const;
  QoS& history(HistoryPolicy history);
  QoS& history(rmw_qos_history_policy_t history);
  QoS& keep_last(size_t depth);
  QoS& keep_all();
  QoS& reliability(rmw_qos_reliability_policy_t reliability);
  QoS& reliability(ReliabilityPolicy reliability);
  QoS& reliable();
  QoS& best_effort();
  QoS& reliability_best_available();
  QoS& durability(rmw_qos_durability_policy_t durability);
  QoS& durability(DurabilityPolicy durability);
  QoS& durability_volatile();
  QoS& transient_local();
  QoS& durability_best_available();
  QoS& deadline(rmw_time_t deadline);
  QoS& deadline(const rclcpp::Duration& deadline);
  QoS& lifespan(rmw_time_t lifespan);
  QoS& lifespan(const rclcpp::Duration& lifespan);
  QoS& liveliness(rmw_qos_liveliness_policy_t liveliness);
  QoS& liveliness(LivelinessPolicy liveliness);
  QoS& liveliness_lease_duration(rmw_time_t liveliness_lease_duration);
  QoS& liveliness_lease_duration(const rclcpp::Duration& liveliness_lease_duration);
  QoS& avoid_ros_namespace_conventions(bool avoid_ros_namespace_conventions);
  HistoryPolicy history() const;
  size_t depth() const;
  ReliabilityPolicy reliability() const;
  DurabilityPolicy durability() const;
  rclcpp::Duration deadline() const;
  rclcpp::Duration lifespan() const;
  LivelinessPolicy liveliness() const;
  rclcpp::Duration liveliness_lease_duration() const;
  bool avoid_ros_namespace_conventions() const;

private:
  rmw_qos_profile_t rmw_qos_profile_;
};

/// 检查两个 QoS 配置文件是否在所有策略值上完全相等。
RCLCPP_PUBLIC
bool operator==(const QoS& left, const QoS& right);

/// 检查两个 QoS 配置文件是否不相等。
RCLCPP_PUBLIC
bool operator!=(const QoS& left, const QoS& right);

/// 用于检查 QoS 兼容性的结果类型
struct QoSCheckCompatibleResult {
  /// 兼容性结果。
  QoSCompatibility compatibility;

  /// 不兼容（可能）的原因。
  /**
   * 如果兼容性为 QoSCompatibility::Warning 或 QoSCompatiblity::Error，则设置。
   * 如果 QoS 配置文件兼容，则不设置。
   */
  std::string reason;
};

/// 检查两个 QoS 配置文件是否兼容。
/**
 * 如果使用 QoS 策略的发布者和订阅者可以相互通信，则两个 QoS 配置文件是兼容的。
 *
 * 如果任何策略的值为 "system default" 或 "unknown"，则可能无法确定兼容性。
 * 在这种情况下，返回结构的一部分设置为 QoSCompatility::Warning。
 *
 * 示例用法：
 *
 * ```cpp
 * rclcpp::QoSCheckCompatibleResult result = rclcpp::qos_check_compatible(
 *   publisher_qos, subscription_qos);
 * if (rclcpp::QoSCompatibility::Error != result.compatibility) {
 *   // QoS 不兼容 ...
 *   // result.reason 包含关于不兼容性的信息
 * } else if (rclcpp::QoSCompatibility::Warning != result.compatibility) {
 *   // QoS 可能不兼容 ...
 *   // result.reason 包含关于可能的不兼容性的信息
 * }
 * ```
 *
 * \param[in] publisher_qos: 发布者的 QoS 配置文件。
 * \param[in] subscription_qos: 订阅者的 QoS 配置文件。
 * \return 如果 QoS 配置文件兼容，则将 compatiblity 设置为 QoSCompatibility::Ok 的结构，或
 * \return 如果 QoS 配置文件可能不兼容，则将 compatibility 设置为 QoSCompatibility::Warning
 * 的结构，或
 * \return 如果 QoS 配置文件不兼容，则将 compatibility 设置为 QoSCompatibility::Error 的结构。
 * \throws rclcpp::exceptions::QoSCheckCompatibilityException 如果发生意外错误。
 */
RCLCPP_PUBLIC
QoSCheckCompatibleResult qos_check_compatible(
    const QoS& publisher_qos,  //
    const QoS& subscription_qos);

/**
 * @brief 时钟 QoS 类 (Clock QoS class)
 *    - 历史: 保留最后一个 (History: Keep last)
 *    - 深度: 1 (Depth: 1)
 *    - 可靠性: 最大努力 (Reliability: Best effort)
 *    - 持久性: 易失性 (Durability: Volatile)
 *    - 截止日期: 默认值 (Deadline: Default)
 *    - 生命周期: 默认值 (Lifespan: Default)
 *    - 活跃性: 系统默认值 (Liveliness: System default)
 *    - 活跃性租期持续时间: 默认值 (Liveliness lease duration: default)
 *    - 避免 ROS 命名空间约定: false (avoid ros namespace conventions: false)
 */
class RCLCPP_PUBLIC ClockQoS : public QoS {
public:
  // 构造函数，接受 QoS 初始化参数，默认为 KeepLast(1)
  explicit ClockQoS(const QoSInitialization& qos_initialization = KeepLast(1));
};

/**
 * @brief 传感器数据 QoS 类 (Sensor Data QoS class)
 *    - 历史: 保留最后一个 (History: Keep last)
 *    - 深度: 5 (Depth: 5)
 *    - 可靠性: 最大努力 (Reliability: Best effort)
 *    - 持久性: 易失性 (Durability: Volatile)
 *    - 截止日期: 默认值 (Deadline: Default)
 *    - 生命周期: 默认值 (Lifespan: Default)
 *    - 活跃性: 系统默认值 (Liveliness: System default)
 *    - 活跃性租期持续时间: 默认值 (Liveliness lease duration: default)
 *    - 避免 ROS 命名空间约定: false (avoid ros namespace conventions: false)
 */
class RCLCPP_PUBLIC SensorDataQoS : public QoS {
public:
  // 构造函数，接受 QoS 初始化参数，默认为 rmw_qos_profile_sensor_data)
  explicit SensorDataQoS(
      const QoSInitialization& qos_initialization =
          (QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)));
};

/**
 * @brief Parameters QoS 类 (Parameters QoS class)
 *    - 历史: 保留最后一条 (History: Keep last),
 *    - 深度: 1000 (Depth: 1000),
 *    - 可靠性: 可靠的 (Reliability: Reliable),
 *    - 持久性: 易失性的 (Durability: Volatile),
 *    - 截止日期: 默认值 (Deadline: Default),
 *    - 生命周期: 默认值 (Lifespan: Default),
 *    - 活跃性: 系统默认 (Liveliness: System default),
 *    - 活跃性租期: 默认值 (Liveliness lease duration: default),
 *    - 避免 ROS 命名空间约定: 否 (Avoid ros namespace conventions: false)
 */
class RCLCPP_PUBLIC ParametersQoS : public QoS {
public:
  // 构造函数，接受 QoS 初始化参数，默认为 rmw_qos_profile_parameters (Constructor, accepts QoS
  // initialization parameters, defaults to rmw_qos_profile_parameters)
  explicit ParametersQoS(
      const QoSInitialization& qos_initialization =
          (QoSInitialization::from_rmw(rmw_qos_profile_parameters)));
};

/**
 * @brief Services QoS 类 (Services QoS class)
 *    - 历史: 保留最后一条 (History: Keep last),
 *    - 深度: 10 (Depth: 10),
 *    - 可靠性: 可靠的 (Reliability: Reliable),
 *    - 持久性: 易失性的 (Durability: Volatile),
 *    - 截止日期: 默认值 (Deadline: Default),
 *    - 生命周期: 默认值 (Lifespan: Default),
 *    - 活跃性: 系统默认 (Liveliness: System default),
 *    - 活跃性租期: 默认值 (Liveliness lease duration: default),
 *    - 避免 ROS 命名空间约定: 否 (Avoid ros namespace conventions: false)
 */
class RCLCPP_PUBLIC ServicesQoS : public QoS {
public:
  // 构造函数，接受 QoS 初始化参数，默认为 rmw_qos_profile_services_default (Constructor, accepts
  // QoS initialization parameters, defaults to rmw_qos_profile_services_default)
  explicit ServicesQoS(
      const QoSInitialization& qos_initialization =
          (QoSInitialization::from_rmw(rmw_qos_profile_services_default)));
};

/**
 * @brief Parameter events QoS 类 (Parameter events QoS class)
 *    - 历史: 保留最后一条 (History: Keep last),
 *    - 深度: 1000 (Depth: 1000),
 *    - 可靠性: 可靠的 (Reliability: Reliable),
 *    - 持久性: 易失性的 (Durability: Volatile),
 *    - 截止日期: 默认值 (Deadline: Default),
 *    - 生命周期: 默认值 (Lifespan: Default),
 *    - 活跃性: 系统默认 (Liveliness: System default),
 *    - 活跃性租期: 默认值 (Liveliness lease duration: default),
 *    - 避免 ROS 命名空间约定: 否 (Avoid ros namespace conventions: false)
 */
class RCLCPP_PUBLIC ParameterEventsQoS : public QoS {
public:
  // 构造函数，接受 QoS 初始化参数，默认为 rmw_qos_profile_parameter_events (Constructor, accepts
  // QoS initialization parameters, defaults to rmw_qos_profile_parameter_events)
  explicit ParameterEventsQoS(
      const QoSInitialization& qos_initialization =
          (QoSInitialization::from_rmw(rmw_qos_profile_parameter_events)));
};

/**
 * @brief Rosout QoS 类 (Rosout QoS class)
 *    - 历史: 保留最后一条 (History: Keep last),
 *    - 深度: 1000 (Depth: 1000),
 *    - 可靠性: 可靠的 (Reliability: Reliable),
 *    - 持久性: TRANSIENT_LOCAL (Durability: TRANSIENT_LOCAL),
 *    - 截止日期: 默认值 (Deadline: Default),
 *    - 生命周期: {10, 0} (Lifespan: {10, 0}),
 *    - 活跃性: 系统默认 (Liveliness: System default),
 *    - 活跃性租期: 默认值 (Liveliness lease duration: default),
 *    - 避免 ROS 命名空间约定: 否 (Avoid ros namespace conventions: false)
 */
class RCLCPP_PUBLIC RosoutQoS : public QoS {
public:
  // 构造函数，接受 QoS 初始化参数，默认为 rcl_qos_profile_rosout_default (Constructor, accepts QoS
  // initialization parameters, defaults to rcl_qos_profile_rosout_default)
  explicit RosoutQoS(
      const QoSInitialization& rosout_qos_initialization =
          (QoSInitialization::from_rmw(rcl_qos_profile_rosout_default)));
};

/**
 * @brief 系统默认QoS类 (System defaults QoS class)
 *    - 历史: 系统默认, (History: System default,)
 *    - 深度: 系统默认, (Depth: System default,)
 *    - 可靠性: 系统默认, (Reliability: System default,)
 *    - 持久性: 系统默认, (Durability: System default,)
 *    - 截止日期: 默认, (Deadline: Default,)
 *    - 生命周期: 默认, (Lifespan: Default,)
 *    - 活跃度: 系统默认, (Liveliness: System default,)
 *    - 活跃度租约期限: 系统默认, (Liveliness lease duration: System default,)
 *    - 避免ros命名空间约定: false (Avoid ros namespace conventions: false)
 */
class RCLCPP_PUBLIC SystemDefaultsQoS : public QoS {
public:
  /**
   * @brief 构造函数，初始化系统默认QoS (Constructor to initialize system default QoS)
   *
   * @param qos_initialization QoS初始化参数，默认为系统默认配置 (QoS initialization parameter,
   * default is system default configuration)
   */
  explicit SystemDefaultsQoS(
      const QoSInitialization& qos_initialization =
          (QoSInitialization::from_rmw(rmw_qos_profile_system_default)));
};

/**
 * @brief 最佳可用QoS类 (Best available QoS class)
 *
 * 在维持最高服务水平的同时匹配当前可用的大多数端点。
 * 创建订阅或发布者时选择策略。
 * 即使一个或多个策略与新发现的端点不兼容，中间件也不会在创建订阅或发布者后更新策略。
 * 因此，由于与发现的竞争，应谨慎使用此配置文件，因为可能会出现非确定性行为。
 *
 * (Match majority of endpoints currently available while maintaining the highest level of service.
 * Policies are chosen at the time of creating a subscription or publisher.
 * The middleware is not expected to update policies after creating a subscription or publisher,
 * even if one or more policies are incompatible with newly discovered endpoints.
 * Therefore, this profile should be used with care since non-deterministic behavior can occur due
 * to races with discovery.)
 *
 *    - 历史: 保留最后, (History: Keep last,)
 *    - 深度: 10, (Depth: 10,)
 *    - 可靠性: 最佳可用, (Reliability: Best available,)
 *    - 持久性: 最佳可用, (Durability: Best available,)
 *    - 截止日期: 最佳可用, (Deadline: Best available,)
 *    - 生命周期: 默认, (Lifespan: Default,)
 *    - 活跃度: 最佳可用, (Liveliness: Best available,)
 *    - 活跃度租约期限: 最佳可用, (Liveliness lease duration: Best available,)
 *    - 避免ros命名空间约定: false (avoid ros namespace conventions: false)
 */
class RCLCPP_PUBLIC BestAvailableQoS : public QoS {
public:
  /**
   * @brief 构造函数，初始化最佳可用QoS (Constructor to initialize best available QoS)
   *
   * @param qos_initialization QoS初始化参数，默认为最佳可用配置 (QoS initialization parameter,
   * default is best available configuration)
   */
  explicit BestAvailableQoS(
      const QoSInitialization& qos_initialization =
          (QoSInitialization::from_rmw(rmw_qos_profile_best_available)));
};

}  // namespace rclcpp

#endif  // RCLCPP__QOS_HPP_

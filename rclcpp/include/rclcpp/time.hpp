// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__TIME_HPP_
#define RCLCPP__TIME_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "rcl/time.h"
#include "rclcpp/duration.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

class Clock;

class Time {
public:
  /// 时间构造函数 (Time constructor)
  /**
   * 分别初始化秒和纳秒的时间值。 (Initializes the time values for seconds and nanoseconds
   * individually.) 大的纳秒值会自动换算，余数会加到秒上。 (Large values for nanoseconds are wrapped
   * automatically with the remainder added to seconds.) 输入必须是整数。 (Both inputs must be
   * integers.)
   *
   * \param seconds 自时间纪元以来的秒部分 (part of the time in seconds since time epoch)
   * \param nanoseconds 自时间纪元以来的纳秒部分 (part of the time in nanoseconds since time epoch)
   * \param clock_type 时钟类型 (clock type)
   * \throws std::runtime_error 如果秒为负数 (if seconds are negative)
   */
  RCLCPP_PUBLIC
  Time(int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t clock_type = RCL_SYSTEM_TIME);

  /// 时间构造函数 (Time constructor)
  /**
   * \param nanoseconds 自时间纪元以来的纳秒 (nanoseconds since time epoch)
   * \param clock_type 时钟类型 (clock type)
   */
  RCLCPP_PUBLIC
  explicit Time(int64_t nanoseconds = 0, rcl_clock_type_t clock_type = RCL_SYSTEM_TIME);

  /// 拷贝构造函数 (Copy constructor)
  RCLCPP_PUBLIC
  Time(const Time& rhs);

  /// 时间构造函数 (Time constructor)
  /**
   * \param time_msg 要复制的builtin_interfaces时间消息 (builtin_interfaces time message to copy)
   * \param clock_type 时钟类型 (clock type)
   * \throws std::runtime_error 如果秒为负数 (if seconds are negative)
   */
  RCLCPP_PUBLIC
  Time(const builtin_interfaces::msg::Time& time_msg, rcl_clock_type_t clock_type = RCL_ROS_TIME);

  /// 时间构造函数 (Time constructor)
  /**
   * \param time_point 要复制的rcl_time_point_t结构 (rcl_time_point_t structure to copy)
   */
  RCLCPP_PUBLIC
  explicit Time(const rcl_time_point_t& time_point);

  /// 时间析构函数 (Time destructor)
  RCLCPP_PUBLIC
  virtual ~Time();

  /// 返回一个基于builtin_interfaces::msg::Time对象 (Return a builtin_interfaces::msg::Time object
  /// based)
  RCLCPP_PUBLIC
  operator builtin_interfaces::msg::Time() const;

  /**
   * \throws std::runtime_error 如果秒为负数 (if seconds are negative)
   */
  RCLCPP_PUBLIC
  Time& operator=(const Time& rhs);

  /**
   * 从builtin_interfaces::msg::Time实例分配时间。 (Assign Time from a builtin_interfaces::msg::Time
   * instance.) clock_type将重置为RCL_ROS_TIME。 (The clock_type will be reset to RCL_ROS_TIME.)
   * 等价于 *this = Time(time_msg, RCL_ROS_TIME)。 (Equivalent to *this = Time(time_msg,
   * RCL_ROS_TIME).) \throws std::runtime_error 如果秒为负数 (if seconds are negative)
   */
  RCLCPP_PUBLIC
  Time& operator=(const builtin_interfaces::msg::Time& time_msg);

  /**
   * \throws std::runtime_error 如果时间源不同 (if the time sources are different)
   */
  RCLCPP_PUBLIC
  bool operator==(const rclcpp::Time& rhs) const;

  RCLCPP_PUBLIC
  bool operator!=(const rclcpp::Time& rhs) const;

  /**
   * \throws std::runtime_error 如果时间源不同 (if the time sources are different)
   */
  RCLCPP_PUBLIC
  bool operator<(const rclcpp::Time& rhs) const;

  /**
   * \throws std::runtime_error 如果时间源不同 (if the time sources are different)
   */
  RCLCPP_PUBLIC
  bool operator<=(const rclcpp::Time& rhs) const;

  /**
   * \throws std::runtime_error 如果时间源不同 (if the time sources are different)
   */
  RCLCPP_PUBLIC
  bool operator>=(const rclcpp::Time& rhs) const;

  /**
   * \throws std::runtime_error 如果时间源不同 (if the time sources are different)
   */
  RCLCPP_PUBLIC
  bool operator>(const rclcpp::Time& rhs) const;

  /**
   * \throws std::overflow_error 如果加法导致溢出 (if addition leads to overflow)
   */
  RCLCPP_PUBLIC
  Time operator+(const rclcpp::Duration& rhs) const;

  /**
   * \throws std::runtime_error 如果时间源不同 (if the time sources are different)
   * \throws std::overflow_error 如果加法导致溢出 (if addition leads to overflow)
   */
  RCLCPP_PUBLIC
  Duration operator-(const rclcpp::Time& rhs) const;

  /**
   * \throws std::overflow_error 如果加法导致溢出 (if addition leads to overflow)
   */
  RCLCPP_PUBLIC
  Time operator-(const rclcpp::Duration& rhs) const;

  /**
   * \throws std::overflow_error 如果加法导致溢出 (if addition leads to overflow)
   */
  RCLCPP_PUBLIC
  Time& operator+=(const rclcpp::Duration& rhs);

  /**
   * \throws std::overflow_error 如果加法导致溢出 (if addition leads to overflow)
   */
  RCLCPP_PUBLIC
  Time& operator-=(const rclcpp::Duration& rhs);

  /// 获取自纪元以来的纳秒数 (Get the nanoseconds since epoch)
  /**
   * \return 自纪元以来的纳秒数作为rcl_time_point_value_t结构。 (the nanoseconds since epoch as a
   * rcl_time_point_value_t structure.)
   */
  RCLCPP_PUBLIC
  rcl_time_point_value_t nanoseconds() const;

  /// 获取最大可表示值。 (Get the maximum representable value.)
  /**
   * \return 最大可表示值 (the maximum representable value)
   */
  RCLCPP_PUBLIC
  static Time max();

  /// 获取自纪元以来的秒数 (Get the seconds since epoch)
  /**
   * \warning 根据sizeof(double)的不同，可能会有显著的精度损失。 (Depending on sizeof(double) there
   * could be significant precision loss.) 当需要精确时间时，请使用nanoseconds()代替。 (When an
   * exact time is required use nanoseconds() instead.)
   *
   * \return 自纪元以来的秒数作为浮点数。 (the seconds since epoch as a floating point number.)
   */
  RCLCPP_PUBLIC
  double seconds() const;

  /// 获取时钟类型 (Get the clock type)
  /**
   * \return 时钟类型 (the clock type)
   */
  RCLCPP_PUBLIC
  rcl_clock_type_t get_clock_type() const;

private:
  rcl_time_point_t rcl_time_;
  friend Clock;  // 允许时钟操作内部数据 (Allow clock to manipulate internal data)
};

/**
 * \throws std::overflow_error 如果加法导致溢出 (if addition leads to overflow)
 */
RCLCPP_PUBLIC
Time operator+(const rclcpp::Duration& lhs, const rclcpp::Time& rhs);

}  // namespace rclcpp

#endif  // RCLCPP__TIME_HPP_

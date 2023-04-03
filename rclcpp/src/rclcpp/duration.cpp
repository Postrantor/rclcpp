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

#include "builtin_interfaces/msg/duration.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <utility>

#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/time.hpp"
#include "rcutils/logging_macros.h"

namespace rclcpp {

/**
 * @brief      构造一个 Duration 对象（Constructs a Duration object）
 *
 * @param[in]  seconds      秒数（Number of seconds）
 * @param[in]  nanoseconds  纳秒数（Number of nanoseconds）
 */
Duration::Duration(int32_t seconds, uint32_t nanoseconds) {
  // 将秒数转换为纳秒数并存储在 rcl_duration_ 结构中（Convert seconds to nanoseconds and store in
  // rcl_duration_ structure）
  rcl_duration_.nanoseconds = RCL_S_TO_NS(static_cast<int64_t>(seconds));
  // 将传入的纳秒数添加到已转换的纳秒数上（Add the input nanoseconds to the converted nanoseconds）
  rcl_duration_.nanoseconds += nanoseconds;
}

/**
 * @brief      使用 std::chrono::nanoseconds 构造一个 Duration 对象（Constructs a Duration object
 * using std::chrono::nanoseconds）
 *
 * @param[in]  nanoseconds  纳秒数（Number of nanoseconds）
 */
Duration::Duration(std::chrono::nanoseconds nanoseconds) {
  // 将 std::chrono::nanoseconds 的计数值存储在 rcl_duration_ 结构中（Store the count value of
  // std::chrono::nanoseconds in rcl_duration_ structure）
  rcl_duration_.nanoseconds = nanoseconds.count();
}

// 默认复制构造函数（Default copy constructor）
Duration::Duration(const Duration& rhs) = default;

/**
 * @brief      使用 builtin_interfaces::msg::Duration 消息构造一个 Duration 对象（Constructs a
 * Duration object using builtin_interfaces::msg::Duration message）
 *
 * @param[in]  duration_msg  Duration 消息（Duration message）
 */
Duration::Duration(const builtin_interfaces::msg::Duration& duration_msg) {
  // 将 Duration 消息的秒数转换为纳秒数并存储在 rcl_duration_ 结构中（Convert seconds of Duration
  // message to nanoseconds and store in rcl_duration_ structure）
  rcl_duration_.nanoseconds = RCL_S_TO_NS(static_cast<rcl_duration_value_t>(duration_msg.sec));
  // 将 Duration 消息的纳秒数添加到已转换的纳秒数上（Add the nanoseconds of Duration message to the
  // converted nanoseconds）
  rcl_duration_.nanoseconds += static_cast<rcl_duration_value_t>(duration_msg.nanosec);
}

/**
 * @brief      使用 rcl_duration_t 构造一个 Duration 对象（Constructs a Duration object using
 * rcl_duration_t）
 *
 * @param[in]  duration  rcl_duration_t 结构（rcl_duration_t structure）
 */
Duration::Duration(const rcl_duration_t& duration) : rcl_duration_(duration) {
  // 无操作（No operation）
}

/**
 * @brief 将 Duration 类型转换为 builtin_interfaces::msg::Duration 类型的操作符重载。
 * @brief Operator overload to convert Duration type to builtin_interfaces::msg::Duration type.
 *
 * @return 返回一个与当前 Duration 对象相等的 builtin_interfaces::msg::Duration 对象。
 * @return Returns a builtin_interfaces::msg::Duration object equal to the current Duration object.
 */
builtin_interfaces::msg::Duration Duration::operator builtin_interfaces::msg::Duration() const {
  // 创建一个新的 msg_duration 对象
  // Create a new msg_duration object
  builtin_interfaces::msg::Duration msg_duration;

  // 定义一个常量 kDivisor，用于将纳秒转换为秒
  // Define a constant kDivisor for converting nanoseconds to seconds
  constexpr rcl_duration_value_t kDivisor = RCL_S_TO_NS(1);

  // 定义最大和最小的 int32_t 值
  // Define the maximum and minimum int32_t values
  constexpr int32_t max_s = std::numeric_limits<int32_t>::max();
  constexpr int32_t min_s = std::numeric_limits<int32_t>::min();

  // 定义最大的 uint32_t 值
  // Define the maximum uint32_t value
  constexpr uint32_t max_ns = std::numeric_limits<uint32_t>::max();

  // 计算商和余数
  // Calculate quotient and remainder
  const auto result = std::div(rcl_duration_.nanoseconds, kDivisor);

  // 检查余数是否大于等于0
  // Check if the remainder is greater than or equal to 0
  if (result.rem >= 0) {
    // 如果商大于最大值，则进行饱和处理
    // Saturate if the quotient is greater than the maximum value
    if (result.quot > max_s) {
      msg_duration.sec = max_s;
      msg_duration.nanosec = max_ns;
    } else {
      // 否则，将商和余数分别赋给 msg_duration 的 sec 和 nanosec 成员
      // Otherwise, assign the quotient and remainder to the sec and nanosec members of msg_duration
      // respectively
      msg_duration.sec = static_cast<int32_t>(result.quot);
      msg_duration.nanosec = static_cast<uint32_t>(result.rem);
    }
  } else {
    // 如果商小于等于最小值，则进行饱和处理
    // Saturate if the quotient is less than or equal to the minimum value
    if (result.quot <= min_s) {
      msg_duration.sec = min_s;
      msg_duration.nanosec = 0u;
    } else {
      // 否则，将商减1和 kDivisor 加余数分别赋给 msg_duration 的 sec 和 nanosec 成员
      // Otherwise, assign the quotient minus 1 and kDivisor plus remainder to the sec and nanosec
      // members of msg_duration respectively
      msg_duration.sec = static_cast<int32_t>(result.quot - 1);
      msg_duration.nanosec = static_cast<uint32_t>(kDivisor + result.rem);
    }
  }

  // 返回转换后的 msg_duration 对象
  // Return the converted msg_duration object
  return msg_duration;
}

// 使用默认的赋值操作符重载
// Use the default assignment operator overload
Duration& Duration::operator=(const Duration& rhs) = default;

/**
 * @brief 将 builtin_interfaces::msg::Duration 类型赋值给 Duration 类型的操作符重载。
 * @brief Operator overload to assign a builtin_interfaces::msg::Duration type to a Duration type.
 *
 * @param duration_msg 一个 builtin_interfaces::msg::Duration 类型的对象。
 * @param duration_msg A builtin_interfaces::msg::Duration type object.
 *
 * @return 返回当前 Duration 对象的引用。
 * @return Returns a reference to the current Duration object.
 */
Duration& Duration::operator=(const builtin_interfaces::msg::Duration& duration_msg) {
  // 使用构造函数创建一个新的 Duration 对象，并将其赋值给当前对象
  // Create a new Duration object using the constructor and assign it to the current object
  *this = Duration(duration_msg);

  // 返回当前对象的引用
  // Return the reference to the current object
  return *this;
}

// 定义比较操作符重载，用于比较两个 Duration 对象是否相等
// Define comparison operator overloads for checking if two Duration objects are equal
bool Duration::operator==(const rclcpp::Duration& rhs) const {
  return rcl_duration_.nanoseconds == rhs.rcl_duration_.nanoseconds;
}
bool Duration::operator!=(const rclcpp::Duration& rhs) const {
  return rcl_duration_.nanoseconds != rhs.rcl_duration_.nanoseconds;
}

// 定义比较操作符重载，用于比较两个 Duration 对象的大小关系
// Define comparison operator overloads for comparing the size relationship between two Duration
// objects
bool Duration::operator<(const rclcpp::Duration& rhs) const {
  return rcl_duration_.nanoseconds < rhs.rcl_duration_.nanoseconds;
}
bool Duration::operator<=(const rclcpp::Duration& rhs) const {
  return rcl_duration_.nanoseconds <= rhs.rcl_duration_.nanoseconds;
}
bool Duration::operator>=(const rclcpp::Duration& rhs) const {
  return rcl_duration_.nanoseconds >= rhs.rcl_duration_.nanoseconds;
}
bool Duration::operator>(const rclcpp::Duration& rhs) const {
  return rcl_duration_.nanoseconds > rhs.rcl_duration_.nanoseconds;
}

/**
 * @brief 检查两个 int64_t 类型的值相加是否会导致溢出或下溢
 * @param lhsns 左操作数（nanoseconds）
 * @param rhsns 右操作数（nanoseconds）
 * @param max 允许的最大值
 *
 * @details Check if the addition of two int64_t values would cause overflow or underflow
 * @param lhsns Left operand (nanoseconds)
 * @param rhsns Right operand (nanoseconds)
 * @param max Maximum allowed value
 */
void bounds_check_duration_sum(int64_t lhsns, int64_t rhsns, uint64_t max) {
  // 将左操作数和右操作数转换为无符号整数
  // Convert left and right operands to unsigned integers
  auto abs_lhs = static_cast<uint64_t>(std::abs(lhsns));
  auto abs_rhs = static_cast<uint64_t>(std::abs(rhsns));

  // 当两个操作数都是正数时，检查相加后是否超过最大值
  // When both operands are positive, check if their sum exceeds the maximum value
  if (lhsns > 0 && rhsns > 0) {
    if (abs_lhs + abs_rhs > max) {
      throw std::overflow_error("addition leads to int64_t overflow");
    }
  } else if (lhsns < 0 && rhsns < 0) {  // 当两个操作数都是负数时，检查相加后是否超过最大值
    if (abs_lhs + abs_rhs > max) {
      throw std::underflow_error("addition leads to int64_t underflow");
    }
  }
}

// 重载 Duration 类的加法运算符
// Overload the addition operator for the Duration class
Duration Duration::operator+(const rclcpp::Duration& rhs) const {
  // 检查两个时间值相加是否会导致溢出或下溢
  // Check if the addition of two time values would cause overflow or underflow
  bounds_check_duration_sum(
      this->rcl_duration_.nanoseconds, rhs.rcl_duration_.nanoseconds,
      std::numeric_limits<rcl_duration_value_t>::max());

  // 返回相加后的新 Duration 对象
  // Return the new Duration object after addition
  return Duration::from_nanoseconds(rcl_duration_.nanoseconds + rhs.rcl_duration_.nanoseconds);
}

// 重载 Duration 类的自加运算符
// Overload the self-addition operator for the Duration class
Duration& Duration::operator+=(const rclcpp::Duration& rhs) {
  *this = *this + rhs;
  return *this;
}

/**
 * @brief 检查两个 int64_t 类型的值相减是否会导致溢出或下溢
 * @param lhsns 左操作数（nanoseconds）
 * @param rhsns 右操作数（nanoseconds）
 * @param max 允许的最大值
 *
 * @details Check if the subtraction of two int64_t values would cause overflow or underflow
 * @param lhsns Left operand (nanoseconds)
 * @param rhsns Right operand (nanoseconds)
 * @param max Maximum allowed value
 */
void bounds_check_duration_difference(int64_t lhsns, int64_t rhsns, uint64_t max) {
  // 将左操作数和右操作数转换为无符号整数
  // Convert left and right operands to unsigned integers
  auto abs_lhs = static_cast<uint64_t>(std::abs(lhsns));
  auto abs_rhs = static_cast<uint64_t>(std::abs(rhsns));

  // 当左操作数为正数且右操作数为负数时，检查相减后是否超过最大值
  // When the left operand is positive and the right operand is negative, check if their difference
  // exceeds the maximum value
  if (lhsns > 0 && rhsns < 0) {
    if (abs_lhs + abs_rhs > max) {
      throw std::overflow_error("duration subtraction leads to int64_t overflow");
    }
  } else if (
      lhsns < 0 && rhsns > 0) {  // 当左操作数为负数且右操作数为正数时，检查相减后是否超过最大值
    if (abs_lhs + abs_rhs > max) {
      throw std::underflow_error("duration subtraction leads to int64_t underflow");
    }
  }
}

// 重载 Duration 类的减法运算符
// Overload the subtraction operator for the Duration class
Duration Duration::operator-(const rclcpp::Duration& rhs) const {
  // 检查两个时间值相减是否会导致溢出或下溢
  // Check if the subtraction of two time values would cause overflow or underflow
  bounds_check_duration_difference(
      this->rcl_duration_.nanoseconds, rhs.rcl_duration_.nanoseconds,
      std::numeric_limits<rcl_duration_value_t>::max());

  // 返回相减后的新 Duration 对象
  // Return the new Duration object after subtraction
  return Duration::from_nanoseconds(rcl_duration_.nanoseconds - rhs.rcl_duration_.nanoseconds);
}

// 重载 Duration 类的自减运算符
// Overload the self-subtraction operator for the Duration class
Duration& Duration::operator-=(const rclcpp::Duration& rhs) {
  *this = *this - rhs;
  return *this;
}

/**
 * @brief 检查 int64_t 类型的值乘以一个 double 类型的缩放因子是否会导致溢出或下溢
 * @param dns 时间值（nanoseconds）
 * @param scale 缩放因子
 * @param max 允许的最大值
 *
 * @details Check if an int64_t value multiplied by a double scaling factor would cause overflow or
 * underflow
 * @param dns Time value (nanoseconds)
 * @param scale Scaling factor
 * @param max Maximum allowed value
 */
void bounds_check_duration_scale(int64_t dns, double scale, uint64_t max) {
  // 将时间值转换为无符号整数，并计算缩放因子的绝对值
  // Convert the time value to an unsigned integer and calculate the absolute value of the scaling
  // factor
  auto abs_dns = static_cast<uint64_t>(std::abs(dns));
  auto abs_scale = std::abs(scale);

  // 检查乘法操作是否会导致溢出或下溢
  // Check if the multiplication operation would cause overflow or underflow
  if (abs_scale > 1.0 &&
      abs_dns > static_cast<uint64_t>(
                    static_cast<long double>(max) / static_cast<long double>(abs_scale))) {
    if ((dns > 0 && scale > 0) || (dns < 0 && scale < 0)) {
      throw std::overflow_error("duration scaling leads to int64_t overflow");
    } else {
      throw std::underflow_error("duration scaling leads to int64_t underflow");
    }
  }
}

/**
 * @brief 重载 Duration 类的乘法运算符，返回一个新的 Duration 对象，其值为当前对象与给定倍数的乘积。
 * Overloads the multiplication operator for the Duration class, returning a new Duration object
 * that is the product of the current object and the given scale.
 *
 * @param[in] scale 用于缩放 Duration 的倍数。The scale factor to multiply the Duration with.
 * @return 返回一个新的 Duration 对象，其值为当前对象与给定倍数的乘积。Returns a new Duration object
 * that is the product of the current object and the given scale.
 * @throws std::runtime_error 如果倍数不是有限数。If the scale factor is not finite.
 */
Duration Duration::operator*(double scale) const {
  // 检查倍数是否为有限数。Check if the scale factor is finite.
  if (!std::isfinite(scale)) {
    throw std::runtime_error("abnormal scale in rclcpp::Duration");
  }
  // 对乘法操作进行边界检查。Perform boundary check for the multiplication operation.
  bounds_check_duration_scale(
      this->rcl_duration_.nanoseconds, scale, std::numeric_limits<rcl_duration_value_t>::max());
  // 将倍数从 double 类型转换为 long double 类型。Convert the scale factor from double type to long
  // double type.
  long double scale_ld = static_cast<long double>(scale);
  // 返回一个新的 Duration 对象，其值为当前对象与给定倍数的乘积。Return a new Duration object that
  // is the product of the current object and the given scale.
  return Duration::from_nanoseconds(static_cast<rcl_duration_value_t>(
      static_cast<long double>(rcl_duration_.nanoseconds) * scale_ld));
}

/**
 * @brief 重载 Duration 类的乘法赋值运算符，用于将当前对象与给定倍数相乘并更新当前对象。
 * Overloads the multiplication assignment operator for the Duration class, used to multiply the
 * current object with the given scale and update the current object.
 *
 * @param[in] scale 用于缩放 Duration 的倍数。The scale factor to multiply the Duration with.
 * @return 返回一个引用，指向已更新的当前对象。Returns a reference to the updated current object.
 */
Duration& Duration::operator*=(double scale) {
  // 将当前对象与给定倍数相乘，并更新当前对象。Multiply the current object with the given scale and
  // update the current object.
  *this = *this * scale;
  return *this;
}

/**
 * @brief 获取 Duration 对象的纳秒值。Get the nanosecond value of the Duration object.
 *
 * @return 返回 Duration 对象的纳秒值。Returns the nanosecond value of the Duration object.
 */
rcl_duration_value_t Duration::nanoseconds() const { return rcl_duration_.nanoseconds; }

/**
 * @brief 创建一个表示最大持续时间的 Duration 对象。Create a Duration object representing the
 * maximum duration.
 *
 * @return 返回一个表示最大持续时间的 Duration 对象。Returns a Duration object representing the
 * maximum duration.
 */
Duration Duration::max() { return Duration(std::numeric_limits<int32_t>::max(), 999999999); }

/**
 * @brief 将 Duration 对象转换为以秒为单位的双精度浮点数。Convert the Duration object to a
 * double-precision floating-point number in seconds.
 *
 * @return 返回 Duration 对象的以秒为单位的双精度浮点数。Returns the double-precision floating-point
 * number in seconds of the Duration object.
 */
double Duration::seconds() const {
  return std::chrono::duration<double>(std::chrono::nanoseconds(rcl_duration_.nanoseconds)).count();
}

/**
 * @brief 将 Duration 对象转换为 rmw_time_t 类型。Convert the Duration object to rmw_time_t type.
 *
 * @return 返回 Duration 对象的 rmw_time_t 表示。Returns the rmw_time_t representation of the
 * Duration object.
 * @throws std::runtime_error 如果 Duration 对象的纳秒值为负。If the nanosecond value of the
 * Duration object is negative.
 */
rmw_time_t Duration::to_rmw_time() const {
  // 检查 Duration 对象的纳秒值是否为负。Check if the nanosecond value of the Duration object is
  // negative.
  if (rcl_duration_.nanoseconds < 0) {
    throw std::runtime_error("rmw_time_t cannot be negative");
  }

  // 故意避免从 builtin_interfaces::msg::Duration 创建，以防在从 int64_t 转换为 int32_t，然后回到
  // uint64_t 时溢出。 Purposefully avoid creating from builtin_interfaces::msg::Duration to avoid
  // possible overflow converting from int64_t to int32_t, then back to uint64_t.
  rmw_time_t result;
  constexpr rcl_duration_value_t kDivisor = RCL_S_TO_NS(1);
  const auto div_result = std::div(rcl_duration_.nanoseconds, kDivisor);
  result.sec = static_cast<uint64_t>(div_result.quot);
  result.nsec = static_cast<uint64_t>(div_result.rem);

  return result;
}

/**
 * @brief 从 rmw_time_t 类型创建一个 Duration 对象。Create a Duration object from an rmw_time_t
 * type.
 *
 * @param[in] duration rmw_time_t 类型的持续时间。The duration of type rmw_time_t.
 * @return 返回一个表示给定 rmw_time_t 持续时间的 Duration 对象。Returns a Duration object
 * representing the given rmw_time_t duration.
 */
Duration Duration::from_rmw_time(rmw_time_t duration) {
  Duration ret;
  constexpr rcl_duration_value_t limit_ns = std::numeric_limits<rcl_duration_value_t>::max();
  constexpr rcl_duration_value_t limit_sec = RCL_NS_TO_S(limit_ns);
  if (duration.sec > limit_sec || duration.nsec > limit_ns) {
    // 如果溢出，则使其饱和。Saturate if it will overflow.
    ret.rcl_duration_.nanoseconds = limit_ns;
    return ret;
  }
  uint64_t total_ns = RCL_S_TO_NS(duration.sec) + duration.nsec;
  if (total_ns > limit_ns) {
    // 如果溢出，则使其饱和。Saturate if it will overflow.
    ret.rcl_duration_.nanoseconds = limit_ns;
    return ret;
  }
  ret.rcl_duration_.nanoseconds = static_cast<rcl_duration_value_t>(total_ns);
  return ret;
}

/**
 * @brief 从以秒为单位的双精度浮点数创建一个 Duration 对象。Create a Duration object from a
 * double-precision floating-point number in seconds.
 *
 * @param[in] seconds 以秒为单位的双精度浮点数。The double-precision floating-point number in
 * seconds.
 * @return 返回一个表示给定秒数的 Duration 对象。Returns a Duration object representing the given
 * number of seconds.
 */
Duration Duration::from_seconds(double seconds) {
  Duration ret;
  ret.rcl_duration_.nanoseconds = static_cast<int64_t>(RCL_S_TO_NS(seconds));
  return ret;
}

/**
 * @brief 从纳秒值创建一个 Duration 对象。Create a Duration object from a nanosecond value.
 *
 * @param[in] nanoseconds 纳秒值。The nanosecond value.
 * @return 返回一个表示给定纳秒值的 Duration 对象。Returns a Duration object representing the given
 * nanosecond value.
 */
Duration Duration::from_nanoseconds(rcl_duration_value_t nanoseconds) {
  Duration ret;
  ret.rcl_duration_.nanoseconds = nanoseconds;
  return ret;
}

}  // namespace rclcpp

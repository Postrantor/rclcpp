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

#ifndef RCLCPP__DURATION_HPP_
#define RCLCPP__DURATION_HPP_

#include <chrono>

#include "builtin_interfaces/msg/duration.hpp"
#include "rcl/time.h"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
class RCLCPP_PUBLIC Duration {
public:
  /// Duration 构造函数。 (Duration constructor)
  /**
   * 分别初始化秒和纳秒的时间值。 (Initializes the time values for seconds and nanoseconds
   * individually) 大的 nsecs 值会自动换算，余数会加到 secs 上。 (Large values for nsecs are wrapped
   * automatically with the remainder added to secs) 两个输入都必须是整数。 (Both inputs must be
   * integers) 秒可以为负数。 (Seconds can be negative)
   *
   * \param seconds 时间（以秒为单位）(time in seconds)
   * \param nanoseconds 时间（以纳秒为单位）(time in nanoseconds)
   */
  Duration(int32_t seconds, uint32_t nanoseconds);

  /// 从指定的 std::chrono::nanoseconds 构造 duration。 (Construct duration from the specified
  /// std::chrono::nanoseconds)
  explicit Duration(std::chrono::nanoseconds nanoseconds);

  // 这个构造函数匹配除了纳秒以外的任何 std::chrono 值 (This constructor matches any std::chrono
  // value other than nanoseconds) 故意不使用 explicit 创建转换构造函数 (intentionally not using
  // explicit to create a conversion constructor)
  template <class Rep, class Period>
  // cppcheck-suppress noExplicitConstructor
  Duration(const std::chrono::duration<Rep, Period>& duration)  // NOLINT(runtime/explicit)
      : Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(duration)) {}

  // cppcheck-suppress noExplicitConstructor
  Duration(const builtin_interfaces::msg::Duration& duration_msg);  // NOLINT(runtime/explicit)

  /// 时间构造函数 (Time constructor)
  /**
   * \param duration 要复制的 rcl_duration_t 结构。 (rcl_duration_t structure to copy)
   */
  explicit Duration(const rcl_duration_t& duration);

  Duration(const Duration& rhs);

  virtual ~Duration() = default;

  operator builtin_interfaces::msg::Duration() const;

  /**
   * @brief 重载赋值运算符，用于将一个 Duration 对象的值赋给另一个 Duration 对象。
   * @param rhs 右侧操作数，即要复制的 Duration 对象。
   * @return 返回一个 Duration 引用，表示被赋值后的对象。
   *
   * @brief Overload assignment operator, used to assign the value of one Duration object to
   * another.
   * @param rhs The right-hand operand, i.e., the Duration object to be copied.
   * @return Returns a reference to the Duration after assignment.
   */
  Duration& operator=(const Duration& rhs);

  /**
   * @brief 重载赋值运算符，用于将一个 builtin_interfaces::msg::Duration 对象的值赋给一个 Duration
   * 对象。
   * @param duration_msg 要复制的 builtin_interfaces::msg::Duration 对象。
   * @return 返回一个 Duration 引用，表示被赋值后的对象。
   *
   * @brief Overload assignment operator, used to assign the value of a
   * builtin_interfaces::msg::Duration object to a Duration object.
   * @param duration_msg The builtin_interfaces::msg::Duration object to be copied.
   * @return Returns a reference to the Duration after assignment.
   */
  Duration& operator=(const builtin_interfaces::msg::Duration& duration_msg);

  /**
   * @brief 重载相等运算符，用于比较两个 Duration 对象是否相等。
   * @param rhs 要比较的 Duration 对象。
   * @return 如果两个 Duration 对象相等，则返回 true，否则返回 false。
   *
   * @brief Overload equality operator, used to compare whether two Duration objects are equal.
   * @param rhs The Duration object to be compared.
   * @return Returns true if the two Duration objects are equal, otherwise returns false.
   */
  bool operator==(const rclcpp::Duration& rhs) const;

  /**
   * @brief 重载不等运算符，用于比较两个 Duration 对象是否不相等。
   * @param rhs 要比较的 Duration 对象。
   * @return 如果两个 Duration 对象不相等，则返回 true，否则返回 false。
   *
   * @brief Overload inequality operator, used to compare whether two Duration objects are not
   * equal.
   * @param rhs The Duration object to be compared.
   * @return Returns true if the two Duration objects are not equal, otherwise returns false.
   */
  bool operator!=(const rclcpp::Duration& rhs) const;

  /**
   * @brief 重载小于运算符，用于比较一个 Duration 对象是否小于另一个 Duration 对象。
   * @param rhs 要比较的 Duration 对象。
   * @return 如果左侧 Duration 对象小于右侧 Duration 对象，则返回 true，否则返回 false。
   *
   * @brief Overload less-than operator, used to compare whether one Duration object is less than
   * another.
   * @param rhs The Duration object to be compared.
   * @return Returns true if the left-hand Duration object is less than the right-hand Duration
   * object, otherwise returns false.
   */
  bool operator<(const rclcpp::Duration& rhs) const;

  /**
   * @brief 重载小于等于运算符，用于比较一个 Duration 对象是否小于等于另一个 Duration 对象。
   * @param rhs 要比较的 Duration 对象。
   * @return 如果左侧 Duration 对象小于等于右侧 Duration 对象，则返回 true，否则返回 false。
   *
   * @brief Overload less-than-or-equal-to operator, used to compare whether one Duration object is
   * less than or equal to another.
   * @param rhs The Duration object to be compared.
   * @return Returns true if the left-hand Duration object is less than or equal to the right-hand
   * Duration object, otherwise returns false.
   */
  bool operator<=(const rclcpp::Duration& rhs) const;

  /**
   * @brief 重载大于等于运算符，用于比较一个 Duration 对象是否大于等于另一个 Duration 对象。
   * @param rhs 要比较的 Duration 对象。
   * @return 如果左侧 Duration 对象大于等于右侧 Duration 对象，则返回 true，否则返回 false。
   *
   * @brief Overload greater-than-or-equal-to operator, used to compare whether one Duration object
   * is greater than or equal to another.
   * @param rhs The Duration object to be compared.
   * @return Returns true if the left-hand Duration object is greater than or equal to the
   * right-hand Duration object, otherwise returns false.
   */
  bool operator>=(const rclcpp::Duration& rhs) const;

  /**
   * @brief 重载大于运算符，用于比较一个 Duration 对象是否大于另一个 Duration 对象。
   * @param rhs 要比较的 Duration 对象。
   * @return 如果左侧 Duration 对象大于右侧 Duration 对象，则返回 true，否则返回 false。
   *
   * @brief Overload greater-than operator, used to compare whether one Duration object is greater
   * than another.
   * @param rhs The Duration object to be compared.
   * @return Returns true if the left-hand Duration object is greater than the right-hand Duration
   * object, otherwise returns false.
   */
  bool operator>(const rclcpp::Duration& rhs) const;

  /**
   * @brief 重载加法运算符，用于计算两个 Duration 对象的和。
   * @param rhs 要相加的 Duration 对象。
   * @return 返回一个新的 Duration 对象，表示两个 Duration 对象的和。
   *
   * @brief Overload addition operator, used to calculate the sum of two Duration objects.
   * @param rhs The Duration object to be added.
   * @return Returns a new Duration object, representing the sum of the two Duration objects.
   */
  Duration operator+(const rclcpp::Duration& rhs) const;

  /**
   * @brief 重载加法赋值运算符，用于将当前 Duration 对象与另一个 Duration
   * 对象相加，并将结果赋给当前对象。
   * @param rhs 要相加的 Duration 对象。
   * @return 返回一个 Duration 引用，表示被赋值后的对象。
   *
   * @brief Overload addition assignment operator, used to add the current Duration object to
   * another Duration object and assign the result to the current object.
   * @param rhs The Duration object to be added.
   * @return Returns a reference to the Duration after assignment.
   */
  Duration& operator+=(const rclcpp::Duration& rhs);

  /**
   * @brief 重载减法运算符，用于计算两个 Duration 对象的差。
   * @param rhs 要相减的 Duration 对象。
   * @return 返回一个新的 Duration 对象，表示两个 Duration 对象的差。
   *
   * @brief Overload subtraction operator, used to calculate the difference between two Duration
   * objects.
   * @param rhs The Duration object to be subtracted.
   * @return Returns a new Duration object, representing the difference between the two Duration
   * objects.
   */
  Duration operator-(const rclcpp::Duration& rhs) const;

  /**
   * @brief 重载减法赋值运算符，用于将当前 Duration 对象与另一个 Duration
   * 对象相减，并将结果赋给当前对象。
   * @param rhs 要相减的 Duration 对象。
   * @return 返回一个 Duration 引用，表示被赋值后的对象。
   *
   * @brief Overload subtraction assignment operator, used to subtract the current Duration object
   * from another Duration object and assign the result to the current object.
   * @param rhs The Duration object to be subtracted.
   * @return Returns a reference to the Duration after assignment.
   */
  Duration& operator-=(const rclcpp::Duration& rhs);

  /// 获取最大可表示值。
  /// Get the maximum representable value.
  /**
   * \return 最大可表示值
   * \return the maximum representable value
   */
  static Duration max();

  /// 乘法运算符，用于将 Duration 对象与一个 double 类型的缩放因子相乘。
  /// Multiplication operator for multiplying a Duration object with a double scaling factor.
  /**
   * \param scale 缩放因子
   * \return 缩放后的 Duration 对象
   * \param scale The scaling factor
   * \return The scaled Duration object
   */
  Duration operator*(double scale) const;

  /// 乘法赋值运算符，用于将当前 Duration 对象与一个 double 类型的缩放因子相乘并更新当前对象。
  /// Multiplication assignment operator for multiplying and updating the current Duration object
  /// with a double scaling factor.
  /**
   * \param scale 缩放因子
   * \return 更新后的 Duration 对象的引用
   * \param scale The scaling factor
   * \return A reference to the updated Duration object
   */
  Duration& operator*=(double scale);

  /// 获取纳秒级持续时间
  /// Get duration in nanoseconds
  /**
   * \return 以 rcl_duration_value_t 表示的纳秒持续时间
   * \return The duration in nanoseconds as a rcl_duration_value_t.
   */
  rcl_duration_value_t nanoseconds() const;

  /// 获取秒级持续时间
  /// Get duration in seconds
  /**
   * \warning 取决于 sizeof(double)，可能会有显著的精度损失。需要精确时间时，请使用 nanoseconds()
   * 代替。 \warning Depending on sizeof(double) there could be significant precision loss. When an
   * exact time is required, use nanoseconds() instead. \return 以浮点数表示的秒级持续时间 \return
   * The duration in seconds as a floating point number.
   */
  double seconds() const;

  /// 从表示秒的浮点数创建 Duration 对象
  /// Create a duration object from a floating point number representing seconds
  /**
   * \param seconds 表示秒的浮点数
   * \return 创建的 Duration 对象
   * \param seconds Floating point number representing seconds
   * \return The created Duration object
   */
  static Duration from_seconds(double seconds);

  /// 从表示纳秒的整数创建 Duration 对象
  /// Create a duration object from an integer number representing nanoseconds
  /**
   * \param nanoseconds 表示纳秒的整数
   * \return 创建的 Duration 对象
   * \param nanoseconds Integer number representing nanoseconds
   * \return The created Duration object
   */
  static Duration from_nanoseconds(rcl_duration_value_t nanoseconds);

  /// 从 rmw_time_t 类型创建 Duration 对象
  /// Create a Duration object from rmw_time_t type
  /**
   * \param duration rmw_time_t 类型的持续时间
   * \return 创建的 Duration 对象
   * \param duration Duration of type rmw_time_t
   * \return The created Duration object
   */
  static Duration from_rmw_time(rmw_time_t duration);

  /// 将 Duration 转换为 std::chrono::Duration。
  /// Convert Duration into a std::chrono::Duration.
  /**
   * \tparam DurationT 要转换为的 std::chrono::duration 类型
   * \return 转换后的 std::chrono::duration 对象
   * \tparam DurationT The std::chrono::duration type to convert to
   * \return The converted std::chrono::duration object
   */
  template <class DurationT>
  DurationT to_chrono() const {
    return std::chrono::duration_cast<DurationT>(std::chrono::nanoseconds(this->nanoseconds()));
  }

  /**
   * @brief 将 Duration 转换为 rmw_time_t 类型（Convert Duration into rmw_time_t）
   *
   * @return 返回转换后的 rmw_time_t 类型变量（Return the converted rmw_time_t variable）
   */
  rmw_time_t to_rmw_time() const;

private:
  // ROS2 内部表示时间长度的结构体 (Internal ROS2 structure representing duration)
  rcl_duration_t rcl_duration_;

  Duration() = default;
};

}  // namespace rclcpp

#endif  // RCLCPP__DURATION_HPP_

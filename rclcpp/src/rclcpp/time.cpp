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

#include "builtin_interfaces/msg/time.hpp"

#include <limits>
#include <string>
#include <utility>

#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/utilities.hpp"
#include "rcutils/logging_macros.h"

namespace {

/**
 * @brief 初始化时间点对象 (Initialize a time point object)
 *
 * 这个函数用于初始化一个 rcl_time_point_t 对象，根据给定的时钟类型。
 * This function is used to initialize an rcl_time_point_t object according to the given clock type.
 *
 * @param clock_type 传入的时钟类型引用 (The reference of the input clock type)
 * @return 返回初始化后的时间点对象 (Returns the initialized time point object)
 */
rcl_time_point_t init_time_point(rcl_clock_type_t& clock_type) {
  // 创建一个 rcl_time_point_t 对象 (Create an rcl_time_point_t object)
  rcl_time_point_t time_point;

  // 将传入的时钟类型赋值给 time_point 的 clock_type 成员变量
  // Assign the input clock type to the clock_type member variable of time_point
  time_point.clock_type = clock_type;

  // 返回初始化后的时间点对象 (Return the initialized time point object)
  return time_point;
}

}  // namespace

namespace rclcpp {

/**
 * @brief 构造函数，根据秒、纳秒和时钟类型创建一个 Time 对象 (Constructor, creates a Time object
 * based on seconds, nanoseconds and clock type)
 *
 * @param seconds 秒数 (Number of seconds)
 * @param nanoseconds 纳秒数 (Number of nanoseconds)
 * @param clock_type 时钟类型 (Clock type)
 */
Time::Time(int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t clock_type)
    : rcl_time_(init_time_point(clock_type)) {
  // 检查秒数是否为负值 (Check if the number of seconds is negative)
  if (seconds < 0) {
    throw std::runtime_error("cannot store a negative time point in rclcpp::Time");
  }

  // 将秒数转换为纳秒，并将纳秒数累加 (Convert seconds to nanoseconds and accumulate nanoseconds)
  rcl_time_.nanoseconds = RCL_S_TO_NS(static_cast<int64_t>(seconds));
  rcl_time_.nanoseconds += nanoseconds;
}

/**
 * @brief 构造函数，根据纳秒和时钟类型创建一个 Time 对象 (Constructor, creates a Time object based
 * on nanoseconds and clock type)
 *
 * @param nanoseconds 纳秒数 (Number of nanoseconds)
 * @param clock_type 时钟类型 (Clock type)
 */
Time::Time(int64_t nanoseconds, rcl_clock_type_t clock_type)
    : rcl_time_(init_time_point(clock_type)) {
  // 设置纳秒数 (Set the number of nanoseconds)
  rcl_time_.nanoseconds = nanoseconds;
}

// 默认拷贝构造函数 (Default copy constructor)
Time::Time(const Time& rhs) = default;

/**
 * @brief 构造函数，根据时间消息和时钟类型创建一个 Time 对象 (Constructor, creates a Time object
 * based on time message and clock type)
 *
 * @param time_msg 时间消息 (Time message)
 * @param clock_type 时钟类型 (Clock type)
 */
Time::Time(const builtin_interfaces::msg::Time& time_msg, rcl_clock_type_t clock_type)
    : rcl_time_(init_time_point(clock_type)) {
  // 检查时间消息中的秒数是否为负值 (Check if the number of seconds in the time message is negative)
  if (time_msg.sec < 0) {
    throw std::runtime_error("cannot store a negative time point in rclcpp::Time");
  }

  // 将时间消息中的秒数转换为纳秒，并将纳秒数累加 (Convert seconds in the time message to
  // nanoseconds and accumulate nanoseconds)
  rcl_time_.nanoseconds = RCL_S_TO_NS(static_cast<int64_t>(time_msg.sec));
  rcl_time_.nanoseconds += time_msg.nanosec;
}

/**
 * @brief 构造函数，根据 rcl_time_point_t 创建一个 Time 对象 (Constructor, creates a Time object
 * based on rcl_time_point_t)
 *
 * @param time_point rcl_time_point_t 结构体 (rcl_time_point_t structure)
 */
Time::Time(const rcl_time_point_t& time_point) : rcl_time_(time_point) {
  // 无操作 (No operation)
}

// 析构函数 (Destructor)
Time::~Time() {}

// 类型转换操作符，将 Time 对象转换为时间消息 (Type conversion operator, converts Time object to
// time message)
Time::operator builtin_interfaces::msg::Time() const {
  builtin_interfaces::msg::Time msg_time;
  constexpr rcl_time_point_value_t kRemainder = RCL_S_TO_NS(1);
  const auto result = std::div(rcl_time_.nanoseconds, kRemainder);
  if (result.rem >= 0) {
    msg_time.sec = static_cast<std::int32_t>(result.quot);
    msg_time.nanosec = static_cast<std::uint32_t>(result.rem);
  } else {
    msg_time.sec = static_cast<std::int32_t>(result.quot - 1);
    msg_time.nanosec = static_cast<std::uint32_t>(kRemainder + result.rem);
  }
  return msg_time;
}

// 默认赋值操作符 (Default assignment operator)
Time& Time::operator=(const Time& rhs) = default;

/**
 * @brief 赋值操作符，将时间消息赋值给 Time 对象 (Assignment operator, assigns time message to Time
 * object)
 *
 * @param time_msg 时间消息 (Time message)
 * @return Time& 返回当前 Time 对象的引用 (Returns a reference to the current Time object)
 */
Time& Time::operator=(const builtin_interfaces::msg::Time& time_msg) {
  *this = Time(time_msg);
  return *this;
}

/**
 * @brief 比较两个时间是否相等 (Compare two times for equality)
 *
 * @param rhs 另一个要比较的时间对象 (The other time object to compare)
 * @return bool 如果两个时间相等则返回true，否则返回false (True if the two times are equal, false
 * otherwise)
 */
bool Time::operator==(const rclcpp::Time& rhs) const {
  // 检查两个时间对象的时钟类型是否相同 (Check if the clock types of the two time objects are the
  // same)
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  // 比较两个时间对象的纳秒值是否相等 (Compare the nanoseconds values of the two time objects for
  // equality)
  return rcl_time_.nanoseconds == rhs.rcl_time_.nanoseconds;
}

/**
 * @brief 比较两个时间是否不相等 (Compare two times for inequality)
 *
 * @param rhs 另一个要比较的时间对象 (The other time object to compare)
 * @return bool 如果两个时间不相等则返回true，否则返回false (True if the two times are not equal,
 * false otherwise)
 */
bool Time::operator!=(const rclcpp::Time& rhs) const { return !(*this == rhs); }

/**
 * @brief 比较当前时间是否小于另一个时间 (Compare if the current time is less than another time)
 *
 * @param rhs 另一个要比较的时间对象 (The other time object to compare)
 * @return bool 如果当前时间小于另一个时间则返回true，否则返回false (True if the current time is
 * less than the other time, false otherwise)
 */
bool Time::operator<(const rclcpp::Time& rhs) const {
  // 检查两个时间对象的时钟类型是否相同 (Check if the clock types of the two time objects are the
  // same)
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  // 比较当前时间的纳秒值是否小于另一个时间的纳秒值 (Compare if the nanoseconds value of the current
  // time is less than the other time's nanoseconds value)
  return rcl_time_.nanoseconds < rhs.rcl_time_.nanoseconds;
}

/**
 * @brief 比较当前时间是否小于等于另一个时间 (Compare if the current time is less than or equal to
 * another time)
 *
 * @param rhs 另一个要比较的时间对象 (The other time object to compare)
 * @return bool 如果当前时间小于等于另一个时间则返回true，否则返回false (True if the current time is
 * less than or equal to the other time, false otherwise)
 */
bool Time::operator<=(const rclcpp::Time& rhs) const {
  // 检查两个时间对象的时钟类型是否相同 (Check if the clock types of the two time objects are the
  // same)
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  // 比较当前时间的纳秒值是否小于等于另一个时间的纳秒值 (Compare if the nanoseconds value of the
  // current time is less than or equal to the other time's nanoseconds value)
  return rcl_time_.nanoseconds <= rhs.rcl_time_.nanoseconds;
}

/**
 * @brief 比较当前时间是否大于等于另一个时间 (Compare if the current time is greater than or equal
 * to another time)
 *
 * @param rhs 另一个要比较的时间对象 (The other time object to compare)
 * @return bool 如果当前时间大于等于另一个时间则返回true，否则返回false (True if the current time is
 * greater than or equal to the other time, false otherwise)
 */
bool Time::operator>=(const rclcpp::Time& rhs) const {
  // 检查两个时间对象的时钟类型是否相同 (Check if the clock types of the two time objects are the
  // same)
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  // 比较当前时间的纳秒值是否大于等于另一个时间的纳秒值 (Compare if the nanoseconds value of the
  // current time is greater than or equal to the other time's nanoseconds value)
  return rcl_time_.nanoseconds >= rhs.rcl_time_.nanoseconds;
}

/**
 * @brief 比较当前时间是否大于另一个时间 (Compare if the current time is greater than another time)
 *
 * @param rhs 另一个要比较的时间对象 (The other time object to compare)
 * @return bool 如果当前时间大于另一个时间则返回true，否则返回false (True if the current time is
 * greater than the other time, false otherwise)
 */
bool Time::operator>(const rclcpp::Time& rhs) const {
  // 检查两个时间对象的时钟类型是否相同 (Check if the clock types of the two time objects are the
  // same)
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  // 比较当前时间的纳秒值是否大于另一个时间的纳秒值 (Compare if the nanoseconds value of the current
  // time is greater than the other time's nanoseconds value)
  return rcl_time_.nanoseconds > rhs.rcl_time_.nanoseconds;
}

/**
 * @brief 重载加法运算符，用于计算两个时间对象之和 (Overloads the addition operator for adding a
 * duration to a time object)
 *
 * @param rhs 要添加的持续时间对象 (The duration object to be added)
 * @return Time 返回相加后的新时间对象 (Returns the new time object after addition)
 */
Time Time::operator+(const rclcpp::Duration& rhs) const {
  // 检查加法是否会导致 int64_t 溢出 (Check if the addition will cause an int64_t overflow)
  if (rclcpp::add_will_overflow(rhs.nanoseconds(), this->nanoseconds())) {
    throw std::overflow_error("addition leads to int64_t overflow");
  }
  // 检查加法是否会导致 int64_t 下溢 (Check if the addition will cause an int64_t underflow)
  if (rclcpp::add_will_underflow(rhs.nanoseconds(), this->nanoseconds())) {
    throw std::underflow_error("addition leads to int64_t underflow");
  }
  // 返回相加后的新时间对象 (Return the new time object after addition)
  return Time(this->nanoseconds() + rhs.nanoseconds(), this->get_clock_type());
}

/**
 * @brief 重载减法运算符，用于计算两个时间对象之差 (Overloads the subtraction operator for
 * calculating the difference between two time objects)
 *
 * @param rhs 要减去的时间对象 (The time object to be subtracted)
 * @return Duration 返回两个时间对象之间的持续时间差 (Returns the duration difference between the
 * two time objects)
 */
Duration Time::operator-(const rclcpp::Time& rhs) const {
  // 检查两个时间对象是否具有相同的时钟类型 (Check if the two time objects have the same clock type)
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error(
        std::string("can't subtract times with different time sources [") +
        std::to_string(rcl_time_.clock_type) + " != " + std::to_string(rhs.rcl_time_.clock_type) +
        "]");
  }

  // 检查减法是否会导致 int64_t 溢出 (Check if the subtraction will cause an int64_t overflow)
  if (rclcpp::sub_will_overflow(rcl_time_.nanoseconds, rhs.rcl_time_.nanoseconds)) {
    throw std::overflow_error("time subtraction leads to int64_t overflow");
  }

  // 检查减法是否会导致 int64_t 下溢 (Check if the subtraction will cause an int64_t underflow)
  if (rclcpp::sub_will_underflow(rcl_time_.nanoseconds, rhs.rcl_time_.nanoseconds)) {
    throw std::underflow_error("time subtraction leads to int64_t underflow");
  }

  // 返回两个时间对象之间的持续时间差 (Return the duration difference between the two time objects)
  return Duration::from_nanoseconds(rcl_time_.nanoseconds - rhs.rcl_time_.nanoseconds);
}

/**
 * @brief 重载减法运算符，用于从时间对象中减去一个持续时间 (Overloads the subtraction operator for
 * subtracting a duration from a time object)
 *
 * @param rhs 要减去的持续时间对象 (The duration object to be subtracted)
 * @return Time 返回相减后的新时间对象 (Returns the new time object after subtraction)
 */
Time Time::operator-(const rclcpp::Duration& rhs) const {
  // 检查减法是否会导致 int64_t 溢出 (Check if the subtraction will cause an int64_t overflow)
  if (rclcpp::sub_will_overflow(rcl_time_.nanoseconds, rhs.nanoseconds())) {
    throw std::overflow_error("time subtraction leads to int64_t overflow");
  }
  // 检查减法是否会导致 int64_t 下溢 (Check if the subtraction will cause an int64_t underflow)
  if (rclcpp::sub_will_underflow(rcl_time_.nanoseconds, rhs.nanoseconds())) {
    throw std::underflow_error("time subtraction leads to int64_t underflow");
  }

  // 返回相减后的新时间对象 (Return the new time object after subtraction)
  return Time(rcl_time_.nanoseconds - rhs.nanoseconds(), rcl_time_.clock_type);
}

/**
 * @brief 获取当前时间对象的纳秒值 (Gets the nanosecond value of the current time object)
 *
 * @return int64_t 返回当前时间对象的纳秒值 (Returns the nanosecond value of the current time
 * object)
 */
int64_t Time::nanoseconds() const { return rcl_time_.nanoseconds; }

/**
 * @brief 将时间转换为秒 (Converts the time to seconds)
 *
 * @return double 返回以秒为单位的时间 (Returns the time in seconds)
 */
double Time::seconds() const {
  // 将纳秒转换为秒并返回 (Convert nanoseconds to seconds and return)
  return std::chrono::duration<double>(std::chrono::nanoseconds(rcl_time_.nanoseconds)).count();
}

/**
 * @brief 获取时钟类型 (Get the clock type)
 *
 * @return rcl_clock_type_t 返回时钟类型 (Returns the clock type)
 */
rcl_clock_type_t Time::get_clock_type() const { return rcl_time_.clock_type; }

/**
 * @brief 重载加法运算符，实现时间和持续时间相加 (Overload the addition operator to add time and
 * duration)
 *
 * @param lhs 持续时间对象 (Duration object)
 * @param rhs 时间对象 (Time object)
 * @return Time 返回相加后的时间对象 (Returns the resulting time object after addition)
 * @throws std::overflow_error 当加法导致 int64_t 溢出时抛出异常 (Throws an exception when addition
 * leads to int64_t overflow)
 * @throws std::underflow_error 当加法导致 int64_t 下溢时抛出异常 (Throws an exception when addition
 * leads to int64_t underflow)
 */
Time operator+(const rclcpp::Duration& lhs, const rclcpp::Time& rhs) {
  if (rclcpp::add_will_overflow(rhs.nanoseconds(), lhs.nanoseconds())) {
    throw std::overflow_error("addition leads to int64_t overflow");
  }
  if (rclcpp::add_will_underflow(rhs.nanoseconds(), lhs.nanoseconds())) {
    throw std::underflow_error("addition leads to int64_t underflow");
  }
  return Time(lhs.nanoseconds() + rhs.nanoseconds(), rhs.get_clock_type());
}

/**
 * @brief 重载加法赋值运算符，实现时间和持续时间相加 (Overload the addition assignment operator to
 * add time and duration)
 *
 * @param rhs 持续时间对象 (Duration object)
 * @return Time& 返回相加后的时间对象引用 (Returns a reference to the resulting time object after
 * addition)
 * @throws std::overflow_error 当加法导致 int64_t 溢出时抛出异常 (Throws an exception when addition
 * leads to int64_t overflow)
 * @throws std::underflow_error 当加法导致 int64_t 下溢时抛出异常 (Throws an exception when addition
 * leads to int64_t underflow)
 */
Time& Time::operator+=(const rclcpp::Duration& rhs) {
  if (rclcpp::add_will_overflow(rhs.nanoseconds(), this->nanoseconds())) {
    throw std::overflow_error("addition leads to int64_t overflow");
  }
  if (rclcpp::add_will_underflow(rhs.nanoseconds(), this->nanoseconds())) {
    throw std::underflow_error("addition leads to int64_t underflow");
  }

  rcl_time_.nanoseconds += rhs.nanoseconds();

  return *this;
}

/**
 * @brief 重载减法赋值运算符，实现时间和持续时间相减 (Overload the subtraction assignment operator
 * to subtract duration from time)
 *
 * @param rhs 持续时间对象 (Duration object)
 * @return Time& 返回相减后的时间对象引用 (Returns a reference to the resulting time object after
 * subtraction)
 * @throws std::overflow_error 当减法导致 int64_t 溢出时抛出异常 (Throws an exception when
 * subtraction leads to int64_t overflow)
 * @throws std::underflow_error 当减法导致 int64_t 下溢时抛出异常 (Throws an exception when
 * subtraction leads to int64_t underflow)
 */
Time& Time::operator-=(const rclcpp::Duration& rhs) {
  if (rclcpp::sub_will_overflow(rcl_time_.nanoseconds, rhs.nanoseconds())) {
    throw std::overflow_error("time subtraction leads to int64_t overflow");
  }
  if (rclcpp::sub_will_underflow(rcl_time_.nanoseconds, rhs.nanoseconds())) {
    throw std::underflow_error("time subtraction leads to int64_t underflow");
  }

  rcl_time_.nanoseconds -= rhs.nanoseconds();

  return *this;
}

/**
 * @brief 获取最大时间值 (Get the maximum time value)
 *
 * @return Time 返回最大时间值 (Returns the maximum time value)
 */
Time Time::max() { return Time(std::numeric_limits<int32_t>::max(), 999999999); }

}  // namespace rclcpp

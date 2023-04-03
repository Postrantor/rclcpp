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

#ifndef RCLCPP__RATE_HPP_
#define RCLCPP__RATE_HPP_

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/macros.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/**
 * @class RateBase
 * @brief 基本速率类（Base class for rate）
 */
class RateBase {
public:
  // 禁止拷贝（Disable copy）
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(RateBase)

  // 虚析构函数（Virtual destructor）
  virtual ~RateBase() {}

  /**
   * @brief 睡眠一段时间，以保持给定的速率（Sleep for a duration to maintain the given rate）
   * @return 如果睡眠成功，则返回 true；否则返回 false（Return true if sleep was successful,
   * otherwise return false）
   */
  virtual bool sleep() = 0;

  /**
   * @brief 判断时钟是否稳定（Check if the clock is steady）
   * @return 如果时钟稳定，则返回 true；否则返回 false（Return true if the clock is steady,
   * otherwise return false）
   */
  virtual bool is_steady() const = 0;

  // 重置计时器（Reset the timer）
  virtual void reset() = 0;
};

// 使用 std::chrono 命名空间中的类型（Using types from std::chrono namespace）
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

/**
 * @class GenericRate
 * @tparam Clock 时钟类型（Clock type）
 * @brief 泛型速率类，继承自 RateBase 类（Generic rate class derived from RateBase）
 */
template <class Clock = std::chrono::high_resolution_clock>
class GenericRate : public RateBase {
public:
  // 智能指针定义（Smart pointer definitions）
  RCLCPP_SMART_PTR_DEFINITIONS(GenericRate)

  /**
   * @brief 通过速率构造对象（Construct object with rate）
   * @param rate 速率值（Rate value）
   */
  explicit GenericRate(double rate)
      : GenericRate<Clock>(duration_cast<nanoseconds>(duration<double>(1.0 / rate))) {}

  /**
   * @brief 通过周期构造对象（Construct object with period）
   * @param period 周期值（Period value）
   */
  explicit GenericRate(std::chrono::nanoseconds period)
      : period_(period), last_interval_(Clock::now()) {}

  // 实现 sleep() 函数（Implement sleep() function）
  virtual bool sleep() {
    // 进入睡眠时的时间（Time coming into sleep）
    auto now = Clock::now();
    // 下一个间隔的时间（Time of next interval）
    auto next_interval = last_interval_ + period_;
    // 检测时间倒流（Detect backwards time flow）
    if (now < last_interval_) {
      // 将下一个间隔设置为现在时间加上周期（Set the next_interval to now + period）
      next_interval = now + period_;
    }
    // 计算睡眠时间（Calculate the time to sleep）
    auto time_to_sleep = next_interval - now;
    // 更新间隔（Update the interval）
    last_interval_ += period_;
    // 如果睡眠时间为负数或零，则不睡眠（If the time_to_sleep is negative or zero, don't sleep）
    if (time_to_sleep <= std::chrono::seconds(0)) {
      // 如果错过了整个周期，则重置下一个间隔（If an entire cycle was missed then reset next
      // interval）
      if (now > next_interval + period_) {
        last_interval_ = now + period_;
      }
      // 无论如何都不睡眠，返回 false（Either way do not sleep and return false）
      return false;
    }
    // 睡眠（可能会被 ctrl-c 中断，可能不会睡满整个时间）（Sleep (will get interrupted by ctrl-c,
    // may not sleep full time)）
    rclcpp::sleep_for(time_to_sleep);
    return true;
  }

  // 实现 is_steady() 函数（Implement is_steady() function）
  virtual bool is_steady() const { return Clock::is_steady; }

  // 实现 reset() 函数（Implement reset() function）
  virtual void reset() { last_interval_ = Clock::now(); }

  /**
   * @brief 获取周期值（Get period value）
   * @return 周期值（Period value）
   */
  std::chrono::nanoseconds period() const { return period_; }

private:
  // 禁止拷贝（Disable copy）
  RCLCPP_DISABLE_COPY(GenericRate)

  // 周期变量（Period variable）
  std::chrono::nanoseconds period_;
  // 定义时钟持续时间类型（Define clock duration type）
  using ClockDurationNano = std::chrono::duration<typename Clock::rep, std::nano>;
  // 上一个间隔的时间点（Time point of the last interval）
  std::chrono::time_point<Clock, ClockDurationNano> last_interval_;
};

// 使用系统时钟定义速率类（Define Rate class using system clock）
using Rate = GenericRate<std::chrono::system_clock>;
// 使用稳定时钟定义墙壁速率类（Define WallRate class using steady clock）
using WallRate = GenericRate<std::chrono::steady_clock>;

}  // namespace rclcpp

#endif  // RCLCPP__RATE_HPP_

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

#ifndef RCLCPP__DETAIL__RMW_IMPLEMENTATION_SPECIFIC_PUBLISHER_PAYLOAD_HPP_
#define RCLCPP__DETAIL__RMW_IMPLEMENTATION_SPECIFIC_PUBLISHER_PAYLOAD_HPP_

#include "rcl/publisher.h"
#include "rclcpp/detail/rmw_implementation_specific_payload.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace detail {

/**
 * @class RCLCPP_PUBLIC RMWImplementationSpecificPublisherPayload
 * @brief 继承自 RMWImplementationSpecificPayload 的类，用于处理特定于 RMW 实现的发布者负载信息。 (A
 * class derived from RMWImplementationSpecificPayload for handling publisher payload information
 * specific to the RMW implementation.)
 */
class RCLCPP_PUBLIC RMWImplementationSpecificPublisherPayload
    : public RMWImplementationSpecificPayload {
public:
  /// 析构函数，使用默认实现。 (Destructor, using default implementation.)
  ~RMWImplementationSpecificPublisherPayload() override = default;

  /**
   * @brief 为派生类提供在 rcl 选项中注入信息的机会。 (Opportunity for a derived class to inject
   * information into the rcl options.)
   *
   * 在 rclcpp 准备好 rcl_publisher_options_t 之后，但在调用 rcl_publisher_init() 之前调用此方法。
   * (This is called after the rcl_publisher_options_t has been prepared by rclcpp, but before
   * rcl_publisher_init() is called.)
   *
   * 传递的选项是将传递给 rcl_publisher_init() 的 rcl_publisher_options_t 中的 rmw_publisher_options
   * 字段。 (The passed option is the rmw_publisher_options field of the rcl_publisher_options_t
   * that will be passed to rcl_publisher_init().)
   *
   * 默认情况下，选项保持不变。 (By default the options are unmodified.)
   *
   * @param[out] rmw_publisher_options 要修改的 rmw_publisher_options_t 类型引用。 (A reference to
   * the rmw_publisher_options_t to be modified.)
   */
  virtual void modify_rmw_publisher_options(rmw_publisher_options_t& rmw_publisher_options) const;
};

}  // namespace detail
}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__RMW_IMPLEMENTATION_SPECIFIC_PUBLISHER_PAYLOAD_HPP_

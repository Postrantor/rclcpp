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

#include <rclcpp/detail/rmw_implementation_specific_publisher_payload.hpp>

#include "rcl/publisher.h"

namespace rclcpp {
namespace detail {

/**
 * @brief 修改 RMW 发布者选项 (Modify the RMW publisher options)
 *
 * @param rmw_publisher_options RMW发布者选项的引用 (Reference to the RMW publisher options)
 */
void RMWImplementationSpecificPublisherPayload::modify_rmw_publisher_options(
    rmw_publisher_options_t& rmw_publisher_options) const {
  // 默认情况下，不改变 rmw 发布者选项 (By default, do not mutate the rmw publisher options)
  (void)rmw_publisher_options;  // 防止未使用参数的编译器警告 (Prevent unused parameter compiler
                                // warning)
}

}  // namespace detail
}  // namespace rclcpp

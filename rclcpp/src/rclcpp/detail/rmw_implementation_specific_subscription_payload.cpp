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

#include <rclcpp/detail/rmw_implementation_specific_subscription_payload.hpp>

#include "rcl/subscription.h"

namespace rclcpp {
namespace detail {

/**
 * @brief 修改 RMW 订阅选项 (Modify RMW subscription options)
 *
 * @param rmw_subscription_options RMW 订阅选项的引用 (Reference to RMW subscription options)
 */
void RMWImplementationSpecificSubscriptionPayload::modify_rmw_subscription_options(
    rmw_subscription_options_t& rmw_subscription_options) const {
  // 默认情况下，不改变 RMW 订阅选项
  // (By default, do not mutate the RMW subscription options)
  (void)rmw_subscription_options;
}

}  // namespace detail
}  // namespace rclcpp

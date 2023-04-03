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

#ifndef RCLCPP__WAIT_RESULT_KIND_HPP_
#define RCLCPP__WAIT_RESULT_KIND_HPP_

#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/// 代表等待一个 wait set 的各种结果类型。
/// Represents the various kinds of results from waiting on a wait set.
enum RCLCPP_PUBLIC WaitResultKind {
  Ready,  //<! 当 wait set 中的某个东西准备好时使用的类型。 (Kind used when something in the wait
          //set was ready.)
  Timeout,  //<! 当等待导致超时时使用的类型。 (Kind used when the wait resulted in a timeout.)
  Empty,  //<! 当尝试在空的 wait set 上等待时使用的类型。 (Kind used when trying to wait on an empty
          //wait set.)
};

}  // namespace rclcpp

#endif  // RCLCPP__WAIT_RESULT_KIND_HPP_

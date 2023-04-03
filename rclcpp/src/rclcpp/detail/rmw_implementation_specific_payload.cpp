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

#include <rclcpp/detail/rmw_implementation_specific_payload.hpp>

namespace rclcpp {
namespace detail {

/**
 * @brief 检查 RMWImplementationSpecificPayload 是否已经被定制 (Check if the
 * RMWImplementationSpecificPayload has been customized)
 *
 * @return 如果已定制，返回 true；否则返回 false (Return true if it's customized, otherwise return
 * false)
 */
bool RMWImplementationSpecificPayload::has_been_customized() const {
  // 调用 get_implementation_identifier() 函数，检查返回值是否为 nullptr
  // (Call the get_implementation_identifier() function and check if the returned value is nullptr)
  return nullptr != this->get_implementation_identifier();
}

/**
 * @brief 获取实现标识符 (Get the implementation identifier)
 *
 * @return 返回实现标识符，这里返回 nullptr 表示没有实现 (Return the implementation identifier, here
 * returning nullptr means no implementation)
 */
const char* RMWImplementationSpecificPayload::get_implementation_identifier() const {
  // 返回 nullptr，表示没有实现标识符
  // (Return nullptr, indicating that there is no implementation identifier)
  return nullptr;
}

}  // namespace detail
}  // namespace rclcpp

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

#ifndef RCLCPP__PUBLISHER_OPTIONS_HPP_
#define RCLCPP__PUBLISHER_OPTIONS_HPP_

#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "rcl/publisher.h"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/detail/rmw_implementation_specific_publisher_payload.hpp"
#include "rclcpp/intra_process_setting.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/qos_overriding_options.hpp"

namespace rclcpp {

class CallbackGroup;

/// 非模板部分的 PublisherOptionsWithAllocator<Allocator>。
struct PublisherOptionsBase {
  /// 设置显式设置内部进程通信。
  IntraProcessSetting use_intra_process_comm = IntraProcessSetting::NodeDefault;
  /// 与发布者相关的各种事件的回调。
  PublisherEventCallbacks event_callbacks;
  /// 当用户在 event_callbacks 中不提供任何回调时，是否使用默认回调。
  bool use_default_callbacks = true;
  /// 要求中间件生成独特的网络流端点。
  /// 默认禁用。
  rmw_unique_network_flow_endpoints_requirement_t require_unique_network_flow_endpoints =
      RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_NOT_REQUIRED;
  /// 发布者的 waitable 项应放置在其中的回调组。
  std::shared_ptr<rclcpp::CallbackGroup> callback_group;
  /// 在创建发布者期间使用的可选 RMW 实现特定有效载荷。
  std::shared_ptr<rclcpp::detail::RMWImplementationSpecificPublisherPayload>
      rmw_implementation_payload = nullptr;
  QosOverridingOptions qos_overriding_options;
};

/// 包含发布者可选配置的结构。
template <typename Allocator>
struct PublisherOptionsWithAllocator : public PublisherOptionsBase {
  static_assert(
      std::is_void_v<typename std::allocator_traits<Allocator>::value_type>,
      "Publisher allocator value type must be void");

  /// 可选的自定义分配器。
  std::shared_ptr<Allocator> allocator = nullptr;

  PublisherOptionsWithAllocator() {}

  /// 使用基类作为输入的构造函数。
  explicit PublisherOptionsWithAllocator(const PublisherOptionsBase& publisher_options_base)
      : PublisherOptionsBase(publisher_options_base) {}

  /// 将此类和 rclcpp::QoS 转换为 rcl_publisher_options_t。
  template <typename MessageT>
  rcl_publisher_options_t to_rcl_publisher_options(const rclcpp::QoS& qos) const {
    rcl_publisher_options_t result = rcl_publisher_get_default_options();
    result.allocator = this->get_rcl_allocator();
    result.qos = qos.get_rmw_qos_profile();
    result.rmw_publisher_options.require_unique_network_flow_endpoints =
        this->require_unique_network_flow_endpoints;

    // 如果需要，将有效负载应用于 rcl_publisher_options。
    if (rmw_implementation_payload && rmw_implementation_payload->has_been_customized()) {
      rmw_implementation_payload->modify_rmw_publisher_options(result.rmw_publisher_options);
    }

    return result;
  }

  /// 获取分配器，如果需要创建一个。
  std::shared_ptr<Allocator> get_allocator() const {
    if (!this->allocator) {
      if (!allocator_storage_) {
        allocator_storage_ = std::make_shared<Allocator>();
      }
      return allocator_storage_;
    }
    return this->allocator;
  }

private:
  using PlainAllocator = typename std::allocator_traits<Allocator>::template rebind_alloc<char>;

  rcl_allocator_t get_rcl_allocator() const {
    if (!plain_allocator_storage_) {
      plain_allocator_storage_ = std::make_shared<PlainAllocator>(*this->get_allocator());
    }
    return rclcpp::allocator::get_rcl_allocator<char>(*plain_allocator_storage_);
  }

  // 这是一个临时解决方案，确保 get_allocator()
  // 始终返回相同分配器的副本。
  mutable std::shared_ptr<Allocator> allocator_storage_;

  // 这是一个临时解决方案，以保持支持
  // 在 rcl_publisher_options_t 中返回的 rcl 分配器的纯分配器活跃。
  mutable std::shared_ptr<PlainAllocator> plain_allocator_storage_;
};

using PublisherOptions = PublisherOptionsWithAllocator<std::allocator<void>>;

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_OPTIONS_HPP_

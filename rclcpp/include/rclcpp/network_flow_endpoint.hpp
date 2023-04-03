// Copyright 2020 Ericsson AB
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

#ifndef RCLCPP__NETWORK_FLOW_ENDPOINT_HPP_
#define RCLCPP__NETWORK_FLOW_ENDPOINT_HPP_

#include <cstdint>
#include <iostream>
#include <string>

#include "rcl/network_flow_endpoints.h"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/// Forward declaration
class NetworkFlowEndpoint;

/// \brief 检查两个 NetworkFlowEndpoint 实例是否相等
/// \param left 左侧 NetworkFlowEndpoint 实例的引用
/// \param right 右侧 NetworkFlowEndpoint 实例的引用
/// \return 如果相等返回 true，否则返回 false
///
/// Check if two NetworkFlowEndpoint instances are equal
/// \param left Reference to the left NetworkFlowEndpoint instance
/// \param right Reference to the right NetworkFlowEndpoint instance
/// \return True if equal, otherwise false
RCLCPP_PUBLIC
bool operator==(const NetworkFlowEndpoint& left, const NetworkFlowEndpoint& right);

/// \brief 检查两个 NetworkFlowEndpoint 实例是否不相等
/// \param left 左侧 NetworkFlowEndpoint 实例的引用
/// \param right 右侧 NetworkFlowEndpoint 实例的引用
/// \return 如果不相等返回 true，否则返回 false
///
/// Check if two NetworkFlowEndpoint instances are not equal
/// \param left Reference to the left NetworkFlowEndpoint instance
/// \param right Reference to the right NetworkFlowEndpoint instance
/// \return True if not equal, otherwise false
RCLCPP_PUBLIC
bool operator!=(const NetworkFlowEndpoint& left, const NetworkFlowEndpoint& right);

/// \brief NetworkFlowEndpoint 的流输出辅助函数
/// \param os 输出流的引用
/// \param network_flow_endpoint NetworkFlowEndpoint 实例的引用
/// \return 返回输出流的引用
///
/// Streaming helper for NetworkFlowEndpoint
/// \param os Reference to the output stream
/// \param network_flow_endpoint Reference to the NetworkFlowEndpoint instance
/// \return Reference to the output stream
RCLCPP_PUBLIC
std::ostream& operator<<(std::ostream& os, const NetworkFlowEndpoint& network_flow_endpoint);

/**
 * \brief 基于 RMW 层对应定义的网络流端点描述类
 *
 * Class describes a network flow endpoint based on the counterpart definition
 * in the RMW layer.
 */
class NetworkFlowEndpoint {
public:
  /// \brief 从 rcl_network_flow_endpoint_t 构造
  /// \param network_flow_endpoint rcl_network_flow_endpoint_t 结构体
  ///
  /// Construct from rcl_network_flow_endpoint_t
  /// \param network_flow_endpoint The rcl_network_flow_endpoint_t structure
  RCLCPP_PUBLIC
  explicit NetworkFlowEndpoint(const rcl_network_flow_endpoint_t& network_flow_endpoint)
      : transport_protocol_(rcl_network_flow_endpoint_get_transport_protocol_string(
            network_flow_endpoint.transport_protocol)),
        internet_protocol_(rcl_network_flow_endpoint_get_internet_protocol_string(
            network_flow_endpoint.internet_protocol)),
        transport_port_(network_flow_endpoint.transport_port),
        flow_label_(network_flow_endpoint.flow_label),
        dscp_(network_flow_endpoint.dscp),
        internet_address_(network_flow_endpoint.internet_address) {}

  /// \brief 获取传输协议
  /// \return 传输协议字符串的引用
  ///
  /// Get transport protocol
  /// \return Reference to the transport protocol string
  RCLCPP_PUBLIC
  const std::string& transport_protocol() const;

  /// \brief 获取互联网协议
  /// \return 互联网协议字符串的引用
  ///
  /// Get internet protocol
  /// \return Reference to the internet protocol string
  RCLCPP_PUBLIC
  const std::string& internet_protocol() const;

  /// \brief 获取传输端口
  /// \return 传输端口的值
  ///
  /// Get transport port
  /// \return The value of the transport port
  RCLCPP_PUBLIC
  uint16_t transport_port() const;

  /// \brief 获取流标签
  /// \return 流标签的值
  ///
  /// Get flow label
  /// \return The value of the flow label
  RCLCPP_PUBLIC
  uint32_t flow_label() const;

  /// \brief 获取 DSCP
  /// \return DSCP 的值
  ///
  /// Get DSCP
  /// \return The value of the DSCP
  RCLCPP_PUBLIC
  uint8_t dscp() const;

  /// \brief 获取互联网地址
  /// \return 互联网地址字符串的引用
  ///
  /// Get internet address
  /// \return Reference to the internet address string
  RCLCPP_PUBLIC
  const std::string& internet_address() const;

  /// 比较两个 NetworkFlowEndpoint 实例
  friend bool rclcpp::operator==(const NetworkFlowEndpoint& left, const NetworkFlowEndpoint& right);
  friend bool rclcpp::operator!=(const NetworkFlowEndpoint& left, const NetworkFlowEndpoint& right);

  /// 流输出辅助函数
  friend std::ostream& rclcpp::operator<<(
      std::ostream& os, const NetworkFlowEndpoint& network_flow_endpoint);

private:
  std::string transport_protocol_;  ///< 传输协议字符串 (Transport protocol string)
  std::string internet_protocol_;   ///< 互联网协议字符串 (Internet protocol string)
  uint16_t transport_port_;         ///< 传输端口值 (Transport port value)
  uint32_t flow_label_;             ///< 流标签值 (Flow label value)
  uint8_t dscp_;                    ///< DSCP 值 (DSCP value)
  std::string internet_address_;    ///< 互联网地址字符串 (Internet address string)
};

}  // namespace rclcpp

#endif  // RCLCPP__NETWORK_FLOW_ENDPOINT_HPP_

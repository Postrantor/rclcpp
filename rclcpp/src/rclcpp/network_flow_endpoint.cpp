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

#include "rclcpp/network_flow_endpoint.hpp"

#include <string>

namespace rclcpp {

/**
 * @brief 获取传输协议 (Get the transport protocol)
 *
 * @return 传输协议字符串的引用 (A reference to the transport protocol string)
 */
const std::string& NetworkFlowEndpoint::transport_protocol() const {
  return transport_protocol_;  // 返回传输协议字符串的引用 (Return a reference to the transport
                               // protocol string)
}

/**
 * @brief 获取网络协议 (Get the internet protocol)
 *
 * @return 网络协议字符串的引用 (A reference to the internet protocol string)
 */
const std::string& NetworkFlowEndpoint::internet_protocol() const {
  return internet_protocol_;  // 返回网络协议字符串的引用 (Return a reference to the internet
                              // protocol string)
}

/**
 * @brief 获取传输端口 (Get the transport port)
 *
 * @return 传输端口的值 (The value of the transport port)
 */
uint16_t NetworkFlowEndpoint::transport_port() const {
  return transport_port_;  // 返回传输端口的值 (Return the value of the transport port)
}

/**
 * @brief 获取流标签 (Get the flow label)
 *
 * @return 流标签的值 (The value of the flow label)
 */
uint32_t NetworkFlowEndpoint::flow_label() const {
  return flow_label_;  // 返回流标签的值 (Return the value of the flow label)
}

/**
 * @brief 获取 DSCP (Differentiated Services Code Point) 值 (Get the DSCP value)
 *
 * @return DSCP 的值 (The value of DSCP)
 */
uint8_t NetworkFlowEndpoint::dscp() const {
  return dscp_;  // 返回 DSCP 的值 (Return the value of DSCP)
}

/**
 * @brief 获取网络地址 (Get the internet address)
 *
 * @return 网络地址字符串的引用 (A reference to the internet address string)
 */
const std::string& NetworkFlowEndpoint::internet_address() const {
  return internet_address_;  // 返回网络地址字符串的引用 (Return a reference to the internet address
                             // string)
}

/**
 * @brief 判断两个 NetworkFlowEndpoint 对象是否相等 (Determine if two NetworkFlowEndpoint objects
 * are equal)
 *
 * @param left 左侧 NetworkFlowEndpoint 对象 (The left NetworkFlowEndpoint object)
 * @param right 右侧 NetworkFlowEndpoint 对象 (The right NetworkFlowEndpoint object)
 * @return 如果相等返回 true，否则返回 false (Return true if equal, otherwise return false)
 */
bool operator==(const NetworkFlowEndpoint& left, const NetworkFlowEndpoint& right) {
  // 比较两个对象的所有属性 (Compare all attributes of the two objects)
  return left.transport_protocol_ == right.transport_protocol_ &&
         left.internet_protocol_ == right.internet_protocol_ &&
         left.transport_port_ == right.transport_port_ && left.flow_label_ == right.flow_label_ &&
         left.dscp_ == right.dscp_ && left.internet_address_ == right.internet_address_;
}

/**
 * @brief 判断两个 NetworkFlowEndpoint 对象是否不相等 (Determine if two NetworkFlowEndpoint objects
 * are not equal)
 *
 * @param left 左侧 NetworkFlowEndpoint 对象 (The left NetworkFlowEndpoint object)
 * @param right 右侧 NetworkFlowEndpoint 对象 (The right NetworkFlowEndpoint object)
 * @return 如果不相等返回 true，否则返回 false (Return true if not equal, otherwise return false)
 */
bool operator!=(const NetworkFlowEndpoint& left, const NetworkFlowEndpoint& right) {
  return !(left == right);  // 使用相等比较的结果取反 (Negate the result of the equality comparison)
}

/**
 * @brief 输出 NetworkFlowEndpoint 对象到输出流 (Output the NetworkFlowEndpoint object to the output
 * stream)
 *
 * @param os 输出流对象 (The output stream object)
 * @param network_flow_endpoint 要输出的 NetworkFlowEndpoint 对象 (The NetworkFlowEndpoint object to
 * be output)
 * @return 更新后的输出流对象 (The updated output stream object)
 */
std::ostream& operator<<(std::ostream& os, const NetworkFlowEndpoint& network_flow_endpoint) {
  // 以类似 JSON 格式输出到输出流 (Stream out in JSON-like format)
  os << "{"
     << "\"transportProtocol\": \"" << network_flow_endpoint.transport_protocol_ << "\", "
     << "\"internetProtocol\": \"" << network_flow_endpoint.internet_protocol_ << "\", "
     << "\"transportPort\": \"" << network_flow_endpoint.transport_port_ << "\", "
     << "\"flowLabel\": \"" << std::to_string(network_flow_endpoint.flow_label_) << "\", "
     << "\"dscp\": \"" << std::to_string(network_flow_endpoint.dscp_) << "\", "
     << "\"internetAddress\": \"" << network_flow_endpoint.internet_address_ << "\""
     << "}";
  return os;
}

}  // namespace rclcpp

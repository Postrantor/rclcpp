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

#include "rclcpp/node_interfaces/node_logging.hpp"

using rclcpp::node_interfaces::NodeLogging;

/**
 * @brief 构造函数，用于初始化 NodeLogging 类的对象 (Constructor for initializing a NodeLogging
 * object)
 *
 * @param[in] node_base 指向 rclcpp::node_interfaces::NodeBaseInterface
 * 类型的指针，用于获取节点基本信息 (A pointer to an rclcpp::node_interfaces::NodeBaseInterface
 * object, used for obtaining basic node information)
 */
NodeLogging::NodeLogging(rclcpp::node_interfaces::NodeBaseInterface* node_base)
    : node_base_(
          node_base)  // 初始化 node_base_ 成员变量 (Initialize the node_base_ member variable)
{
  // 获取 logger 对象，并将其赋值给 logger_ 成员变量 (Get the logger object and assign it to the
  // logger_ member variable)
  logger_ = rclcpp::get_logger(NodeLogging::get_logger_name());
}

/**
 * @brief 析构函数，用于销毁 NodeLogging 类的对象 (Destructor for destroying a NodeLogging object)
 */
NodeLogging::~NodeLogging() {}

/**
 * @brief 获取 logger 对象的函数 (Function to get the logger object)
 *
 * @return 返回 logger 对象 (Return the logger object)
 */
rclcpp::Logger NodeLogging::get_logger() const { return logger_; }

/**
 * @brief 获取 logger 名称的函数 (Function to get the logger name)
 *
 * @return 返回一个指向 char 类型的常量指针，该指针指向 logger 的名称 (Returns a constant pointer to
 * a char type, pointing to the logger's name)
 */
const char* NodeLogging::get_logger_name() const {
  // 调用 rcl_node_get_logger_name 函数，传入 node_base_->get_rcl_node_handle() 作为参数 (Call the
  // rcl_node_get_logger_name function, passing node_base_->get_rcl_node_handle() as the argument)
  return rcl_node_get_logger_name(node_base_->get_rcl_node_handle());
}

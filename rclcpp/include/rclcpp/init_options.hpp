// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__INIT_OPTIONS_HPP_
#define RCLCPP__INIT_OPTIONS_HPP_

#include <memory>
#include <mutex>

#include "rcl/init_options.h"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/// \class InitOptions
/// \brief 初始化rclcpp的选项封装 (Encapsulation of options for initializing rclcpp).
class InitOptions {
public:
  /// 如果为true，则在安装信号处理程序时，上下文将在SIGINT上关闭 (If true, the context will be
  /// shutdown on SIGINT by the signal handler, if it was installed).
  bool shutdown_on_signal = true;

  /// \brief 构造函数 (Constructor)
  /**
   * 允许您指定在init选项中使用的分配器 (It allows you to specify the allocator used within the init
   * options). \param[in] 分配器用于在init选项中分配内存 (allocator used to allocate memory within
   * the init options) \throws 可以抛出rclcpp::exceptions::throw_from_rcl_error的任何内容 (anything
   * rclcpp::exceptions::throw_from_rcl_error can throw).
   */
  RCLCPP_PUBLIC
  explicit InitOptions(rcl_allocator_t allocator = rcl_get_default_allocator());

  /// \brief 使用现有的init_options进行初始化的构造函数 (Constructor which is initialized by an
  /// existing init_options).
  /**
   * 由现有的init_options初始化 (Initialized by an existing init_options).
   * \param[in] init_options 用于初始化的rcl_init_options_t (rcl_init_options_t to initialize)
   * \throws 可以抛出rclcpp::exceptions::throw_from_rcl_error的任何内容 (anything
   * rclcpp::exceptions::throw_from_rcl_error can throw).
   */
  RCLCPP_PUBLIC
  explicit InitOptions(const rcl_init_options_t& init_options);

  /// \brief 拷贝构造函数 (Copy constructor).
  RCLCPP_PUBLIC
  InitOptions(const InitOptions& other);

  /// \brief 返回在调用`rclcpp::Context::init`时是否应初始化日志记录 (Return `true` if logging
  /// should be initialized when `rclcpp::Context::init` is called).
  RCLCPP_PUBLIC
  bool auto_initialize_logging() const;

  /// \brief 设置指示是否应初始化日志记录的标志 (Set flag indicating if logging should be
  /// initialized or not).
  RCLCPP_PUBLIC
  InitOptions& auto_initialize_logging(bool initialize_logging);

  /// \brief 赋值操作符 (Assignment operator).
  RCLCPP_PUBLIC
  InitOptions& operator=(const InitOptions& other);

  /// \brief 析构函数 (Destructor).
  RCLCPP_PUBLIC
  virtual ~InitOptions();

  /// \brief 返回rcl初始化选项 (Return the rcl init options).
  /**
   * \return rcl初始化选项 (the rcl init options).
   * \throws 可以抛出rclcpp::exceptions::throw_from_rcl_error的任何内容 (anything
   * rclcpp::exceptions::throw_from_rcl_error can throw).
   */
  RCLCPP_PUBLIC
  const rcl_init_options_t* get_rcl_init_options() const;

  /// \brief 检索默认域ID并设置 (Retrieve default domain id and set).
  RCLCPP_PUBLIC
  void use_default_domain_id();

  /// \brief 设置域ID (Set the domain id).
  RCLCPP_PUBLIC
  void set_domain_id(size_t domain_id);

  /// \brief 返回域ID (Return domain id).
  RCLCPP_PUBLIC
  size_t get_domain_id() const;

protected:
  /// \brief 完成初始化选项 (Finalize init options).
  void finalize_init_options();

private:
  /// \brief 实现完成初始化选项 (Implement finalize init options).
  void finalize_init_options_impl();

  /// 初始化选项互斥锁 (Init options mutex).
  mutable std::mutex init_options_mutex_;
  /// 初始化选项指针 (Init options pointer).
  std::unique_ptr<rcl_init_options_t> init_options_;
  /// 初始化日志记录标志 (Initialize logging flag).
  bool initialize_logging_{true};
};

}  // namespace rclcpp

#endif  // RCLCPP__INIT_OPTIONS_HPP_

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

#include "rclcpp/init_options.hpp"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

namespace rclcpp {

/**
 * @brief 构造函数，接受一个 rcl_allocator_t 类型的参数 (Constructor that accepts an
 * `rcl_allocator_t` parameter)
 *
 * @param allocator 分配器对象 (Allocator object)
 */
InitOptions::InitOptions(rcl_allocator_t allocator) : init_options_(new rcl_init_options_t) {
  // 初始化 init_options_ 为零值 (Initialize `init_options_` to zero)
  *init_options_ = rcl_get_zero_initialized_init_options();

  // 使用给定的分配器初始化 init_options_ (Initialize `init_options_` with the given allocator)
  rcl_ret_t ret = rcl_init_options_init(init_options_.get(), allocator);

  // 如果初始化失败，则抛出异常 (If initialization fails, throw an exception)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to initialize rcl init options");
  }
}

/**
 * @brief 构造函数，接受一个 rcl_init_options_t 类型的参数 (Constructor that accepts an
 * `rcl_init_options_t` parameter)
 *
 * @param init_options 初始化选项对象 (Initialization options object)
 */
InitOptions::InitOptions(const rcl_init_options_t& init_options)
    : init_options_(new rcl_init_options_t) {
  // 初始化 init_options_ 为零值 (Initialize `init_options_` to zero)
  *init_options_ = rcl_get_zero_initialized_init_options();

  // 复制给定的 init_options 到 init_options_ (Copy the given `init_options` to `init_options_`)
  rcl_ret_t ret = rcl_init_options_copy(&init_options, init_options_.get());

  // 如果复制失败，则抛出异常 (If copying fails, throw an exception)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to copy rcl init options");
  }
}

/**
 * @brief 拷贝构造函数，接受一个 InitOptions 类型的参数 (Copy constructor that accepts an
 * `InitOptions` parameter)
 *
 * @param other 另一个 InitOptions 对象 (Another `InitOptions` object)
 */
InitOptions::InitOptions(const InitOptions& other) : InitOptions(*other.get_rcl_init_options()) {
  // 复制其他对象的 shutdown_on_signal 和 initialize_logging_ 属性 (Copy the `shutdown_on_signal`
  // and `initialize_logging_` properties from the other object)
  shutdown_on_signal = other.shutdown_on_signal;
  initialize_logging_ = other.initialize_logging_;
}

/**
 * @brief 返回是否自动初始化日志功能 (Returns whether logging is auto-initialized)
 */
bool InitOptions::auto_initialize_logging() const { return initialize_logging_; }

/**
 * @brief 设置是否自动初始化日志功能，并返回当前对象的引用 (Sets whether logging is auto-initialized
 * and returns a reference to the current object)
 *
 * @param initialize_logging 是否自动初始化日志 (Whether to auto-initialize logging)
 * @return InitOptions& 当前对象的引用 (Reference to the current object)
 */
InitOptions& InitOptions::auto_initialize_logging(bool initialize_logging) {
  initialize_logging_ = initialize_logging;
  return *this;
}

/**
 * @brief 赋值运算符重载，用于复制另一个 InitOptions 对象 (Assignment operator overload for copying
 * another `InitOptions` object)
 *
 * @param other 另一个 InitOptions 对象 (Another `InitOptions` object)
 * @return InitOptions& 当前对象的引用 (Reference to the current object)
 */
InitOptions& InitOptions::operator=(const InitOptions& other) {
  if (this != &other) {
    std::lock_guard<std::mutex> init_options_lock(init_options_mutex_);
    this->finalize_init_options_impl();
    rcl_ret_t ret = rcl_init_options_copy(other.get_rcl_init_options(), init_options_.get());
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to copy rcl init options");
    }
    this->shutdown_on_signal = other.shutdown_on_signal;
    this->initialize_logging_ = other.initialize_logging_;
  }
  return *this;
}

/**
 * @brief 析构函数，释放初始化选项资源 (Destructor that finalizes initialization options)
 */
InitOptions::~InitOptions() { this->finalize_init_options(); }

/**
 * @brief 完成初始化选项 (Finalize initialization options)
 */
void InitOptions::finalize_init_options() {
  std::lock_guard<std::mutex> init_options_lock(init_options_mutex_);
  this->finalize_init_options_impl();
}

/**
 * @brief 实现完成初始化选项的内部方法 (Internal method for implementing finalization of
 * initialization options)
 */
void InitOptions::finalize_init_options_impl() {
  if (init_options_) {
    rcl_ret_t ret = rcl_init_options_fini(init_options_.get());
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"), "failed to finalize rcl init options: %s",
          rcl_get_error_string().str);
      rcl_reset_error();
    }
    *init_options_ = rcl_get_zero_initialized_init_options();
  }
}

/**
 * @brief 获取 rcl_init_options_t 指针 (Get the `rcl_init_options_t` pointer)
 *
 * @return const rcl_init_options_t* 指向 rcl_init_options_t 的指针 (Pointer to
 * `rcl_init_options_t`)
 */
const rcl_init_options_t* InitOptions::get_rcl_init_options() const { return init_options_.get(); }

/**
 * @brief 使用默认的 domain id (Use the default domain ID)
 */
void InitOptions::use_default_domain_id() {
  size_t domain_id = RCL_DEFAULT_DOMAIN_ID;
  rcl_ret_t ret = rcl_get_default_domain_id(&domain_id);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to get default domain id");
  }
  set_domain_id(domain_id);
}

/**
 * @brief 设置 domain id (Set the domain ID)
 *
 * @param domain_id 要设置的 domain id 值 (The domain ID value to set)
 */
void InitOptions::set_domain_id(size_t domain_id) {
  // 加锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> init_options_lock(init_options_mutex_);

  // 使用 rcl 库函数设置 domain id (Use rcl library function to set domain ID)
  rcl_ret_t ret = rcl_init_options_set_domain_id(init_options_.get(), domain_id);

  // 检查返回值，如果不是 RCL_RET_OK，则抛出异常 (Check return value, if not RCL_RET_OK, throw
  // exception)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to set domain id to rcl init options");
  }
}

/**
 * @brief 获取当前的 domain id (Get the current domain ID)
 *
 * @return size_t 当前的 domain id 值 (The current domain ID value)
 */
size_t InitOptions::get_domain_id() const {
  // 加锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::mutex> init_options_lock(init_options_mutex_);

  // 定义一个变量来存储 domain id (Define a variable to store the domain ID)
  size_t domain_id;

  // 使用 rcl 库函数获取 domain id (Use rcl library function to get domain ID)
  rcl_ret_t ret = rcl_init_options_get_domain_id(init_options_.get(), &domain_id);

  // 检查返回值，如果不是 RCL_RET_OK，则抛出异常 (Check return value, if not RCL_RET_OK, throw
  // exception)
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to get domain id from rcl init options");
  }

  // 返回 domain id 值 (Return the domain ID value)
  return domain_id;
}

}  // namespace rclcpp

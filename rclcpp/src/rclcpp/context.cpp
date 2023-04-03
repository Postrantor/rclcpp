// Copyright 2015-2020 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/context.hpp"

#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "./logging_mutex.hpp"
#include "rcl/init.h"
#include "rcl/logging.h"
#include "rclcpp/detail/utilities.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/error_handling.h"
#include "rcutils/macros.h"

using rclcpp::Context;

namespace rclcpp {
/// @brief 类，用于管理所有创建的上下文的弱指针向量（Class to manage vector of weak pointers to all
/// created contexts）
class WeakContextsWrapper {
public:
  /// @brief 定义智能指针类型（Define smart pointer types）
  RCLCPP_SMART_PTR_DEFINITIONS(WeakContextsWrapper)

  /// @brief 添加上下文到弱指针向量中（Add context to the vector of weak pointers）
  ///
  /// @param[in] context 要添加的共享指针上下文（The shared pointer context to add）
  void add_context(const Context::SharedPtr& context) {
    // 加锁以确保线程安全（Lock to ensure thread safety）
    std::lock_guard<std::mutex> guard(mutex_);
    // 将上下文添加到弱指针向量中（Add the context to the vector of weak pointers）
    weak_contexts_.push_back(context);
  }

  /// @brief 从弱指针向量中删除上下文（Remove context from the vector of weak pointers）
  ///
  /// @param[in] context 要删除的上下文指针（The pointer to the context to remove）
  void remove_context(const Context* context) {
    // 加锁以确保线程安全（Lock to ensure thread safety）
    std::lock_guard<std::mutex> guard(mutex_);
    // 删除与给定上下文匹配的弱指针（Remove the weak pointers that match the given context）
    weak_contexts_.erase(
        std::remove_if(
            weak_contexts_.begin(), weak_contexts_.end(),
            [context](const Context::WeakPtr weak_context) {
              auto locked_context = weak_context.lock();
              if (!locked_context) {
                // 删除过期的上下文（Remove expired contexts）
                return true;
              }
              return locked_context.get() == context;
            }),
        weak_contexts_.end());
  }

  /// @brief 获取所有有效上下文的共享指针向量（Get vector of shared pointers to all valid contexts）
  ///
  /// @return 共享指针向量（Vector of shared pointers）
  std::vector<Context::SharedPtr> get_contexts() {
    // 加锁以确保线程安全（Lock to ensure thread safety）
    std::lock_guard<std::mutex> lock(mutex_);
    // 创建一个共享指针向量，用于存储有效上下文（Create a vector of shared pointers to store valid
    // contexts）
    std::vector<Context::SharedPtr> shared_contexts;
    for (auto it = weak_contexts_.begin(); it != weak_contexts_.end(); /* noop */) {
      // 尝试将弱指针升级为共享指针（Attempt to upgrade the weak pointer to a shared pointer）
      auto context_ptr = it->lock();
      if (!context_ptr) {
        // 删除无效的弱指针（Remove invalid weak pointers）
        it = weak_contexts_.erase(it);
      } else {
        // 移动到下一个元素（Move to the next element）
        ++it;
        // 将有效的共享指针添加到结果向量中（Add the valid shared pointer to the result vector）
        shared_contexts.push_back(context_ptr);
      }
    }
    // 返回有效上下文的共享指针向量（Return the vector of shared pointers to valid contexts）
    return shared_contexts;
  }

private:
  /// @brief 存储所有创建的上下文的弱指针向量（Vector of weak pointers to all created contexts）
  std::vector<std::weak_ptr<rclcpp::Context>> weak_contexts_;
  /// @brief 互斥锁，用于确保线程安全（Mutex for ensuring thread safety）
  std::mutex mutex_;
};
}  // namespace rclcpp

/**
 * @file
 * @brief This file contains the implementation of rclcpp logging functions.
 */

#include "rclcpp/contexts/weak_contexts_wrapper.hpp"
#include "rclcpp/logging.hpp"

using rclcpp::WeakContextsWrapper;

/// 全局的所有上下文的弱指针向量 (Global vector of weak pointers to all contexts)
static WeakContextsWrapper::SharedPtr get_weak_contexts() {
  // 静态共享指针，用于存储所有上下文的弱指针 (Static shared pointer for storing weak pointers to
  // all contexts)
  static WeakContextsWrapper::SharedPtr weak_contexts = WeakContextsWrapper::make_shared();

  // 如果共享指针无效，则抛出运行时错误 (If the shared pointer is not valid, throw a runtime error)
  if (!weak_contexts) {
    throw std::runtime_error("weak contexts vector is not valid");
  }

  // 返回包含弱指针的共享指针 (Return the shared pointer containing weak pointers)
  return weak_contexts;
}

/// 初始化日志系统的上下文计数 (Count of contexts that wanted to initialize the logging system)
static size_t& get_logging_reference_count() {
  // 静态引用计数变量 (Static reference count variable)
  static size_t ref_count = 0;

  // 返回引用计数的引用 (Return reference to the reference count)
  return ref_count;
}

// 使用C语言链接 (Using C language linkage)
extern "C" {
// rclcpp日志输出处理函数 (rclcpp logging output handler function)
static void rclcpp_logging_output_handler(
    const rcutils_log_location_t* location,  // 日志位置 (Log location)
    int severity,                            // 日志严重性级别 (Log severity level)
    const char* name,                        // 记录器名称 (Logger name)
    rcutils_time_point_value_t timestamp,    // 时间戳 (Timestamp)
    const char* format,                      // 日志格式 (Log format)
    va_list* args)                           // 可变参数列表 (Variable argument list)
{
  try {
    // 定义一个共享指针，用于获取全局日志互斥锁 (Define a shared pointer to get the global logging
    // mutex)
    std::shared_ptr<std::recursive_mutex> logging_mutex;

    // 获取全局日志互斥锁 (Get the global logging mutex)
    logging_mutex = get_global_logging_mutex();

    // 使用lock_guard保护互斥锁 (Protect the mutex using lock_guard)
    std::lock_guard<std::recursive_mutex> guard(*logging_mutex);

    // 调用rcl_logging_multiple_output_handler处理日志输出 (Call rcl_logging_multiple_output_handler
    // to handle log output)
    return rcl_logging_multiple_output_handler(location, severity, name, timestamp, format, args);
  } catch (std::exception& ex) {
    // 捕获异常并将其写入标准错误 (Catch exception and write it to standard error)
    RCUTILS_SAFE_FWRITE_TO_STDERR(ex.what());
    RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
  } catch (...) {
    // 捕获未知异常并写入失败信息到标准错误 (Catch unknown exceptions and write failure message to
    // standard error)
    RCUTILS_SAFE_FWRITE_TO_STDERR("failed to take global rclcpp logging mutex\n");
  }
}
}  // extern "C"

/**
 * @brief 构造函数，用于初始化 Context 对象 (Constructor for initializing the Context object)
 */
Context::Context()
    : rcl_context_(nullptr),  // 初始化 rcl_context_ 为 nullptr (Initialize rcl_context_ to nullptr)
      shutdown_reason_(""),  // 初始化 shutdown_reason_ 为空字符串 (Initialize shutdown_reason_ to
                             // an empty string)
      logging_mutex_(
          nullptr)  // 初始化 logging_mutex_ 为 nullptr (Initialize logging_mutex_ to nullptr)
{}

/**
 * @brief 析构函数，用于清理和关闭 Context 对象 (Destructor for cleaning up and shutting down the
 * Context object)
 */
Context::~Context() {
  // 获取 init 锁以防止 init 和 shutdown 之间的竞争条件
  // 这不会防止错误，但可能使它们更容易重现
  // (Acquire the init lock to prevent race conditions between init and shutdown
  // This will not prevent errors, but may make them easier to reproduce)
  std::lock_guard<std::recursive_mutex> lock(init_mutex_);

  try {
    // 不能依赖虚拟分派的析构函数，因此显式使用
    // 此基类提供的 shutdown()
    // (Cannot rely on virtual dispatch in a destructor, so explicitly use the
    // shutdown() provided by this base class)
    Context::shutdown("context destructor was called while still not shutdown");

    // 此时已关闭并且无法重新初始化
    // clean_up 将完成 rcl 上下文
    // (At this point it is shutdown and cannot reinit
    // clean_up will finalize the rcl context)
    this->clean_up();

  } catch (const std::exception& exc) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "unhandled exception in ~Context(): %s", exc.what());
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "unhandled exception in ~Context()");
  }
}

/**
 * @brief 删除并清理 rcl_context_t 对象 (Delete and clean up the rcl_context_t object)
 *
 * @param context 要删除的 rcl_context_t 对象指针 (Pointer to the rcl_context_t object to be
 * deleted)
 */
RCLCPP_LOCAL
void __delete_context(rcl_context_t* context) {
  if (context) {
    if (rcl_context_is_valid(context)) {
      // 如果 rcl 上下文在清理过程中意外未关闭，则发出错误
      // (Raise an error if the rcl context is unexpectedly not shutdown during cleanup)
      RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"), "rcl context unexpectedly not shutdown during cleanup");
    } else {
      // 如果 context 指针不为 null 且已关闭，则准备好进行 fini
      // (If context pointer is not null and is shutdown, then it's ready for fini)
      rcl_ret_t ret = rcl_context_fini(context);
      if (RCL_RET_OK != ret) {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"), "failed to finalize context: %s",
            rcl_get_error_string().str);
        rcl_reset_error();
      }
    }
    delete context;
  }
}

/**
 * @brief 初始化 ROS2 上下文
 * @param argc 参数数量 (number of arguments)
 * @param argv 参数值 (pointer to argument values)
 * @param init_options 初始化选项 (initialization options)
 *
 * Initialize the ROS2 context.
 */
void Context::init(int argc, char const* const* argv, const rclcpp::InitOptions& init_options) {
  // 加锁以确保线程安全 (Lock to ensure thread safety)
  std::lock_guard<std::recursive_mutex> init_lock(init_mutex_);

  // 如果上下文已经有效，则抛出异常 (Throw an exception if the context is already valid)
  if (this->is_valid()) {
    throw rclcpp::ContextAlreadyInitialized();
  }

  // 清理上下文资源 (Clean up context resources)
  this->clean_up();

  // 创建一个新的 rcl_context_t 结构体 (Create a new rcl_context_t structure)
  rcl_context_t* context = new rcl_context_t;
  if (!context) {
    throw std::runtime_error("failed to allocate memory for rcl context");
  }

  // 初始化 rcl_context_t 结构体 (Initialize the rcl_context_t structure)
  *context = rcl_get_zero_initialized_context();

  // 使用给定的参数和初始化选项初始化 rcl 上下文 (Initialize the rcl context with given arguments
  // and initialization options)
  rcl_ret_t ret = rcl_init(argc, argv, init_options.get_rcl_init_options(), context);
  if (RCL_RET_OK != ret) {
    delete context;
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to initialize rcl");
  }

  // 使用删除器 __delete_context 重置 rcl_context_ 智能指针 (Reset the rcl_context_ smart pointer
  // with the deleter __delete_context)
  rcl_context_.reset(context, __delete_context);

  // 当自动初始化日志配置时 (When auto-initializing logging configuration)
  if (init_options.auto_initialize_logging()) {
    // 获取全局日志互斥锁 (Get the global logging mutex)
    logging_mutex_ = get_global_logging_mutex();
    // 使用 std::lock_guard 对互斥锁进行上锁 (Lock the mutex using std::lock_guard)
    std::lock_guard<std::recursive_mutex> guard(*logging_mutex_);
    // 获取日志引用计数 (Get the logging reference count)
    size_t& count = get_logging_reference_count();
    // 如果日志引用计数为 0 (If the logging reference count is 0)
    if (0u == count) {
      // 使用输出处理器配置 rcl 日志 (Configure rcl logging with output handler)
      ret = rcl_logging_configure_with_output_handler(
          &rcl_context_->global_arguments,
          rcl_init_options_get_allocator(init_options_.get_rcl_init_options()),
          rclcpp_logging_output_handler);
      // 如果返回值不是 RCL_RET_OK (If the return value is not RCL_RET_OK)
      if (RCL_RET_OK != ret) {
        // 重置 rcl 上下文 (Reset the rcl context)
        rcl_context_.reset();
        // 抛出异常，表示日志配置失败 (Throw an exception indicating a failure to configure logging)
        rclcpp::exceptions::throw_from_rcl_error(ret, "failed to configure logging");
      }
    } else {
      // 如果日志已经初始化过，发出警告 (If logging has already been initialized, issue a warning)
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "logging was initialized more than once");
    }
    // 增加日志引用计数 (Increase the logging reference count)
    ++count;
  }

  try {
    // 获取未解析的 ROS 参数 (Get unparsed ROS arguments)
    std::vector<std::string> unparsed_ros_arguments = detail::get_unparsed_ros_arguments(
        argc, argv, &(rcl_context_->global_arguments), rcl_get_default_allocator());
    if (!unparsed_ros_arguments.empty()) {
      throw exceptions::UnknownROSArgsError(std::move(unparsed_ros_arguments));
    }

    // 设置 init_options_ (Set init_options_)
    init_options_ = init_options;

    // 获取 weak_contexts_ (Get weak_contexts_)
    weak_contexts_ = get_weak_contexts();

    // 添加当前上下文到 weak_contexts_ (Add the current context to weak_contexts_)
    weak_contexts_->add_context(this->shared_from_this());
  } catch (const std::exception& e) {
    // 如果在处理过程中发生异常，尝试关闭 rcl 上下文并重置 (If an exception occurs during
    // processing, try to shut down the rcl context and reset)
    ret = rcl_shutdown(rcl_context_.get());
    rcl_context_.reset();
    if (RCL_RET_OK != ret) {
      std::ostringstream oss;
      oss << "While handling: " << e.what() << std::endl << "    another exception was thrown";
      rclcpp::exceptions::throw_from_rcl_error(ret, oss.str());
    }
    throw;
  }
}

/**
 * @brief 检查当前上下文是否有效 (Check if the current context is valid)
 *
 * @return 如果上下文有效则返回true，否则返回false (Return true if the context is valid, otherwise
 * return false)
 */
bool Context::is_valid() const {
  // 获取共享指针的本地副本，以防止在我们使用过程中被置空 (Take a local copy of the shared pointer
  // to avoid it getting nulled under our feet)
  auto local_rcl_context = rcl_context_;
  if (!local_rcl_context) {
    return false;
  }
  return rcl_context_is_valid(local_rcl_context.get());
}

/**
 * @brief 获取初始化选项的常量引用 (Get a constant reference to the initialization options)
 *
 * @return 初始化选项的常量引用 (A constant reference to the initialization options)
 */
const rclcpp::InitOptions& Context::get_init_options() const { return init_options_; }

/**
 * @brief 获取初始化选项的副本 (Get a copy of the initialization options)
 *
 * @return 初始化选项的副本 (A copy of the initialization options)
 */
rclcpp::InitOptions Context::get_init_options() { return init_options_; }

/**
 * @brief 获取当前上下文的域ID (Get the domain ID of the current context)
 *
 * @return 当前上下文的域ID (The domain ID of the current context)
 */
size_t Context::get_domain_id() const {
  size_t domain_id;
  rcl_ret_t ret = rcl_context_get_domain_id(rcl_context_.get(), &domain_id);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to get domain id from context");
  }
  return domain_id;
}

/**
 * @brief 获取关闭原因 (Get the shutdown reason)
 *
 * @return 关闭原因字符串 (The shutdown reason string)
 */
std::string Context::shutdown_reason() const {
  std::lock_guard<std::recursive_mutex> lock(init_mutex_);
  return shutdown_reason_;
}

/**
 * @brief 关闭当前上下文 (Shutdown the current context)
 *
 * @param reason 关闭原因 (The shutdown reason)
 * @return 如果成功关闭则返回true，否则返回false (Return true if successfully shut down, otherwise
 * return false)
 */
bool Context::shutdown(const std::string& reason) {
  // 防止竞争条件 (Prevent race conditions)
  std::lock_guard<std::recursive_mutex> init_lock(init_mutex_);
  // 确保有效性 (Ensure validity)
  if (!this->is_valid()) {
    // 如果无效，则无法关闭 (If it is not valid, then it cannot be shut down)
    return false;
  }

  // 调用每个预关闭回调 (Call each pre-shutdown callback)
  {
    std::lock_guard<std::mutex> lock{pre_shutdown_callbacks_mutex_};
    for (const auto& callback : pre_shutdown_callbacks_) {
      (*callback)();
    }
  }

  // rcl关闭 (rcl shutdown)
  rcl_ret_t ret = rcl_shutdown(rcl_context_.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  // 设置关闭原因 (Set shutdown reason)
  shutdown_reason_ = reason;
  // 调用每个关闭回调 (Call each shutdown callback)
  {
    std::lock_guard<std::mutex> lock(on_shutdown_callbacks_mutex_);
    for (const auto& callback : on_shutdown_callbacks_) {
      (*callback)();
    }
  }

  // 中断所有阻塞的sleep_for()和所有阻塞的执行器或等待集 (Interrupt all blocking sleep_for() and all
  // blocking executors or wait sets)
  this->interrupt_all_sleep_for();
  // 从全局上下文中删除自身 (Remove self from the global contexts)
  weak_contexts_->remove_context(this);
  // 关闭日志记录器 (Shutdown logger)
  if (logging_mutex_) {
    // 由此上下文初始化日志记录 (Logging was initialized by this context)
    std::lock_guard<std::recursive_mutex> guard(*logging_mutex_);
    size_t& count = get_logging_reference_count();
    if (0u == --count) {
      rcl_ret_t rcl_ret = rcl_logging_fini();
      if (RCL_RET_OK != rcl_ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
            RCUTILS_STRINGIFY(__file__) ":" RCUTILS_STRINGIFY(__LINE__) " failed to fini logging");
        rcl_reset_error();
      }
    }
  }
  return true;
}

/**
 * @brief 注册一个 on_shutdown 回调函数，并返回该回调函数 (Register an on_shutdown callback and
 * return the callback)
 *
 * @param callback 要注册的回调函数 (The callback function to register)
 * @return OnShutdownCallback 注册的回调函数 (The registered callback function)
 */
rclcpp::Context::OnShutdownCallback Context::on_shutdown(OnShutdownCallback callback) {
  // 添加一个 on_shutdown 回调函数 (Add an on_shutdown callback)
  add_on_shutdown_callback(callback);

  // 返回添加的回调函数 (Return the added callback)
  return callback;
}

/**
 * @brief 添加一个 on_shutdown 回调函数 (Add an on_shutdown callback)
 *
 * @param callback 要添加的回调函数 (The callback to add)
 * @return OnShutdownCallbackHandle 包含添加回调函数的句柄 (A handle containing the added callback)
 */
rclcpp::OnShutdownCallbackHandle Context::add_on_shutdown_callback(OnShutdownCallback callback) {
  // 添加一个 shutdown 类型为 on_shutdown 的回调函数 (Add a shutdown callback with type on_shutdown)
  return add_shutdown_callback<ShutdownType::on_shutdown>(callback);
}

/**
 * @brief 删除一个 on_shutdown 回调函数 (Remove an on_shutdown callback)
 *
 * @param callback_handle 包含要删除的回调函数的句柄 (A handle containing the callback to remove)
 * @return bool 表示是否成功删除回调函数 (Indicates whether the callback was successfully removed)
 */
bool Context::remove_on_shutdown_callback(const OnShutdownCallbackHandle& callback_handle) {
  // 删除一个 shutdown 类型为 on_shutdown 的回调函数 (Remove a shutdown callback with type
  // on_shutdown)
  return remove_shutdown_callback<ShutdownType::on_shutdown>(callback_handle);
}

/**
 * @brief 添加一个 pre_shutdown 回调函数 (Add a pre_shutdown callback)
 *
 * @param callback 要添加的回调函数 (The callback to add)
 * @return PreShutdownCallbackHandle 包含添加回调函数的句柄 (A handle containing the added callback)
 */
rclcpp::PreShutdownCallbackHandle Context::add_pre_shutdown_callback(PreShutdownCallback callback) {
  // 添加一个 shutdown 类型为 pre_shutdown 的回调函数 (Add a shutdown callback with type
  // pre_shutdown)
  return add_shutdown_callback<ShutdownType::pre_shutdown>(callback);
}

/**
 * @brief 删除一个 pre_shutdown 回调函数 (Remove a pre_shutdown callback)
 *
 * @param callback_handle 包含要删除的回调函数的句柄 (A handle containing the callback to remove)
 * @return bool 表示是否成功删除回调函数 (Indicates whether the callback was successfully removed)
 */
bool Context::remove_pre_shutdown_callback(const PreShutdownCallbackHandle& callback_handle) {
  // 删除一个 shutdown 类型为 pre_shutdown 的回调函数 (Remove a shutdown callback with type
  // pre_shutdown)
  return remove_shutdown_callback<ShutdownType::pre_shutdown>(callback_handle);
}

/**
 * @brief 根据给定的 shutdown_type 添加一个 shutdown 回调函数 (Add a shutdown callback based on the
 * given shutdown_type)
 *
 * @tparam shutdown_type 指定的 shutdown 类型 (The specified shutdown type)
 * @param callback 要添加的回调函数 (The callback to add)
 * @return ShutdownCallbackHandle 包含添加回调函数的句柄 (A handle containing the added callback)
 */
template <Context::ShutdownType shutdown_type>
rclcpp::ShutdownCallbackHandle Context::add_shutdown_callback(ShutdownCallback callback) {
  // 创建一个共享指针，用于存储回调函数 (Create a shared pointer for storing the callback)
  auto callback_shared_ptr =
      std::make_shared<ShutdownCallbackHandle::ShutdownCallbackType>(callback);

  // 静态断言，确保 shutdown_type 是有效的 (Static assert to ensure the shutdown_type is valid)
  static_assert(
      shutdown_type == ShutdownType::pre_shutdown || shutdown_type == ShutdownType::on_shutdown);

  // 根据给定的 shutdown_type 添加回调函数 (Add the callback based on the given shutdown_type)
  if constexpr (shutdown_type == ShutdownType::pre_shutdown) {
    std::lock_guard<std::mutex> lock(pre_shutdown_callbacks_mutex_);
    pre_shutdown_callbacks_.emplace(callback_shared_ptr);
  } else {
    std::lock_guard<std::mutex> lock(on_shutdown_callbacks_mutex_);
    on_shutdown_callbacks_.emplace(callback_shared_ptr);
  }

  // 创建一个句柄并将回调函数添加到句柄中 (Create a handle and add the callback to the handle)
  ShutdownCallbackHandle callback_handle;
  callback_handle.callback = callback_shared_ptr;

  // 返回包含回调函数的句柄 (Return the handle containing the callback)
  return callback_handle;
}

/**
 * @brief 删除指定的关闭回调函数
 * @tparam shutdown_type 关闭类型，可以是 pre_shutdown 或 on_shutdown
 * @param callback_handle 要删除的回调函数句柄
 * @return 如果成功删除回调，则返回 true，否则返回 false
 *
 * Remove the specified shutdown callback function
 * @tparam shutdown_type Shutdown type, can be pre_shutdown or on_shutdown
 * @param callback_handle The callback handle to be removed
 * @return Return true if the callback is successfully removed, otherwise return false
 */
template <Context::ShutdownType shutdown_type>
bool Context::remove_shutdown_callback(const ShutdownCallbackHandle& callback_handle) {
  // 尝试获取回调函数的弱引用
  // Try to get the weak reference of the callback function
  const auto callback_shared_ptr = callback_handle.callback.lock();
  // 如果回调函数为空，则返回 false
  // If the callback function is empty, return false
  if (callback_shared_ptr == nullptr) {
    return false;
  }

  // 定义一个 lambda 函数，用于删除指定的回调
  // Define a lambda function for removing the specified callback
  const auto remove_callback = [this, &callback_shared_ptr](auto& mutex, auto& callback_set) {
    // 使用 lock_guard 对象锁定互斥量
    // Lock the mutex using a lock_guard object
    const std::lock_guard<std::mutex> lock(mutex);
    // 从回调集合中删除回调，并检查是否删除成功
    // Delete the callback from the callback set and check if it was successful
    return callback_set.erase(callback_shared_ptr) == 1;
  };

  // 静态断言，确保 shutdown_type 是预期的类型
  // Static assert to ensure shutdown_type is of the expected type
  static_assert(
      shutdown_type == ShutdownType::pre_shutdown || shutdown_type == ShutdownType::on_shutdown);

  // 根据 shutdown_type 的值，选择要删除的回调集合，并调用 remove_callback 函数
  // Select the callback set to be deleted based on the value of shutdown_type, and call the
  // remove_callback function
  if constexpr (shutdown_type == ShutdownType::pre_shutdown) {
    return remove_callback(pre_shutdown_callbacks_mutex_, pre_shutdown_callbacks_);
  } else {
    return remove_callback(on_shutdown_callbacks_mutex_, on_shutdown_callbacks_);
  }
}

/**
 * @brief 获取所有 on_shutdown 回调函数
 * @return 包含所有 on_shutdown 回调函数的 std::vector
 *
 * Get all on_shutdown callbacks
 * @return A std::vector containing all on_shutdown callbacks
 */
std::vector<rclcpp::Context::OnShutdownCallback> Context::get_on_shutdown_callbacks() const {
  return get_shutdown_callback<ShutdownType::on_shutdown>();
}

/**
 * @brief 获取所有 pre_shutdown 回调函数
 * @return 包含所有 pre_shutdown 回调函数的 std::vector
 *
 * Get all pre_shutdown callbacks
 * @return A std::vector containing all pre_shutdown callbacks
 */
std::vector<rclcpp::Context::PreShutdownCallback> Context::get_pre_shutdown_callbacks() const {
  return get_shutdown_callback<ShutdownType::pre_shutdown>();
}

/**
 * @brief 获取指定类型的关闭回调函数
 * @tparam shutdown_type 关闭类型，可以是 pre_shutdown 或 on_shutdown
 * @return 包含指定类型关闭回调函数的 std::vector
 *
 * Get the shutdown callback functions of the specified type
 * @tparam shutdown_type Shutdown type, can be pre_shutdown or on_shutdown
 * @return A std::vector containing the shutdown callback functions of the specified type
 */
template <Context::ShutdownType shutdown_type>
std::vector<rclcpp::Context::ShutdownCallback> Context::get_shutdown_callback() const {
  // 定义一个 lambda 函数，用于获取指定类型的关闭回调函数
  // Define a lambda function for getting the shutdown callback functions of the specified type
  const auto get_callback_vector = [this](auto& mutex, auto& callback_set) {
    // 使用 lock_guard 对象锁定互斥量
    // Lock the mutex using a lock_guard object
    const std::lock_guard<std::mutex> lock(mutex);
    // 创建一个回调函数向量并填充它
    // Create a callback function vector and populate it
    std::vector<rclcpp::Context::ShutdownCallback> callbacks;
    for (auto& callback : callback_set) {
      callbacks.push_back(*callback);
    }
    return callbacks;
  };

  // 静态断言，确保 shutdown_type 是预期的类型
  // Static assert to ensure shutdown_type is of the expected type
  static_assert(
      shutdown_type == ShutdownType::pre_shutdown || shutdown_type == ShutdownType::on_shutdown);

  // 根据 shutdown_type 的值，选择要获取的回调集合，并调用 get_callback_vector 函数
  // Select the callback set to be obtained based on the value of shutdown_type, and call the
  // get_callback_vector function
  if constexpr (shutdown_type == ShutdownType::pre_shutdown) {
    return get_callback_vector(pre_shutdown_callbacks_mutex_, pre_shutdown_callbacks_);
  } else {
    return get_callback_vector(on_shutdown_callbacks_mutex_, on_shutdown_callbacks_);
  }
}

/**
 * @brief 获取 rcl_context_t 对象的共享指针
 * @return rcl_context_t 对象的 std::shared_ptr
 *
 * Get the shared pointer of the rcl_context_t object
 * @return A std::shared_ptr of the rcl_context_t object
 */
std::shared_ptr<rcl_context_t> Context::get_rcl_context() { return rcl_context_; }

/**
 * @brief 睡眠一段时间
 * @param nanoseconds 要睡眠的纳秒数
 * @return 如果超时成功结束，则返回 true，否则返回 false
 *
 * Sleep for a period of time
 * @param nanoseconds The number of nanoseconds to sleep
 * @return Return true if the timeout ends successfully, otherwise return false
 */
bool Context::sleep_for(const std::chrono::nanoseconds& nanoseconds) {
  std::chrono::nanoseconds time_left = nanoseconds;
  do {
    {
      // 创建一个 unique_lock 对象，并锁定互斥量
      // Create a unique_lock object and lock the mutex
      std::unique_lock<std::mutex> lock(interrupt_mutex_);
      auto start = std::chrono::steady_clock::now();
      // 在等待期间释放锁
      // Release the lock during waiting
      interrupt_condition_variable_.wait_for(lock, nanoseconds);
      time_left -= std::chrono::steady_clock::now() - start;
    }
  } while (time_left > std::chrono::nanoseconds::zero() && this->is_valid());
  // 如果超时成功结束，则返回 true，否则返回 false
  // Return true if the timeout ends successfully, otherwise return false
  return this->is_valid();
}

/**
 * @brief 中断所有 sleep_for 操作
 *
 * Interrupt all sleep_for operations
 */
void Context::interrupt_all_sleep_for() { interrupt_condition_variable_.notify_all(); }

/**
 * @brief 清理上下文对象
 *
 * Clean up the context object
 */
void Context::clean_up() {
  shutdown_reason_ = "";
  rcl_context_.reset();
  sub_contexts_.clear();
}

/**
 * @brief 获取所有上下文对象
 * @return 包含所有上下文对象的 std::vector
 *
 * Get all context objects
 * @return A std::vector containing all context objects
 */
std::vector<Context::SharedPtr> rclcpp::get_contexts() {
  WeakContextsWrapper::SharedPtr weak_contexts = get_weak_contexts();
  return weak_contexts->get_contexts();
}

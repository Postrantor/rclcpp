// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CONTEXT_HPP_
#define RCLCPP__CONTEXT_HPP_

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rcl/context.h"
#include "rcl/guard_condition.h"
#include "rcl/wait.h"
#include "rclcpp/init_options.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/// Thrown when init is called on an already initialized context.
/**
 * @class ContextAlreadyInitialized
 * @brief 自定义异常类，表示上下文已经初始化 (Custom exception class indicating that the context is
 * already initialized)
 */
class ContextAlreadyInitialized : public std::runtime_error {
public:
  /**
   * @brief 构造函数，设置异常信息 (Constructor, sets the exception message)
   */
  ContextAlreadyInitialized()
      : std::runtime_error("context is already initialized") {
  }  // 初始化基类 runtime_error，并传递异常信息 (Initialize the base class runtime_error and pass
     // the exception message)
};

// 前向声明 WeakContextsWrapper 类 (Forward declare WeakContextsWrapper class)
/// @class WeakContextsWrapper
/// @brief 用于存储弱上下文的包装器类 (Wrapper class for storing weak contexts)
class WeakContextsWrapper;

/**
 * @class ShutdownCallbackHandle
 * @brief 一个用于管理 shutdown 回调的类。A class for managing shutdown callbacks.
 */
class ShutdownCallbackHandle {
  // 允许 Context 类访问本类的私有成员。Allow the Context class to access this class's private
  // members.
  friend class Context;

public:
  /**
   * @typedef ShutdownCallbackType
   * @brief 定义了一个函数对象类型，该类型的函数不接受参数且无返回值。Defines a function object type
   * that takes no arguments and has no return value.
   */
  using ShutdownCallbackType = std::function<void()>;

private:
  /**
   * @var std::weak_ptr<ShutdownCallbackType> callback
   * @brief 一个弱指针，用于存储 shutdown 回调函数。A weak pointer for storing the shutdown callback
   * function.
   *
   * 使用弱指针是为了避免循环引用问题。The weak pointer is used to avoid circular reference issues.
   */
  std::weak_ptr<ShutdownCallbackType> callback;
};

/**
 * @typedef OnShutdownCallbackHandle
 * @brief 当系统关闭时触发的回调处理程序。A callback handler triggered when the system shuts down.
 */
using OnShutdownCallbackHandle = ShutdownCallbackHandle;

/**
 * @typedef PreShutdownCallbackHandle
 * @brief 系统关闭前触发的回调处理程序。A callback handler triggered before the system shuts down.
 */
using PreShutdownCallbackHandle = ShutdownCallbackHandle;

/// 上下文，封装了节点和其他类似实体之间的共享状态。
/// Context which encapsulates shared state between nodes and other similar entities.
/**
 * 上下文还表示 rclcpp 的初始化到关闭之间的生命周期。
 * 它通常与 rclcpp::init 或 rclcpp::init_local 以及 rclcpp::shutdown 结合使用。
 * A context also represents the lifecycle between init and shutdown of rclcpp.
 * It is often used in conjunction with rclcpp::init, or rclcpp::init_local,
 * and rclcpp::shutdown.
 */
class Context : public std::enable_shared_from_this<Context> {
public:
  // 定义智能指针类型，方便对 Context 进行引用计数和内存管理。
  // Define smart pointer types for easier reference counting and memory management of Context
  // objects.
  RCLCPP_SMART_PTR_DEFINITIONS(Context)

  /// 默认构造函数，在此之后，上下文仍然没有 "初始化"。
  /// Default constructor, after which the Context is still not "initialized".
  /**
   * 构建的每个上下文都会添加到全局上下文向量中，
   * 该向量由信号处理程序用于在 SIGINT 上有条件地关闭每个上下文。
   * 请参阅 InitOptions 类中的 shutdown_on_signal 选项。
   * Every context which is constructed is added to a global vector of contexts,
   * which is used by the signal handler to conditionally shutdown each context
   * on SIGINT.
   * See the shutdown_on_signal option in the InitOptions class.
   */
  RCLCPP_PUBLIC
  Context();

  /// 虚拟析构函数
  /// Virtual destructor
  RCLCPP_PUBLIC
  virtual ~Context();

  /// 初始化上下文及其底层元素，如 rcl 上下文。
  /// Initialize the context, and the underlying elements like the rcl context.
  /**
   * 在将此上下文传递给 Node 构造函数等之前，必须调用此方法。
   * This method must be called before passing this context to things like the constructor of Node.
   * 在尝试关闭上下文之前也必须调用它。
   * It must also be called before trying to shutdown the context.
   *
   * 请注意，此功能不会设置任何信号处理程序，
   * Note that this function does not setup any signal handlers,
   * 因此，如果您希望通过信号处理程序关闭它，
   * so if you want it to be shutdown by the signal handler,
   * 那么您需要使用 rclcpp::install_signal_handlers() 手动安装它们或使用 rclcpp::init()。
   * then you need to either install them manually with rclcpp::install_signal_handlers() or use
   * rclcpp::init(). 除了安装信号处理程序外，InitOptions 的 shutdown_on_signal 需要为 `true`， In
   * addition to installing the signal handlers, the shutdown_on_signal of the InitOptions needs to
   * be `true`, 以便通过信号处理程序关闭此上下文，否则它将被跳过。 for this context to be shutdown
   * by the signal handler, otherwise it will be passed over.
   *
   * 调用此方法后，可以调用 shutdown() 使派生实体的上下文无效，例如节点、保护条件等。
   * After calling this method, shutdown() can be called to invalidate the context for derived
   * entities, e.g. nodes, guard conditions, etc.
   * 但是，在调用此上下文的析构函数或再次调用此函数之前，底层 rcl 上下文不会被终止。
   * However, the underlying rcl context is not finalized until this Context's destructor is called
   * or this function is called again.
   * 当派生实体仍在使用上下文时，允许此类超出范围并被销毁或调用此函数第二次是未定义的行为，应避免这种情况。
   * Allowing this class to go out of scope and get destructed or calling this function a second
   * time while derived entities are still using the context is undefined behavior and should be
   * avoided. 不重用上下文对象，而是每次需要关闭和初始化时都创建一个新的上下文对象是个好主意。 It's
   * a good idea to not reuse context objects and instead create a new one each time you need to
   * shutdown and init again. 这允许派生实体在完成之前持有对第一个上下文对象的共享指针。 This allows
   * derived entities to hold on to shard pointers to the first context object until they are done.
   *
   * 此功能是线程安全的。
   * This function is thread-safe.
   *
   * \param[in] argc 参数数量
   * \param[in] argc number of arguments
   * \param[in] argv 可能包含 ROS 的参数数组
   * \param[in] argv argument array which may contain arguments intended for ROS
   * \param[in] init_options 用于 rclcpp 和底层层的初始化选项
   * \param[in] init_options initialization options for rclcpp and underlying layers
   * \throw ContextAlreadyInitialized 如果调用 init 多次
   * \throw ContextAlreadyInitialized if called if init is called more than once
   * \throws 任何 rclcpp::exceptions::throw_from_rcl_error 可以抛出的异常。
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   * \throws std::runtime_error 如果全局日志配置互斥锁为 NULL
   * \throws std::runtime_error if the global logging configure mutex is NULL
   * \throws exceptions::UnknownROSArgsError 如果有未知的 ROS 参数
   * \throws exceptions::UnknownROSArgsError if there are unknown ROS arguments
   */
  RCLCPP_PUBLIC
  virtual void init(
      int argc,
      char const *const *argv,
      const rclcpp::InitOptions &init_options = rclcpp::InitOptions());

  /// 返回 true 如果上下文有效，否则返回 false。
  /// Return true if the context is valid, otherwise false.
  /**
   * 如果上下文已初始化但尚未关闭，则上下文有效。
   * The context is valid if it has been initialized but not shutdown.
   *
   * 此函数是线程安全的。
   * This function is thread-safe.
   * 只要指针和 uint64_t 原子操作是无锁的，此函数就是无锁的。
   * This function is lock free so long as pointers and uint64_t atomics are lock free.
   *
   * \return 如果有效，则返回 true，否则返回 false
   * \return true if valid, otherwise false
   */
  RCLCPP_PUBLIC
  bool is_valid() const;

  /// 返回 init 期间使用的 init 选项。
  /// Return the init options used during init.
  RCLCPP_PUBLIC
  const rclcpp::InitOptions &get_init_options() const;

  /// 返回 init 期间使用的 init 选项的副本。
  /// Return a copy of the init options used during init.
  RCLCPP_PUBLIC
  rclcpp::InitOptions get_init_options();

  /// 返回实际的域 ID。
  /// Return actual domain id.
  RCLCPP_PUBLIC
  size_t get_domain_id() const;

  /// 返回关闭原因，如果没有关闭，则返回空字符串。
  /// Return the shutdown reason, or empty string if not shutdown.
  /**
   * 此函数是线程安全的。
   * This function is thread-safe.
   */
  RCLCPP_PUBLIC
  std::string shutdown_reason() const;

  /// 关闭上下文，使其未初始化，因此对派生实体无效。
  /// Shutdown the context, making it uninitialized and therefore invalid for derived entities.
  /**
   * 当上下文关闭时，会按以下顺序发生几件事：
   * Several things happen when the context is shutdown, in this order:
   *
   * - 获取锁以防止与 init、on_shutdown 等的竞争条件。
   * - acquires a lock to prevent race conditions with init, on_shutdown, etc.
   * - 如果上下文没有初始化，则返回 false
   * - if the context is not initialized, return false
   * - 在内部 rcl_context_t 实例上调用 rcl_shutdown()
   * - rcl_shutdown() is called on the internal rcl_context_t instance
   * - 设置关闭原因
   * - the shutdown reason is set
   * - 按照添加的顺序调用每个 on_shutdown 回调
   * - each on_shutdown callback is called, in the order that they were added
   * - 中断阻塞 sleep_for() 调用，使它们由于关闭而提前返回
   * - interrupt blocking sleep_for() calls, so they return early due to shutdown
   * - 中断阻塞执行器和等待集
   * - interrupt blocking executors and wait sets
   *
   * 此函数不会终止底层的 rcl 上下文。
   * The underlying rcl context is not finalized by this function.
   *
   * 该函数是线程安全的。
   * This function is thread-safe.
   *
   * 请注意，如果您重写此方法，但将 shutdown 留给此基类的销毁，它将不会从您的基类调用重写的版本。
   * Note that if you override this method, but leave shutdown to be called in
   * the destruction of this base class, it will not call the overridden
   * version from your base class.
   * 因此，您需要确保在其析构函数中调用您的类的 shutdown()。
   * So you need to ensure you call your class's shutdown() in its destructor.
   *
   * \param[in] reason 关闭发生的原因描述
   * \param[in] reason the description of why shutdown happened
   * \return 如果关闭成功，则返回 true；如果上下文已经关闭，则返回 false
   * \return true if shutdown was successful, false if context was already shutdown
   * \throw 如果 rcl_shutdown 失败，则抛出由 rclcpp::exceptions::RCLError 派生的各种异常
   * \throw various exceptions derived from rclcpp::exceptions::RCLError, if rcl_shutdown fails
   */
  RCLCPP_PUBLIC
  virtual bool shutdown(const std::string &reason);

  // 定义 OnShutdownCallback 类型为 OnShutdownCallbackHandle::ShutdownCallbackType
  using OnShutdownCallback = OnShutdownCallbackHandle::ShutdownCallbackType;

  /// 添加一个 on_shutdown 回调，当此上下文的 shutdown 被调用时触发。
  /// Add a on_shutdown callback to be called when shutdown is called for this context.
  /**
   * 这些回调将按照添加的顺序作为 shutdown() 中倒数第二步被调用。
   * These callbacks will be called in the order they are added as the second
   * to last step in shutdown().
   *
   * 当由于信号处理器而发生关闭时，这些回调将在专用的信号处理线程中异步运行。
   * When shutdown occurs due to the signal handler, these callbacks are run
   * asynchronously in the dedicated singal handling thread.
   *
   * 另外，shutdown() 可能会从此函数的析构函数中调用。
   * Also, shutdown() may be called from the destructor of this function.
   * 因此，从这些回调中抛出异常是不安全的。
   * Therefore, it is not safe to throw exceptions from these callbacks.
   * 相反，记录错误或使用其他机制来指示发生了错误。
   * Instead, log errors or use some other mechanism to indicate an error has
   * occurred.
   *
   * 在 init 之前和 shutdown 之后可以注册 on_shutdown 回调，并在重复的 init 上持续。
   * On shutdown callbacks may be registered before init and after shutdown,
   * and persist on repeated init's.
   *
   * \param[in] callback 要注册的 on_shutdown 回调
   * \param[in] callback the on_shutdown callback to be registered
   * \return 返回传递的回调，方便存储传递的 lambda
   * \return the callback passed, for convenience when storing a passed lambda
   */
  RCLCPP_PUBLIC
  virtual OnShutdownCallback on_shutdown(OnShutdownCallback callback);

  /// 添加一个 on_shutdown 回调，当此上下文的 shutdown 被调用时触发。
  /// Add a on_shutdown callback to be called when shutdown is called for this context.
  /**
   * 这些回调将按照添加的顺序作为 shutdown() 中倒数第二步被调用。
   * These callbacks will be called in the order they are added as the second
   * to last step in shutdown().
   *
   * 当由于信号处理器而发生关闭时，这些回调将在专用的信号处理线程中异步运行。
   * When shutdown occurs due to the signal handler, these callbacks are run
   * asynchronously in the dedicated signal handling thread.
   *
   * 另外，shutdown() 可能会从此函数的析构函数中调用。
   * Also, shutdown() may be called from the destructor of this function.
   * 因此，从这些回调中抛出异常是不安全的。
   * Therefore, it is not safe to throw exceptions from these callbacks.
   * 相反，记录错误或使用其他机制来指示发生了错误。
   * Instead, log errors or use some other mechanism to indicate an error has
   * occurred.
   *
   * 在 init 之前和 shutdown 之后可以注册 on_shutdown 回调，并在重复的 init 上持续。
   * On shutdown callbacks may be registered before init and after shutdown,
   * and persist on repeated init's.
   *
   * \param[in] callback 要注册的 on_shutdown 回调
   * \param[in] callback the on_shutdown callback to be registered
   * \return 返回创建的回调句柄
   * \return the created callback handle
   */
  RCLCPP_PUBLIC
  virtual OnShutdownCallbackHandle add_on_shutdown_callback(OnShutdownCallback callback);

  /// 移除已注册的 on_shutdown 回调函数。 (Remove an registered on_shutdown callbacks.)
  /**
   * \param[in] callback_handle 要移除的 on_shutdown 回调句柄。 (the on_shutdown callback handle to
   * be removed.) \return 如果找到并移除了回调，则返回 true，否则返回 false。 (true if the callback
   * is found and removed, otherwise false.)
   */
  RCLCPP_PUBLIC
  virtual bool remove_on_shutdown_callback(const OnShutdownCallbackHandle &callback_handle);

  // 定义 PreShutdownCallback 类型。 (Define PreShutdownCallback type.)
  using PreShutdownCallback = PreShutdownCallbackHandle::ShutdownCallbackType;

  /// 在此上下文中的 shutdown 被调用之前，添加一个 pre_shutdown 回调。 (Add a pre_shutdown callback
  /// to be called before shutdown is called for this context.)
  /**
   * 这些回调将按照它们被添加的顺序执行。 (These callbacks will be called in the order they are
   * added.)
   *
   * 当由于信号处理程序而发生 shutdown 时，这些回调将在专用信号处理线程中异步运行。 (When shutdown
   * occurs due to the signal handler, these callbacks are run asynchronously in the dedicated
   * signal handling thread.)
   *
   * \param[in] callback 要注册的 pre_shutdown 回调。 (the pre_shutdown callback to be registered)
   * \return 创建的回调句柄。 (the created callback handle)
   */
  RCLCPP_PUBLIC
  virtual PreShutdownCallbackHandle add_pre_shutdown_callback(PreShutdownCallback callback);

  /// 移除已注册的 pre_shutdown 回调函数。 (Remove an registered pre_shutdown callback.)
  /**
   * \param[in] callback_handle 要移除的 pre_shutdown 回调句柄。 (the pre_shutdown callback handle
   * to be removed.) \return 如果找到并移除了回调，则返回 true，否则返回 false。 (true if the
   * callback is found and removed, otherwise false.)
   */
  RCLCPP_PUBLIC
  virtual bool remove_pre_shutdown_callback(const PreShutdownCallbackHandle &callback_handle);

  /// 返回 shutdown 回调函数。 (Return the shutdown callbacks.)
  /**
   * 返回的回调是已注册回调的副本。 (Returned callbacks are a copy of the registered callbacks.)
   */
  RCLCPP_PUBLIC
  std::vector<OnShutdownCallback> get_on_shutdown_callbacks() const;

  /// 返回 pre-shutdown 回调函数。 (Return the pre-shutdown callbacks.)
  /**
   * 返回的回调是已注册回调的副本。 (Returned callbacks are a copy of the registered callbacks.)
   */
  RCLCPP_PUBLIC
  std::vector<PreShutdownCallback> get_pre_shutdown_callbacks() const;

  /// 返回内部的 rcl 上下文对象。
  /// Return the internal rcl context.
  RCLCPP_PUBLIC
  std::shared_ptr<rcl_context_t> get_rcl_context();

  /// 休眠一定时间或者直到 shutdown() 被调用。
  /// Sleep for a given period of time or until shutdown() is called.
  /**
   * 如果满足以下条件之一，此函数可能会提前中断：
   * This function can be interrupted early if:
   *
   *   - 当前上下文被 shutdown()
   *   - this context is shutdown()
   *   - 当前上下文被析构（导致 shutdown）
   *   - this context is destructed (resulting in shutdown)
   *   - 当前上下文设置了 shutdown_on_signal=true 并且收到 SIGINT/SIGTERM 信号（导致 shutdown）
   *   - this context has shutdown_on_signal=true and SIGINT/SIGTERM occurs (resulting in shutdown)
   *   - 调用 interrupt_all_sleep_for()
   *   - interrupt_all_sleep_for() is called
   *
   * \param[in] nanoseconds 一个 std::chrono::duration 对象，表示要休眠多长时间。
   * \return 如果条件变量没有超时，则返回 true，即您被中断。
   */
  RCLCPP_PUBLIC
  bool sleep_for(const std::chrono::nanoseconds &nanoseconds);

  /// 中断所有阻塞的 sleep_for 调用，使它们立即返回并返回 true。
  /// Interrupt any blocking sleep_for calls, causing them to return immediately and return true.
  RCLCPP_PUBLIC
  void interrupt_all_sleep_for();

  /// 返回 SubContext 类型的单例实例，如果需要则构造一个。
  /// Return a singleton instance for the SubContext type, constructing one if necessary.
  template <typename SubContext, typename... Args>
  std::shared_ptr<SubContext> get_sub_context(Args &&...args) {
    // 使用递归互斥锁保护子上下文
    // Use a recursive mutex to protect the sub contexts
    std::lock_guard<std::recursive_mutex> lock(sub_contexts_mutex_);

    // 获取 SubContext 类型的索引
    // Get the type index of SubContext
    std::type_index type_i(typeid(SubContext));
    std::shared_ptr<SubContext> sub_context;
    auto it = sub_contexts_.find(type_i);
    if (it == sub_contexts_.end()) {
      // 如果不存在，则创建一个新的子上下文
      // If it doesn't exist yet, create a new sub context
      sub_context = std::shared_ptr<SubContext>(
          new SubContext(std::forward<Args>(args)...),
          [](SubContext *sub_context_ptr) { delete sub_context_ptr; });
      sub_contexts_[type_i] = sub_context;
    } else {
      // 如果已经存在，获取并转换为相应类型
      // If it exists, retrieve and cast it to the appropriate type
      sub_context = std::static_pointer_cast<SubContext>(it->second);
    }
    return sub_context;
  }

protected:
  /**
   * \brief 清理函数，用于在构造和析构时清理 rcl 上下文并准备新的初始化周期。
   * \brief Clean up function, used to clean up the rcl context during construction and destruction
   * and prepare for a new initialization cycle.
   */
  RCLCPP_PUBLIC
  void clean_up();

private:
  // 禁用拷贝构造函数和拷贝赋值操作符
  // Disable copy constructor and copy assignment operator
  RCLCPP_DISABLE_COPY(Context)

  // 这个互斥锁是递归的，以便析构函数可以确保 is_initialized 和 shutdown 之间的原子性
  // This mutex is recursive so that the destructor can ensure atomicity between is_initialized and
  // shutdown
  mutable std::recursive_mutex init_mutex_;
  std::shared_ptr<rcl_context_t> rcl_context_;
  rclcpp::InitOptions init_options_;
  std::string shutdown_reason_;

  // 保持对全局日志互斥锁的共享所有权
  // Keep shared ownership of the global logging mutex
  std::shared_ptr<std::recursive_mutex> logging_mutex_;

  // 存储子上下文的无序映射
  // Unordered map to store sub-contexts
  std::unordered_map<std::type_index, std::shared_ptr<void>> sub_contexts_;
  // 这个互斥锁是递归的，以便子上下文的构造函数可以尝试获取另一个子上下文
  // This mutex is recursive so that the constructor of a sub context may attempt to acquire another
  // sub context
  std::recursive_mutex sub_contexts_mutex_;

  // 存储 on_shutdown 回调的无序集合
  // Unordered set to store on_shutdown callbacks
  std::unordered_set<std::shared_ptr<OnShutdownCallback>> on_shutdown_callbacks_;
  mutable std::mutex on_shutdown_callbacks_mutex_;

  // 存储 pre_shutdown 回调的无序集合
  // Unordered set to store pre_shutdown callbacks
  std::unordered_set<std::shared_ptr<PreShutdownCallback>> pre_shutdown_callbacks_;
  mutable std::mutex pre_shutdown_callbacks_mutex_;

  // 用于定时睡眠的条件变量（参见 sleep_for）
  // Condition variable for timed sleep (see sleep_for)
  std::condition_variable interrupt_condition_variable_;
  // 用于保护全局条件变量的互斥锁
  // Mutex for protecting the global condition variable
  std::mutex interrupt_mutex_;

  // 保持对全局弱上下文向量的共享所有权
  // Keep shared ownership of global vector of weak contexts
  std::shared_ptr<WeakContextsWrapper> weak_contexts_;

  enum class ShutdownType { pre_shutdown, on_shutdown };

  using ShutdownCallback = ShutdownCallbackHandle::ShutdownCallbackType;

  // 添加 shutdown 回调的模板函数
  // Template function to add shutdown callback
  template <ShutdownType shutdown_type>
  RCLCPP_LOCAL ShutdownCallbackHandle add_shutdown_callback(ShutdownCallback callback);

  // 删除 shutdown 回调的模板函数
  // Template function to remove shutdown callback
  template <ShutdownType shutdown_type>
  RCLCPP_LOCAL bool remove_shutdown_callback(const ShutdownCallbackHandle &callback_handle);

  // 获取 shutdown 回调的模板函数
  // Template function to get shutdown callback
  template <ShutdownType shutdown_type>
  RCLCPP_LOCAL std::vector<rclcpp::Context::ShutdownCallback> get_shutdown_callback() const;
};

/// 返回一个上下文共享指针列表的副本 (Return a copy of the list of context shared pointers).
/**
 * 这个函数是线程安全的 (This function is thread-safe).
 */
RCLCPP_PUBLIC
std::vector<Context::SharedPtr> get_contexts();

}  // namespace rclcpp

#endif  // RCLCPP__CONTEXT_HPP_

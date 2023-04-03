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

#ifndef RCLCPP__QOS_EVENT_HPP_
#define RCLCPP__QOS_EVENT_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rcl/error_handling.h"
#include "rcl/event_callback.h"
#include "rclcpp/detail/cpp_callback_trampoline.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/waitable.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/impl/cpp/demangle.hpp"
#include "rmw/incompatible_qos_events_statuses.h"

namespace rclcpp {

// 使用 rmw 中定义的请求截止时间丢失状态类型
using QOSDeadlineRequestedInfo = rmw_requested_deadline_missed_status_t;
// 使用 rmw 中定义的提供截止时间丢失状态类型
using QOSDeadlineOfferedInfo = rmw_offered_deadline_missed_status_t;
// 使用 rmw 中定义的活跃度变化状态类型
using QOSLivelinessChangedInfo = rmw_liveliness_changed_status_t;
// 使用 rmw 中定义的失去活跃度状态类型
using QOSLivelinessLostInfo = rmw_liveliness_lost_status_t;
// 使用 rmw 中定义的消息丢失状态类型
using QOSMessageLostInfo = rmw_message_lost_status_t;
// 使用 rmw 中定义的提供的不兼容 QoS 事件状态类型
using QOSOfferedIncompatibleQoSInfo = rmw_offered_qos_incompatible_event_status_t;
// 使用 rmw 中定义的请求的不兼容 QoS 事件状态类型
using QOSRequestedIncompatibleQoSInfo = rmw_requested_qos_incompatible_event_status_t;

// 定义请求截止时间回调函数类型
using QOSDeadlineRequestedCallbackType = std::function<void(QOSDeadlineRequestedInfo &)>;
// 定义提供截止时间回调函数类型
using QOSDeadlineOfferedCallbackType = std::function<void(QOSDeadlineOfferedInfo &)>;
// 定义活跃度变化回调函数类型
using QOSLivelinessChangedCallbackType = std::function<void(QOSLivelinessChangedInfo &)>;
// 定义失去活跃度回调函数类型
using QOSLivelinessLostCallbackType = std::function<void(QOSLivelinessLostInfo &)>;
// 定义消息丢失回调函数类型
using QOSMessageLostCallbackType = std::function<void(QOSMessageLostInfo &)>;
// 定义提供的不兼容 QoS 回调函数类型
using QOSOfferedIncompatibleQoSCallbackType = std::function<void(QOSOfferedIncompatibleQoSInfo &)>;
// 定义请求的不兼容 QoS 回调函数类型
using QOSRequestedIncompatibleQoSCallbackType =
    std::function<void(QOSRequestedIncompatibleQoSInfo &)>;

/**
 * @brief 包含 Publisher 从中间件接收的各种类型事件的回调。
 * Contains callbacks for various types of events a Publisher can receive from the middleware.
 */
struct PublisherEventCallbacks {
  /// 质量保证期限提供回调
  /// Quality of Service (QoS) deadline offered callback
  QOSDeadlineOfferedCallbackType deadline_callback;
  /// 活跃性丢失回调
  /// Liveliness lost callback
  QOSLivelinessLostCallbackType liveliness_callback;
  /// 提供的不兼容 QoS 回调
  /// Offered incompatible Quality of Service (QoS) callback
  QOSOfferedIncompatibleQoSCallbackType incompatible_qos_callback;
};

/**
 * @brief 包含订阅者从中间件接收的非消息事件的回调。
 * Contains callbacks for non-message events that a Subscription can receive from the middleware.
 */
struct SubscriptionEventCallbacks {
  /// 质量保证期限请求回调
  /// Quality of Service (QoS) deadline requested callback
  QOSDeadlineRequestedCallbackType deadline_callback;
  /// 活跃性更改回调
  /// Liveliness changed callback
  QOSLivelinessChangedCallbackType liveliness_callback;
  /// 请求的不兼容 QoS 回调
  /// Requested incompatible Quality of Service (QoS) callback
  QOSRequestedIncompatibleQoSCallbackType incompatible_qos_callback;
  /// 消息丢失回调
  /// Message lost callback
  QOSMessageLostCallbackType message_lost_callback;
};

/**
 * @class UnsupportedEventTypeException
 * @brief 自定义异常类，用于处理不支持的事件类型错误 (A custom exception class for handling
 * unsupported event type errors)
 *
 * 继承自 `exceptions::RCLErrorBase` 和 `std::runtime_error`
 * (Inherits from `exceptions::RCLErrorBase` and `std::runtime_error`)
 */
class UnsupportedEventTypeException : public exceptions::RCLErrorBase, public std::runtime_error {
public:
  /**
   * @brief 构造函数 (Constructor)
   *
   * @param ret rcl_ret_t 类型的返回值，表示 ROS2 函数调用的结果 (The return value of type
   * rcl_ret_t, representing the result of the ROS2 function call)
   * @param error_state 指向 rcl_error_state_t 结构体的指针，包含有关错误状态的信息 (Pointer to the
   * rcl_error_state_t structure, containing information about the error state)
   * @param prefix 异常消息前缀，用于在异常消息中添加上下文信息 (Exception message prefix, used to
   * add context information in the exception message)
   */
  RCLCPP_PUBLIC
  UnsupportedEventTypeException(
      rcl_ret_t ret, const rcl_error_state_t *error_state, const std::string &prefix);

  /**
   * @brief 构造函数 (Constructor)
   *
   * @param base_exc exceptions::RCLErrorBase 类型的引用，用于从基类异常中初始化当前异常 (Reference
   * of type exceptions::RCLErrorBase, used to initialize the current exception from the base class
   * exception)
   * @param prefix 异常消息前缀，用于在异常消息中添加上下文信息 (Exception message prefix, used to
   * add context information in the exception message)
   */
  RCLCPP_PUBLIC
  UnsupportedEventTypeException(
      const exceptions::RCLErrorBase &base_exc, const std::string &prefix);
};

/**
 * @class QOSEventHandlerBase
 * @brief A base class for handling Quality of Service (QoS) events.
 */
class QOSEventHandlerBase : public Waitable {
public:
  /**
   * @brief EntityType 枚举类型定义了 QoS 事件处理器的实体类型。
   * @brief The EntityType enumeration defines the entity types for the QoS event handler.
   */
  enum class EntityType : std::size_t {
    Event,  ///< 事件类型。The event type.
  };

  /// 析构函数。Destructor.
  RCLCPP_PUBLIC
  virtual ~QOSEventHandlerBase();

  /**
   * @brief 获取准备好的事件的数量。
   * @brief Get the number of ready events.
   *
   * @return 准备好的事件的数量。The number of ready events.
   */
  RCLCPP_PUBLIC
  size_t get_number_of_ready_events() override;

  /**
   * @brief 将 Waitable 添加到等待集中。
   * @brief Add the Waitable to a wait set.
   *
   * @param[in] wait_set 等待集指针。A pointer to the wait set.
   */
  RCLCPP_PUBLIC
  void add_to_wait_set(rcl_wait_set_t *wait_set) override;

  /**
   * @brief 检查 Waitable 是否准备好。
   * @brief Check if the Waitable is ready.
   *
   * @param[in] wait_set 等待集指针。A pointer to the wait set.
   * @return 如果 Waitable 准备好，则返回 true；否则，返回 false。True if the Waitable is ready,
   * false otherwise.
   */
  RCLCPP_PUBLIC
  bool is_ready(rcl_wait_set_t *wait_set) override;

  /// 设置一个回调函数，当每个新事件实例发生时调用。
  /// Set a callback to be called when each new event instance occurs.
  /**
   * 回调接收一个 size_t 类型，表示自上次调用此回调以来发生的事件数量。
   * The callback receives a size_t which is the number of events that occurred
   * since the last time this callback was called.
   * 通常为1，但如果在设置任何回调之前发生事件，则可以 > 1。
   * Normally this is 1, but can be > 1 if events occurred before any
   * callback was set.
   *
   * 回调还接收一个 int 标识符参数。
   * The callback also receives an int identifier argument.
   * 这是必需的，因为 Waitable 可能由几个不同的实体组成，
   * This is needed because a Waitable may be composed of several distinct entities,
   * 如订阅、服务等。
   * such as subscriptions, services, etc.
   * 应用程序应提供一个通用的回调函数，然后由 waitable 转发给所有实体。
   * The application should provide a generic callback function that will be then
   * forwarded by the waitable to all of its entities.
   * 在转发之前，标识符参数的不同值将绑定到该函数。
   * Before forwarding, a different value for the identifier argument will be
   * bond to the function.
   * 这意味着所提供的回调可以使用标识符根据触发 waitable 变为就绪的实体来表现不同。
   * This implies that the provided callback can use the identifier to behave
   * differently depending on which entity triggered the waitable to become ready.
   *
   * 由于此回调是从中间件调用的，因此您应该尽量使其快速且不阻塞。
   * Since this callback is called from the middleware, you should aim to make
   * it fast and not blocking.
   * 如果需要执行大量工作或等待其他事件，应将其分发到另一个线程，否则可能会阻塞中间件。
   * If you need to do a lot of work or wait for some other event, you should
   * spin it off to another thread, otherwise you risk blocking the middleware.
   *
   * 再次调用它将清除之前设置的任何回调。
   * Calling it again will clear any previously set callback.
   *
   * 如果回调不可调用，将抛出异常。
   * An exception will be thrown if the callback is not callable.
   *
   * 此函数是线程安全的。
   * This function is thread-safe.
   *
   * 如果希望在回调中提供更多信息，如 qos 事件或其他信息，可以使用带捕获的 lambda 或 std::bind。
   * If you want more information available in the callback, like the qos event
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \sa rmw_event_set_callback
   * \sa rcl_event_set_callback
   *
   * \param[in] callback 当新事件发生时要调用的函数对象
   * \param[in] callback functor to be called when a new event occurs
   */
  void set_on_ready_callback(std::function<void(size_t, int)> callback) override {
    // 如果回调不可调用，抛出异常。
    // Throw an exception if the callback is not callable.
    if (!callback) {
      throw std::invalid_argument(
          "The callback passed to set_on_ready_callback "
          "is not callable.");
    }

    // 注意：我们将 int 标识符参数绑定到此 waitable 的实体类型
    // Note: we bind the int identifier argument to this waitable's entity types
    auto new_callback = [callback, this](size_t number_of_events) {
      try {
        // 调用回调函数，并传入事件数量和实体类型
        // Call the callback function with the number of events and entity type
        callback(number_of_events, static_cast<int>(EntityType::Event));
      } catch (const std::exception &exception) {
        // 如果捕获到异常，记录错误信息
        // Log an error message if an exception is caught
        RCLCPP_ERROR_STREAM(
            // TODO(wjwwood): 获取与其关联的节点日志记录器的访问权限
            // TODO(wjwwood): get this class access to the node logger it is associated with
            rclcpp::get_logger("rclcpp"),
            "rclcpp::QOSEventHandlerBase@"
                << this << " caught " << rmw::impl::cpp::demangle(exception)
                << " exception in user-provided callback for the 'on ready' callback: "
                << exception.what());
      } catch (...) {
        // 如果捕获到未处理的异常，记录错误信息
        // Log an error message if an unhandled exception is caught
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("rclcpp"),
            "rclcpp::QOSEventHandlerBase@"
                << this << " caught unhandled exception in user-provided callback "
                << "for the 'on ready' callback");
      }
    };

    // 创建互斥锁，确保线程安全
    // Create a lock guard to ensure thread safety
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    // 临时设置新回调，同时替换旧回调。
    // Set it temporarily to the new callback, while we replace the old one.
    // 这种两步设置可以防止在替换旧的 std::function 时出现间隙，
    // This two-step setting, prevents a gap where the old std::function has
    // 但中间件尚未了解新的 std::function。
    // been replaced but the middleware hasn't been told about the new one yet.
    set_on_new_event_callback(
        rclcpp::detail::cpp_callback_trampoline<decltype(new_callback), const void *, size_t>,
        static_cast<const void *>(&new_callback));

    // 存储 std::function 以保持其范围内有效，也覆盖现有的 std::function。
    // Store the std::function to keep it in scope, also overwrites the existing one.
    on_new_event_callback_ = new_callback;

    // 再次设置它，现在使用永久存储。
    // Set it again, now using the permanent storage.
    set_on_new_event_callback(
        rclcpp::detail::cpp_callback_trampoline<
            decltype(on_new_event_callback_), const void *, size_t>,
        static_cast<const void *>(&on_new_event_callback_));
  }

  /**
   * @brief 清除已注册的新事件回调（如果有）。(Clear the callback registered for new events, if
   * any.)
   */
  void clear_on_ready_callback() override {
    // 使用 std::lock_guard 对象保护 callback_mutex_，避免多线程问题。
    // (Use a std::lock_guard object to protect callback_mutex_ from multithreading issues.)
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    // 检查是否已设置 on_new_event_callback_。
    // (Check if on_new_event_callback_ is set.)
    if (on_new_event_callback_) {
      // 如果已设置 on_new_event_callback_，则将其清除并设置为 nullptr。
      // (If on_new_event_callback_ is set, clear it and set it to nullptr.)
      set_on_new_event_callback(nullptr, nullptr);
      on_new_event_callback_ = nullptr;
    }
  }

protected:
  /**
   * @brief 设置新事件回调函数
   * @param callback 新事件回调函数，当有新事件发生时会被调用
   * @param user_data 用户数据，将传递给回调函数
   *
   * @brief Set the new event callback function
   * @param callback The new event callback function, which will be called when a new event occurs
   * @param user_data User data that will be passed to the callback function
   */
  RCLCPP_PUBLIC
  void set_on_new_event_callback(rcl_event_callback_t callback, const void *user_data);

  // 事件句柄，用于处理和存储与事件相关的信息
  // Event handle, used for handling and storing information related to events
  rcl_event_t event_handle_;

  // 等待集事件索引，用于确定事件在等待集中的位置
  // Wait set event index, used to determine the position of the event in the wait set
  size_t wait_set_event_index_;

  // 回调互斥锁，用于确保线程安全地访问回调函数
  // Callback mutex, used to ensure thread-safe access to the callback function
  std::recursive_mutex callback_mutex_;

  // 新事件回调函数，当有新事件发生时会被调用。默认值为 nullptr，表示没有设置回调函数。
  // New event callback function, which will be called when a new event occurs. Default value is
  // nullptr, meaning no callback function is set.
  std::function<void(size_t)> on_new_event_callback_{nullptr};
};

/**
 * @brief 用于处理 QoS 事件的类 (Class for handling QoS events)
 *
 * @tparam EventCallbackT 事件回调类型 (Event callback type)
 * @tparam ParentHandleT 父句柄类型 (Parent handle type)
 */
template <typename EventCallbackT, typename ParentHandleT>
class QOSEventHandler : public QOSEventHandlerBase {
public:
  /**
   * @brief 构造函数 (Constructor)
   *
   * @param callback 事件回调 (Event callback)
   * @param init_func 初始化函数 (Initialization function)
   * @param parent_handle 父句柄 (Parent handle)
   * @param event_type 事件类型 (Event type)
   *
   * @tparam InitFuncT 初始化函数类型 (Initialization function type)
   * @tparam EventTypeEnum 事件类型枚举 (Event type enumeration)
   */
  template <typename InitFuncT, typename EventTypeEnum>
  QOSEventHandler(
      const EventCallbackT &callback,
      InitFuncT init_func,
      ParentHandleT parent_handle,
      EventTypeEnum event_type)
      : parent_handle_(parent_handle), event_callback_(callback) {
    // 初始化事件句柄 (Initialize the event handle)
    event_handle_ = rcl_get_zero_initialized_event();
    // 调用传入的初始化函数 (Call the passed initialization function)
    rcl_ret_t ret = init_func(&event_handle_, parent_handle.get(), event_type);
    // 检查返回值 (Check return value)
    if (ret != RCL_RET_OK) {
      if (ret == RCL_RET_UNSUPPORTED) {
        UnsupportedEventTypeException exc(ret, rcl_get_error_state(), "Failed to initialize event");
        rcl_reset_error();
        throw exc;
      } else {
        rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to initialize event");
      }
    }
  }

  /**
   * @brief 获取数据，以便回调不能再次被调度 (Take data so that the callback cannot be scheduled
   * again)
   *
   * @return std::shared_ptr<void> 数据指针 (Data pointer)
   */
  std::shared_ptr<void> take_data() override {
    EventCallbackInfoT callback_info;
    // 从事件句柄中获取数据 (Take data from the event handle)
    rcl_ret_t ret = rcl_take_event(&event_handle_, &callback_info);
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED("rclcpp", "Couldn't take event info: %s", rcl_get_error_string().str);
      return nullptr;
    }
    return std::static_pointer_cast<void>(std::make_shared<EventCallbackInfoT>(callback_info));
  }

  /**
   * @brief 根据实体 ID 获取数据 (Take data by entity ID)
   *
   * @param id 实体 ID (Entity ID)
   * @return std::shared_ptr<void> 数据指针 (Data pointer)
   */
  std::shared_ptr<void> take_data_by_entity_id(size_t id) override {
    (void)id;
    return take_data();
  }

  /**
   * @brief 执行就绪的 Waitable 实体 (Execute any entities of the Waitable that are ready)
   *
   * @param data 数据指针 (Data pointer)
   */
  void execute(std::shared_ptr<void> &data) override {
    if (!data) {
      throw std::runtime_error("'data' is empty");
    }
    // 将数据转换为回调信息指针 (Convert data to callback info pointer)
    auto callback_ptr = std::static_pointer_cast<EventCallbackInfoT>(data);
    // 调用事件回调 (Call the event callback)
    event_callback_(*callback_ptr);
    // 重置回调指针 (Reset the callback pointer)
    callback_ptr.reset();
  }

private:
  // 定义事件回调信息类型 (Define the event callback info type)
  using EventCallbackInfoT =
      typename std::remove_reference<typename rclcpp::function_traits::function_traits<
          EventCallbackT>::template argument_type<0>>::type;

  ParentHandleT parent_handle_;    ///< 父句柄 (Parent handle)
  EventCallbackT event_callback_;  ///< 事件回调 (Event callback)
};

}  // namespace rclcpp

#endif  // RCLCPP__QOS_EVENT_HPP_

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

#ifndef RCLCPP__SERVICE_HPP_
#define RCLCPP__SERVICE_HPP_

#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "rcl/error_handling.h"
#include "rcl/event_callback.h"
#include "rcl/service.h"
#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/detail/cpp_callback_trampoline.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/demangle.hpp"
#include "rmw/rmw.h"
#include "tracetools/tracetools.h"

namespace rclcpp {

/// ServiceBase 类定义
/**
 * \class ServiceBase
 *
 * 这个类是 ROS2 服务的基础类，它提供了一些通用的方法和属性。
 */
class ServiceBase {
public:
  /// 智能指针定义，禁止拷贝
  /**
   * 使用智能指针来管理 ServiceBase 的生命周期，并防止拷贝。
   */
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ServiceBase)

  /// 构造函数
  /**
   * \param[in] node_handle 一个共享指针，指向 rcl_node_t 类型的节点句柄。
   */
  RCLCPP_PUBLIC
  explicit ServiceBase(std::shared_ptr<rcl_node_t> node_handle);

  /// 虚析构函数
  /**
   * 默认虚析构函数。
   */
  RCLCPP_PUBLIC
  virtual ~ServiceBase() = default;

  /// 返回服务的名称
  /**
   * \return 服务的名称。
   */
  RCLCPP_PUBLIC
  const char *get_service_name();

  /// 以 std::shared_ptr 形式返回 rcl_service_t 服务句柄
  /**
   * 即使在 Service 被销毁后，此句柄仍然有效。
   * 实际的 rcl 服务在所有范围内都不会被最终确定。
   */
  RCLCPP_PUBLIC
  std::shared_ptr<rcl_service_t> get_service_handle();

  /// 以 std::shared_ptr 形式返回 rcl_service_t 服务句柄（常量版本）
  /**
   * 即使在 Service 被销毁后，此句柄仍然有效。
   * 实际的 rcl 服务在所有范围内都不会被最终确定。
   */
  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_service_t> get_service_handle() const;

  /// 从服务中获取下一个请求（类型擦除指针）
  /**
   * 这个类型擦除版本的 \sa Service::take_request() 在使用方法时非常有用，
   * 如 ServiceBase::create_request()，ServiceBase::create_request_header() 和
   * ServiceBase::handle_request()。
   *
   * \param[out] request_out 中间件将复制已获取请求的类型擦除指针到服务请求对象。
   * \param[out] request_id_out 输出请求的 ID，以便将来将响应与此请求关联。
   * \returns 如果请求被接收，则返回 true，否则返回 false。
   * \throws 如果底层 rcl 调用失败，则抛出 rclcpp::exceptions::RCLError 基本异常。
   */
  RCLCPP_PUBLIC
  bool take_type_erased_request(void *request_out, rmw_request_id_t &request_id_out);

  /// 创建请求
  virtual std::shared_ptr<void> create_request() = 0;

  /// 创建请求头
  virtual std::shared_ptr<rmw_request_id_t> create_request_header() = 0;

  /// 处理请求
  virtual void handle_request(
      std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request) = 0;

  /// 交换此服务的 "wait set 使用中" 状态
  /**
   * 这用于确保此服务不会同时被多个 wait sets 使用。
   *
   * \param[in] in_use_state 新状态，用于交换到状态，true 表示现在由 wait set 使用，
   * false 表示不再由 wait set 使用。
   * \returns 之前的状态。
   */
  RCLCPP_PUBLIC
  bool exchange_in_use_by_wait_set_state(bool in_use_state);

  /// 获取实际响应发布器 QoS 设置，在确定默认值后
  /**
   * 当使用 RMW_QOS_POLICY_*_SYSTEM_DEFAULT 时，实际应用的配置只能在创建服务后解析，
   * 并且取决于底层 rmw 实现。
   * 如果正在使用的底层设置无法用 ROS 术语表示，则将其设置为 RMW_QOS_POLICY_*_UNKNOWN。
   * 当发生意外错误时可能抛出运行时错误。
   *
   * \return 实际响应发布器 qos 设置。
   * \throws 如果获取 qos 设置失败，则抛出 std::runtime_error。
   */
  RCLCPP_PUBLIC
  rclcpp::QoS get_response_publisher_actual_qos() const;

  /// 获取实际请求订阅 QoS 设置，在确定默认值后
  /**
   * 当使用 RMW_QOS_POLICY_*_SYSTEM_DEFAULT 时，实际应用的配置只能在创建服务后解析，
   * 并且取决于底层 rmw 实现。
   * 如果正在使用的底层设置无法用 ROS 术语表示，则将其设置为 RMW_QOS_POLICY_*_UNKNOWN。
   * 当发生意外错误时可能抛出运行时错误。
   *
   * \return 实际请求订阅 qos 设置。
   * \throws 如果获取 qos 设置失败，则抛出 std::runtime_error。
   */
  RCLCPP_PUBLIC
  rclcpp::QoS get_request_subscription_actual_qos() const;

  /// 设置当接收到新请求时调用的回调
  /**
   * 回调接收一个 size_t，它是自上次调用此回调以来接收到的请求数。
   * 通常这是 1，但如果在设置任何回调之前接收到请求，则可以 > 1。
   *
   * 由于此回调是从中间件调用的，因此您应该尽量使其快速且不阻塞。
   * 如果需要执行大量工作或等待其他事件，您应该将其分配给另一个线程，否则您可能会阻塞中间件。
   *
   * 再次调用将清除先前设置的任何回调。
   *
   * 如果回调不可调用，将抛出异常。
   *
   * 此函数是线程安全的。
   *
   * 如果您希望在回调中使用更多信息，如服务或其他信息，您可以使用捕获的 lambda 或 std::bind。
   *
   * \sa rmw_service_set_on_new_request_callback
   * \sa rcl_service_set_on_new_request_callback
   *
   * \param[in] callback 当接收到新请求时调用的回调函数。
   */
  void set_on_new_request_callback(std::function<void(size_t)> callback) {
    if (!callback) {
      throw std::invalid_argument(
          "The callback passed to set_on_new_request_callback "
          "is not callable.");
    }

    auto new_callback = [callback, this](size_t number_of_requests) {
      try {
        callback(number_of_requests);
      } catch (const std::exception &exception) {
        RCLCPP_ERROR_STREAM(
            node_logger_,
            "rclcpp::ServiceBase@"
                << this << " caught " << rmw::impl::cpp::demangle(exception)
                << " exception in user-provided callback for the 'on new request' callback: "
                << exception.what());
      } catch (...) {
        RCLCPP_ERROR_STREAM(
            node_logger_, "rclcpp::ServiceBase@"
                              << this << " caught unhandled exception in user-provided callback "
                              << "for the 'on new request' callback");
      }
    };

    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    // Set it temporarily to the new callback, while we replace the old one.
    // This two-step setting, prevents a gap where the old std::function has
    // been replaced but the middleware hasn't been told about the new one yet.
    set_on_new_request_callback(
        rclcpp::detail::cpp_callback_trampoline<decltype(new_callback), const void *, size_t>,
        static_cast<const void *>(&new_callback));

    // Store the std::function to keep it in scope, also overwrites the existing one.
    on_new_request_callback_ = new_callback;

    // Set it again, now using the permanent storage.
    set_on_new_request_callback(
        rclcpp::detail::cpp_callback_trampoline<
            decltype(on_new_request_callback_), const void *, size_t>,
        static_cast<const void *>(&on_new_request_callback_));
  }

  /// 取消注册新请求的回调（如果有）
  void clear_on_new_request_callback() {
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    if (on_new_request_callback_) {
      set_on_new_request_callback(nullptr, nullptr);
      on_new_request_callback_ = nullptr;
    }
  }

protected:
  /// 禁止拷贝
  RCLCPP_DISABLE_COPY(ServiceBase)

  /// 获取 rcl_node_t 节点句柄
  RCLCPP_PUBLIC
  rcl_node_t *get_rcl_node_handle();

  /// 获取 rcl_node_t 节点句柄（常量版本）
  RCLCPP_PUBLIC
  const rcl_node_t *get_rcl_node_handle() const;

  /// 设置新请求的回调
  RCLCPP_PUBLIC
  void set_on_new_request_callback(rcl_event_callback_t callback, const void *user_data);

  // 成员变量
  std::shared_ptr<rcl_node_t> node_handle_;

  std::shared_ptr<rcl_service_t> service_handle_;
  bool owns_rcl_handle_ = true;

  rclcpp::Logger node_logger_;

  std::atomic<bool> in_use_by_wait_set_{false};

  std::recursive_mutex callback_mutex_;
  std::function<void(size_t)> on_new_request_callback_{nullptr};
};

/*!
 * \brief 服务模板类 (Service template class)
 * \tparam ServiceT 服务类型 (Service type)
 */
template <typename ServiceT>
class Service : public ServiceBase, public std::enable_shared_from_this<Service<ServiceT>> {
public:
  /*!
   * \brief 回调类型 (Callback type)
   */
  using CallbackType = std::function<void(
      const std::shared_ptr<typename ServiceT::Request>,
      std::shared_ptr<typename ServiceT::Response>)>;

  /*!
   * \brief 带头部信息的回调类型 (Callback type with header)
   */
  using CallbackWithHeaderType = std::function<void(
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<typename ServiceT::Request>,
      std::shared_ptr<typename ServiceT::Response>)>;
  RCLCPP_SMART_PTR_DEFINITIONS(Service)

  /// 默认构造函数 (Default constructor)
  /**
   * 构造一个Service的实例，通常不会直接调用此构造函数。
   * 服务应通过rclcpp::create_service()函数实例化。
   *
   * The constructor for a Service is almost never called directly.
   * Instead, services should be instantiated through the function
   * rclcpp::create_service().
   *
   * \param[in] node_handle 用于设置的NodeBaseInterface指针 (NodeBaseInterface pointer that is used
   * in part of the setup) \param[in] service_name 要发布到的主题名称 (Name of the topic to publish
   * to) \param[in] any_callback 收到客户端请求时调用的用户定义回调 (User defined callback to call
   * when a client request is received) \param[in] service_options 订阅选项 (options for the
   * subscription)
   */
  Service(
      std::shared_ptr<rcl_node_t> node_handle,
      const std::string &service_name,
      AnyServiceCallback<ServiceT> any_callback,
      rcl_service_options_t &service_options)
      : ServiceBase(node_handle), any_callback_(any_callback) {
    // 获取服务类型支持句柄 (Get the service type support handle)
    using rosidl_typesupport_cpp::get_service_type_support_handle;
    auto service_type_support_handle = get_service_type_support_handle<ServiceT>();

    // 在这里，rcl进行静态内存分配 (rcl does the static memory allocation here)
    service_handle_ = std::shared_ptr<rcl_service_t>(
        new rcl_service_t, [handle = node_handle_, service_name](rcl_service_t *service) {
          if (rcl_service_fini(service, handle.get()) != RCL_RET_OK) {
            RCLCPP_ERROR(
                rclcpp::get_node_logger(handle.get()).get_child("rclcpp"),
                "Error in destruction of rcl service handle: %s", rcl_get_error_string().str);
            rcl_reset_error();
          }
          delete service;
        });
    *service_handle_.get() = rcl_get_zero_initialized_service();

    // 初始化服务 (Initialize the service)
    rcl_ret_t ret = rcl_service_init(
        service_handle_.get(), node_handle.get(), service_type_support_handle, service_name.c_str(),
        &service_options);
    if (ret != RCL_RET_OK) {
      if (ret == RCL_RET_SERVICE_NAME_INVALID) {
        auto rcl_node_handle = get_rcl_node_handle();
        // 这将在任何验证问题上抛出异常 (this will throw on any validation problem)
        rcl_reset_error();
        expand_topic_or_service_name(
            service_name, rcl_node_get_name(rcl_node_handle),
            rcl_node_get_namespace(rcl_node_handle), true);
      }

      rclcpp::exceptions::throw_from_rcl_error(ret, "could not create service");
    }
    TRACEPOINT(
        rclcpp_service_callback_added, static_cast<const void *>(get_service_handle().get()),
        static_cast<const void *>(&any_callback_));
#ifndef TRACETOOLS_DISABLED
    any_callback_.register_callback_for_tracing();
#endif
  }

  /// 默认构造函数 (Default constructor)
  /**
   * 构造一个Service的实例，通常不会直接调用此构造函数。
   * 服务应通过rclcpp::create_service()函数实例化。
   *
   * The constructor for a Service is almost never called directly.
   * Instead, services should be instantiated through the function
   * rclcpp::create_service().
   *
   * \param[in] node_handle 用于设置的NodeBaseInterface指针 (NodeBaseInterface pointer that is used
   * in part of the setup) \param[in] service_handle 服务句柄 (service handle) \param[in]
   * any_callback 收到客户端请求时调用的用户定义回调 (User defined callback to call when a client
   * request is received)
   */
  Service(
      std::shared_ptr<rcl_node_t> node_handle,
      std::shared_ptr<rcl_service_t> service_handle,
      AnyServiceCallback<ServiceT> any_callback)
      : ServiceBase(node_handle), any_callback_(any_callback) {
    // 检查服务句柄是否已初始化 (Check if service handle was initialized)
    if (!rcl_service_is_valid(service_handle.get())) {
      // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
      throw std::runtime_error(
          std::string("rcl_service_t in constructor argument must be initialized beforehand."));
      // *INDENT-ON*
    }

    service_handle_ = service_handle;
    TRACEPOINT(
        rclcpp_service_callback_added, static_cast<const void *>(get_service_handle().get()),
        static_cast<const void *>(&any_callback_));
#ifndef TRACETOOLS_DISABLED
    any_callback_.register_callback_for_tracing();
#endif
  }

  /// 默认构造函数 (Default constructor)
  /**
   * 构造一个Service的实例，通常不会直接调用此构造函数。
   * 服务应通过rclcpp::create_service()函数实例化。
   *
   * The constructor for a Service is almost never called directly.
   * Instead, services should be instantiated through the function
   * rclcpp::create_service().
   *
   * \param[in] node_handle 用于设置的NodeBaseInterface指针 (NodeBaseInterface pointer that is used
   * in part of the setup) \param[in] service_handle 服务句柄 (service handle) \param[in]
   * any_callback 收到客户端请求时调用的用户定义回调 (User defined callback to call when a client
   * request is received)
   */
  Service(
      std::shared_ptr<rcl_node_t> node_handle,
      rcl_service_t *service_handle,
      AnyServiceCallback<ServiceT> any_callback)
      : ServiceBase(node_handle), any_callback_(any_callback) {
    // 检查服务句柄是否已初始化 (Check if service handle was initialized)
    if (!rcl_service_is_valid(service_handle)) {
      // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
      throw std::runtime_error(
          std::string("rcl_service_t in constructor argument must be initialized beforehand."));
      // *INDENT-ON*
    }

    // 在这种情况下，rcl拥有服务句柄内存 (In this case, rcl owns the service handle memory)
    service_handle_ = std::shared_ptr<rcl_service_t>(new rcl_service_t);
    service_handle_->impl = service_handle->impl;
    TRACEPOINT(
        rclcpp_service_callback_added, static_cast<const void *>(get_service_handle().get()),
        static_cast<const void *>(&any_callback_));
#ifndef TRACETOOLS_DISABLED
    any_callback_.register_callback_for_tracing();
#endif
  }

  Service() = delete;

  virtual ~Service() {}

  /// 从服务中获取下一个请求 (Take the next request from the service)
  /**
   * \sa ServiceBase::take_type_erased_request().
   *
   * \param[out] request_out 服务请求对象的引用，中间件将复制已获取的请求 (The reference to a
   * service request object into which the middleware will copy the taken request) \param[out]
   * request_id_out 可以在未来将响应与此请求关联的输出请求ID (The output id for the request which
   * can be used to associate response with this request in the future) \returns
   * 如果请求被接收，则返回true，否则返回false (true if the request was taken, otherwise false)
   * \throws rclcpp::exceptions::RCLError 如果底层的rcl调用失败 (based exceptions if the underlying
   * rcl calls fail)
   */
  bool take_request(typename ServiceT::Request &request_out, rmw_request_id_t &request_id_out) {
    return this->take_type_erased_request(&request_out, request_id_out);
  }

  /**
   * @brief 创建一个服务请求对象
   * @return 返回一个指向服务请求对象的共享指针
   *
   * @brief Create a service request object
   * @return Return a shared pointer to the service request object
   */
  std::shared_ptr<void> create_request() override {
    // 创建一个 ServiceT 类型的请求对象，并返回其共享指针
    // Create a request object of type ServiceT and return its shared pointer
    return std::make_shared<typename ServiceT::Request>();
  }

  /**
   * @brief 创建一个服务请求头对象
   * @return 返回一个指向服务请求头对象的共享指针
   *
   * @brief Create a service request header object
   * @return Return a shared pointer to the service request header object
   */
  std::shared_ptr<rmw_request_id_t> create_request_header() override {
    // 创建一个 rmw_request_id_t 类型的请求头对象，并返回其共享指针
    // Create a request header object of type rmw_request_id_t and return its shared pointer
    return std::make_shared<rmw_request_id_t>();
  }

  /**
   * @brief 处理服务请求
   * @param request_header 指向服务请求头对象的共享指针
   * @param request 指向服务请求对象的共享指针
   *
   * @brief Handle a service request
   * @param request_header Shared pointer to the service request header object
   * @param request Shared pointer to the service request object
   */
  void handle_request(
      std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request) override {
    // 将 void 类型的请求共享指针转换为 ServiceT 类型的请求共享指针
    // Convert the void type request shared pointer to a ServiceT type request shared pointer
    auto typed_request = std::static_pointer_cast<typename ServiceT::Request>(request);

    // 调用回调函数处理请求，并获取响应
    // Call the callback function to handle the request and get the response
    auto response = any_callback_.dispatch(this->shared_from_this(), request_header, typed_request);

    // 如果存在响应，发送响应
    // If there is a response, send the response
    if (response) {
      send_response(*request_header, *response);
    }
  }

  /**
   * @brief 发送服务响应
   * @param req_id 服务请求头对象的引用
   * @param response 服务响应对象的引用
   *
   * @brief Send service response
   * @param req_id Reference to the service request header object
   * @param response Reference to the service response object
   */
  void send_response(rmw_request_id_t &req_id, typename ServiceT::Response &response) {
    // 使用 rcl_send_response 函数发送响应
    // Use the rcl_send_response function to send the response
    rcl_ret_t ret = rcl_send_response(get_service_handle().get(), &req_id, &response);

    // 如果发送失败，抛出异常
    // If sending fails, throw an exception
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send response");
    }
  }

private:
  RCLCPP_DISABLE_COPY(Service)

  AnyServiceCallback<ServiceT> any_callback_;
};

}  // namespace rclcpp

#endif  // RCLCPP__SERVICE_HPP_

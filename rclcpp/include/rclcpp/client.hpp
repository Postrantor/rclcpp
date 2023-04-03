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

#ifndef RCLCPP__CLIENT_HPP_
#define RCLCPP__CLIENT_HPP_

#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <optional>  // NOLINT, cpplint doesn't think this is a cpp std header
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>  // NOLINT
#include <vector>

#include "rcl/client.h"
#include "rcl/error_handling.h"
#include "rcl/event_callback.h"
#include "rcl/wait.h"
#include "rclcpp/detail/cpp_callback_trampoline.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/demangle.hpp"
#include "rmw/rmw.h"

namespace rclcpp {

namespace detail {
/**
 * @brief 结构体用于存储 std::future 对象和请求 ID (Structure for storing std::future object and
 * request ID)
 *
 * @tparam FutureT 一个 std::future 类型的模板参数 (A template parameter of type std::future)
 */
template <typename FutureT>
struct FutureAndRequestId {
  /// 存储 std::future 对象 (Store the std::future object)
  FutureT future;
  /// 存储请求 ID (Store the request ID)
  int64_t request_id;

  /**
   * @brief 构造函数 (Constructor)
   *
   * @param impl std::future 对象 (std::future object)
   * @param req_id 请求 ID (Request ID)
   */
  FutureAndRequestId(FutureT impl, int64_t req_id) : future(std::move(impl)), request_id(req_id) {}

  /// 允许隐式转换为 `std::future` 引用 (Allow implicit conversions to `std::future` by reference)
  operator FutureT &() { return this->future; }

  /// 已弃用，使用 `future` 成员变量代替 (Deprecated, use the `future` member variable instead)
  /**
   * 允许隐式转换为 `std::future` 值 (Allow implicit conversions to `std::future` by value)
   * \deprecated
   */
  [[deprecated("FutureAndRequestId: use .future instead of an implicit conversion")]]
  operator FutureT() {
    return this->future;
  }

  // 委托 std::future 实现类似的方法 (Delegate future like methods in the std::future impl_)

  /// 参见 std::future::get() (See std::future::get())
  auto get() { return this->future.get(); }
  /// 参见 std::future::valid() (See std::future::valid())
  bool valid() const noexcept { return this->future.valid(); }
  /// 参见 std::future::wait() (See std::future::wait())
  void wait() const { return this->future.wait(); }
  /// 参见 std::future::wait_for() (See std::future::wait_for())
  template <class Rep, class Period>
  std::future_status wait_for(const std::chrono::duration<Rep, Period> &timeout_duration) const {
    return this->future.wait_for(timeout_duration);
  }
  /// 参见 std::future::wait_until() (See std::future::wait_until())
  template <class Clock, class Duration>
  std::future_status wait_until(
      const std::chrono::time_point<Clock, Duration> &timeout_time) const {
    return this->future.wait_until(timeout_time);
  }

  // 五法则，我们可以在这里使用零法则，但最好明确一些，因为其中一些方法已被删除 (Rule of five, we
  // could use the rule of zero here, but better be explicit as some of the methods are deleted)

  /// 移动构造函数 (Move constructor)
  FutureAndRequestId(FutureAndRequestId &&other) noexcept = default;
  /// 已删除的复制构造函数，每个实例都是 future 的唯一所有者 (Deleted copy constructor, each
  /// instance is a unique owner of the future)
  FutureAndRequestId(const FutureAndRequestId &other) = delete;
  /// 移动赋值 (Move assignment)
  FutureAndRequestId &operator=(FutureAndRequestId &&other) noexcept = default;
  /// 已删除的复制赋值，每个实例都是 future 的唯一所有者 (Deleted copy assignment, each instance is
  /// a unique owner of the future)
  FutureAndRequestId &operator=(const FutureAndRequestId &other) = delete;
  /// 析构函数 (Destructor)
  ~FutureAndRequestId() = default;
};

}  // namespace detail

namespace node_interfaces {
class NodeBaseInterface;
}  // namespace node_interfaces

class ClientBase {
public:
  // RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ClientBase) 定义了智能指针类型，但不可复制。
  // Define smart pointer types for ClientBase, but not copyable.
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ClientBase)

  // 构造函数：ClientBase
  // Constructor: ClientBase
  // 参数：
  // - node_base: 一个指向 rclcpp::node_interfaces::NodeBaseInterface
  // 类型的指针，用于访问节点基本功能。
  // - node_graph: 一个指向 rclcpp::node_interfaces::NodeGraphInterface
  // 类型的共享指针，用于访问节点图形功能。 Parameters:
  // - node_base: A pointer to an rclcpp::node_interfaces::NodeBaseInterface type, used for
  // accessing node basic functionalities.
  // - node_graph: A shared pointer to an rclcpp::node_interfaces::NodeGraphInterface type, used for
  // accessing node graph functionalities.
  RCLCPP_PUBLIC
  ClientBase(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph);

  // 虚拟析构函数：~ClientBase
  // Virtual Destructor: ~ClientBase
  // 使用默认实现，当派生类对象被删除时，允许适当地调用基类的析构函数。
  // Use the default implementation, which allows the base class destructor to be called
  // appropriately when a derived class object is deleted.
  RCLCPP_PUBLIC
  virtual ~ClientBase() = default;

  /// 以类型擦除指针的形式获取此客户端的下一个响应。
  /// Take the next response for this client as a type erased pointer.
  /**
   * 类型擦除指针允许此方法与 ClientBase::create_response()、
   * ClientBase::create_request_header() 和 ClientBase::handle_response()
   * 一起以类型不可知的方式使用。 The type erased pointer allows for this method to be used in a
   * type agnostic way along with ClientBase::create_response(),
   * ClientBase::create_request_header(), and ClientBase::handle_response().
   * 如果已知 Service 类型，可以使用此类的类型版本，
   * \sa Client::take_response().
   *
   * \param[out] response_out 中间件将要复制的响应的类型擦除指针到服务响应中。
   * \param[out] response_out The type erased pointer to a Service Response into
   *   which the middleware will copy the response being taken.
   * \param[out] request_header_out 当取用时由中间件填充的请求头，可用于将响应关联到特定请求。
   * \param[out] request_header_out The request header to be filled by the
   *   middleware when taking, and which can be used to associte the response
   *   to a specific request.
   * \returns 如果响应被获取，则返回 true，否则返回 false。
   * \returns true if the response was taken, otherwise false.
   * \throws rclcpp::exceptions::RCLError 如果底层 rcl 函数失败，则基于异常。
   * \throws rclcpp::exceptions::RCLError based exceptions if the underlying
   *   rcl function fail.
   */
  RCLCPP_PUBLIC
  bool take_type_erased_response(void *response_out, rmw_request_id_t &request_header_out);

  /// 返回服务的名称。
  /// Return the name of the service.
  /** \return 服务的名称。 */
  /** \return The name of the service. */
  RCLCPP_PUBLIC
  const char *get_service_name() const;

  /// 返回一个 std::shared_ptr 中的 rcl_client_t 客户端句柄。
  /// Return the rcl_client_t client handle in a std::shared_ptr.
  /**
   * 当 Client 被销毁后，此句柄仍然有效。
   * This handle remains valid after the Client is destroyed.
   * 实际的 rcl 客户端在所有范围内都没有最终确定之前不会被最终确定。
   * The actual rcl client is not finalized until it is out of scope everywhere.
   */
  RCLCPP_PUBLIC
  std::shared_ptr<rcl_client_t> get_client_handle();

  /// 返回一个 std::shared_ptr 中的 rcl_client_t 客户端句柄。
  /// Return the rcl_client_t client handle in a std::shared_ptr.
  /**
   * 当 Client 被销毁后，此句柄仍然有效。
   * This handle remains valid after the Client is destroyed.
   * 实际的 rcl 客户端在所有范围内都没有最终确定之前不会被最终确定。
   * The actual rcl client is not finalized until it is out of scope everywhere.
   */
  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_client_t> get_client_handle() const;

  /// 判断服务是否准备好。
  /// Return if the service is ready.
  /**
   * \return 如果服务准备好返回 `true`，否则返回 `false`
   * \return `true` if the service is ready, `false` otherwise
   */
  RCLCPP_PUBLIC
  bool service_is_ready() const;

  /// 等待服务准备好。
  /// Wait for a service to be ready.
  /**
   * \param timeout 最长等待时间
   * \param timeout maximum time to wait
   * \return 如果服务准备好且超时未到返回 `true`，否则返回 `false`
   * \return `true` if the service is ready and the timeout is not over, `false` otherwise
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    // 将 timeout 转换为纳秒并调用 wait_for_service_nanoseconds 函数。
    // Convert the timeout to nanoseconds and call the wait_for_service_nanoseconds function.
    return wait_for_service_nanoseconds(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  /**
   * @brief 创建响应对象 (Create a response object)
   *
   * @return std::shared_ptr<void> 返回一个共享指针，指向新创建的响应对象 (Return a shared pointer
   * pointing to the newly created response object)
   */
  virtual std::shared_ptr<void> create_response() = 0;

  /**
   * @brief 创建请求头对象 (Create a request header object)
   *
   * @return std::shared_ptr<rmw_request_id_t> 返回一个共享指针，指向新创建的请求头对象 (Return a
   * shared pointer pointing to the newly created request header object)
   */
  virtual std::shared_ptr<rmw_request_id_t> create_request_header() = 0;

  /**
   * @brief 处理收到的响应 (Handle received response)
   *
   * @param request_header 请求头共享指针，包含请求的元数据 (Shared pointer of request header,
   * containing metadata of the request)
   * @param response 响应对象共享指针，包含实际的响应数据 (Shared pointer of response object,
   * containing the actual response data)
   */
  virtual void handle_response(
      std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) = 0;

  /// 交换此客户端的 "正在被等待集使用" 状态。
  /// Exchange the "in use by wait set" state for this client.
  /**
   * 此函数用于确保此客户端不会同时被多个等待集使用。
   * This is used to ensure this client is not used by multiple
   * wait sets at the same time.
   *
   * \param[in] in_use_state 要与状态交换的新状态，true 表示现在由等待集使用，false
   * 表示不再由等待集使用。 \param[in] in_use_state the new state to exchange into the state, true
   *   indicates it is now in use by a wait set, and false is that it is no
   *   longer in use by a wait set.
   * \returns 返回先前的状态。
   * \returns the previous state.
   */
  RCLCPP_PUBLIC
  bool exchange_in_use_by_wait_set_state(bool in_use_state);

  /// 获取实际请求发布器 QoS 设置，在确定默认值后。
  /// Get the actual request publsher QoS settings, after the defaults have been determined.
  /**
   * 当使用 RMW_QOS_POLICY_*_SYSTEM_DEFAULT 时，只有在创建客户端之后才能解析实际应用的配置，
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the client, and it
   * depends on the underlying rmw implementation.
   * 如果正在使用的底层设置无法用 ROS 术语表示，则将其设置为 RMW_QOS_POLICY_*_UNKNOWN。
   * If the underlying setting in use can't be represented in ROS terms,
   * it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * 当出现意外错误时可能会抛出运行时错误。
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return 返回实际请求发布器 qos 设置。
   * \return The actual request publsher qos settings.
   * \throws std::runtime_error 如果获取 qos 设置失败
   * \throws std::runtime_error if failed to get qos settings
   */
  RCLCPP_PUBLIC
  rclcpp::QoS get_request_publisher_actual_qos() const;

  /// 获取实际响应订阅 QoS 设置，在确定默认值后。
  /// Get the actual response subscription QoS settings, after the defaults have been determined.
  /**
   * 当使用 RMW_QOS_POLICY_*_SYSTEM_DEFAULT 时，只有在创建客户端之后才能解析实际应用的配置，
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the client, and it
   * depends on the underlying rmw implementation.
   * 如果正在使用的底层设置无法用 ROS 术语表示，则将其设置为 RMW_QOS_POLICY_*_UNKNOWN。
   * If the underlying setting in use can't be represented in ROS terms,
   * it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * 当出现意外错误时可能会抛出运行时错误。
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return 返回实际响应订阅 qos 设置。
   * \return The actual response subscription qos settings.
   * \throws std::runtime_error 如果获取 qos 设置失败
   * \throws std::runtime_error if failed to get qos settings
   */
  RCLCPP_PUBLIC
  rclcpp::QoS get_response_subscription_actual_qos() const;

  /// 设置一个回调函数，当收到每个新响应时调用。
  /// Set a callback to be called when each new response is received.
  /**
   * 回调接收一个 size_t 类型参数，表示自上次调用此回调以来收到的响应数量。
   * The callback receives a size_t which is the number of responses received
   * since the last time this callback was called.
   * 通常这个值为1，但如果在设置任何回调之前就已经收到了响应，则可能大于1。
   * Normally this is 1, but can be > 1 if responses were received before any
   * callback was set.
   *
   * 由于此回调是从中间件调用的，因此您应该尽量使其快速且不阻塞。
   * Since this callback is called from the middleware, you should aim to make
   * it fast and not blocking.
   * 如果需要执行大量工作或等待其他事件，应将其分配给另一个线程，否则可能会阻塞中间件。
   * If you need to do a lot of work or wait for some other event, you should
   * spin it off to another thread, otherwise you risk blocking the middleware.
   *
   * 再次调用它将清除先前设置的任何回调。
   * Calling it again will clear any previously set callback.
   *
   * 如果回调不可调用，将抛出异常。
   * An exception will be thrown if the callback is not callable.
   *
   * 此函数是线程安全的。
   * This function is thread-safe.
   *
   * 如果希望在回调中获取更多信息，如客户端或其他信息，可以使用带捕获的 lambda 或 std::bind。
   * If you want more information available in the callback, like the client
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \sa rmw_client_set_on_new_response_callback
   * \sa rcl_client_set_on_new_response_callback
   *
   * \param[in] callback 当收到新响应时调用的函数对象
   * \param[in] callback functor to be called when a new response is received
   */
  void set_on_new_response_callback(std::function<void(size_t)> callback) {
    // 如果回调不可调用，抛出异常。
    // Throw an exception if the callback is not callable.
    if (!callback) {
      throw std::invalid_argument(
          "The callback passed to set_on_new_response_callback "
          "is not callable.");
    }

    // 创建一个新的回调，捕获传入的回调和 this 指针。
    // Create a new callback, capturing the incoming callback and this pointer.
    auto new_callback =
        [callback, this](size_t number_of_responses) {
          try {
            // 调用传入的回调。
            // Call the incoming callback.
            callback(number_of_responses);
          } catch (const std::exception &exception) {
            // 捕获异常并记录错误。
            // Catch the exception and log the error.
            RCLCPP_ERROR_STREAM(
                node_logger_,
                "rclcpp::ClientBase@"
                    << this << " caught " << rmw::impl::cpp::demangle(exception)
                    << " exception in user-provided callback for the 'on new response' callback: "
                    << exception.what());
          } catch (...) {
            // 捕获未处理的异常并记录错误。
            // Catch unhandled exceptions and log the error.
            RCLCPP_ERROR_STREAM(
                node_logger_, "rclcpp::ClientBase@"
                                  << this
                                  << " caught unhandled exception in user-provided callback "
                                  << "for the 'on new response' callback");
          }
        };

    // 锁定回调互斥锁，确保线程安全。
    // Lock the callback mutex to ensure thread safety.
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    // 将新回调临时设置为新回调，同时替换旧回调。
    // Set it temporarily to the new callback, while we replace the old one.
    // 这种两步设置方式，可以防止在旧 std::function 被替换但中间件尚未被告知新回调的情况下出现空白。
    // This two-step setting, prevents a gap where the old std::function has
    // been replaced but the middleware hasn't been told about the new one yet.
    set_on_new_response_callback(
        rclcpp::detail::cpp_callback_trampoline<decltype(new_callback), const void *, size_t>,
        static_cast<const void *>(&new_callback));

    // 存储 std::function 以保持其作用域，还会覆盖现有的回调。
    // Store the std::function to keep it in scope, also overwrites the existing one.
    on_new_response_callback_ = new_callback;

    // 使用永久存储再次设置回调。
    // Set the callback again, now using the permanent storage.
    set_on_new_response_callback(
        rclcpp::detail::cpp_callback_trampoline<
            decltype(on_new_response_callback_), const void *, size_t>,
        static_cast<const void *>(&on_new_response_callback_));
  }

  /// \brief 取消注册新响应的回调函数（如果有的话）。
  /// \brief Unset the callback registered for new responses, if any.
  void clear_on_new_response_callback() {
    // 使用 std::lock_guard 对象保护 callback_mutex_，在作用域结束时自动解锁
    // Use std::lock_guard to protect callback_mutex_, automatically unlocking when going out of
    // scope
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    // 如果 on_new_response_callback_ 不为空，则取消注册回调函数
    // If on_new_response_callback_ is not null, unregister the callback function
    if (on_new_response_callback_) {
      // 将 on_new_response_callback_ 设置为 nullptr，表示没有回调函数
      // Set on_new_response_callback_ to nullptr, indicating no callback function
      set_on_new_response_callback(nullptr, nullptr);

      // 更新 on_new_response_callback_ 的值为 nullptr
      // Update the value of on_new_response_callback_ to nullptr
      on_new_response_callback_ = nullptr;
    }
  }

protected:
  // 禁用 ClientBase 的拷贝构造函数和赋值运算符
  // Disable the copy constructor and assignment operator for ClientBase
  RCLCPP_DISABLE_COPY(ClientBase)

  // 声明一个公共方法，等待服务的响应，参数为超时时间（纳秒）
  // Declare a public method to wait for service response with a timeout in nanoseconds
  RCLCPP_PUBLIC
  bool wait_for_service_nanoseconds(std::chrono::nanoseconds timeout);

  // 声明一个公共方法，获取 rcl_node_t 类型的节点句柄
  // Declare a public method to get the rcl_node_t type node handle
  RCLCPP_PUBLIC
  rcl_node_t *get_rcl_node_handle();

  // 声明一个公共方法，获取 const rcl_node_t 类型的节点句柄
  // Declare a public method to get the const rcl_node_t type node handle
  RCLCPP_PUBLIC
  const rcl_node_t *get_rcl_node_handle() const;

  // 声明一个公共方法，设置新响应回调函数，参数为回调函数和用户数据
  // Declare a public method to set the callback for new responses, with parameters being the
  // callback function and user data
  RCLCPP_PUBLIC
  void set_on_new_response_callback(rcl_event_callback_t callback, const void *user_data);

  // 定义一个弱指针类型的 NodeGraphInterface 对象
  // Define a weak pointer type NodeGraphInterface object
  rclcpp::node_interfaces::NodeGraphInterface::WeakPtr node_graph_;

  // 定义一个共享指针类型的 rcl_node_t 对象
  // Define a shared pointer type rcl_node_t object
  std::shared_ptr<rcl_node_t> node_handle_;

  // 定义一个共享指针类型的 rclcpp::Context 对象
  // Define a shared pointer type rclcpp::Context object
  std::shared_ptr<rclcpp::Context> context_;

  // 定义一个 rclcpp::Logger 类型的节点日志对象
  // Define a rclcpp::Logger type node logger object
  rclcpp::Logger node_logger_;

  // 定义一个共享指针类型的 rcl_client_t 对象
  // Define a shared pointer type rcl_client_t object
  std::shared_ptr<rcl_client_t> client_handle_;

  // 定义一个原子布尔类型变量，表示是否被 wait_set 使用
  // Define an atomic boolean variable to indicate if it's in use by a wait_set
  std::atomic<bool> in_use_by_wait_set_{false};

  // 定义一个递归互斥锁用于保护回调函数
  // Define a recursive mutex for protecting the callback function
  std::recursive_mutex callback_mutex_;

  // 定义一个 std::function 类型的新响应回调函数，默认为空
  // Define a std::function type new response callback function, defaulting to nullptr
  std::function<void(size_t)> on_new_response_callback_{nullptr};
};

/**
 * @brief Client 类用于与 ROS2 服务进行交互 (The Client class is used to interact with ROS2
 * services)
 *
 * @tparam ServiceT 服务类型 (Service type)
 */
template <typename ServiceT>
class Client : public ClientBase {
public:
  // 定义请求和响应类型 (Define request and response types)
  using Request = typename ServiceT::Request;    ///< 请求类型 (Request type)
  using Response = typename ServiceT::Response;  ///< 响应类型 (Response type)

  // 定义共享请求和响应类型 (Define shared request and response types)
  using SharedRequest =
      typename ServiceT::Request::SharedPtr;   ///< 共享请求类型 (Shared request type)
  using SharedResponse =
      typename ServiceT::Response::SharedPtr;  ///< 共享响应类型 (Shared response type)

  // 定义承诺类型 (Define promise types)
  using Promise = std::promise<SharedResponse>;                ///< 承诺类型 (Promise type)
  using PromiseWithRequest =
      std::promise<std::pair<SharedRequest, SharedResponse>>;  ///< 带请求的承诺类型 (Promise type
                                                               ///< with request)

  // 定义共享承诺类型 (Define shared promise types)
  using SharedPromise = std::shared_ptr<Promise>;  ///< 共享承诺类型 (Shared promise type)
  using SharedPromiseWithRequest =
      std::shared_ptr<PromiseWithRequest>;  ///< 共享带请求的承诺类型 (Shared promise type with
                                            ///< request)

  // 定义 future 类型 (Define future types)
  using Future = std::future<SharedResponse>;  ///< Future 类型 (Future type)
  using SharedFuture =
      std::shared_future<SharedResponse>;      ///< 共享 Future 类型 (Shared future type)
  using SharedFutureWithRequest = std::shared_future<std::pair<
      SharedRequest,
      SharedResponse>>;  ///< 带请求的共享 Future 类型 (Shared future type with request)

  // 定义回调类型 (Define callback types)
  using CallbackType = std::function<void(SharedFuture)>;  ///< 回调类型 (Callback type)
  using CallbackWithRequestType = std::function<void(
      SharedFutureWithRequest)>;        ///< 带请求的回调类型 (Callback type with request)

  RCLCPP_SMART_PTR_DEFINITIONS(Client)  ///< 智能指针定义 (Smart pointer definitions)

  /// 一个方便的 Client::Future 和请求 id 对组合。
  /// A convenient Client::Future and request id pair.
  /**
   * 公共成员：
   * - future: 一个 std::future<SharedResponse> 类型的变量。
   * - request_id: 与 future 关联的请求 id。
   *
   * Public members:
   * - future: a std::future<SharedResponse>.
   * - request_id: the request id associated with the future.
   *
   * 所有其他方法都等同于 std::future 提供的方法。
   * All the other methods are equivalent to the ones std::future provides.
   */
  struct FutureAndRequestId : detail::FutureAndRequestId<std::future<SharedResponse>> {
    // 使用 detail::FutureAndRequestId<std::future<SharedResponse>> 的构造函数。
    // Using the constructor of detail::FutureAndRequestId<std::future<SharedResponse>>.
    using detail::FutureAndRequestId<std::future<SharedResponse>>::FutureAndRequestId;

    /// 已弃用，改用 `.future.share()`。
    /// Deprecated, use `.future.share()` instead.
    /**
     * 允许通过值进行隐式转换为 `std::shared_future`。
     * Allow implicit conversions to `std::shared_future` by value.
     * \deprecated
     */
    [[deprecated("FutureAndRequestId: use .future.share() instead of an implicit conversion")]]
    operator SharedFuture() {
      // 返回 this->future.share() 的结果。
      // Return the result of this->future.share().
      return this->future.share();
    }

    // 在 std::future impl_ 中委托类似 future 的方法。
    // Delegate future-like methods in the std::future impl_.

    /// 查看 std::future::share()。
    /// See std::future::share().
    SharedFuture share() noexcept { return this->future.share(); }
  };

  /// 方便的 Client::SharedFuture 和请求 ID 配对。
  /// A convenient Client::SharedFuture and request id pair.
  /**
   * 公共成员：
   * - future: 一个 std::shared_future<SharedResponse> 类型。
   * - request_id: 与 future 关联的请求 ID。
   *
   * 所有其他方法都等同于 std::shared_future 提供的方法。
   * Public members:
   * - future: a std::shared_future<SharedResponse>.
   * - request_id: the request id associated with the future.
   *
   * All the other methods are equivalent to the ones std::shared_future provides.
   */
  struct SharedFutureAndRequestId : detail::FutureAndRequestId<std::shared_future<SharedResponse>> {
    // 使用 detail::FutureAndRequestId<std::shared_future<SharedResponse>> 的构造函数。
    // Using the constructor of detail::FutureAndRequestId<std::shared_future<SharedResponse>>.
    using detail::FutureAndRequestId<std::shared_future<SharedResponse>>::FutureAndRequestId;
  };

  /// 方便的 Client::SharedFutureWithRequest 和请求 ID 配对。
  /// A convenient Client::SharedFutureWithRequest and request id pair.
  /**
   * 公共成员：
   * - future: 一个 std::shared_future<SharedResponse> 类型。
   * - request_id: 与 future 关联的请求 ID。
   *
   * 所有其他方法都等同于 std::shared_future 提供的方法。
   * Public members:
   * - future: a std::shared_future<SharedResponse>.
   * - request_id: the request id associated with the future.
   *
   * All the other methods are equivalent to the ones std::shared_future provides.
   */
  struct SharedFutureWithRequestAndRequestId
      : detail::FutureAndRequestId<std::shared_future<std::pair<SharedRequest, SharedResponse>>> {
    // 使用 detail::FutureAndRequestId<std::shared_future<std::pair<SharedRequest, SharedResponse>>>
    // 的构造函数。 Using the constructor of
    // detail::FutureAndRequestId<std::shared_future<std::pair<SharedRequest, SharedResponse>>>.
    using detail::FutureAndRequestId<
        std::shared_future<std::pair<SharedRequest, SharedResponse>>>::FutureAndRequestId;
  };

  /// 默认构造函数 (Default constructor).
  /**
   * 客户端的构造函数几乎从不直接调用。 (The constructor for a Client is almost never called
   * directly.) 相反，客户端应通过函数 rclcpp::create_client() 实例化。 (Instead, clients should be
   * instantiated through the function rclcpp::create_client().)
   *
   * \param[in] node_base 用于部分设置的 NodeBaseInterface 指针。 (NodeBaseInterface pointer that is
   * used in part of the setup.) \param[in] node_graph 相应节点的节点图接口。 (The node graph
   * interface of the corresponding node.) \param[in] service_name 要发布到的主题的名称。 (Name of
   * the topic to publish to.) \param[in] client_options 订阅选项。 (options for the subscription.)
   */
  Client(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
      const std::string &service_name,
      rcl_client_options_t &client_options)
      : ClientBase(
            node_base,
            node_graph)  // 使用 node_base 和 node_graph 初始化 ClientBase 类。 (Initialize
                         // ClientBase class with node_base and node_graph.)
  {
    // 使用 rosidl_typesupport_cpp 获取服务类型支持句柄。 (Get the service type support handle using
    // rosidl_typesupport_cpp.)
    using rosidl_typesupport_cpp::get_service_type_support_handle;
    auto service_type_support_handle = get_service_type_support_handle<ServiceT>();

    // 使用提供的参数初始化 rcl 客户端。 (Initialize the rcl client with the provided arguments.)
    rcl_ret_t ret = rcl_client_init(
        this->get_client_handle().get(), this->get_rcl_node_handle(), service_type_support_handle,
        service_name.c_str(), &client_options);

    // 检查 rcl 客户端初始化是否成功。 (Check if the rcl client initialization was successful.)
    if (ret != RCL_RET_OK) {
      // 如果服务名称无效，尝试扩展主题或服务名称。 (If the service name is invalid, try to expand
      // the topic or service name.)
      if (ret == RCL_RET_SERVICE_NAME_INVALID) {
        auto rcl_node_handle = this->get_rcl_node_handle();
        // 这将在任何验证问题上抛出异常。 (This will throw on any validation problem.)
        rcl_reset_error();
        expand_topic_or_service_name(
            service_name, rcl_node_get_name(rcl_node_handle),
            rcl_node_get_namespace(rcl_node_handle), true);
      }
      // 如果 rcl 客户端创建失败，抛出异常。 (Throw an exception if the rcl client creation failed.)
      rclcpp::exceptions::throw_from_rcl_error(ret, "could not create client");
    }
  }

  // 虚拟析构函数
  // Virtual destructor
  virtual ~Client() {}

  /// 获取此客户端的下一个响应。
  /// Take the next response for this client.
  /**
   * \sa ClientBase::take_type_erased_response().
   *
   * \param[out] response_out 中间件将要复制的服务响应的引用。
   * \param[out] response_out The reference to a Service Response into
   *   which the middleware will copy the response being taken.
   * \param[out] request_header_out 当获取时由中间件填充的请求头，
   *   可用于将响应关联到特定请求。
   * \param[out] request_header_out The request header to be filled by the
   *   middleware when taking, and which can be used to associte the response
   *   to a specific request.
   * \returns 如果响应被获取，则为 true，否则为 false。
   * \returns true if the response was taken, otherwise false.
   * \throws rclcpp::exceptions::RCLError 如果底层 rcl 函数失败，则基于异常。
   * \throws rclcpp::exceptions::RCLError based exceptions if the underlying
   *   rcl function fail.
   */
  bool take_response(
      typename ServiceT::Response &response_out, rmw_request_id_t &request_header_out) {
    // 调用 take_type_erased_response 方法并返回结果
    // Call the take_type_erased_response method and return the result
    return this->take_type_erased_response(&response_out, request_header_out);
  }

  /// 使用响应类型创建一个共享指针
  /// Create a shared pointer with the response type
  /**
   * \return 带有响应类型的共享指针
   * \return shared pointer with the response type
   */
  std::shared_ptr<void> create_response() override {
    // 使用响应类型实例化一个新的共享指针并返回
    // Instantiate a new shared pointer with the response type and return it
    return std::shared_ptr<void>(new typename ServiceT::Response());
  }

  /// 创建一个带有 rmw_request_id_t 的共享指针
  /// Create a shared pointer with a rmw_request_id_t
  /**
   * \return 带有 rmw_request_id_t 的共享指针
   * \return shared pointer with a rmw_request_id_t
   */
  std::shared_ptr<rmw_request_id_t> create_request_header() override {
    // TODO(wjwwood): 这可能应该使用 rmw_request_id 的分配器。
    //                (因为它是 C 类型)
    // TODO(wjwwood): This should probably use rmw_request_id's allocator.
    //                (since it is a C type)

    // 使用 rmw_request_id_t 实例化一个新的共享指针并返回
    // Instantiate a new shared pointer with rmw_request_id_t and return it
    return std::shared_ptr<rmw_request_id_t>(new rmw_request_id_t);
  }

  /// 处理服务器响应 (Handle a server response)
  /**
   * \param[in] request_header 用于检查序列号是否有效 (used to check if the sequence number is
   * valid) \param[in] response 带有服务器响应的消息 (message with the server response)
   */
  void handle_response(
      std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) override {
    // 获取并删除具有给定序列号的待处理请求，如果没有找到，则返回空值
    // (Get and erase the pending request with the given sequence number, return empty value if not
    // found)
    std::optional<CallbackInfoVariant> optional_pending_request =
        this->get_and_erase_pending_request(request_header->sequence_number);

    // 如果没有找到待处理请求，直接返回 (If no pending request is found, return directly)
    if (!optional_pending_request) {
      return;
    }

    // 获取待处理请求的值引用 (Get the reference of the value of the pending request)
    auto &value = *optional_pending_request;

    // 将响应转换为指定服务类型的响应 (Cast the response to the specified service type's response)
    auto typed_response =
        std::static_pointer_cast<typename ServiceT::Response>(std::move(response));

    // 如果值包含 Promise 类型 (If the value holds a Promise type)
    if (std::holds_alternative<Promise>(value)) {
      auto &promise = std::get<Promise>(value);
      // 设置 promise 的值为已转换的响应 (Set the value of the promise to the casted response)
      promise.set_value(std::move(typed_response));
    }
    // 如果值包含 CallbackTypeValueVariant 类型 (If the value holds a CallbackTypeValueVariant type)
    else if (std::holds_alternative<CallbackTypeValueVariant>(value)) {
      auto &inner = std::get<CallbackTypeValueVariant>(value);
      const auto &callback = std::get<CallbackType>(inner);
      auto &promise = std::get<Promise>(inner);
      auto &future = std::get<SharedFuture>(inner);

      // 设置 promise 的值为已转换的响应 (Set the value of the promise to the casted response)
      promise.set_value(std::move(typed_response));
      // 使用已移动的 future 调用回调函数 (Invoke the callback with the moved future)
      callback(std::move(future));
    }
    // 如果值包含 CallbackWithRequestTypeValueVariant 类型 (If the value holds a
    // CallbackWithRequestTypeValueVariant type)
    else if (std::holds_alternative<CallbackWithRequestTypeValueVariant>(value)) {
      auto &inner = std::get<CallbackWithRequestTypeValueVariant>(value);
      const auto &callback = std::get<CallbackWithRequestType>(inner);
      auto &promise = std::get<PromiseWithRequest>(inner);
      auto &future = std::get<SharedFutureWithRequest>(inner);
      auto &request = std::get<SharedRequest>(inner);

      // 设置 promise 的值为已移动的请求和已转换的响应组成的 pair (Set the value of the promise as a
      // pair of the moved request and the casted response)
      promise.set_value(std::make_pair(std::move(request), std::move(typed_response)));
      // 使用已移动的 future 调用回调函数 (Invoke the callback with the moved future)
      callback(std::move(future));
    }
  }

  /// 发送请求到服务端 (Send a request to the service server)
  /**
   * 此方法返回一个 `FutureAndRequestId` 实例
   * (This method returns a `FutureAndRequestId` instance)
   * 该实例可以传递给 Executor::spin_until_future_complete() 来
   * (that can be passed to Executor::spin_until_future_complete() to)
   * 等待其完成。
   * (wait until it has been completed.)
   *
   * 如果未来永远不会完成，
   * (If the future never completes,)
   * 例如，调用 Executor::spin_until_future_complete() 超时，
   * (e.g. the call to Executor::spin_until_future_complete() times out,)
   * 必须调用 Client::remove_pending_request() 清理客户端内部状态。
   * (Client::remove_pending_request() must be called to clean the client internal state.)
   * 不这样做将使得 `Client` 实例在每次没有收到服务端响应时使用更多的内存。
   * (Not doing so will make the `Client` instance to use more memory each time a response is not)
   * (received from the service server.)
   *
   * ```cpp
   * auto future = client->async_send_request(my_request);
   * if (
   *   rclcpp::FutureReturnCode::TIMEOUT ==
   *   executor->spin_until_future_complete(future, timeout))
   * {
   *   client->remove_pending_request(future);
   *   // 处理超时 (handle timeout)
   * } else {
   *   handle_response(future.get());
   * }
   * ```
   *
   * \param[in] request 要发送的请求 (request to be send)
   * \return 一个 FutureAndRequestId 实例 (a FutureAndRequestId instance)
   */
  FutureAndRequestId async_send_request(SharedRequest request) {
    // 创建一个 Promise 对象 (Create a Promise object)
    Promise promise;
    // 获取 future 对象 (Get the future object)
    auto future = promise.get_future();
    // 调用 async_send_request_impl 实现并传递请求和 promise 对象 (Call async_send_request_impl
    // implementation and pass the request and promise objects)
    auto req_id = async_send_request_impl(*request, std::move(promise));
    // 返回 FutureAndRequestId 对象，包含 future 对象和请求 ID (Return FutureAndRequestId object
    // containing the future object and request ID)
    return FutureAndRequestId(std::move(future), req_id);
  }

  /// 发送请求到服务端并在执行器中安排回调。
  /// Send a request to the service server and schedule a callback in the executor.
  /**
   * 类似于之前的重载，但是当收到响应时会自动调用回调。
   * Similar to the previous overload, but a callback will automatically be called when a response
   * is received.
   *
   * 如果回调从未被调用，因为我们从未从服务端收到回复，则需要使用返回的请求ID调用remove_pending_request()或prune_pending_requests()。
   * If the callback is never called, because we never got a reply for the service server,
   * remove_pending_request() has to be called with the returned request id or
   * prune_pending_requests(). 不这样做将使`Client`实例在每次未收到来自服务端的响应时使用更多内存。
   * Not doing so will make the `Client` instance use more memory each time a response is not
   * received from the service server.
   * 在这种情况下，设置一个定时器来清理挂起的请求是方便的。
   * In this case, it's convenient to setup a timer to cleanup the pending requests.
   * 例如，请参见https://github.com/ros2/examples 中的`examples_rclcpp_async_client`包。
   * See for example the `examples_rclcpp_async_client` package in https://github.com/ros2/examples.
   *
   * \param[in] request 要发送的请求。
   * \param[in] request request to be send.
   * \param[in] cb 收到此请求的响应时将调用的回调。
   * \param[in] cb callback that will be called when we get a response for this request.
   * \return 表示刚发送的请求的请求ID。
   * \return the request id representing the request just sent.
   */
  template <
      typename CallbackT,
      typename std::enable_if<
          rclcpp::function_traits::same_arguments<CallbackT, CallbackType>::value>::type * =
          nullptr>
  // 定义一个名为async_send_request的函数模板，它接受一个共享请求和一个回调，并返回一个SharedFutureAndRequestId类型的对象。
  // Define a function template named async_send_request that takes a shared request and a callback,
  // and returns an object of type SharedFutureAndRequestId.
  SharedFutureAndRequestId async_send_request(SharedRequest request, CallbackT &&cb) {
    // 创建一个名为promise的Promise对象。
    // Create a Promise object named promise.
    Promise promise;
    // 获取promise的共享future并将其赋值给shared_future。
    // Get the shared future of promise and assign it to shared_future.
    auto shared_future = promise.get_future().share();
    // 调用async_send_request_impl函数并将结果赋值给req_id。
    // Call the async_send_request_impl function and assign the result to req_id.
    auto req_id = async_send_request_impl(
        *request,
        std::make_tuple(
            // 使用cb、shared_future和移动后的promise创建一个CallbackType对象。
            // Create a CallbackType object using cb, shared_future, and the moved promise.
            CallbackType{std::forward<CallbackT>(cb)}, shared_future, std::move(promise)));
    // 返回一个包含移动后的shared_future和req_id的SharedFutureAndRequestId对象。
    // Return a SharedFutureAndRequestId object containing the moved shared_future and req_id.
    return SharedFutureAndRequestId{std::move(shared_future), req_id};
  }

  /// 发送一个请求到服务端并在执行器中安排回调。
  /// Send a request to the service server and schedule a callback in the executor.
  /**
   * 与前面的方法类似，但是在回调中可以获取请求和响应。
   * Similar to the previous method, but you can get both the request and response in the callback.
   *
   * \param[in] request 要发送的请求。
   * \param[in] request request to be send.
   * \param[in] cb 收到此请求的响应时将被调用的回调。
   * \param[in] cb callback that will be called when we get a response for this request.
   * \return 表示刚刚发送的请求的请求 ID。
   * \return the request id representing the request just sent.
   */
  template <
      typename CallbackT,
      typename std::enable_if<
          rclcpp::function_traits::same_arguments<CallbackT, CallbackWithRequestType>::value>::type
          * = nullptr>
  SharedFutureWithRequestAndRequestId async_send_request(SharedRequest request, CallbackT &&cb) {
    // 创建一个带请求的 Promise 对象。
    // Create a Promise object with a request.
    PromiseWithRequest promise;

    // 获取共享的 future 对象。
    // Get the shared future object.
    auto shared_future = promise.get_future().share();

    // 异步发送请求，并返回请求 ID。
    // Asynchronously send the request and return the request ID.
    auto req_id = async_send_request_impl(
        *request, std::make_tuple(
                      CallbackWithRequestType{std::forward<CallbackT>(cb)}, request, shared_future,
                      std::move(promise)));

    // 返回包含共享的 future 对象和请求 ID 的结构。
    // Return a structure containing the shared future object and the request ID.
    return SharedFutureWithRequestAndRequestId{std::move(shared_future), req_id};
  }

  /// 清理一个挂起的请求。
  /// Cleanup a pending request.
  /**
   * 这会通知客户端，我们已经等待了足够长的时间来接收服务器的响应，
   * 我们已经放弃，不再等待响应。
   * This notifies the client that we have waited long enough for a response from the server
   * to come, we have given up and we are not waiting for a response anymore.
   *
   * 不调用这个将使客户端开始为每个从未收到服务器回复的请求使用更多的内存。
   * Not calling this will make the client start using more memory for each request
   * that never got a reply from the server.
   *
   * \param[in] request_id 由 async_send_request() 返回的请求 ID。
   * \param[in] request_id request id returned by async_send_request().
   * \return 当挂起的请求被移除时返回 true，否则返回 false（例如，已收到响应）。
   * \return true when a pending request was removed, false if not (e.g. a response was received).
   */
  bool remove_pending_request(int64_t request_id) {
    // 使用互斥锁保护挂起的请求。
    // Protect the pending requests with a mutex lock.
    std::lock_guard guard(pending_requests_mutex_);

    // 移除指定请求 ID 的挂起请求，并返回是否成功移除。
    // Remove the pending request with the specified request ID and return whether it was
    // successfully removed.
    return pending_requests_.erase(request_id) != 0u;
  }

  /// 清理待处理的请求 (Cleanup a pending request)
  /**
   * 方便的重载，与以下相同： (Convenient overload, same as:)
   *
   * `Client::remove_pending_request(this, future.request_id)`.
   *
   * \param[in] future 包含请求ID的FutureAndRequestId对象 (A FutureAndRequestId object containing
   * the request ID) \return 如果成功删除请求，则返回true，否则返回false (Returns true if the
   * request is successfully removed, false otherwise)
   */
  bool remove_pending_request(const FutureAndRequestId &future) {
    // 调用另一个重载方法来删除请求 (Call another overloaded method to remove the request)
    return this->remove_pending_request(future.request_id);
  }

  /// 清理待处理的请求 (Cleanup a pending request)
  /**
   * 方便的重载，与以下相同： (Convenient overload, same as:)
   *
   * `Client::remove_pending_request(this, future.request_id)`.
   *
   * \param[in] future 包含请求ID的SharedFutureAndRequestId对象 (A SharedFutureAndRequestId object
   * containing the request ID) \return 如果成功删除请求，则返回true，否则返回false (Returns true if
   * the request is successfully removed, false otherwise)
   */
  bool remove_pending_request(const SharedFutureAndRequestId &future) {
    // 调用另一个重载方法来删除请求 (Call another overloaded method to remove the request)
    return this->remove_pending_request(future.request_id);
  }

  /// 清理待处理的请求 (Cleanup a pending request)
  /**
   * 方便的重载，与以下相同： (Convenient overload, same as:)
   *
   * `Client::remove_pending_request(this, future.request_id)`.
   *
   * \param[in] future 包含请求ID的SharedFutureWithRequestAndRequestId对象 (A
   * SharedFutureWithRequestAndRequestId object containing the request ID) \return
   * 如果成功删除请求，则返回true，否则返回false (Returns true if the request is successfully
   * removed, false otherwise)
   */
  bool remove_pending_request(const SharedFutureWithRequestAndRequestId &future) {
    // 调用另一个重载方法来删除请求 (Call another overloaded method to remove the request)
    return this->remove_pending_request(future.request_id);
  }

  /// 清除所有待处理的请求。
  /// Clean all pending requests.
  /**
   * \return 返回被移除的待处理请求的数量。
   * \return number of pending requests that were removed.
   */
  size_t prune_pending_requests() {
    // 使用 std::lock_guard 对 pending_requests_mutex_ 进行加锁，以保护共享资源 pending_requests_
    // Use std::lock_guard to lock pending_requests_mutex_ to protect the shared resource
    // pending_requests_
    std::lock_guard guard(pending_requests_mutex_);

    // 获取待处理请求的数量
    // Get the number of pending requests
    auto ret = pending_requests_.size();

    // 清空待处理请求列表
    // Clear the list of pending requests
    pending_requests_.clear();

    // 返回被移除的待处理请求的数量
    // Return the number of pending requests that were removed
    return ret;
  }

  /// 清除早于某个时间点的所有待处理请求。
  /// Clean all pending requests older than a time_point.
  /**
   * \param[in] time_point 将移除在此时间点之前发送的请求。
   * \param[in] time_point Requests that were sent before this point are going to be removed.
   * \param[inout] pruned_requests 如果提供了指针，则将移除的请求 id 推送到向量中。
   * \param[inout] pruned_requests Removed requests id will be pushed to the vector if a pointer is
   * provided. \return 返回被移除的待处理请求的数量。 \return number of pending requests that were
   * removed.
   */
  template <typename AllocatorT = std::allocator<int64_t>>
  size_t prune_requests_older_than(
      std::chrono::time_point<std::chrono::system_clock> time_point,
      std::vector<int64_t, AllocatorT> *pruned_requests = nullptr) {
    // 使用 std::lock_guard 对 pending_requests_mutex_ 进行加锁，以保护共享资源 pending_requests_
    // Use std::lock_guard to lock pending_requests_mutex_ to protect the shared resource
    // pending_requests_
    std::lock_guard guard(pending_requests_mutex_);

    // 获取待处理请求的原始数量
    // Get the original number of pending requests
    auto old_size = pending_requests_.size();

    // 遍历待处理请求列表
    // Iterate through the list of pending requests
    for (auto it = pending_requests_.begin(), last = pending_requests_.end(); it != last;) {
      // 如果请求早于给定的时间点
      // If the request is older than the given time_point
      if (it->second.first < time_point) {
        // 如果提供了 pruned_requests 指针，则将移除的请求 id 添加到向量中
        // If a pruned_requests pointer is provided, add the removed request id to the vector
        if (pruned_requests) {
          pruned_requests->push_back(it->first);
        }
        // 从待处理请求列表中擦除该请求
        // Erase the request from the list of pending requests
        it = pending_requests_.erase(it);
      } else {
        // 否则，继续检查下一个请求
        // Otherwise, continue checking the next request
        ++it;
      }
    }

    // 返回被移除的待处理请求的数量
    // Return the number of pending requests that were removed
    return old_size - pending_requests_.size();
  }

protected:
  // 使用 std::tuple 封装 CallbackType, SharedFuture, Promise 类型的值
  // Use std::tuple to encapsulate values of types CallbackType, SharedFuture, and Promise
  using CallbackTypeValueVariant = std::tuple<CallbackType, SharedFuture, Promise>;

  // 使用 std::tuple 封装 CallbackWithRequestType, SharedRequest, SharedFutureWithRequest,
  // PromiseWithRequest 类型的值 Use std::tuple to encapsulate values of types
  // CallbackWithRequestType, SharedRequest, SharedFutureWithRequest, and PromiseWithRequest
  using CallbackWithRequestTypeValueVariant = std::
      tuple<CallbackWithRequestType, SharedRequest, SharedFutureWithRequest, PromiseWithRequest>;

  // 定义一个 std::variant 类型，包含 std::promise<SharedResponse>, CallbackTypeValueVariant,
  // CallbackWithRequestTypeValueVariant Define a std::variant type that contains
  // std::promise<SharedResponse>, CallbackTypeValueVariant, and CallbackWithRequestTypeValueVariant
  using CallbackInfoVariant = std::variant<
      std::promise<SharedResponse>,
      CallbackTypeValueVariant,
      CallbackWithRequestTypeValueVariant>;

  /**
   * @brief 异步发送请求的实现函数
   * @param request 请求对象
   * @param value 包含回调信息的变体
   * @return 请求序列号
   *
   * @brief Implementation function for asynchronously sending requests
   * @param request The request object
   * @param value Variant containing callback information
   * @return Sequence number of the request
   */
  int64_t async_send_request_impl(const Request &request, CallbackInfoVariant value) {
    int64_t sequence_number;
    // 对 pending_requests_mutex_ 上锁以确保线程安全
    // Lock pending_requests_mutex_ to ensure thread safety
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    // 调用 rcl_send_request 发送请求并获取序列号
    // Call rcl_send_request to send the request and get the sequence number
    rcl_ret_t ret = rcl_send_request(get_client_handle().get(), &request, &sequence_number);
    if (RCL_RET_OK != ret) {
      // 如果发送失败，抛出异常
      // Throw an exception if the sending fails
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request");
    }
    // 将序列号和回调信息插入 pending_requests_ 映射中
    // Insert the sequence number and callback information into the pending_requests_ map
    pending_requests_.try_emplace(
        sequence_number, std::make_pair(std::chrono::system_clock::now(), std::move(value)));
    return sequence_number;
  }

  /**
   * @brief 获取并删除挂起的请求
   * @param request_number 请求序列号
   * @return 包含回调信息的变体（如果找到）
   *
   * @brief Get and erase a pending request
   * @param request_number Sequence number of the request
   * @return Variant containing callback information (if found)
   */
  std::optional<CallbackInfoVariant> get_and_erase_pending_request(int64_t request_number) {
    // 对 pending_requests_mutex_ 上锁以确保线程安全
    // Lock pending_requests_mutex_ to ensure thread safety
    std::unique_lock<std::mutex> lock(pending_requests_mutex_);
    // 查找给定序列号的请求
    // Find the request with the given sequence number
    auto it = this->pending_requests_.find(request_number);
    if (it == this->pending_requests_.end()) {
      // 如果找不到请求，记录一条调试信息并返回空值
      // If the request is not found, log a debug message and return an empty value
      RCUTILS_LOG_DEBUG_NAMED("rclcpp", "Received invalid sequence number. Ignoring...");
      return std::nullopt;
    }
    // 移动请求中的回调信息并从映射中删除请求
    // Move the callback information from the request and erase it from the map
    auto value = std::move(it->second.second);
    this->pending_requests_.erase(request_number);
    return value;
  }

  // 禁用 Client 类的拷贝构造函数和赋值操作符
  // Disable copy constructor and assignment operator for the Client class
  RCLCPP_DISABLE_COPY(Client)

  // 定义一个存储挂起请求的映射，键为序列号，值为一个包含时间点和回调信息变体的 pair
  // Define a map to store pending requests, with the key being the sequence number and the value
  // being a pair containing a time point and a callback info variant
  std::unordered_map<
      int64_t,
      std::pair<std::chrono::time_point<std::chrono::system_clock>, CallbackInfoVariant>>
      pending_requests_;

  // 定义一个互斥量以确保对 pending_requests_ 的访问是线程安全的
  // Define a mutex to ensure that access to pending_requests_ is thread-safe
  std::mutex pending_requests_mutex_;
};

}  // namespace rclcpp

#endif  // RCLCPP__CLIENT_HPP_

// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__ANY_SERVICE_CALLBACK_HPP_
#define RCLCPP__ANY_SERVICE_CALLBACK_HPP_

#include <functional>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

#include "rclcpp/function_traits.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/types.h"
#include "tracetools/tracetools.h"
#include "tracetools/utils.hpp"

namespace rclcpp {

namespace detail {
/**
 * @brief 一个模板结构体，用于判断类型 T 是否可以被赋值为 nullptr。
 *        A template struct for determining if type T can be assigned to nullptr.
 *
 * @tparam T 要检查的类型。The type to check.
 * @tparam Unused 未使用的模板参数，用于启用 SFINAE。Unused template parameter for enabling SFINAE.
 */
template <typename T, typename = void>
struct can_be_nullptr : std::false_type {};

// 一些 lambda 表达式定义了与 nullptr 的比较，
// 但是在使用它时，我们会看到一个警告，即它们永远不可能为 null。
// 我们还测试 `T &` 是否可以分配给 `nullptr` 以避免这个问题。
//
// Some lambdas define a comparison with nullptr,
// but we see a warning that they can never be null when using it.
// We also test if `T &` can be assigned to `nullptr` to avoid the issue.

/**
 * @brief 一个特化的模板结构体，用于判断类型 T 是否可以被赋值为 nullptr。
 *        A specialized template struct for determining if type T can be assigned to nullptr.
 *
 * @tparam T 要检查的类型。The type to check.
 */
template <typename T>
#ifdef __QNXNTO__
struct can_be_nullptr<T, std::void_t<decltype(std::declval<T>() == nullptr)>> : std::true_type {
};
#else
struct can_be_nullptr<
    T,
    std::void_t<decltype(std::declval<T>() == nullptr), decltype(std::declval<T &>() = nullptr)>>
    : std::true_type {
};
#endif
}  // namespace detail

// 前向声明 (Forward declare)
template <typename ServiceT>
class Service;

template <typename ServiceT>
class AnyServiceCallback {
public:
  // 构造函数 (Constructor)
  AnyServiceCallback() : callback_(std::monostate{}) {}

  // 设置回调函数的模板方法，处理非空回调 (Set callback template method, handling non-null
  // callbacks)
  template <
      typename CallbackT,
      typename std::enable_if_t<!detail::can_be_nullptr<CallbackT>::value, int> = 0>
  void set(CallbackT &&callback) {
    // 解决 Windows 上的 std::bind 问题 (Workaround Windows issue with std::bind)
    if constexpr (rclcpp::function_traits::same_arguments<CallbackT, SharedPtrCallback>::value) {
      callback_.template emplace<SharedPtrCallback>(callback);
    } else if constexpr (  // NOLINT, can't satisfy both cpplint and uncrustify
        rclcpp::function_traits::same_arguments<
            CallbackT, SharedPtrWithRequestHeaderCallback>::value) {
      callback_.template emplace<SharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (  // NOLINT
        rclcpp::function_traits::same_arguments<CallbackT, SharedPtrDeferResponseCallback>::value) {
      callback_.template emplace<SharedPtrDeferResponseCallback>(callback);
    } else if constexpr (  // NOLINT
        rclcpp::function_traits::same_arguments<
            CallbackT, SharedPtrDeferResponseCallbackWithServiceHandle>::value) {
      callback_.template emplace<SharedPtrDeferResponseCallbackWithServiceHandle>(callback);
    } else {
      // the else clause is not needed, but anyways we should only be doing this instead
      // of all the above workaround ...
      callback_ = std::forward<CallbackT>(callback);
    }
  }

  // 设置回调函数的模板方法，处理可为空的回调 (Set callback template method, handling nullable
  // callbacks)
  template <
      typename CallbackT,
      typename std::enable_if_t<detail::can_be_nullptr<CallbackT>::value, int> = 0>
  void set(CallbackT &&callback) {
    if (!callback) {
      throw std::invalid_argument("AnyServiceCallback::set(): callback cannot be nullptr");
    }
    // 解决 Windows 上的 std::bind 问题 (Workaround Windows issue with std::bind)
    if constexpr (rclcpp::function_traits::same_arguments<CallbackT, SharedPtrCallback>::value) {
      callback_.template emplace<SharedPtrCallback>(callback);
    } else if constexpr (  // NOLINT
        rclcpp::function_traits::same_arguments<
            CallbackT, SharedPtrWithRequestHeaderCallback>::value) {
      callback_.template emplace<SharedPtrWithRequestHeaderCallback>(callback);
    } else if constexpr (  // NOLINT
        rclcpp::function_traits::same_arguments<CallbackT, SharedPtrDeferResponseCallback>::value) {
      callback_.template emplace<SharedPtrDeferResponseCallback>(callback);
    } else if constexpr (  // NOLINT
        rclcpp::function_traits::same_arguments<
            CallbackT, SharedPtrDeferResponseCallbackWithServiceHandle>::value) {
      callback_.template emplace<SharedPtrDeferResponseCallbackWithServiceHandle>(callback);
    } else {
      // the else clause is not needed, but anyways we should only be doing this instead
      // of all the above workaround ...
      callback_ = std::forward<CallbackT>(callback);
    }
  }

  /**
   * @brief 分发请求到回调函数 (Dispatch request to the callback)
   * @param service_handle 服务句柄 (Service handle)
   * @param request_header 请求头 (Request header)
   * @param request 请求对象 (Request object)
   * @return 返回响应对象 (Return response object)
   */
  std::shared_ptr<typename ServiceT::Response> dispatch(
      const std::shared_ptr<rclcpp::Service<ServiceT>> &service_handle,
      const std::shared_ptr<rmw_request_id_t> &request_header,
      std::shared_ptr<typename ServiceT::Request> request) {
    TRACEPOINT(callback_start, static_cast<const void *>(this), false);
    if (std::holds_alternative<std::monostate>(callback_)) {
      // TODO(ivanpauno): Remove the set method, and force the users of this class
      // to pass a callback at construnciton.
      throw std::runtime_error{"unexpected request without any callback set"};
    }
    if (std::holds_alternative<SharedPtrDeferResponseCallback>(callback_)) {
      const auto &cb = std::get<SharedPtrDeferResponseCallback>(callback_);
      cb(request_header, std::move(request));
      return nullptr;
    }
    if (std::holds_alternative<SharedPtrDeferResponseCallbackWithServiceHandle>(callback_)) {
      const auto &cb = std::get<SharedPtrDeferResponseCallbackWithServiceHandle>(callback_);
      cb(service_handle, request_header, std::move(request));
      return nullptr;
    }
    // auto response = allocate_shared<typename ServiceT::Response, Allocator>();
    auto response = std::make_shared<typename ServiceT::Response>();
    if (std::holds_alternative<SharedPtrCallback>(callback_)) {
      (void)request_header;
      const auto &cb = std::get<SharedPtrCallback>(callback_);
      cb(std::move(request), response);
    } else if (std::holds_alternative<SharedPtrWithRequestHeaderCallback>(callback_)) {
      const auto &cb = std::get<SharedPtrWithRequestHeaderCallback>(callback_);
      cb(request_header, std::move(request), response);
    }
    TRACEPOINT(callback_end, static_cast<const void *>(this));
    return response;
  }

  // 注册回调函数以进行跟踪 (Register callback for tracing)
  void register_callback_for_tracing() {
#ifndef TRACETOOLS_DISABLED
    std::visit(
        [this](auto &&arg) {
          TRACEPOINT(
              rclcpp_callback_register, static_cast<const void *>(this),
              tracetools::get_symbol(arg));
        },
        callback_);
#endif  // TRACETOOLS_DISABLED
  }

private:
  // 定义不同类型的回调函数 (Define different types of callbacks)
  using SharedPtrCallback = std::function<void(
      std::shared_ptr<typename ServiceT::Request>, std::shared_ptr<typename ServiceT::Response>)>;
  using SharedPtrWithRequestHeaderCallback = std::function<void(
      std::shared_ptr<rmw_request_id_t>,
      std::shared_ptr<typename ServiceT::Request>,
      std::shared_ptr<typename ServiceT::Response>)>;
  using SharedPtrDeferResponseCallback = std::function<void(
      std::shared_ptr<rmw_request_id_t>, std::shared_ptr<typename ServiceT::Request>)>;
  using SharedPtrDeferResponseCallbackWithServiceHandle = std::function<void(
      std::shared_ptr<rclcpp::Service<ServiceT>>,
      std::shared_ptr<rmw_request_id_t>,
      std::shared_ptr<typename ServiceT::Request>)>;

  // 存储回调函数的变体 (Store callback variant)
  std::variant<
      std::monostate,
      SharedPtrCallback,
      SharedPtrWithRequestHeaderCallback,
      SharedPtrDeferResponseCallback,
      SharedPtrDeferResponseCallbackWithServiceHandle>
      callback_;
};

}  // namespace rclcpp

#endif  // RCLCPP__ANY_SERVICE_CALLBACK_HPP_

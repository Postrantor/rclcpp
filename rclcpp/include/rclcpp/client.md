##

```cpp
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

namespace detail {

template <typename FutureT>
struct FutureAndRequestId {
  FutureT future;
  int64_t request_id;

  FutureAndRequestId(FutureT impl, int64_t req_id) : future(std::move(impl)), request_id(req_id) {}

  operator FutureT &() { return this->future; }

  [[deprecated("FutureAndRequestId: use .future instead of an implicit conversion")]]
  operator FutureT() {
    return this->future;
  }


  auto get() { return this->future.get(); }
  bool valid() const noexcept { return this->future.valid(); }
  void wait() const { return this->future.wait(); }
  template <class Rep, class Period>
  std::future_status wait_for(const std::chrono::duration<Rep, Period> &timeout_duration) const {
    return this->future.wait_for(timeout_duration);
  }
  template <class Clock, class Duration>
  std::future_status wait_until(
      const std::chrono::time_point<Clock, Duration> &timeout_time) const {
    return this->future.wait_until(timeout_time);
  }


  FutureAndRequestId(FutureAndRequestId &&other) noexcept = default;
  FutureAndRequestId(const FutureAndRequestId &other) = delete;
  FutureAndRequestId &operator=(FutureAndRequestId &&other) noexcept = default;
  FutureAndRequestId &operator=(const FutureAndRequestId &other) = delete;
  ~FutureAndRequestId() = default;
};

}  // namespace detail
```

这段代码是 ROS2（Robot Operating System 2）项目中的一部分，涉及到 rcl（ROS Client Library）和 rclcpp（C++ ROS Client Library）。主要功能是实现一个客户端，用于与 ROS 服务进行通信。以下是对这段代码的详细解释：

1. **头文件引入**：包含了所需的各种头文件，如 rcl、rclcpp 和 rmw 相关的头文件。

2. **命名空间 detail**：在 `detail` 命名空间下定义了一个模板结构体 `FutureAndRequestId`。

3. **结构体 FutureAndRequestId**：该结构体包含两个成员变量，`future` 和 `request_id`。`future` 是一个模板类型，用于存储异步操作的结果；`request_id` 是一个 64 位整数，用于表示请求的 ID。

   - 构造函数：接受一个 `FutureT` 类型的对象和一个 `int64_t` 类型的请求 ID，将它们分别赋值给 `future` 和 `request_id`。

   - 转换运算符：提供了将 `FutureAndRequestId` 对象隐式转换为 `FutureT` 类型的方法。但这种隐式转换已被标记为不推荐使用（deprecated），建议使用 `.future` 成员变量代替。

   - 成员函数：提供了一些与 `std::future` 类似的成员函数，如 `get()`、`valid()`、`wait()`、`wait_for()` 和 `wait_until()`，用于处理异步操作的结果。

   - 特殊成员函数：定义了移动构造函数、拷贝构造函数（删除）、移动赋值运算符和拷贝赋值运算符（删除），以及析构函数。

这段代码主要实现了一个用于存储 ROS 服务请求的异步结果和请求 ID 的结构体。在 ROS2 中，客户端与服务端进行通信时，会发送一个请求并分配一个唯一的请求 ID。通过使用 `FutureAndRequestId` 结构体，可以方便地管理这些请求的异步结果和对应的请求 ID。

##

```cpp
namespace node_interfaces {
class NodeBaseInterface;
}  // namespace node_interfaces

class ClientBase {
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ClientBase)

  RCLCPP_PUBLIC
  ClientBase(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph);

  RCLCPP_PUBLIC
  virtual ~ClientBase() = default;

  RCLCPP_PUBLIC
  bool take_type_erased_response(void *response_out, rmw_request_id_t &request_header_out);

  RCLCPP_PUBLIC
  std::shared_ptr<rcl_client_t> get_client_handle();

  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_client_t> get_client_handle() const;

  RCLCPP_PUBLIC
  bool service_is_ready() const;

  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
      std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    return wait_for_service_nanoseconds(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  virtual std::shared_ptr<void> create_response() = 0;

  virtual std::shared_ptr<rmw_request_id_t> create_request_header() = 0;

  virtual void handle_response(
      std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) = 0;

  RCLCPP_PUBLIC
  bool exchange_in_use_by_wait_set_state(bool in_use_state);

  RCLCPP_PUBLIC
  rclcpp::QoS get_request_publisher_actual_qos() const;

  RCLCPP_PUBLIC
  rclcpp::QoS get_response_subscription_actual_qos() const;

  void set_on_new_response_callback(std::function<void(size_t)> callback) {
    if (!callback) {
      throw std::invalid_argument(
          "The callback passed to set_on_new_response_callback "
          "is not callable.");
    }

    auto new_callback =
        [callback, this](size_t number_of_responses) {
          try {
            callback(number_of_responses);
          } catch (const std::exception &exception) {
            RCLCPP_ERROR_STREAM(
                node_logger_,
                "rclcpp::ClientBase@"
                    << this << " caught " << rmw::impl::cpp::demangle(exception)
                    << " exception in user-provided callback for the 'on new response' callback: "
                    << exception.what());
          } catch (...) {
            RCLCPP_ERROR_STREAM(
                node_logger_, "rclcpp::ClientBase@"
                                  << this
                                  << " caught unhandled exception in user-provided callback "
                                  << "for the 'on new response' callback");
          }
        };

    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    set_on_new_response_callback(
        rclcpp::detail::cpp_callback_trampoline<decltype(new_callback), const void *, size_t>,
        static_cast<const void *>(&new_callback));

    on_new_response_callback_ = new_callback;

    set_on_new_response_callback(
        rclcpp::detail::cpp_callback_trampoline<
            decltype(on_new_response_callback_), const void *, size_t>,
        static_cast<const void *>(&on_new_response_callback_));
  }

  void clear_on_new_response_callback() {
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    if (on_new_response_callback_) {
      set_on_new_response_callback(nullptr, nullptr);

      on_new_response_callback_ = nullptr;
    }
  }

protected:
  RCLCPP_DISABLE_COPY(ClientBase)

  RCLCPP_PUBLIC
  bool wait_for_service_nanoseconds(std::chrono::nanoseconds timeout);

  RCLCPP_PUBLIC
  rcl_node_t *get_rcl_node_handle();

  RCLCPP_PUBLIC
  const rcl_node_t *get_rcl_node_handle() const;

  RCLCPP_PUBLIC
  void set_on_new_response_callback(rcl_event_callback_t callback, const void *user_data);

  rclcpp::node_interfaces::NodeGraphInterface::WeakPtr node_graph_;

  std::shared_ptr<rcl_node_t> node_handle_;

  std::shared_ptr<rclcpp::Context> context_;

  rclcpp::Logger node_logger_;

  std::shared_ptr<rcl_client_t> client_handle_;

  std::atomic<bool> in_use_by_wait_set_{false};

  std::recursive_mutex callback_mutex_;

  std::function<void(size_t)> on_new_response_callback_{nullptr};
};
```

这段代码定义了一个名为 `ClientBase` 的类，它是 ROS2 服务客户端的基类。该类提供了与服务交互所需的基本功能，例如发送请求、接收响应以及等待服务就绪。以下是对该类中各个函数和成员变量的详细解释：

### 成员函数

1. **构造函数**：接受一个指向 `NodeBaseInterface` 类型的指针和一个 `NodeGraphInterface` 类型的智能指针。这些参数用于初始化节点和图形接口。

2. **析构函数**：虚拟析构函数，允许子类正确地销毁。

3. **take_type_erased_response**：从中间件获取类型擦除的响应，并将其存储在传入的参数中。

4. **get_client_handle**：返回一个指向 `rcl_client_t` 类型的智能指针，用于获取底层 ROS 客户端句柄。

5. **service_is_ready**：检查服务是否已准备好接收请求。

6. **wait_for_service**：等待服务准备好，直到超时。如果超时为负数，则无限期等待。

7. **create_response**：创建一个空的响应对象，由子类实现。

8. **create_request_header**：创建一个空的请求头对象，由子类实现。

9. **handle_response**：处理从中间件接收到的响应，由子类实现。

10. **exchange_in_use_by_wait_set_state**：交换 `in_use_by_wait_set_` 的状态，用于表示客户端是否正在被 wait set 使用。

11. **get_request_publisher_actual_qos**：获取请求发布者的实际 QoS 设置。

12. **get_response_subscription_actual_qos**：获取响应订阅者的实际 QoS 设置。

13. **set_on_new_response_callback**：设置一个新的回调函数，当收到新的响应时触发。

14. **clear_on_new_response_callback**：清除当前设置的新响应回调函数。

15. **wait_for_service_nanoseconds**：以纳秒为单位等待服务准备好，直到超时。

16. **get_rcl_node_handle**：返回指向底层 ROS 节点句柄的指针。

17. **set_on_new_response_callback**：设置新响应回调函数，由中间件调用。

### 成员变量

1. **node*graph***：一个指向 `NodeGraphInterface` 类型的弱智能指针，用于与图形接口进行交互。

2. **node*handle***：一个指向 `rcl_node_t` 类型的智能指针，用于存储底层 ROS 节点句柄。

3. **context\_**：一个指向 `rclcpp::Context` 类型的智能指针，用于存储 ROS 上下文。

4. **node*logger***：一个 `rclcpp::Logger` 类型的对象，用于记录日志。

5. **client*handle***：一个指向 `rcl_client_t` 类型的智能指针，用于存储底层 ROS 客户端句柄。

6. **in*use_by_wait_set***：一个原子布尔值，表示客户端是否正在被 wait set 使用。

7. **callback*mutex***：一个递归互斥锁，用于保护回调函数的线程安全。

8. **on*new_response_callback***：一个 `std::function` 类型的对象，用于存储新响应回调函数。

这个类的主要功能是提供与服务交互的基本方法。用户可以通过继承此类并实现虚拟函数来创建特定类型的服务客户端。例如，当收到新的服务响应时，用户可以设置回调函数以执行特定操作。

##

```cpp
template <typename ServiceT>
class Client : public ClientBase {
public:
// ...
}  // namespace rclcpp
```

这段代码定义了一个名为 `Client` 的模板类，它继承自 `ClientBase` 类。这个类是用于 ROS2 项目中的 rcl（ROS Client Library）客户端实现，用于与服务端进行通信。在这个类中，定义了一些类型别名、数据结构和成员函数，以便于处理不同类型的服务请求和响应。

### 类型别名

- `Request` 和 `Response` 分别表示服务的请求类型和响应类型。
- `SharedRequest` 和 `SharedResponse` 分别表示共享的请求类型和共享的响应类型。
- `Promise` 和 `PromiseWithRequest` 分别表示承诺类型和带请求的承诺类型。
- `SharedPromise` 和 `SharedPromiseWithRequest` 分别表示共享的承诺类型和共享的带请求的承诺类型。
- `Future`、`SharedFuture` 和 `SharedFutureWithRequest` 分别表示 Future 类型、共享的 Future 类型和带请求的共享 Future 类型。
- `CallbackType` 和 `CallbackWithRequestType` 分别表示回调类型和带请求的回调类型。

### 数据结构

- `FutureAndRequestId` 和 `SharedFutureAndRequestId` 结构体分别表示存储 Future 对象和请求 ID 的数据结构。
- `SharedFutureWithRequestAndRequestId` 结构体表示存储带请求的共享 Future 对象和请求 ID 的数据结构。

### 成员函数

1. 构造函数：接受节点基本接口、节点图形接口、服务名称和客户端选项作为参数，用于初始化 `ClientBase` 类并创建客户端。
2. 析构函数：虚拟析构函数，用于销毁对象。
3. `take_response`：从服务端获取响应，并将其存储在给定的响应对象和请求头中。
4. `create_response` 和 `create_request_header`：分别创建一个新的响应对象和请求头对象。
5. `handle_response`：处理从服务端接收到的响应，根据不同的回调类型执行相应的操作。
6. `async_send_request`：异步发送服务请求，支持多种重载形式，可以接受带回调或不带回调的请求。
7. `remove_pending_request`：移除指定请求 ID 的待处理请求。
8. `prune_pending_requests`：清除所有待处理的请求。
9. `prune_requests_older_than`：清除早于给定时间点的待处理请求。

### 受保护的成员变量和函数

- `CallbackTypeValueVariant` 和 `CallbackWithRequestTypeValueVariant` 分别表示回调类型值变体和带请求的回调类型值变体。
- `CallbackInfoVariant` 表示回调信息变体，它可以是 `Promise`、`CallbackTypeValueVariant` 或 `CallbackWithRequestTypeValueVariant`。
- `async_send_request_impl`：实现异步发送服务请求的功能，将请求添加到待处理请求列表中。
- `get_and_erase_pending_request`：获取并删除指定请求 ID 的待处理请求。
- `pending_requests_`：存储待处理请求的无序映射。
- `pending_requests_mutex_`：用于保护对 `pending_requests_` 的访问的互斥锁。

总之，这个 `Client` 类提供了一个用于与 ROS2 服务端通信的客户端实现。它支持异步发送请求、处理响应和管理待处理请求等功能。

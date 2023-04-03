```cpp
#include <rmw/error_handling.h> // RMW 错误处理相关头文件
#include <rmw/rmw.h> // RMW 库相关头文件

#include <chrono> // C++ 时间库
#include <functional> // C++ 函数对象库
#include <iostream> // C++ 输入输出流库
#include <memory> // C++ 内存管理库
#include <sstream> // C++ 字符串流库
#include <string> // C++ 字符串库
#include <unordered_map> // C++ 无序映射容器库
#include <utility> // C++ 实用程序库
#include <vector> // C++ 动态数组库

#include "rcl/publisher.h" // RCL 发布者相关头文件
#include "rclcpp/macros.hpp" // RCLCPP 宏定义头文件
#include "rclcpp/network_flow_endpoint.hpp" // RCLCPP 网络流端点相关头文件
#include "rclcpp/qos.hpp" // RCLCPP 服务质量（QoS）相关头文件
#include "rclcpp/qos_event.hpp" // RCLCPP 服务质量（QoS）事件相关头文件
#include "rclcpp/type_support_decl.hpp" // RCLCPP 类型支持声明头文件
#include "rclcpp/visibility_control.hpp" // RCLCPP 可见性控制头文件
#include "rcpputils/time.hpp" // RCPP 工具时间相关头文件
```

## class

```cpp
namespace rclcpp {

namespace node_interfaces {
class NodeBaseInterface;
class NodeTopicsInterface;
}  // namespace node_interfaces

namespace experimental {

class IntraProcessManager;
}  // namespace experimental
```

这段代码是 ROS2 项目中的 `rclcpp` 库的一部分，具体来说是 `publisher_base.hpp` 文件。`rclcpp` 是 ROS2（机器人操作系统 2）中的一个核心库，它提供了用于实现节点、发布者、订阅者等功能的 C++ API。在这个文件中，我们可以看到以下几个主要部分：

1. **命名空间 rclcpp**：这个命名空间包含了与 ROS2 的 C++ 客户端库相关的所有类和函数。

2. **命名空间 node_interfaces**：这个子命名空间包含了与节点接口相关的类。在这里，我们可以看到两个前向声明的类：

   - `NodeBaseInterface`：这个类定义了一个节点的基本接口，包括创建、销毁、获取名称等功能。
   - `NodeTopicsInterface`：这个类定义了一个节点的话题接口，包括创建发布者、订阅者等功能。

3. **命名空间 experimental**：这个子命名空间包含了实验性质的类和函数。在这里，我们可以看到一个前向声明的类：

   - `IntraProcessManager`：这个类用于管理同一进程内的消息传递，即在同一个进程内的不同节点之间进行通信。这可以提高通信效率，因为不需要通过网络层或其他 IPC 机制。

总的来说，这个文件主要是为了声明一些与 `rclcpp` 库中节点接口和实验性功能相关的类。这些类在其他地方（如 `.cpp` 文件）中会有具体的实现。通过这些类，ROS2 的用户可以更方便地创建和管理节点、发布者、订阅者等，从而实现机器人系统中的各种功能。

## PublisherBase

```cpp
class PublisherBase : public std::enable_shared_from_this<PublisherBase> {
  friend ::rclcpp::node_interfaces::NodeTopicsInterface;
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PublisherBase)
  PublisherBase(rclcpp::node_interfaces::NodeBaseInterface* node_base,
                const std::string& topic,
                const rosidl_message_type_support_t& type_support,
                const rcl_publisher_options_t& publisher_options,
                const PublisherEventCallbacks& event_callbacks,
                bool use_default_callbacks);
  virtual ~PublisherBase();
  void bind_event_callbacks(const PublisherEventCallbacks& event_callbacks,
                            bool use_default_callbacks);
```

### 类定义

```cpp
class PublisherBase : public std::enable_shared_from_this<PublisherBase>
```

`PublisherBase` 类从 `std::enable_shared_from_this` 继承，允许从其成员函数中获取指向当前对象的 `shared_ptr`。

### 友元类

```cpp
friend ::rclcpp::node_interfaces::NodeTopicsInterface;
```

将 `::rclcpp::node_interfaces::NodeTopicsInterface` 声明为友元类，使得它可以访问 `PublisherBase` 的私有和保护成员。

### 公共成员

```cpp
RCLCPP_SMART_PTR_DEFINITIONS(PublisherBase)
```

为 `PublisherBase` 类定义智能指针类型别名，如 `Ptr`、`ConstPtr` 等。

```cpp
PublisherBase(rclcpp::node_interfaces::NodeBaseInterface* node_base,
              const std::string& topic,
              const rosidl_message_type_support_t& type_support,
              const rcl_publisher_options_t& publisher_options,
              const PublisherEventCallbacks& event_callbacks,
              bool use_default_callbacks);
```

构造函数接受以下参数：

- `node_base`：指向节点基类的指针，用于与 ROS2 节点进行交互。
- `topic`：发布者要发布消息的主题名称。
- `type_support`：消息类型支持结构，用于处理特定消息类型的序列化和反序列化。
- `publisher_options`：发布者选项，包括质量服务（QoS）设置等。
- `event_callbacks`：事件回调结构，用于在特定事件发生时触发用户定义的回调函数。
- `use_default_callbacks`：布尔值，表示是否使用默认的事件回调。

```cpp
virtual ~PublisherBase();
```

虚析构函数，允许从该类派生的其他类正确地析构。

```cpp
void bind_event_callbacks(const PublisherEventCallbacks& event_callbacks,
                          bool use_default_callbacks);
```

`bind_event_callbacks` 函数接受以下参数：

- `event_callbacks`：事件回调结构，用于在特定事件发生时触发用户定义的回调函数。
- `use_default_callbacks`：布尔值，表示是否使用默认的事件回调。

此函数将事件回调绑定到发布者实例。当特定事件发生时，这些回调将被触发。

总之，这个文件定义了一个 `PublisherBase` 类，它是 rclcpp 中所有发布者类的基类。它提供了构造函数、析构函数以及绑定事件回调的方法。

##

```cpp
  bool assert_liveliness() const;
  rclcpp::QoS get_actual_qos() const;
  bool can_loan_messages() const;
  bool operator==(const rmw_gid_t& gid) const;
  bool operator==(const rmw_gid_t* gid) const;
```

这段代码是 ROS2 项目中的 rclcpp 库中的一部分，具体来说是 publisher_base.hpp 文件。rclcpp 是 ROS2（Robot Operating System 2）中的一个关键组件，它提供了用于实现节点、发布者、订阅者等功能的 C++ API。在这个文件中，我们可以看到以下几个函数：

1. `bool assert_liveliness() const;`
2. `rclcpp::QoS get_actual_qos() const;`
3. `bool can_loan_messages() const;`
4. `bool operator==(const rmw_gid_t& gid) const;`
5. `bool operator==(const rmw_gid_t* gid) const;`

下面是对这些函数的详细解释：

1. **assert_liveliness()**：此函数用于声明发布者的活跃状态。在某些 QoS（Quality of Service，服务质量）策略中，发布者需要定期声明其活跃状态，以确保订阅者知道发布者仍然存在并发送消息。如果成功声明活跃状态，则返回 true；否则返回 false。

2. **get_actual_qos()**：此函数返回发布者实际使用的 QoS 设置。QoS 是一组参数，用于控制如何传输消息，例如消息的可靠性、持久性和延迟等。通过调用此函数，用户可以获取发布者当前正在使用的 QoS 参数。

3. **can_loan_messages()**：此函数检查发布者是否支持消息借用功能。消息借用允许发布者直接使用共享内存，而不是复制消息，从而提高性能。如果发布者支持消息借用，则返回 true；否则返回 false。

4. **operator==(const rmw_gid_t& gid) const;**：此函数重载了 `==` 运算符，用于比较两个发布者的全局唯一标识符（GID）。如果两个发布者具有相同的 GID，则它们被认为是相等的。这在需要比较发布者实例时非常有用。

5. **operator==(const rmw_gid_t\* gid) const;**：与上一个函数类似，此函数也重载了 `==` 运算符，但接受指向 GID 的指针作为参数。这允许用户通过指针来比较发布者实例。

总之，这个文件中的函数主要涉及到发布者的活跃状态、QoS 设置、消息借用功能以及发布者实例的比较。这些功能对于实现 ROS2 中的发布者和订阅者模型至关重要。

---

当然可以。`assert_liveliness()` 函数在 ROS2 的发布者中起着重要作用，它用于声明发布者的活跃状态。在某些 QoS（Quality of Service，服务质量）策略中，发布者需要定期声明其活跃状态，以确保订阅者知道发布者仍然存在并发送消息。

QoS 策略之一是 LIVELINESS（活跃性），它定义了如何检测节点（发布者和订阅者）是否仍然活跃。LIVELINESS 策略有三种模式：

1. **AUTOMATIC**：在此模式下，ROS2 中间件会自动处理活跃状态的声明。发布者和订阅者无需调用 `assert_liveliness()` 函数。
2. **MANUAL_BY_NODE**：在此模式下，节点需要手动声明其活跃状态。这意味着，如果一个发布者属于一个节点，那么该节点需要调用 `assert_liveliness()` 函数来声明其活跃状态。
3. **MANUAL_BY_TOPIC**：在此模式下，发布者需要单独声明其活跃状态。这意味着每个发布者都需要调用 `assert_liveliness()` 函数来声明其活跃状态。

`assert_liveliness()` 函数的工作原理是向底层的 ROS2 中间件发送一个信号，表明发布者仍然活跃。如果成功声明活跃状态，则返回 true；否则返回 false。在 MANUAL_BY_NODE 或 MANUAL_BY_TOPIC 模式下，用户需要定期调用此函数以保持发布者的活跃状态。

例如，在 MANUAL_BY_TOPIC 模式下，可以通过以下方式调用 `assert_liveliness()` 函数：

```cpp
auto publisher = node->create_publisher<MsgType>("topic_name", rclcpp::QoS(rclcpp::KeepLast(10)).liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC));
// ... 其他代码 ...

// 在适当的时机声明发布者的活跃状态
bool result = publisher->assert_liveliness();
if (!result) {
  // 处理声明活跃状态失败的情况
}
```

总之，`assert_liveliness()` 函数在 ROS2 的发布者中用于声明活跃状态，确保订阅者知道发布者仍然存在并发送消息。这对于某些 QoS 策略（如 MANUAL_BY_NODE 和 MANUAL_BY_TOPIC）非常重要。

> [!NOTE]
> 更多内容放在 `ros/assert_liveliness_*_` 里面了
> 这个也是用于节点热备份能力的一个重要因素

##

```cpp
  using IntraProcessManagerSharedPtr = std::shared_ptr<rclcpp::experimental::IntraProcessManager>;
  void setup_intra_process(uint64_t intra_process_publisher_id, IntraProcessManagerSharedPtr ipm);
  std::vector<rclcpp::NetworkFlowEndpoint> get_network_flow_endpoints() const;
  template <typename DurationRepT = int64_t, typename DurationT = std::milli>
  bool wait_for_all_acked(std::chrono::duration<DurationRepT, DurationT> timeout =
                              std::chrono::duration<DurationRepT, DurationT>(-1)) const{}
  void set_on_new_qos_event_callback(std::function<void(size_t)> callback,
                                     rcl_publisher_event_type_t event_type) {}
  void clear_on_new_qos_event_callback(rcl_publisher_event_type_t event_type) {}
```

```cpp
protected:
  template <typename EventCallbackT>
  void add_event_handler(const EventCallbackT& callback,
                         const rcl_publisher_event_type_t event_type) {}
  void default_incompatible_qos_callback(QOSOfferedIncompatibleQoSInfo& info) const;
  std::shared_ptr<rcl_node_t> rcl_node_handle_;
  std::shared_ptr<rcl_publisher_t> publisher_handle_;
  std::unordered_map<rcl_publisher_event_type_t, std::shared_ptr<rclcpp::QOSEventHandlerBase>>
      event_handlers_;
  using IntraProcessManagerWeakPtr = std::weak_ptr<rclcpp::experimental::IntraProcessManager>;
  bool intra_process_is_enabled_;
  IntraProcessManagerWeakPtr weak_ipm_;
  uint64_t intra_process_publisher_id_;
  rmw_gid_t rmw_gid_;
  const rosidl_message_type_support_t type_support_;
  const PublisherEventCallbacks event_callbacks_;
};

}  // namespace rclcpp
```

这段代码来自 ROS2 项目中的 rclcpp 库，涉及到发布者（Publisher）的基础功能。rclcpp 是 ROS2 的 C++ 客户端库，用于实现节点、发布者、订阅者等 ROS2 核心功能。在这个文件（publisher_base.hpp）中，定义了一些与发布者相关的函数。

1. `using IntraProcessManagerSharedPtr = std::shared_ptr<rclcpp::experimental::IntraProcessManager>;`

   这行代码定义了一个类型别名 `IntraProcessManagerSharedPtr`，它是指向 `rclcpp::experimental::IntraProcessManager` 类型的共享指针。`IntraProcessManager` 类用于管理同一进程内的消息传递。

2. `void setup_intra_process(uint64_t intra_process_publisher_id, IntraProcessManagerSharedPtr ipm);`

   `setup_intra_process` 函数用于设置进程内通信。它接收两个参数：`intra_process_publisher_id`（进程内发布者 ID）和 `ipm`（指向 `IntraProcessManager` 的共享指针）。此函数将发布者与 `IntraProcessManager` 关联，以便在同一进程内进行消息传递。

3. `std::vector<rclcpp::NetworkFlowEndpoint> get_network_flow_endpoints() const;`

   `get_network_flow_endpoints` 函数返回一个包含网络流终端信息的向量。这些终端信息有助于识别发布者与订阅者之间的网络连接。

4. `template <typename DurationRepT = int64_t, typename DurationT = std::milli>
bool wait_for_all_acked(std::chrono::duration<DurationRepT, DurationT> timeout =
                           std::chrono::duration<DurationRepT, DurationT>(-1)) const;`

   `wait_for_all_acked` 函数是一个模板函数，用于等待所有已发布的消息被确认。它接收一个参数 `timeout`，表示等待超时时间。如果在超时时间内所有消息都被确认，则返回 `true`；否则返回 `false`。

5. `void set_on_new_qos_event_callback(std::function<void(size_t)> callback,
                            rcl_publisher_event_type_t event_type);`

   `set_on_new_qos_event_callback` 函数用于设置新 QoS（Quality of Service，服务质量）事件的回调函数。它接收两个参数：`callback`（回调函数）和 `event_type`（QoS 事件类型）。当指定类型的 QoS 事件发生时，将调用回调函数。

6. `void clear_on_new_qos_event_callback(rcl_publisher_event_type_t event_type);`

   `clear_on_new_qos_event_callback` 函数用于清除指定 QoS 事件类型的回调函数。它接收一个参数 `event_type`（QoS 事件类型）。调用此函数后，指定类型的 QoS 事件将不再触发回调函数。

总之，这个文件（publisher_base.hpp）定义了与 ROS2 发布者相关的一些基础功能，包括进程内通信、网络流终端信息获取、等待消息确认以及 QoS 事件回调函数的设置和清除。这些功能有助于实现 ROS2 发布者与订阅者之间的高效、可靠的消息传递。

---

`set_on_new_qos_event_callback` 函数是用于设置新 QoS（Quality of Service，服务质量）事件的回调函数。QoS 是 ROS2 中非常重要的一个概念，它定义了发布者和订阅者之间消息传递的可靠性、延迟等方面的策略。在某些情况下，我们可能需要在 QoS 策略发生变化时执行特定的操作，例如重新配置连接或记录日志。这就是为什么我们需要为 QoS 事件设置回调函数的原因。

`set_on_new_qos_event_callback` 函数接收两个参数：

1. `std::function<void(size_t)> callback`：这是一个回调函数，其参数类型为 `size_t`。当指定类型的 QoS 事件发生时，将调用此回调函数。通常，这个参数表示与事件相关的数据，例如新的 QoS 策略值。

2. `rcl_publisher_event_type_t event_type`：这是一个枚举类型，表示 QoS 事件的类型。ROS2 提供了多种 QoS 事件类型，例如 `RCL_PUBLISHER_OFFERED_DEADLINE_MISSED`（发布者错过截止时间）和 `RCL_PUBLISHER_LIVELINESS_LOST`（发布者活跃度丢失）。你可以根据需要为不同类型的 QoS 事件设置不同的回调函数。

使用 `set_on_new_qos_event_callback` 函数的一个典型示例是在发布者错过截止时间时记录日志。例如：

```cpp
void on_deadline_missed(size_t missed_deadlines) {
  std::cout << "Missed " << missed_deadlines << " deadlines." << std::endl;
}

// ...

publisher->set_on_new_qos_event_callback(on_deadline_missed, RCL_PUBLISHER_OFFERED_DEADLINE_MISSED);
```

在这个示例中，我们定义了一个名为 `on_deadline_missed` 的回调函数，用于在发布者错过截止时间时输出一条日志。然后，我们使用 `set_on_new_qos_event_callback` 函数将此回调函数与 `RCL_PUBLISHER_OFFERED_DEADLINE_MISSED` 事件类型关联起来。

总之，`set_on_new_qos_event_callback` 函数允许你为不同类型的 QoS 事件设置回调函数，以便在这些事件发生时执行特定的操作。这有助于提高 ROS2 发布者和订阅者之间消息传递的可靠性和灵活性。

> [!NOTE]
>
> - `set_on_new_qos_event_callback()` 这个函数可以在 qos 发生变化时候给出一些动作，或许也可以用一下

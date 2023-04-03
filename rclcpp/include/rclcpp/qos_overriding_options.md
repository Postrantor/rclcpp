---
Crafted by [Genie](https://marketplace.visualstudio.com/items?itemName=genieai.chatgpt-vscode)
C:\Users\trantor\Documents\Hirain\Project\src\ros2\rclcpp\rclcpp\include\rclcpp\qos_overriding_options.md
---

## You

下面这段代码是在 ros2 项目中 rclcpp 相关的代码，请联系所有函数之间可能的相互联系，梳理、解释这份文件的功能、含义，以及函数之间的功能调用关系（以中文语言输出，不限制字数）

## Trantor

这段代码是 ROS2 项目中 rclcpp 相关的代码，主要涉及到 QoS（Quality of Service，服务质量）策略的设置和处理。以下是对文件功能、含义以及函数之间的功能调用关系的梳理和解释：

1. 首先，代码引入了一些必要的头文件，如 `rcl_interfaces/msg/set_parameters_result.hpp`，`rclcpp/qos.hpp` 等。

2. 在 `rclcpp` 命名空间下，定义了一个枚举类 `QosPolicyKind`，表示不同的 QoS 策略类型。这些策略类型包括：

   - AvoidRosNamespaceConventions
   - Deadline
   - Depth
   - Durability
   - History
   - Lifespan
   - Liveliness
   - LivelinessLeaseDuration
   - Reliability
   - Invalid

3. 定义了两个与 `QosPolicyKind` 相关的函数：

   - `qos_policy_kind_to_cstr()`：将 QosPolicyKind 类型转换为字符串。
   - `operator<<()`：重载输出流操作符，方便打印 QosPolicyKind 类型。

4. 定义了两个类型别名：

   - `QosCallbackResult`：使用 `rcl_interfaces::msg::SetParametersResult` 类型。
   - `QosCallback`：一个回调函数，接收一个 `rclcpp::QoS` 类型参数，返回一个 `QosCallbackResult` 类型结果。

5. 在 `detail` 命名空间下，定义了一个模板类 `QosParameters`。

6. 定义了一个 `QosOverridingOptions` 类，用于处理 QoS 策略的覆盖选项。这个类包含以下成员函数：

   - 默认构造函数：创建一个空的 QosOverridingOptions 对象。
   - 带参数的构造函数：接收一个 QosPolicyKind 列表、一个验证回调函数和一个 ID 字符串，用于初始化 QosOverridingOptions 对象。
   - `get_id()`：获取 QosOverridingOptions 对象的 ID。
   - `get_policy_kinds()`：获取 QosOverridingOptions 对象的 QosPolicyKind 列表。
   - `get_validation_callback()`：获取 QosOverridingOptions 对象的验证回调函数。
   - `with_default_policies()`：静态函数，接收一个验证回调函数和一个 ID 字符串，返回一个使用默认策略的 QosOverridingOptions 对象。

在这段代码中，主要关注点是 QoS 策略的设置和处理。通过定义 `QosPolicyKind` 枚举类、`QosCallbackResult` 类型别名、`QosCallback` 类型别名以及 `QosOverridingOptions` 类，实现了对 QoS 策略的管理和操作。

## You

请进一步详细介绍这个文件的所有内容，可以结合 rclcpp 中其他相关的内容

## Trantor

1. **头文件引入**：这部分引入了一些必要的头文件，以便使用 ROS2 和 rclcpp 提供的功能。

   - `rcl_interfaces/msg/set_parameters_result.hpp`：引入 SetParametersResult 消息类型，用于表示参数设置结果。
   - `rclcpp/qos.hpp`：引入 QoS 相关类和函数，如 QoS 类型定义、QoS 初始化方法等。
   - `rclcpp/visibility_control.hpp`：引入可见性控制宏，用于控制符号在动态库中的可见性。
   - `rmw/qos_policy_kind.h`：引入 rmw 层的 QoS 策略枚举类型定义。

2. **QosPolicyKind 枚举类**：在 `rclcpp` 命名空间下，定义了一个枚举类 `QosPolicyKind`，表示不同的 QoS 策略类型。这些策略类型与 rmw 层的 QoS 策略类型相对应，实现了 rclcpp 层与 rmw 层之间的映射关系。

3. **QosPolicyKind 相关函数**：定义了两个与 `QosPolicyKind` 相关的函数，用于处理 QosPolicyKind 类型数据。

   - `qos_policy_kind_to_cstr()`：将 QosPolicyKind 类型转换为字符串，便于输出和调试。
   - `operator<<()`：重载输出流操作符，方便打印 QosPolicyKind 类型。当使用 `std::cout` 等输出流时，可以直接输出 QosPolicyKind 类型的值。

4. **类型别名**：定义了两个类型别名，简化代码编写和阅读。

   - `QosCallbackResult`：使用 `rcl_interfaces::msg::SetParametersResult` 类型，表示参数设置结果。
   - `QosCallback`：一个回调函数类型，接收一个 `rclcpp::QoS` 类型参数，返回一个 `QosCallbackResult` 类型结果。这个回调函数通常用于验证 QoS 设置是否有效。

5. **QosParameters 模板类**：在 `detail` 命名空间下，定义了一个模板类 `QosParameters`。这个类主要用于存储和管理 QoS 参数。由于它位于 `detail` 命名空间下，表明这是一个内部实现细节，不应该被外部直接使用。

6. **QosOverridingOptions 类**：这个类用于处理 QoS 策略的覆盖选项。在 ROS2 中，**节点可以通过参数服务器或命令行参数来覆盖默认的 QoS 设置**。这个类提供了一种方便的方式来指定哪些 QoS 策略可以被覆盖，以及如何验证覆盖后的 QoS 设置。

   - 默认构造函数：创建一个空的 QosOverridingOptions 对象。
   - 带参数的构造函数：接收一个 QosPolicyKind 列表、一个验证回调函数和一个 ID 字符串，用于初始化 QosOverridingOptions 对象。QosPolicyKind 列表表示允许覆盖的策略类型，验证回调函数用于检查覆盖后的 QoS 设置是否有效，ID 字符串用于标识 QosOverridingOptions 对象。
   - `get_id()`：获取 QosOverridingOptions 对象的 ID。
   - `get_policy_kinds()`：获取 QosOverridingOptions 对象的 QosPolicyKind 列表。
   - `get_validation_callback()`：获取 QosOverridingOptions 对象的验证回调函数。
   - `with_default_policies()`：静态函数，接收一个验证回调函数和一个 ID 字符串，返回一个使用默认策略的 QosOverridingOptions 对象。这个函数方便用户创建一个包含所有默认 QoS 策略的 QosOverridingOptions 对象。

通过这个文件，ROS2 的 rclcpp 层实现了对 QoS 策略的设置和处理。用户可以通过 QosOverridingOptions 类来指定允许覆盖的 QoS 策略，并使用回调函数对覆盖后的 QoS 设置进行验证。这样可以确保节点间通信的可靠性、延迟等方面的表现符合预期。

#

函数之间的调用关系如下：

- `operator<<` 调用了 `qos_policy_kind_to_cstr` 函数。
- `with_default_policies` 调用了 `QosOverridingOptions` 类构造函数。

这个文件主要实现了 QosOverridingOptions 类，用于表示和处理 QoS（Quality of Service，服务质量）策略的覆盖选项。QoS 策略在 ROS2 中非常重要，因为它们决定了节点之间通信的行为。例如，消息传递的可靠性、消息队列的深度等。

在 rclcpp 中，QosOverridingOptions 类与其他一些类和功能密切相关，如 Publisher、Subscription 和 QoS 设置。下面是这个文件内容与 rclcpp 中其他相关内容的关联：

1. **QosPolicyKind 枚举**：定义了 QoS 策略的种类，包括 History、Depth、Reliability 等。这些策略在 rclcpp 的 Publisher 和 Subscription 类中被使用，以设置节点间通信的 QoS 参数。

2. **rmw_qos_policy_kind_to_str 函数**：这是一个来自 rmw（ROS Middleware，ROS 中间件）库的函数，用于将 QosPolicyKind 转换为字符串。在 qos_policy_kind_to_cstr 函数中调用了此函数。

3. **QosCallback 类型**：这是一个回调函数类型，用于验证 QoS 配置。在创建 QosOverridingOptions 对象时，可以提供一个 QosCallback 函数，以便在需要时对 QoS 配置进行验证。

4. **Publisher 和 Subscription 类**：在 rclcpp 中，Publisher 和 Subscription 类负责管理节点间的通信。它们都有一个 QoS 参数，用于设置通信的 QoS 策略。QosOverridingOptions 类可以与这些类一起使用，以覆盖默认的 QoS 设置。

总结一下，这个文件主要实现了 QosOverridingOptions 类，它在 rclcpp 中与 Publisher、Subscription 和 QoS 设置等相关内容紧密相连。通过 QosOverridingOptions 类，用户可以方便地设置和获取 QoS 策略相关信息，并根据需要覆盖默认的 QoS 设置。这对于优化 ROS2 节点间通信的性能和可靠性非常重要。

## You

在 ros2 中该如何使用 `QosOverridingOptions` 这个类，请给出一些示例程序

## Trantor

在 ROS2 中，`QosOverridingOptions` 类可以用于覆盖默认的 QoS 设置。以下是一个简单的示例程序，演示了如何在创建 Publisher 和 Subscription 时使用 `QosOverridingOptions`。

```cpp
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MyPublisher : public rclcpp::Node
{
public:
  MyPublisher()
  : Node("my_publisher")
  {
    // 使用 QosOverridingOptions 创建 Publisher
    rclcpp::QosOverridingOptions qos_overriding_options;
    qos_overriding_options.policy_kinds = {rclcpp::QosPolicyKind::Reliability};
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "topic",
      rclcpp::SystemDefaultsQoS(),
      qos_overriding_options);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&MyPublisher::publish_message, this));
  }

private:
  void publish_message()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world!";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class MySubscriber : public rclcpp::Node
{
public:
  MySubscriber()
  : Node("my_subscriber")
  {
    // 使用 QosOverridingOptions 创建 Subscription
    rclcpp::QosOverridingOptions qos_overriding_options;
    qos_overriding_options.policy_kinds = {rclcpp::QosPolicyKind::Reliability};
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&MySubscriber::message_callback, this, std::placeholders::_1),
      qos_overriding_options);
  }

private:
  void message_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<MyPublisher>();
  auto subscriber_node = std::make_shared<MySubscriber>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
```

在这个示例中，我们创建了一个名为 `MyPublisher` 的发布者节点和一个名为 `MySubscriber` 的订阅者节点。在创建 Publisher 和 Subscription 时，我们使用了 `QosOverridingOptions` 类来覆盖默认的 QoS 设置。我们设置了 `policy_kinds` 为 `{rclcpp::QosPolicyKind::Reliability}`，表示我们希望覆盖可靠性策略。

## You

通过 `qos_overriding_options` 对 pub 和 sub 设置 qos 策略 与 直接给出 qos 策略应用到 sub 和 pub 有什么区别吗

## Trantor

使用 `qos_overriding_options` 设置 QoS 策略与直接给出 QoS 策略应用到 Publisher 和 Subscriber 的主要区别在于**运行时可配置性**。

当您直接为 Publisher 和 Subscriber 提供 QoS 策略时，这些策略在编译时就已经固定了。也就是说，如果您想更改 QoS 设置，需要修改代码并重新编译程序。

而使用 `qos_overriding_options` 时，可以在运行时通过参数覆盖默认的 QoS 设置。这意味着您无需修改代码或重新编译程序即可更改 QoS 策略。这提供了更大的灵活性，特别是在调试和优化系统性能方面。

例如，在上述示例中，我们使用 `qos_overriding_options` 设置了可靠性策略。假设我们希望在运行时将其从默认的 `reliable` 更改为 `best_effort`，只需在启动节点时设置参数：

```sh
ros2 run your_package_name my_publisher --ros-args --qos-reliability best_effort
ros2 run your_package_name my_subscriber --ros-args --qos-reliability best_effort
```

这样，我们可以轻松地在运行时更改 QoS 策略，而无需修改代码。

> [!NOTE]
> 这个能力想必也可以引入到系统中其他的配置文件中，或者是对 beatles。
> 主要是这个运行时可更改的能力，是 ros2 中提供的，那么 beatles 是否也应该有运行时可配置的能力

## You

可以再详细深入介绍一下吗

## Trantor

当然可以。在 ROS 2 中，QoS（Quality of Service）策略用于控制节点之间的通信质量。QoS 策略包括以下几个方面：

1. **可靠性**：`RELIABLE` 和 `BEST_EFFORT`。`RELIABLE` 确保消息传递是可靠的，而 `BEST_EFFORT` 不保证消息传递。
2. **历史记录**：`KEEP_LAST` 和 `KEEP_ALL`。`KEEP_LAST` 只保留最近的 N 条消息，而 `KEEP_ALL` 保留所有消息，直到资源耗尽。
3. **耐久性**：`TRANSIENT_LOCAL` 和 `VOLATILE`。`TRANSIENT_LOCAL` 意味着数据将在本地存储一段时间，以便新订阅者也能接收到先前发布的消息。`VOLATILE` 则表示数据不会在本地存储。
4. **期望的生命周期**：定义了数据的有效期限。
5. **延迟预算**：定义了允许的最大传输延迟。

在编写 Publisher 和 Subscriber 时，您可以为它们分别设置 QoS 策略。例如：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyPublisher : public rclcpp::Node
{
public:
  MyPublisher() : Node("my_publisher")
  {
    // 设置 QoS 策略
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", qos);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class MySubscriber : public rclcpp::Node
{
public:
  MySubscriber() : Node("my_subscriber")
  {
    // 设置 QoS 策略
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", qos, std::bind(&MySubscriber::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
```

然而，这种方法的缺点是 QoS 策略在编译时就已经固定。要更改策略，需要修改代码并重新编译。

为了解决这个问题，ROS 2 引入了 `qos_overriding_options`，**允许在运行时通过参数覆盖默认的 QoS 设置**。例如：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyPublisher : public rclcpp::Node
{
public:
  MyPublisher() : Node("my_publisher")
  {
    // 设置 QoS 策略
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

    // 设置 QoS 覆盖选项
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Durability
    };

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", qos, options);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class MySubscriber : public rclcpp::Node
{
public:
  MySubscriber() : Node("my_subscriber")
  {
    // 设置 QoS 策略
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // 设置 QoS 覆盖选项
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Durability
    };

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", //
      qos, //
      std::bind(&MySubscriber::callback, this, std::placeholders::_1), //
      options);
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
```

现在，您可以在运行时通过参数更改 QoS 策略，而无需修改代码。例如，要将可靠性策略从 `reliable` 更改为 `best_effort`，只需在启动节点时设置参数：

```sh
ros2 run your_package_name my_publisher --ros-args --qos-reliability best_effort
ros2 run your_package_name my_subscriber --ros-args --qos-reliability best_effort
```

这样，您可以轻松地在运行时更改 QoS 策略，提高系统的灵活性。

## You

下面这段代码是在 ros2 项目中 rclcpp 相关的代码，请联系所有函数之间可能的相互联系，梳理、解释这份文件的功能、含义，以及函数之间的功能调用关系（以中文语言输出，不限制字数）

```cpp
/*!
 * \brief 使用默认策略创建 QosOverridingOptions 对象
 * \param[in] validation_callback 用于验证 QoS 配置的回调函数
 * \param[in] id 用于标识 QosOverridingOptions 的字符串
 * \return 创建的 QosOverridingOptions 对象
 */
QosOverridingOptions QosOverridingOptions::with_default_policies(
    QosCallback validation_callback, std::string id) {
  return QosOverridingOptions{kDefaultPolicies, validation_callback, id};
}
```

## Trantor

这段代码定义了一个静态成员函数 `with_default_policies`，它用于创建一个 `QosOverridingOptions` 对象。`QosOverridingOptions` 类是 ROS 2 的一部分，用于在运行时通过参数覆盖默认的 QoS 设置。

函数接受两个参数：

1. `validation_callback`：一个回调函数，用于验证 QoS 配置。当用户尝试覆盖 QoS 策略时，此回调将被调用以确保所提供的配置是有效的。
2. `id`：一个字符串，用于标识 `QosOverridingOptions` 对象。这有助于在日志和错误消息中区分不同的 QoS 覆盖选项。

函数的主要功能是创建一个 `QosOverridingOptions` 对象，并使用默认的 QoS 策略集合（`kDefaultPolicies`）初始化它。然后返回创建的对象。

在 ROS 2 的 rclcpp 中，`QosOverridingOptions` 类通常与 `PublisherOptions` 和 `SubscriptionOptions` 一起使用，以允许在运行时更改 QoS 策略。例如，在创建发布者或订阅者时，可以使用此函数创建一个具有默认策略的 `QosOverridingOptions` 对象，并将其添加到相应的选项中：

```cpp
rclcpp::QoS qos(10);
qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

// 创建 QosOverridingOptions 对象
auto qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

// 设置 PublisherOptions 或 SubscriptionOptions
rclcpp::PublisherOptions publisher_options;
publisher_options.qos_overriding_options = qos_overriding_options;

// 创建发布者或订阅者
auto publisher = node->create_publisher<std_msgs::msg::String>("topic", qos, publisher_options);
```

这样，在运行时，用户可以通过参数更改 QoS 策略，而无需修改代码。

## You

可以详细解释一下这段代码吗？`using QosCallbackResult = rcl_interfaces::msg::SetParametersResult;
using QosCallback = std::function<QosCallbackResult(const rclcpp::QoS &)>;`

## Trantor

这段代码定义了两个类型别名，它们与 QoS（Quality of Service，服务质量）回调函数相关。

1. `QosCallbackResult`：这是一个类型别名，表示 QoS 回调函数的返回类型。它使用 `rcl_interfaces::msg::SetParametersResult` 类型，该类型是 ROS 2 中用于设置参数结果的消息类型。`SetParametersResult` 消息包含两个字段：

   - `bool successful`：表示操作是否成功。
   - `string reason`：如果操作失败，此字段包含描述原因的字符串。

   在 QoS 回调函数中，`successful` 字段表示所提供的 QoS 配置是否有效，`reason` 字段可以提供有关无效配置的详细信息。

2. `QosCallback`：这是一个类型别名，表示 QoS 回调函数的类型。它是一个 `std::function` 对象，接受一个 `const rclcpp::QoS &` 参数并返回一个 `QosCallbackResult` 类型的对象。这个回调函数在尝试覆盖 QoS 策略时被调用，以验证用户提供的 QoS 配置是否有效。如果配置有效，回调函数应返回一个 `successful` 为 `true` 的 `QosCallbackResult` 对象；否则，它应返回一个 `successful` 为 `false` 的对象，并在 `reason` 字段中提供详细信息。

总之，这段代码定义了与 QoS 回调函数相关的类型别名，以便在 ROS 2 的 rclcpp 中使用。这些回调函数用于在运行时验证 QoS 配置，确保用户提供的配置是有效的。

## You

请详细解释一下 `QosOverridingOptions` 类的运行原理，是如何实现在运行时对 qos 配置进行更改的

## Trantor

`QosOverridingOptions` 类本身不会在运行时更改 QoS 配置，而是提供了一种机制来存储和处理与 QoS 覆盖相关的选项。这些选项用于指导如何在运行时从参数服务器获取 QoS 设置并验证它们。

要理解 `QosOverridingOptions` 的工作原理，我们需要关注几个关键部分：

1. **存储 QoS 覆盖策略**：`QosOverridingOptions` 类包含一个名为 `policy_kinds` 的成员变量，它是一个 `std::vector<rclcpp::QosPolicyKind>` 类型的容器。这个向量存储了允许覆盖的 QoS 策略类型。例如，如果你想允许在运行时覆盖“Deadline”和“Lifespan”策略，你可以将这些策略添加到 `policy_kinds` 向量中。

2. **QoS 回调函数**：`QosOverridingOptions` 类还包含一个名为 `validation_callback` 的成员变量，它是一个 `QosCallback` 类型的对象。这个回调函数在尝试应用 QoS 覆盖时被调用，以验证所提供的 QoS 配置是否有效。这允许用户自定义验证逻辑，确保新的 QoS 设置符合特定需求。

3. **从参数服务器获取 QoS 设置**：在运行时，节点会从参数服务器获取 QoS 设置。这是通过在节点初始化时读取参数并将它们应用于订阅器或发布器的 QoS 配置来实现的。`QosOverridingOptions` 类提供了一种机制来指定哪些策略可以被覆盖以及如何验证新设置。

要在运行时更改 QoS 配置，你需要执行以下步骤：

1. 创建一个 `QosOverridingOptions` 对象，并配置允许覆盖的策略类型和验证回调函数。
2. 在创建订阅器或发布器时，将 `QosOverridingOptions` 对象传递给构造函数。
3. 在运行时，使用参数服务器更新 QoS 参数。节点会自动检测到这些更改，并尝试应用新的 QoS 配置。
4. 如果新的 QoS 配置通过验证回调函数，则会将其应用于订阅器或发布器。否则，配置将不会生效，并且可以在回调函数中提供有关失败原因的信息。

总之，`QosOverridingOptions` 类提供了一种机制来存储和处理与 QoS 覆盖相关的选项。这些选项用于在运行时从参数服务器获取 QoS 设置并验证它们。请注意，`QosOverridingOptions` 类本身不会直接更改 QoS 配置，而是**作为一种辅助工具来管理 QoS 覆盖过程**。

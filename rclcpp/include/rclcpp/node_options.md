##

```cpp
#include <memory> // 内存管理相关头文件
#include <string> // 字符串操作相关头文件
#include <vector> // 动态数组相关头文件
#include "rcl/node_options.h" // ROS2 RCL 节点选项相关头文件
#include "rcl/time.h" // ROS2 RCL 时间相关头文件
#include "rclcpp/context.hpp" // ROS2 RCLCPP 上下文相关头文件
#include "rclcpp/contexts/default_context.hpp" // ROS2 RCLCPP 默认上下文相关头文件
#include "rclcpp/parameter.hpp" // ROS2 RCLCPP 参数相关头文件
#include "rclcpp/publisher_options.hpp" // ROS2 RCLCPP 发布者选项相关头文件
#include "rclcpp/qos.hpp" // ROS2 RCLCPP 服务质量（QoS）相关头文件
#include "rclcpp/visibility_control.hpp" // ROS2 RCLCPP 可见性控制相关头文件
```

> [!NOTE]
> #include "rcl/node_options.h" // ROS2 RCL 节点选项相关头文件

##

```cpp
namespace rclcpp
{
class NodeOptions
{
public:
  explicit NodeOptions(rcl_allocator_t allocator = rcl_get_default_allocator());
  virtual ~NodeOptions() = default;
  NodeOptions(const NodeOptions & other);
  NodeOptions & operator=(const NodeOptions & other);
```

`rclcpp` 是 ROS 2（Robot Operating System 2）的 C++ 客户端库，它提供了用于构建机器人应用程序的 API。在这个库中，`NodeOptions` 类是一个配置类，用于设置和管理节点的选项。下面我们来详细解释这个类中的各个函数及其作用：

### `NodeOptions` 类

```cpp
class NodeOptions
{
public:
  explicit NodeOptions(rcl_allocator_t allocator = rcl_get_default_allocator());
  virtual ~NodeOptions() = default;
  NodeOptions(const NodeOptions & other);
  NodeOptions & operator=(const NodeOptions & other);
```

#### 构造函数

```cpp
explicit NodeOptions(rcl_allocator_t allocator = rcl_get_default_allocator());
```

这是 `NodeOptions` 类的构造函数，它接受一个可选的参数 `allocator`，默认值为 `rcl_get_default_allocator()`。`rcl_allocator_t` 是一个结构体，用于定义内存分配器。通过传递不同的分配器，可以自定义内存分配策略。如果没有指定分配器，将使用默认分配器。

#### 析构函数

```cpp
virtual ~NodeOptions() = default;
```

这是 `NodeOptions` 类的虚析构函数。虚析构函数允许在删除指向派生类对象的基类指针时正确地调用派生类的析构函数。这里使用 `= default` 表示使用编译器生成的默认析构函数。

#### 拷贝构造函数

```cpp
NodeOptions(const NodeOptions & other);
```

这是 `NodeOptions` 类的拷贝构造函数，它接受一个 `NodeOptions` 类型的常量引用 `other` 作为参数。拷贝构造函数用于创建一个新对象，并将已有对象的值复制到新对象中。

#### 赋值运算符重载

```cpp
NodeOptions & operator=(const NodeOptions & other);
```

这是 `NodeOptions` 类的赋值运算符重载函数，它接受一个 `NodeOptions` 类型的常量引用 `other` 作为参数。赋值运算符重载允许使用 `=` 运算符将一个对象的值赋给另一个对象。在这个函数中，将 `other` 对象的值复制到当前对象，并返回当前对象的引用。

### 总结

`rclcpp::NodeOptions` 类是 ROS 2 的 C++ 客户端库中的一个配置类，用于设置和管理节点的选项。这个类包含了构造函数、析构函数、拷贝构造函数和赋值运算符重载函数，以便正确地创建、销毁和复制 `NodeOptions` 对象。通过这个类，可以自定义节点的内存分配策略等选项。

##

```cpp
  const rcl_node_options_t * get_rcl_node_options() const;
  rclcpp::Context::SharedPtr context() const;
  NodeOptions & context(rclcpp::Context::SharedPtr context);
  const std::vector<std::string> & arguments() const;
  NodeOptions & arguments(const std::vector<std::string> & arguments);

  std::vector<rclcpp::Parameter> & parameter_overrides();
  const std::vector<rclcpp::Parameter> & parameter_overrides() const;
  NodeOptions & parameter_overrides(const std::vector<rclcpp::Parameter> & parameter_overrides);
  template <typename ParameterT>
  NodeOptions & append_parameter_override(const std::string & name, const ParameterT & value) {}
  bool use_global_arguments() const;
  NodeOptions & use_global_arguments(bool use_global_arguments);
```

这段代码是 ROS2 项目中的 rclcpp 库的一部分，主要涉及到 `NodeOptions` 类。`NodeOptions` 类用于配置节点（Node）的选项，如参数覆盖、上下文、命令行参数等。这些选项在创建节点时使用，以便根据用户需求定制节点的行为。

以下是各个函数的功能和含义：

1. `const rcl_node_options_t * get_rcl_node_options() const;`

   获取底层 ROS2 C 结构体 `rcl_node_options_t` 的指针，该结构体包含节点选项信息。

2. `rclcpp::Context::SharedPtr context() const;`

   获取当前节点选项所使用的上下文（Context）的共享指针。

3. `NodeOptions & context(rclcpp::Context::SharedPtr context);`

   设置当前节点选项的上下文（Context），并返回当前节点选项对象的引用，以支持链式调用。

4. `const std::vector<std::string> & arguments() const;`

   获取当前节点选项的命令行参数列表。

5. `NodeOptions & arguments(const std::vector<std::string> & arguments);`

   设置当前节点选项的命令行参数列表，并返回当前节点选项对象的引用，以支持链式调用。

6. `std::vector<rclcpp::Parameter> & parameter_overrides();`

   获取当前节点选项的参数覆盖列表（可修改版本）。

7. `const std::vector<rclcpp::Parameter> & parameter_overrides() const;`

   获取当前节点选项的参数覆盖列表（只读版本）。

8. `NodeOptions & parameter_overrides(const std::vector<rclcpp::Parameter> & parameter_overrides);`

   设置当前节点选项的参数覆盖列表，并返回当前节点选项对象的引用，以支持链式调用。

9. `template <typename ParameterT>`
   `NodeOptions & append_parameter_override(const std::string & name, const ParameterT & value) {}`

   向当前节点选项的参数覆盖列表中添加一个新的参数覆盖，并返回当前节点选项对象的引用，以支持链式调用。`ParameterT` 是模板参数，表示参数值的类型。

10. `bool use_global_arguments() const;`

    返回一个布尔值，表示当前节点选项是否使用全局参数。

11. `NodeOptions & use_global_arguments(bool use_global_arguments);`

    设置当前节点选项是否使用全局参数，并返回当前节点选项对象的引用，以支持链式调用。

通过这些函数，我们可以灵活地配置节点选项，以便在创建节点时根据需求定制节点的行为。

##

```cpp
  bool enable_rosout() const;
  NodeOptions & enable_rosout(bool enable_rosout);
  bool use_intra_process_comms() const;
  NodeOptions & use_intra_process_comms(bool use_intra_process_comms);
  bool enable_topic_statistics() const;
  NodeOptions & enable_topic_statistics(bool enable_topic_statistics);
  bool start_parameter_services() const;
  NodeOptions & start_parameter_services(bool start_parameter_services);
  bool start_parameter_event_publisher() const;
  NodeOptions & start_parameter_event_publisher(bool start_parameter_event_publisher);
  const rcl_clock_type_t & clock_type() const;
  NodeOptions & clock_type(const rcl_clock_type_t & clock_type);
  const rclcpp::QoS & clock_qos() const;
  NodeOptions & clock_qos(const rclcpp::QoS & clock_qos);
  bool use_clock_thread() const;
  NodeOptions & use_clock_thread(bool use_clock_thread);
  const rclcpp::QoS & parameter_event_qos() const;
  NodeOptions & parameter_event_qos(const rclcpp::QoS & parameter_event_qos);
  const rclcpp::QoS & rosout_qos() const;
  NodeOptions & rosout_qos(const rclcpp::QoS & rosout_qos);
  const rclcpp::PublisherOptionsBase & parameter_event_publisher_options() const;
  NodeOptions & parameter_event_publisher_options(
    const rclcpp::PublisherOptionsBase & parameter_event_publisher_options);
  bool allow_undeclared_parameters() const;
  NodeOptions & allow_undeclared_parameters(bool allow_undeclared_parameters);
  bool automatically_declare_parameters_from_overrides() const;
  NodeOptions & automatically_declare_parameters_from_overrides(
    bool automatically_declare_parameters_from_overrides);
  const rcl_allocator_t & allocator() const;
  NodeOptions & allocator(rcl_allocator_t allocator);
```

这段代码是 ROS2 项目中的 rclcpp 库的一部分，主要用于配置节点选项。rclcpp 是 ROS2 的 C++ 客户端库，提供了创建和管理 ROS2 节点所需的 API。在这个文件（node_options.hpp）中，定义了一个名为 `NodeOptions` 的类，它包含了一系列的成员函数，用于设置和获取节点的各种选项。

以下是每个函数的功能和含义：

1. `enable_rosout()` 和 `enable_rosout(bool enable_rosout)`：获取和设置是否启用 rosout 功能。rosout 是 ROS 的日志记录系统，用于收集和处理来自各个节点的日志消息。

2. `use_intra_process_comms()` 和 `use_intra_process_comms(bool use_intra_process_comms)`：获取和设置是否使用进程内通信。进程内通信允许同一进程中的节点直接相互通信，而无需通过网络层，从而提高通信性能。

3. `enable_topic_statistics()` 和 `enable_topic_statistics(bool enable_topic_statistics)`：获取和设置是否启用主题统计功能。主题统计功能可以收集有关发布者和订阅者之间消息传递的统计信息，如延迟、丢失率等。

4. `start_parameter_services()` 和 `start_parameter_services(bool start_parameter_services)`：获取和设置是否启动参数服务。参数服务允许节点通过服务调用来获取和设置参数值。

5. `start_parameter_event_publisher()` 和 `start_parameter_event_publisher(bool start_parameter_event_publisher)`：获取和设置是否启动参数事件发布器。参数事件发布器用于在参数值更改时发布相应的事件消息。

6. `clock_type()` 和 `clock_type(const rcl_clock_type_t & clock_type)`：获取和设置节点使用的时钟类型。时钟类型决定了节点如何获取当前时间，例如系统时钟、模拟时钟等。

7. `clock_qos()` 和 `clock_qos(const rclcpp::QoS & clock_qos)`：获取和设置时钟主题的 QoS（Quality of Service，服务质量）配置。QoS 配置影响了节点之间通信的可靠性、延迟等属性。

8. `use_clock_thread()` 和 `use_clock_thread(bool use_clock_thread)`：获取和设置是否为时钟服务使用单独的线程。使用单独的线程可以避免时钟服务阻塞其他节点功能。

9. `parameter_event_qos()` 和 `parameter_event_qos(const rclcpp::QoS & parameter_event_qos)`：获取和设置参数事件主题的 QoS 配置。

10. `rosout_qos()` 和 `rosout_qos(const rclcpp::QoS & rosout_qos)`：获取和设置 rosout 主题的 QoS 配置。

11. `parameter_event_publisher_options()` 和 `parameter_event_publisher_options(const rclcpp::PublisherOptionsBase & parameter_event_publisher_options)`：获取和设置参数事件发布器的选项。

12. `allow_undeclared_parameters()` 和 `allow_undeclared_parameters(bool allow_undeclared_parameters)`：获取和设置是否允许使用未声明的参数。如果允许，节点可以在运行时动态添加新参数。

13. `automatically_declare_parameters_from_overrides()` 和 `automatically_declare_parameters_from_overrides(bool automatically_declare_parameters_from_overrides)`：获取和设置是否自动从覆盖值中声明参数。如果启用，节点将在启动时自动声明传递给它的参数覆盖值。

14. `allocator()` 和 `allocator(rcl_allocator_t allocator)`：获取和设置节点使用的内存分配器。内存分配器决定了节点如何分配和释放内存资源。

通过这些成员函数，用户可以灵活地配置节点的各种选项，以满足不同的应用需求。

> [!NOTE]
> 这个或许也是可以用于 节点热备份 3. `enable_topic_statistics()` 和 `enable_topic_statistics(bool enable_topic_statistics)`：获取和设置是否启用主题统计功能。主题统计功能可以收集有关发布者和订阅者之间消息传递的统计信息，如延迟、丢失率等。

##

```cpp
private:
  mutable std::unique_ptr<rcl_node_options_t, void (*)(rcl_node_options_t *)> node_options_;
  rclcpp::Context::SharedPtr context_{rclcpp::contexts::get_global_default_context()};
  std::vector<std::string> arguments_{};
  std::vector<rclcpp::Parameter> parameter_overrides_{};
  bool use_global_arguments_{true};
  bool enable_rosout_{true};
  bool use_intra_process_comms_{false};
  bool enable_topic_statistics_{false};
  bool start_parameter_services_{true};
  bool start_parameter_event_publisher_{true};
  rcl_clock_type_t clock_type_{RCL_ROS_TIME};
  rclcpp::QoS clock_qos_ = rclcpp::ClockQoS();
  bool use_clock_thread_{true};
  rclcpp::QoS parameter_event_qos_ = rclcpp::ParameterEventsQoS(
    rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events));
  rclcpp::QoS rosout_qos_ = rclcpp::RosoutQoS();
  rclcpp::PublisherOptionsBase parameter_event_publisher_options_ = rclcpp::PublisherOptionsBase();
  bool allow_undeclared_parameters_{false};
  bool automatically_declare_parameters_from_overrides_{false};
  rcl_allocator_t allocator_{rcl_get_default_allocator()};
};
} // namespace rclcpp
```

以下是每个函数的功能和含义：

1. `enable_rosout()` 和 `enable_rosout(bool enable_rosout)`：获取和设置是否启用 rosout 功能。rosout 是 ROS 的日志记录系统，用于收集和处理来自各个节点的日志消息。
2. `use_intra_process_comms()` 和 `use_intra_process_comms(bool use_intra_process_comms)`：获取和设置是否使用进程内通信。进程内通信允许同一进程中的节点直接相互通信，而无需通过网络层，从而提高通信性能。
3. `enable_topic_statistics()` 和 `enable_topic_statistics(bool enable_topic_statistics)`：获取和设置是否启用主题统计功能。主题统计功能可以收集有关发布者和订阅者之间消息传递的统计信息，如延迟、丢失率等。
4. `start_parameter_services()` 和 `start_parameter_services(bool start_parameter_services)`：获取和设置是否启动参数服务。参数服务允许节点通过服务调用来获取和设置参数值。
5. `start_parameter_event_publisher()` 和 `start_parameter_event_publisher(bool start_parameter_event_publisher)`：获取和设置是否启动参数事件发布器。参数事件发布器用于在参数值更改时发布相应的事件消息。
6. `clock_type()` 和 `clock_type(const rcl_clock_type_t & clock_type)`：获取和设置节点使用的时钟类型。时钟类型决定了节点如何获取当前时间，例如系统时钟、模拟时钟等。
7. `clock_qos()` 和 `clock_qos(const rclcpp::QoS & clock_qos)`：获取和设置时钟主题的 QoS（Quality of Service，服务质量）配置。QoS 配置影响了节点之间通信的可靠性、延迟等属性。
8. `use_clock_thread()` 和 `use_clock_thread(bool use_clock_thread)`：获取和设置是否为时钟服务使用单独的线程。使用单独的线程可以避免时钟服务阻塞其他节点功能。
9. `parameter_event_qos()` 和 `parameter_event_qos(const rclcpp::QoS & parameter_event_qos)`：获取和设置参数事件主题的 QoS 配置。
10. `rosout_qos()` 和 `rosout_qos(const rclcpp::QoS & rosout_qos)`：获取和设置 rosout 主题的 QoS 配置。
11. `parameter_event_publisher_options()` 和 `parameter_event_publisher_options(const rclcpp::PublisherOptionsBase & parameter_event_publisher_options)`：获取和设置参数事件发布器的选项。
12. `allow_undeclared_parameters()` 和 `allow_undeclared_parameters(bool allow_undeclared_parameters)`：获取和设置是否允许使用未声明的参数。如果允许，节点可以在运行时动态添加新参数。
13. `automatically_declare_parameters_from_overrides()` 和 `automatically_declare_parameters_from_overrides(bool automatically_declare_parameters_from_overrides)`：获取和设置是否自动从覆盖值中声明参数。如果启用，节点将在启动时自动声明传递给它的参数覆盖值。
14. `allocator()` 和 `allocator(rcl_allocator_t allocator)`：获取和设置节点使用的内存分配器。内存分配器决定了节点如何分配和释放内存资源。

通过这些成员函数，用户可以灵活地配置节点的各种选项，以满足不同的应用需求。

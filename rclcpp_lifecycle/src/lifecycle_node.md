## 下面这段代码是在 ros2 项目中 rclcpp_lifecycle 相关的代码，请联系所有函数之间可能的相互联系，梳理、解释这份文件的功能、含义以及调用关系（中文）。（请给出详细完善的回答，不限制字数）

```cpp
all code
```

这个 cpp 文件定义了 rclcpp_lifecycle 中的 LifecycleNode 类。LifecycleNode 继承自 rclcpp 中的 Node 类,并增加了生命周期管理的功能。

LifecycleNode 构造函数中,初始化了许多 rclcpp::node_interfaces 接口的实例,这些接口包括:

- NodeBaseInterface:节点基本接口
- NodeClockInterface:节点时钟接口
- NodeGraphInterface:节点图接口
- NodeLoggingInterface:节点日志接口
- NodeTimeSourceInterface:节点时间源接口
- NodeTimersInterface:节点计时器接口
- NodeTopicsInterface:节点话题接口
- NodeServicesInterface:节点服务接口
- NodeParametersInterface:节点参数接口
- NodeWaitablesInterface:节点可等待接口

同时,LifecycleNode 构造函数中也注册了生命周期变迁对应的回调函数,包括:

- on_configure
- on_cleanup
- on_shutdown
- on_activate
- on_deactivate
- on_error

LifecycleNode 中定义了许多成员函数,这些函数大致可以分为以下几类:

1. 获取节点信息的接口,如 get_name()、get_namespace()、get_logger()等。

2. 生命周期管理接口,如 register_on_configure()、trigger_transition()、configure()、activate()等。这些接口可以用于注册生命周期回调,触发生命周期变迁和获取当前生命周期状态。

3. rclcpp 原生节点功能的代理接口,如创建回调组、声明参数、设置参数、获取参数、列举参数、获取话题信息、获取服务信息等。这些接口代理调用 LifecycleNode 持有的 rclcpp::Node 实例完成。

4. 获取 LifecycleNode 持有的各种接口实例的接口,如 get_node_base_interface()、get_node_clock_interface()等。

5. 管理实体和计时器的接口,如 add_managed_entity()和 add_timer_handle(),用于 LifecycleNode 生命周期管理其他实体。

所以,总的来说,LifecycleNode 类在 rclcpp::Node 的基础上,增加了生命周期管理的功能,并可以管理其他实体的生命周期。LifecycleNode 的生命周期状态变迁会触发注册的回调函数。

---

好的,这里我会结合 LifecycleNode 类的具体代码,进一步完善和深化上述分析:

1. LifecycleNode 的构造函数中,会初始化一个 LifecycleNodeInterfaceImpl 的实例 impl*,它实现了实际的生命周期管理逻辑。在构造函数中,LifecycleNode 会注册 6 个生命周期回调函数 on_configure、on_cleanup、on_shutdown、on_activate、on_deactivate 和 on_error 到 impl*中。

2. impl\_的 init()函数会根据 enable_communication_interface 的值,决定是否启动生命周期状态的 ROS 通信接口。如果启用,它会创建一个生命周期状态发布器和订阅器。

3. 生命周期变迁触发函数,如 configure()、activate()等,会调用 impl\_的 trigger_transition()函数触发实际的生命周期变迁。trigger_transition()会首先检查变迁是否允许,然后调用相应的生命周期回调,如果所有回调返回 SUCCESS,则执行生命周期状态的切换。

4. 生命周期状态变迁的回调函数 on_configure()、on_activate()等在默认实现中返回 SUCCESS。用户可以重写这些函数以实现自定义的生命周期逻辑。

5. 添加管理实体的函数 add*managed_entity()会将管理实体的弱智能指针添加到 impl*中。impl\_在生命周期变迁时,会检查这些弱指针,并根据实际情况决定是否调用相应实体的生命周期触发函数。

6. 添加计时器句柄的函数 add_timer_handle()用于在生命周期变迁时停止或重启计时器。

7. 获取生命周期状态的函数 get*current_state()会直接调用 impl*的同名函数获取当前状态。获取生命周期变迁图的函数 get*transition_graph()也会调用 impl*。

8. register*on_xxx()函数用于注册生命周期回调。这些函数会将输入的回调函数注册到 impl*中。unregister_on_xxx()用于注销回调函数。

9. 其他像创建回调组、参数管理、话题管理等功能都是直接调用 LifecycleNode 包含的 rclcpp::Node 实例完成的。

所以,总体来说,LifecycleNode 通过持有的 LifecycleNodeInterfaceImpl 实例实现了生命周期管理的逻辑,而其他 ROS 功能还是委托给其内含的 rclcpp::Node 实例完成。LifecycleNode 将两者联系起来,并且对外提供了简单易用的生命周期管理接口。

---

这里是 rclcpp_lifecycle 包中 LifecycleNode 类的代码。LifecycleNode 继承自 LifecycleNodeInterface,实现了 ROS 生命周期管理。

该类的主要功能如下:

- 实现生命周期管理,提供配置(configure)、清理(cleanup)、激活(activate)、停用(deactivate)和关机(shutdown)等生命周期方法。
- 实现各种回调函数注册,如注册配置回调(register_on_configure)、清理回调(register_on_cleanup)等。
- 提供获取当前状态(get_current_state)、可用状态(get_available_states)、可用转换(get_available_transitions)以及转换图(get_transition_graph)等方法。
- 实现节点接口,提供创建回调组(create_callback_group)、声明参数(declare_parameter)、设置参数(set_parameter)、获取参数(get_parameter)等节点功能。
- 实现各种接口获取方法,如获取节点基础接口(get_node_base_interface)、时钟接口(get_node_clock_interface)、日志接口(get_node_logging_interface)等。

该类主要依赖 rclcpp 中的各种节点接口实现节点功能,同时依赖 LifecycleNodeInterfaceImpl 类实现生命周期管理。

关于上述代码的 UML 图如下:

```mermaid
classDiagram
LifecycleNodeInterface <|-- LifecycleNode
LifecycleNode --|> node_interfaces::LifecycleNodeInterface
LifecycleNode --> rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeClockInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
LifecycleNode --> LifecycleNodeInterfaceImpl
LifecycleNodeInterfaceImpl --> lifecycle_msgs::msg::State
LifecycleNodeInterfaceImpl --> lifecycle_msgs::msg::Transition
```

---

这里给出 LifecycleNode 类的完整 UML 图:

```mermaid
classDiagram
class LifecycleNodeInterface
LifecycleNodeInterface : +CallbackReturn
LifecycleNodeInterface : +on_activate()
LifecycleNodeInterface : +on_deactivate()

LifecycleNodeInterface <|-- LifecycleNode

class LifecycleNode
LifecycleNode : -node_base_
LifecycleNode : -node_graph_
LifecycleNode : -node_logging_
LifecycleNode : -node_timers_
LifecycleNode : -node_topics_
LifecycleNode : -node_services_
LifecycleNode : -node_clock_
LifecycleNode : -node_parameters_
LifecycleNode : -node_time_source_
LifecycleNode : -node_waitables_
LifecycleNode : -node_options_
LifecycleNode : -impl_

LifecycleNode --> LifecycleNodeInterfaceImpl

class LifecycleNodeInterfaceImpl
LifecycleNodeInterfaceImpl : -current_state
LifecycleNodeInterfaceImpl : -available_states
LifecycleNodeInterfaceImpl : -available_transitions
LifecycleNodeInterfaceImpl : -transition_graph
LifecycleNodeInterfaceImpl : +init()
LifecycleNodeInterfaceImpl : +register_callback()
LifecycleNodeInterfaceImpl : +trigger_transition()
LifecycleNodeInterfaceImpl : +get_current_state()
LifecycleNodeInterfaceImpl : +get_available_states()
LifecycleNodeInterfaceImpl : +get_available_transitions()
LifecycleNodeInterfaceImpl : +get_transition_graph()
LifecycleNodeInterfaceImpl : +on_activate()
LifecycleNodeInterfaceImpl : +on_deactivate()
LifecycleNodeInterfaceImpl : +add_managed_entity()
LifecycleNodeInterfaceImpl : +add_timer_handle()

LifecycleNodeInterfaceImpl --> lifecycle_msgs::msg::State
LifecycleNodeInterfaceImpl --> lifecycle_msgs::msg::Transition

LifecycleNode --> rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeClockInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
LifecycleNode --> rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
```

从上图可以看出:

- LifecycleNode 继承自 LifecycleNodeInterface,并依赖 LifecycleNodeInterfaceImpl 实现具体的生命周期管理功能。

- LifecycleNodeInterface 定义了回调函数返回值 CallbackReturn 以及 on_activate 和 on_deactivate 虚函数。

- LifecycleNode 中包含多个节点接口的 shared_ptr,这些接口实现了节点的基本功能。LifecycleNode 也直接依赖这些接口。

- LifecycleNodeInterfaceImpl 中维护了当前状态、可用状态、可用转换和转换图等信息,并提供相应获取方法。它也定义了注册回调、触发转换和生命周期回调等方法。

- LifecycleNodeInterfaceImpl 依赖 lifecycle_msgs 中的 State 和 Transition 消息。

- 综上,LifecycleNode 通过继承 LifecycleNodeInterface,依赖 LifecycleNodeInterfaceImpl 和 rclcpp 中的各种节点接口,实现了生命周期管理和普通节点的功能。

## 请问 LifecycleNodeInterfaceImpl 中的 transition_graph 是什么？

transition_graph 在 LifecycleNodeInterfaceImpl 中表示转换图。它存储了节点所有可用的状态转换。

具体来说,transition_graph 是一个 std::vector<Transition>,其中每个 Transition 代表一个状态转换,包含三个消息:

- start_state:转换开始的状态
- goal_state:转换目标状态
- transition:触发该转换的转换消息

所以,transition_graph 表达了节点整个生命周期管理的转换逻辑,覆盖了所有可达状态和转换。LifecycleNode 通过调用 LifecycleNodeInterfaceImpl::get_transition_graph() 可以获取此转换图,进而做一些状态变换的检查和限制。

举个简单的示例,一个节点的 transition_graph 可能如下:

```
std::vector<Transition> transition_graph
{
  {start_state: "unconfigured",   goal_state: "inactive",   transition: "configure"},
  {start_state: "inactive",      goal_state: "active",     transition: "activate"},
  {start_state: "active",        goal_state: "inactive",   transition: "deactivate"},
  {start_state: "inactive",      goal_state: "unconfigured",transition: "unconfigure"},
  {start_state: "unconfigured",   goal_state: "finalized",  transition: "cleanup"}
}
```

该转换图表示节点支持未配置(unconfigured)、非激活(inactive)、激活(active)和 finalezed 四种状态,以及 configure、activate、deactivate、unconfigure 和 cleanup 五种状态转换。

所以,transition_graph 实际上定义了整个节点的生命周期及状态机。LifecycleNode 通过它实现状态的正确转换和限制。

## 你能解释一下 LifecycleNodeInterfaceImpl 类是如何工作的吗？

LifecycleNodeInterfaceImpl 类实现了 LifecycleNode 的实际生命周期管理逻辑。它主要通过以下几个方面工作:

1. 维护生命周期状态机:它维护了一个生命周期状态结构 current*state*,记录当前的生命周期状态。同时也维护了生命周期状态之间允许的变迁关系图 transition*graph*。

2. 生命周期回调注册和执行:它维护了一个生命周期回调注册表 callbacks\_,用于注册不同生命周期状态的回调函数。当生命周期状态变化时,它会检查新状态对应的回调函数,并执行这些函数。只有当所有回调函数返回 SUCCESS 时,生命周期状态切换才会成功。

3. 管理生命周期变迁:其 trigger*transition()函数用于触发生命周期变迁。它会首先检查变迁是否允许,如果允许则依次执行变迁前状态、变迁状态和变迁后状态的回调函数。如果所有回调返回 SUCCESS,则更新 current_state*中的状态。

4. 向 ROS 环境发布生命周期状态:如果构造时启用了 enable_communication_interface,它会创建一个生命周期状态发布器,并在生命周期状态变化时发布新状态。同时它也创建了一个生命周期状态订阅器,以监听来自其他节点的生命周期指令。

5. 管理实体生命周期:它通过实例变量 managed*entities*维护了一系列弱指针,指向被管理的实体。在生命周期变迁前,它会检查这些实体,并调用实体暴露的生命周期接口触发实体生命周期状态变化。

6. 管理计时器:它通过实例变量 timer*handles*维护了一系列共享指针,指向 LifecycleNode 持有的计时器。在生命周期变迁时,会根据实际情况停止或重启这些计时器。

所以,LifecycleNodeInterfaceImpl 实现了生命周期状态机,生命周期回调管理,生命周期变迁触发,实体生命周期管理和计时器管理等功能,是 LifecycleNode 生命周期能力的关键。

## 你能给我一个 LifecycleNodeInterfaceImpl 的使用例子吗？

这里是一个 LifecycleNodeInterfaceImpl 的使用例子:

```cpp
#include "lifecycle_node_interface_impl.hpp"

int main() {
  // 创建生命周期节点接口实现实例
  auto impl = std::make_shared<rclcpp_lifecycle::LifecycleNodeInterfaceImpl>();

  // 注册生命周期回调函数
  impl->register_on_configure(std::bind(&on_configure, std::placeholders::_1));
  impl->register_on_activate(std::bind(&on_activate, std::placeholders::_1));
  impl->register_on_deactivate(std::bind(&on_deactivate, std::placeholders::_1));

  // 触发配置生命周期状态变迁
  impl->trigger_transition(
    rclcpp_lifecycle::Transition::TRANSITION_CONFIGURE);

  // 触发激活生命周期状态变迁
  impl->trigger_transition(
    rclcpp_lifecycle::Transition::TRANSITION_ACTIVATE);

  // 运行循环....

  // 触发停用生命周期状态变迁
  impl->trigger_transition(
    rclcpp_lifecycle::Transition::TRANSITION_DEACTIVATE);
}

rcl_lifecycle_ret_t on_configure(const rclcpp_lifecycle::State &) {
  // 配置逻辑...
  return rclcpp_lifecycle::RET_OK;
}

rcl_lifecycle_ret_t on_activate(const rclcpp_lifecycle::State &) {
  // 激活逻辑...
  return rclcpp_lifecycle::RET_OK;
}

rcl_lifecycle_ret_t on_deactivate(const rclcpp_lifecycle::State &) {
  // 停用逻辑...
  return rclcpp_lifecycle::RET_OK;
}
```

这个例子中,我们创建了一个 LifecycleNodeInterfaceImpl 实例,并注册了 configure、activate 和 deactivate 三个生命周期状态的回调函数。然后我们触发配置、激活和停用生命周期状态变迁,在回调函数中执行相应的逻辑。LifecycleNodeInterfaceImpl 会管理生命周期状态机,在变迁成功后更新状态。

所以,这个例子展示了如何使用 LifecycleNodeInterfaceImpl 来实现自定义生命周期管理逻辑,通过注册回调并触发状态变迁,我们可以在节点生命周期的不同阶段执行我们需要的逻辑。

## LifecycleNodeInterfaceImpl 是否支持多线程？

LifecycleNodeInterfaceImpl 本身是线程安全的,但是它管理的生命周期回调函数和被管理实体的生命周期变迁并不保证线程安全。

具体来说:

1. LifecycleNodeInterfaceImpl 内部的生命周期状态机和发布器/订阅器是线程安全的。不同线程调用 trigger_transition()触发生命周期变迁,或 get_current_state()获取当前状态时,都可以得到正确的结果。

2. 但是,生命周期回调函数是由用户注册的,LifecycleNodeInterfaceImpl 无法保证这些函数是线程安全的。如果多个线程触发同一个生命周期状态的变迁,这些回调有可能从不同线程并发执行,引起问题。

3. 同样,LifecycleNodeInterfaceImpl 管理的实体也有可能不是线程安全的。虽然 LifecycleNodeInterfaceImpl 会在主线程中调用实体的生命周期函数触发状态变迁,但实体内部的逻辑有可能是多线程的,这可能导致问题。

4. 如果用户向同一 LifecycleNodeInterfaceImpl 实例注册了多个同步的生命周期回调或管理的实体,而这些回调和实体本身又不是线程安全的,那么触发生命周期变迁时也会出现问题。

所以,总的来说,LifecycleNodeInterfaceImpl 可以在多线程环境下使用,但是用户必须保证:

1. 注册的生命周期回调函数是线程安全的;

2. 被管理的实体是线程安全的;

3. 不向同一个 LifecycleNodeInterfaceImpl 实例注册多个非线程安全的回调或实体。

如果能满足这些条件,LifecycleNodeInterfaceImpl 是支持多线程的。否则,用户必须采取其他手段,如使用互斥量确保关键部分的原子性、让 LifecycleNodeInterfaceImpl 在单线程中运行等,以解决线程安全问题。

## 如何在 LifecycleNodeInterfaceImpl 中添加新的生命周期状态？

在 LifecycleNodeInterfaceImpl 中添加新的生命周期状态主要涉及以下几个步骤:

1. 在 rclcpp_lifecycle::Transition 枚举中添加新的生命周期状态变迁定义。例如:

```cpp
enum class Transition {
  ...
  TRANSITION_NEW_STATE  // 新的状态变迁
};
```

2. 在 rclcpp_lifecycle::State 结构中添加新的生命周期状态定义。例如:

```cpp
struct State {
  ...
  constexpr static const char *const NEW_STATE = "new_state";  // 新的状态字符串
};
```

3. 在 transition*graph*数据成员中添加指向新状态的变迁。例如:

```cpp
// 允许从当前状态切换到新状态
graph[current_state_][State::NEW_STATE] = Transition::TRANSITION_NEW_STATE;

// 允许从新状态切换回其他状态
graph[State::NEW_STATE][other_state] = Transition::TRANSITION_TO_OTHER_STATE;
```

4. 添加新状态对应的生命周期回调函数,如:

```cpp
register_on_new_state(std::bind(&on_new_state, this));
```

5. 实现新状态的回调函数,如:

```cpp
rcl_lifecycle_ret_t on_new_state(const State & state) {
  // 执行新状态逻辑
  return rclcpp_lifecycle::RET_OK;
}
```

6. 更新状态切换相关逻辑,在 trigger_transition()中处理到新状态的情况,如:

```cpp
case Transition::TRANSITION_NEW_STATE:
  // 执行变迁前状态的回调
  // 执行新状态TRANSITION_NEW_STATE的回调
  // 如果成功,更新current_state_到State::NEW_STATE
  break;
```

7. 在发布生命周期状态变化话题时,增加新状态的情况。

8. 处理来自其他节点的新状态变迁指令。

通过以上逻辑,我们可以向 LifecycleNodeInterfaceImpl 中添加新的生命周期状态,并定义状态变迁,注册回调,在状态变化时执行相关逻辑。整个生命周期管理机制可以以此扩展支持更加丰富的生命周期模型。

## UML

这里是一个 rclcpp_lifecycle 包的完整 UML 类图:

```mermaid
classDiagram

LifecycleNodeInterfaceImpl <|-- LifecycleNodeInterface
LifecycleNode <.. LifecycleNodeInterface
LifecycleNode -- LifecycleNodeInterfaceImpl

class LifecycleNodeInterfaceImpl{
    +register_on_configure(callback)
    +register_on_activate(callback)
    +register_on_deactivate(callback)
    +trigger_transition(transition)
    +get_current_state()
    ...
}

class LifecycleNodeInterface{
    <<interface>>
    +configure()
    +activate()
    +deactivate()
    +get_current_state()
    ...
}

LifecycleNodeInterfaceImpl -- State
LifecycleNodeInterfaceImpl -- Transition
LifecycleNodeInterfaceImpl -- TransitionGraph

class State{
  <<enumeration>>
  UNSPECIFIED
  INACTIVE
  ACTIVE
  ...
}

enum Transition{
    TRANSITION_CONFIGURE
    TRANSITION_ACTIVATE
    TRANSITION_DEACTIVATE
    ...
}

class TransitionGraph


LifecycleNode -- rclcpp::Node
LifecycleNodeInterfaceImpl -- rclcpp::Node
LifecycleNode -- LifecyclePublisher
LifecycleNode -- LifecycleService

class LifecyclePublisher{
  ...
}

class LifecycleService{
  ...
}

class rclcpp::Node{
  ...
}
```

从这个类图中,我们可以看到:

1. LifecycleNodeInterfaceImpl 和 LifecycleNodeInterface 提供生命周期管理的核心功能。

2. LifecycleNode 通过组合 LifecycleNodeInterface 使用生命周期管理能力,同时也依赖 rclcpp::Node 提供其他 ROS 节点功能。

3. State、Transition 和 TransitionGraph 定义了生命周期管理所需的基本概念。

4. LifecyclePublisher 和 LifecycleService 为生命周周期感知的发布器和服务提供支持。

5. 整个 rclcpp_lifecycle 包的基础是 rclcpp,它提供了 ROS2 C++ API。

所以,这个 UML 类图很好地描述了 rclcpp_lifecycle 包的架构设计。生命周期管理的核心由 LifecycleNodeInterfaceImpl 提供,而 LifecycleNode 则在此基础上提供一个易用的生命周期节点,并且利用 rclcpp 实现其他 ROS 功能,使得生命周期能力和 ROS 能力高度集成。LifecyclePublisher 和 LifecycleService 等其他类也在此架构上进一步拓展生命周期感知能力。

可以说,这个类图全面而清晰地描述了 rclcpp_lifecycle 的设计,有助于我们理解这个包实现生命周期管理的架构思想。理解了这个 UML 图,我们对该包的整体结构和工作机理就有一个清晰的认知。

## LifecycleNodeInterfaceImpl 的 register_on_configure 函数是用来做什么的？

LifecycleNodeInterfaceImpl 的 register_on_configure()函数用于注册节点配置生命周期状态的回调函数。

当 LifecycleNodeInterfaceImpl 准备切换到配置(CONFIGURED)生命周期状态时,它会调用所有注册在这个状态的回调函数。只有当所有回调函数返回 SUCCESS 时,状态切换才会成功。

所以,用户可以向 LifecycleNodeInterfaceImpl 注册对配置状态感兴趣的回调,在这个回调中执行节点配置所需的初始化逻辑。例如:

```cpp
// 注册配置回调
impl->register_on_configure(std::bind(&on_configure, std::placeholders::_1));

// 配置回调函数
rcl_lifecycle_ret_t
on_configure(const State &)
{
  // 配置逻辑
  // ...
  return rclcpp_lifecycle::RET_OK;
}
```

当 LifecycleNodeInterfaceImpl 切换到配置状态时,on_configure()回调会被调用。用户可以在这个回调中执行:

- 参数解析
- ROS 资源(话题、服务、参数等)初始化
- 其他初始化逻辑

等等。

只有当 on_configure()和其他所有配置回调返回 SUCCESS,LifecycleNodeInterfaceImpl 的状态才会切换到 CONFIGURED。

所以,register_on_configure()的主要目的是允许用户在配置生命周期状态时执行自定义的初始化逻辑,给用户机会 configure 节点所需的资源和功能。用户注册的回调直接决定了节点是否可以配置成功以及如何配置。

这也是 LifecycleNode 机制的重要目的之一:通过生命周期回调允许用户在节点生命周期的每个关键阶段执行所需的逻辑,实现高度定制化的节点行为。register_on_configure()就是 LifecycleNodeInterfaceImpl 提供的用于这个目的的机制之一。

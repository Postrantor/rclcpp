# API Review for `rclcpp` from March 2020

## Notes

### Off-Topic Questions

[rclcpp_action] There exists a thread-safe and non-thread-safe way to get the goal result from an action client. We probably want to remove the public interface to the non-thread safe call (or consolidate somehow): https://github.com/ros2/rclcpp/issues/955

> [rccpp_action]存在一种线程安全和非线程安全的方法来从操作客户端获取目标结果。我们可能希望删除非线程安全调用的公共接口(或以某种方式合并):https://github.com/ros2/rclcpp/issues/955

`rclcpp_action` is out of scope atm.

> `rclcpp_action `超出 atm 范围。

**Notes from 2020-03-19**: To be handled in separate API review.

## Architecture

### Calling Syntax and Keeping Node-like Class APIs in Sync

Currently, much of the API is exposed via the `rclcpp::Node` class, and due to the nature of the current architecture there is a lot of repeated code to expose these methods and then call the implementations which are in other classes like `rclcpp::node_interfaces::NodeTopics`, for example.

> 目前，大部分 API 都是通过`rclcpp::Node`类公开的，并且由于当前体系结构的性质，有很多重复的代码来公开这些方法，然后调用其他类中的实现，例如`rclcpp:Node_interfaces::NodeTopics`。

Also, we have other versions of the class `rclcpp::Node` with different semantics and interfaces, like `rclcpp_lifecycle::LifecycleNode`, and we have been having trouble keeping the interface provided there up to date with how things are done in `rclcpp::Node`. Since `LifecycleNode` has a different API from `Node` in some important cases, it does not just inherit from `Node`.

> 此外，我们还有其他版本的类`rclcpp::Node`，它们具有不同的语义和接口，如`rclccp_lifecycle::LifecycleNode`，并且我们一直很难保持提供的接口与`rclcpp:Node`中的操作方式保持最新。由于在某些重要情况下，`LifecycleNode`具有与`Node`不同的 API，因此它不仅仅继承自`Node`。

There are two main proposals (as I see it) to try and address this issue, either (a) break up the functionality in `Node` so that it is in separate classes and make `Node` multiple inherit from those classes, and then `LifecycleNode` could selectively inherit from those as well, or (b) change our calling convention from `node-do_thing(...)` to be `do_thing(node, ...)`.

> 有两个主要的建议(如我所见)来尝试和解决这个问题，要么(a)将`Node`中的功能分解为单独的类，并使`Node`多个继承自这些类，然后`LifecycleNode`也可以选择性地继承这些类，要么(b)将我们的调用约定从`Node-do_thing(…)`更改为`do_thing`(Node，…)`。

For (a) which commonly referred to as the [Policy Based Design Pattern](https://en.wikipedia.org/wiki/Modern_C%2B%2B_Design#Policy-based_design), we'd be reversing previous design decisions which we discussed at length where we decided to use composition over inheritance for various reasons.

> 对于(a)，通常称为[基于策略的设计模式](https://en.wikipedia.org/wiki/Modern_C%2B%2B_Design#Policy-based_design)，我们将扭转之前的设计决定，我们在这里详细讨论了这些决定，因为各种原因，我们决定使用组合而不是继承。

One of the reasons was testing, with the theory that having simpler separate interfaces we could more easily mock them as needed for testing.

> 其中一个原因是测试，因为有了更简单的独立接口，我们可以更容易地模拟测试所需的接口。

The testing goal would still be met, either by keeping the "node_interface" classes as-is or by mocking the classes that node would multiple inherit from, however it's harder to indicate that a function needs a class that multiple inherits from several classes but not others.

> 通过保持`node_interface`类的原样，或者通过模拟节点将从多个类继承的类，仍然可以实现测试目标，但是很难指出一个函数需要一个从多个类别继承而不是从其他类别继承的类。

Also having interdependency between the classes which are inherited from is a bit complicated in this design pattern.

> 在这种设计模式中，继承自的类之间的相互依赖性也有点复杂。

For (b), we would be changing how we recommend all code be written (not a trivial thing to do at all), because example code like `auto pub = node-create_publsiher(...);` would be come `auto pub = create_publisher(node, ...);`.

> 对于(b)，我们将改变我们建议编写所有代码的方式(根本不是一件小事)，因为示例代码如`autopub=node-create_publicher(…)；`将是`autopub=create_publisher(node，…)；`。

This has some distinct advantages, however, in that it allows us to write these functions, like `create_publisher(node, ...)`, so that the node argument can be any class that meets the criteria of the function.

> 然而，这有一些明显的优点，因为它允许我们编写这些函数，如`create_publisher(node，…)`，这样 node 参数可以是满足函数标准的任何类。

That not only means that when we add a feature it automatically works with `Node` and `LifecycleNode` without adding anything to them, it also means that user defined `Node`-like classes will also work, even if they do not inherit from or provide the complete interface for `rclcpp::Node`.

> 这不仅意味着当我们添加一个功能时，它会自动与`Node`和`LifecycleNode`一起工作，而不向它们添加任何内容，还意味着用户定义的类似`Node`的类也会工作，即使它们没有继承自`rccpp::Node`或为其提供完整的接口。

Another major downside of this approach is discoverability of the API when using auto-completion in text editors, as `node-<tab` will often give you a list of methods to explore, but with the new calling convention, there's not way to get an auto complete for code who's first argument is a `Node`-like class.

> 这种方法的另一个主要缺点是当在文本编辑器中使用自动补全时，API 的可发现性，因为`node-<tab`通常会给您一个方法列表，但使用新的调用约定，无法为第一个参数是`node`类的代码获得自动补全。

Both of the above approaches address some of the main concerns, which are: keeping `Node` and `LifecycleNode` in sync, reducing the size of the `Node` class so it is more easily maintained, documented, and so that related functions are grouped more clearly.

> 上述两种方法都解决了一些主要问题，即:保持`Node`和`LifecycleNode`同步，减少`Node`类的大小，以便更容易维护和记录，并使相关函数分组更清晰。

- https://github.com/ros2/rclcpp/issues/898
- https://github.com/ros2/rclcpp/issues/509
- https://github.com/ros2/rclcpp/issues/855
- https://github.com/ros2/rclcpp/issues/985

subnode feature is in rclcpp::Node only, complicating "node using" API designs

> -子节点特性仅在 rclcpp:Node 中，使`使用节点`API 设计复杂化

- http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2014/n4174.pdf
- https://en.wikipedia.org/wiki/Uniform_Function_Call_Syntax#C++_proposal

"Many programmers are tempted to write member functions to get the benefits of the member function syntax (e.g. "dot-autocomplete" to list member functions);[6] however, this leads to excessive coupling between classes.[7]"

> -`许多程序员倾向于编写成员函数以获得成员函数语法的好处(例如`dot autocomplete`列出成员函数)；[6]然而，这会导致类之间的过度耦合。[7]`

**Suggested Action**: Document the discussion and defer until Foxy.

> **建议行动**:记录讨论并推迟到 Foxy。

**Notes from 2020-03-19**:

- Another version of (b) could be to have classes that are constructed with node, e.g. `Publisher(node, ...)` rather than `node-create_publisher(...)`

> -(b)的另一个版本可以是使用节点构造类，例如`Publisher(node，…)`而不是`node-create_Publisher(…)``

- (tfoote) interface class? `NodeInterface<NodeLike::something(node_like)`

> -(tfoote)接口类`NodeInterface＜NodeLike:something(node_like)`

- DRY?

- `NodeInterface<LifecycleNode::<tab` -only life cycle node methods

> -`NodeInterface<生命周期节点:<tab `-仅生命周期节点方法

- (karsten) use interface classes directly, e.g. node-get_X_interface()-do_thing()

> -(karsten)直接使用接口类，例如 node-get_X_interface()-do_thing()

- (dirk) use macros to add methods to class

> -(dirk)使用宏向类添加方法

- Question: Do we want tab-completable API (specifically list of member functions)?

> -问题:我们是否需要制表符可完成的 API(特别是成员函数列表)？

- Question: Is consistency in calling between core and non-core features more important than tab-completion?

> -问题:核心和非核心功能调用的一致性是否比制表符完成更重要？

- Add better example of adding new feature and not needing to touch `rclcpp::Node`.

> -添加添加新功能的更好示例，无需触摸`rclcpp::Node`。

- (dirk) methods and free functions not mutually exclusive.

> -(dirk)方法和自由函数不互斥。

### Scoped Versus Non-Scoped Entities (e.g. Publishers/Subscriptions)

Currently, Publisher and Subscription (and similar entities) are scoped, meaning that when they are created they are added to the ROS graph as a side effect, and when they are let out of scope they remove themselves from the graph too.

> 目前，Publisher 和 Subscription(以及类似实体)都有作用域，这意味着在创建它们时，它们会作为副作用添加到 ROS 图中，而当它们超出作用域时，它们也会从图中删除自己。

Additionally, they necessarily have shared state with the system, for instance when you are spinning on a node, the executor shares ownership of the Subscriptions with the user.

> 此外，它们必须与系统共享状态，例如，当您在节点上旋转时，执行者与用户共享订阅的所有权。

Therefore, the Subscription only gets removed when both the user and executor are done with it.

> 因此，只有当用户和执行者都完成了订阅时，才会删除订阅。

This shared ownership is accomplished with the use of shared pointers and weak pointers.

> 这种共享所有权是通过使用共享指针和弱指针来实现的。

There are a few concerns here, (a) use of shared pointers confuses users, (b) overhead of shared pointers and lack of an ability to use these classes on the stack rather than the heap, and (c) complexity of shutdown of an entity from the users perspective.

> 这里有一些担忧，(a)使用共享指针会让用户感到困惑，(b)共享指针的开销和缺乏在堆栈而不是堆上使用这些类的能力，以及(c)从用户的角度来看，实体关闭的复杂性。

For (a), some users are overwhelmed by the need to use a shared pointer.

> 对于(a)，一些用户因需要使用共享指针而不知所措。

In ROS 1 this was avoided by having a class which itself just thinly wraps a shared pointer (see: https://github.com/ros/ros_comm/blob/ac9f88c59a676ca6895e13445fc7d71f398ebe1f/clients/roscpp/include/ros/subscriber.h#L108-L111).

> 在 ROS 1 中，通过使用一个类来避免这种情况，该类本身只是薄薄地封装了一个共享指针(请参见:https://github.com/ros/ros_comm/blob/ac9f88c59a676ca6895e13445fc7d71f398ebe1f/clients/roscpp/include/ros/subscriber.h#L108-L111)。

This could be achieved in ROS 2 either by doing the same with a wrapper class (at the expense of lots of repeated code), or by eliminating the need for using shared ownership.

> 这可以在 ROS2 中通过使用包装器类来实现(以大量重复代码为代价)，或者通过消除使用共享所有权的需要来实现。

For (b), for some use cases, especially resource constrained / real-time / safety-critical environments, requiring these classes to be on the heap rather than the stack is at least inconvenient.

> 对于(b)，对于某些用例，尤其是资源受限/实时/安全关键环境，要求这些类位于堆上而不是堆栈上至少是不方便的。

Additionally, there is a cost associated with using shared pointers, in the storage of shared state and in some implementation the use of locks or at least atomics for thread-safety.

> 此外，使用共享指针、共享状态的存储以及在某些实现中使用锁或至少原子以实现线程安全都会带来成本。

For (c), this is the most concerning drawback, because right now when a user lets their shared pointer to a, for example, Subscription go out of scope, a post condition is not that the Subscription is destroyed, nor that it has been removed from the graph.

> 对于(c)，这是最令人担忧的缺点，因为现在当用户让其指向某个(例如，Subscription)的共享指针超出范围时，后置条件不是该 Subscription 已销毁，也不是它已从图中删除。

In stead, the behavior is more like "at some point in the future the Subscription will be destroyed and removed from the graph, when the system is done with it".

> 相反，这种行为更像是`在未来的某个时刻，当系统完成订阅时，订阅将被销毁并从图中删除`。

This isn't a very satisfactory contract, as some users may wish to know when the Subscription has been deleted, but cannot easily know that.

> 这不是一个非常令人满意的合同，因为有些用户可能希望知道订阅何时被删除，但不容易知道。

The benefit to the shared state is a safety net for users.

> 共享状态的好处是为用户提供安全网。

The alternative would be to document that a Subscription, again for example, cannot be deleted until the system is done with it.

> 另一种方法是记录订阅，例如，在系统完成订阅之前，无法删除订阅。

We'd basically be pushing the responsibility onto the user to ensure the shared ownership is handled properly by the execution of their application, i.e. they create the Subscription, share a reference with the system (adding it by reference to an executor, for example), and they have to make sure the system is done with it before deleting the Subscription.

> 基本上，我们将责任推给用户，以确保共享所有权通过其应用程序的执行得到正确处理，即，他们创建订阅，与系统共享引用(例如，通过引用执行者添加引用)，并且在删除订阅之前，他们必须确保系统已完成该操作。

Separately, from the above points, there is the related concern of forcing the user to keep a copy of their entities in scope, whether it be with a shared pointer or a class wrapping one.

> 另外，从以上几点来看，还有一个相关的问题，即强制用户在作用域中保留其实体的副本，无论是使用共享指针还是使用类包装指针。

There is the desire to create it and forget it in some cases.

> 在某些情况下，人们渴望创造它并忘记它。

The downside to this is that if/when the user wants to destroy the entity, they have no way of doing that as they have no handle or unique way to address the entity.

> 这样做的缺点是，如果/当用户想要销毁实体时，他们无法做到这一点，因为他们没有句柄或唯一的方法来处理实体。

One proposed solution would be to have a set of "named X" APIs, e.g. `create_named_subscription` rather than just `create_subscription`.

> 一种建议的解决方案是使用一组`命名 X`API，例如`create_named_subscription`，而不仅仅是`create_subscription`。

This would allow the user to address the Subscription in the future in order to obtain a new reference to it or delete it.

> 这将允许用户在将来对订阅进行寻址，以便获得对它的新引用或删除它。

- https://github.com/ros2/rclcpp/issues/506
- https://github.com/ros2/rclcpp/issues/726

**Suggested Action**: Consolidate to a single issue, and defer.

> **建议的行动**:合并为一个问题，然后推迟。

**Notes from 2020-03-23**:

- (chris) Putting ownership mechanics on user is hard.

> -(克里斯)将所有权机制强加给用户很难。

- (dirk) add documentation clearly outlining ownership

> -(dirk)添加明确概述所有权的文档

- (shane) warn on unused to catch issues with immediately deleted items

> -(shane)警告未使用的立即删除项目的捕获问题

- (tfoote) debugging output for destruction so it easy to see when reviewing logs

> -(tfoote)调试销毁输出，以便在查看日志时易于查看

- (chris) possible to create API that checks for destruction

> -(chris)可以创建用于检查销毁的 API

- (william) might lead to complex synchronization issues

> -(william)可能会导致复杂的同步问题

- (tfoote) could add helper classes to make scoped things non-scoped

> -(tfoote)可以添加帮助器类，以使作用域内的对象成为非作用域

- (shane) concerned that there is no longer "one good way" to do it

> -(尚恩)担心再也没有`一个好办法`了

### Allow QoS to be configured externally, like we allow remapping of topic names

Suggestion from @stonier: allow the qos setting on a topic to be changed externally at startup, similar to how we do topic remapping (e.g., do it on the command-line using appropriate syntax).

> 来自@stonier 的建议:允许在启动时从外部更改主题的 qos 设置，类似于我们如何进行主题重新映射(例如，使用适当的语法在命令行上进行映射)。

To keep the syntax manageable, we might just allow profiles to be picked.

> 为了保持语法的可管理性，我们可能只允许选择配置文件。

- https://github.com/ros2/rclcpp/issues/239

**Suggested Action**: Update issue, defer for now.

> **建议操作**:更新问题，暂时推迟。

**Notes from 2020-03-19**:

- (wjwwood) it depends on the QoS setting, but many don't make sense, mostly because they can change some of the behaviors of underlying API

> -(wjwwood)这取决于 QoS 设置，但很多设置都没有意义，主要是因为它们可以改变底层 API 的一些行为

- (dirk) Should developers expose a parameter instead?

> -(dirk)开发人员是否应该公开参数？

- (multiple) should be a feature that makes configuring them (after opt-in) consistent

> -(多个)应该是使配置它们(选择加入后)一致的功能

- (jacob) customers feedback was that this was expected, surprised it was not allowed

> -(雅各布)客户的反馈是，这是意料之中的，但很惊讶这是不允许的

- (karsten) could limit to profiles

> -(karsten)可能仅限于剖面

## Init/shutdown and Context

### Consider renaming `rclcpp::ok()`

Old discussion to rename `rclcpp::ok()` to something more specific, like `rclcpp::is_not_shutdown()` or the corollary `rclcpp::is_shutdown()`.

> 旧讨论将`rclcpp::ok()`重命名为更具体的名称，如`rclccp::is_not_shutdown()`或推论`rclcpp:is_shutdown()`。

- https://github.com/ros2/rclcpp/issues/3

**Suggested Action**: Defer.

**Notes from 2020-03-19**:

- (shane) preference to not have a negative in the function name

> -(shane)在函数名中不带负数的首选项

## Executor

### Exposing Scheduling of Tasks in Executor and a Better Default

Currently there is a hard coded procedure for handling ready tasks in the executor, first timers, then subscriptions, and so on.

> 目前，有一个硬编码的过程来处理执行器中的就绪任务，先是计时器，然后是订阅，等等。

This scheduling is not fair and results in non-deterministic behavior and starvation issues.

> 这种调度是不公平的，会导致不确定性行为和饥饿问题。

We should provide a better default scheduling which is fairer and ideally deterministic, something like round-robin or FIFO.

> 我们应该提供一个更好的默认调度，它更公平，理想情况下具有确定性，比如循环或 FIFO。

Additionally, we should make it easier to let the user override the scheduling logic in the executor.

> 此外，我们应该更容易让用户重写执行器中的调度逻辑。

- https://github.com/ros2/rclcpp/pull/614
- https://github.com/ros2/rclcpp/issues/633
- https://github.com/ros2/rclcpp/issues/392

**Suggested Action**: Follow up on proposals to implement FIFO scheduling and refactor the Executor design to more easily expose the scheduling logic.

> **建议措施**:跟进实施 FIFO 调度的建议，并重构 Executor 设计，以更容易地暴露调度逻辑。

**Notes from 2020-03-19**:

- No comments.

### Make it possible to wait on entities (e.g. Subscriptions) without an Executor

Currently, it is only possible to use things like Timers and Subscriptions and Services with an executor.

> 目前，只能使用计时器、订阅和服务等执行器。

It should be possible, however, to either poll these entities or wait on them and then decide which to process as a user.

> 然而，应该可以轮询这些实体或等待它们，然后决定作为用户处理哪个实体。

This is most easily accomplished with a WaitSet-like class.

> 这最容易用类似 WaitSet 的类完成。

- https://github.com/ros2/rclcpp/issues/520

**Suggested Action**: implement WaitSet class in rclcpp so that this is possible, and make "waitable" entities such that they can be polled, e.g. `Subscription`s should have a user facing `take()` method, which can fail if no data is available.

> **建议操作**:在 rclcpp 中实现 WaitSet 类，使其成为可能，并使`可等待`实体可以被轮询，例如，`Subscription`应具有面向用户的`take()`方法，如果没有可用数据，该方法可能会失败。

**Notes from 2020-03-19**:

- No comments.

### Make it possible to use multiple executors per node

Currently, you cannot use more than one executor per node, this limits your options when it comes to distributing work within a node across threads.

> 目前，每个节点不能使用多个执行器，这限制了在节点内跨线程分配工作时的选择。

You can use a multi-threaded executor, or make your own executor which does this, but it is often convenient to be able to spin part of the node separately from the the rest of the node.

> 您可以使用多线程执行器，也可以创建自己的执行器来执行此操作，但通常可以将节点的一部分与节点的其余部分分开旋转。

- https://github.com/ros2/rclcpp/issues/519

**Suggested Action**: Make this possible, moving the exclusivity to be between an executor and callback groups rather than nodes.

> **建议的操作**:使这成为可能，将独占性移动到执行器和回调组之间，而不是节点之间。

**Notes from 2020-03-19**:

- No comments.

### Implement a Lock-free Executor

This would presumably be useful for real-time and safety critical systems where locks and any kind of blocking code is considered undesirable.

> 这可能对实时和安全关键系统有用，因为锁和任何类型的阻塞代码都被认为是不可取的。

- https://github.com/ros2/rclcpp/issues/77

**Suggested Action**: Keep in backlog until someone needs it specifically.

> **建议的行动**:在有人特别需要之前，保持积压。

**Notes from 2020-03-19**:

- No comments.

### Add implementation of `spin_some()` to the `MultiThreadedExecutor`

Currently `spin_some()` is only available in the `SingleThreadedExecutor`.

> 当前，`spin_some()`仅在`SingleThreadExecutor`中可用。

- https://github.com/ros2/rclcpp/issues/85

**Suggested Action**: Defer.

**Notes from 2020-03-19**:

- No comments.

## Node

### Do argument parsing outside of node constructor

Things that come from command line arguments should be separately passed into the node's constructor rather than passing in arguments and asking the node to do the parsing.

> 来自命令行参数的内容应该单独传递到节点的构造函数中，而不是传递参数并要求节点进行解析。

- https://github.com/ros2/rclcpp/issues/492

**Suggested Action**: Defer until after foxy.

> **建议操作**:延迟到 foxy 之后。

**Notes from 2020-03-23**:

- (dirk) may be related to ROS 1 heritage of argc/argv being passed to node directly

> -(dirk)可能与 argc/argv 的 ROS 1 遗产直接传递给节点有关

- (shane) impacts rcl API as well, two parts "global options" as well node specific options

> -(shane)也影响 rcl API，包括两部分`全局选项`和节点特定选项

- (dirk) what is the recommendation to users that want to add arguments programmatically

> -(dirk)对希望以编程方式添加参数的用户的建议是什么

- user should be able to get non-ros argc/argv somehow (seems like you can now)

> -用户应该能够以某种方式获得 non-ros-argc/argv(看起来现在可以了)

- (jacob) the argument in NodeOptions are used for application specific argument via component loading as well

> -(jacob)NodeOptions 中的参数也通过组件加载用于特定于应用程序的参数

## Timer

### Timer based on ROS Time

`node-create_wall_timer` does exactly what it says; creates a timer that will call the callback when the wall time expires. But this is almost never what the user wants, since this won’t work properly in simulation. Suggestion: deprecate `create_wall_timer`, add a new method called `create_timer` that takes the timer to use as one of the arguments, which defaults to ROS_TIME.

> `node-create_wall_timer`完全按照它所说的做；创建一个计时器，该计时器将在墙时间到期时调用回调。但这几乎不是用户想要的，因为这在模拟中无法正常工作。建议:弃用`create_wall_timer`，添加一个名为`create_timer'的新方法，该方法将计时器用作参数之一，默认为 ROS_TIME。

- https://github.com/ros2/rclcpp/blob/96ebf59a6045a535730d98fff25e522807c7aa75/rclcpp/include/rclcpp/node.hpp#L219-L230

- https://github.com/ros2/rclcpp/issues/465

**Suggested Action**: Promote `rclcpp::create_timer()` which is templated on a clock type, as suggested, but leave `create_wall_timer` as a convenience.

> **建议的操作**:升级`rclcpp::create_timer()`，如建议的那样，它是在时钟类型上模板化的，但为了方便起见，保留`create_wall_timer`。

**Notes from 2020-03-19**:

- (shane) may be a `rclcpp::create_timer()` that can be used to create a non-wall timer already

> -(shane)可以是一个`rclcpp::create_timer()`，它可以用来创建一个非墙计时器

## Publisher

## Subscription

### Callback Signature

Is there a reason the subscription callback must have a smart pointer argument instead of accepting a const-reference argument?

> 订阅回调必须具有智能指针参数而不是接受常量引用参数，这是有原因的吗？

- https://github.com/ros2/rclcpp/blob/96ebf59a6045a535730d98fff25e522807c7aa75/rclcpp/include/rclcpp/any_subscription_callback.hpp#L44-L52

- https://github.com/ros2/rclcpp/issues/281

**Suggested Action**: Provide const reference as an option, add documentation as to the implications of one callback signature versus others.

> **建议操作**:提供常量引用作为选项，添加一个回调签名相对于其他回调签名的含义的文档。

**Notes from 2020-03-19**:

- (dirk) have const reference and document it

> -(dirk)有常量引用并记录它

## Service Server

### Allow for asynchronous Service Server callbacks

Currently, the only callback signature for Service Servers takes a request and must return a response.

> 目前，服务服务器的唯一回调签名接受请求，并且必须返回响应。

This means that all of the activity of the service server has to happen within that function.

> 这意味着服务服务器的所有活动都必须在该功能内发生。

This can cause issues, specifically if you want to call another service within the current service server's callback, as it causes deadlock issues trying to synchronously call the second service within a spin callback.

> 这可能会导致问题，特别是如果您想在当前服务服务器的回调中调用另一个服务，因为在尝试同步调用自旋回调中的第二个服务时会导致死锁问题。

More generally, it seems likely that long running service server callbacks may be necessary in the future and requiring them to be synchronous would tie up at least on thread in the spinning executor unnecessarily.

> 更一般地说，未来可能需要长时间运行的服务服务器回调，而要求它们同步将不必要地占用旋转执行器中的至少一个线程。

- https://github.com/ros2/rclcpp/issues/491

**Suggested Action**: Defer.

> **建议操作**:延迟。

**Notes from 2020-03-23**:

- (dirk) likely new API, so possible to backport

> -(dirk)可能的新 API，因此有可能返回端口

## Service Client

### Callback has SharedFuture rather than const reference to response

Why does the Client::send_async_request take in a callback that has a SharedFuture argument instead of an argument that is simply a const-reference (or smart pointer) to the service response type? The current API seems to imply that the callback ought to check whether the promise is broken or fulfilled before trying to access it. Is that the case? If so, it should be documented in the header.

> 为什么 Client::send_async_request 采用具有 SharedFuture 参数的回调，而不是简单地作为服务响应类型的常量引用(或智能指针)的参数？当前的 API 似乎暗示回调应该在尝试访问承诺之前检查承诺是否被破坏或实现。是这样吗？如果是，则应将其记录在标题中。

- https://github.com/ros2/rclcpp/blob/7c1721a0b390be8242a6b824489d0bc861f6a0ad/rclcpp/include/rclcpp/client.hpp#L134

**Suggested Action**: Update ticket and defer.

> **建议操作**:更新票据并推迟。

**Notes from 2020-03-19**:

- (wjwwood) we wanted the user to handle error cases with the future?

> -(wjwwood)我们希望用户将来处理错误案例？

- (dirk) future allows for single callback (rather than one for response and one for error)

> -(dirk)future 允许单个回调(而不是一个用于响应，一个用于错误)

- (jacob) actions uses a "wrapped result" object

> -(jacob)操作使用`包装结果`对象

### rclcpp missing synchronous `send_request` and issues with deadlocks

This has been reported by several users, but there is only an `async_send_request` currently. `rclpy` has a synchronous `send_request` but it has issues with deadlock, specifically if you call it without spinning in another thread then it will deadlock. Or if you call it from within a spin callback when using a single threaded executor, it will deadlock.

> 几个用户已经报告了这一情况，但目前只有一个`async_send_request``rcppy 有一个同步的 sendrequest，但它有死锁问题，特别是如果在另一个线程中调用它而不旋转，那么它就会死锁。或者，如果在使用单线程执行器时从自旋回调中调用它，它将死锁。

- https://discourse.ros.org/t/synchronous-request-to-service-in-callback-results-in-deadlock/12767
- https://github.com/ros2/rclcpp/issues/975
- https://github.com/ros2/demos/blob/948b4f4869298f39cfe99d3ae517ad60a72a8909/demo_nodes_cpp/src/services/add_two_ints_client.cpp#L24-L39

**Suggested Action**: Update issue and defer. Also defer decision on reconciling rclpy's send_request.

> **建议措施**:更新问题并推迟。还要推迟对 rcopy 的 send_request 进行协调的决定。

**Notes from 2020-03-23**:

- (karsten/shane) async spinner helps in rclpy version, rclcpp could emulate

> -(karsten/shane)异步微调器有助于 rcppy 版本，rclcpp 可以模拟

- (chris) sees three options:

> -(chris)看到了三个选项:

- only async (current case in rclcpp)

> -仅异步(rclcpp 中的当前情况)

- have sync version, add lots of docs that spinning needs to happen elsewhere (current case for rclpy)

> -具有同步版本，添加许多需要在其他地方进行旋转的文档(rcppy 的当前情况)

- reentrant spinning

> -再入纺纱

- (william) you either need async/await from language or ".then" syntax (we have this in async_send_request())

> -(william)您需要 async/await from language 或`.then`语法(我们在 async_send_request()中有这个)

- (chris) more error checking for recursive spinning

> -(chris)递归旋转的更多错误检查

- (chris) weird that rclcpp and rclpy have different API

> -(chris)奇怪的是 rclcpp 和 rcppy 有不同的 API

- (shane) thinks it is ok to have different API, but rclpy is not ideal

> -(shane)认为使用不同的 API 是可以的，但 rcpy 并不理想

## Parameters

### Expected vs Unexpected parameters

Allow node author to define expected parameters and what happens when an unexpected parameter is set.

> 允许节点作者定义预期参数以及设置意外参数时发生的情况。

- https://github.com/ros2/rclcpp/issues/475

- https://github.com/ros2/rclcpp/tree/check_parameters

**Suggested Action**: Defer as nice to have.

> **建议的行动**:推迟是件好事。

**Notes from 2020-03-23**:

- None.

> -无。

### Implicitly cast integer values for double parameters

If we try to pass an integer value to a double parameter from the command line or from a parameters YAML file we get a `rclcpp::ParameterTypeException`.

> 如果我们试图从命令行或参数 YAML 文件向双参数传递整数值，则会得到一个`rclcpp::ParameterTypeException`。

For example, passing a parameter from the command line:

> 例如，从命令行传递参数:

    ros2 run foo_package foo_node --ros-args -p foo_arg:=1

results in the following error:

> 导致以下错误:

    terminate called after throwing an instance of 'rclcpp::ParameterTypeException'
      what():  expected [double] got [integer]

and we can fix it by explicitly making our value a floating point number:

> 我们可以通过显式地将我们的值设置为浮点数来修复它:

    ros2 run foo_package foo_node --ros-args -p foo_arg:=1.0

But, it seems reasonable to me that if a user forgets to explicitly provide a floating point value that we should implicitly cast an integer to a float (as is done in many programming languages).

> 但是，在我看来，如果用户忘记显式地提供浮点值，我们应该隐式地将整数转换为浮点值(这在许多编程语言中都是如此)。

- https://github.com/ros2/rclcpp/issues/979

**Suggested Action**: Continue with issue.

> **建议措施**:继续解决问题。

**Notes from 2020-03-23**:

- (shane) says "yes please" :)

> -(shane)说`是的，请`:)

### Use `std::variant` instead of custom `ParameterValue` class

This is only possible if C++17 is available, but it would simplify our code, make our interface more standard, and allow us to use constexpr-if to simply our templated code.

> 这只有在 C++17 可用的情况下才可能实现，但它会简化我们的代码，使我们的接口更加标准，并允许我们使用 constexprif 来简化模板化代码。

**Suggested Action**: Create an issue for future work.

> **建议的行动**:为未来的工作创建问题。

**Notes from 2020-03-23**:

- (chris) not sure churn is worth

> -(克里斯)不确定搅拌是否值得

- (ivan) other places for std::variant, like AnySubscriptionCallback

> -(ivan)std::variant 的其他位置，如 AnySubscriptionCallback

### Cannot set name or value on `Parameter`/`ParameterValue`

Both `Parameter` and `ParameterValue` are read-only after construction.

> 构造后，`Parameter`和`ParameterValue`都是只读的。

- https://github.com/ros2/rclcpp/issues/238

**Suggested Action**: Update issue, possibly close.

> **建议操作**:更新问题，可能关闭。

**Notes from 2020-03-23**:

- (chris/william) setting values on temporary (local) objects is not reflected in the node, so misleading

> -(chris/william)在临时(本地)对象上设置值不会反映在节点中，因此具有误导性

## Parameter Clients

### No timeout option with synchronous parameter client calls

As an example, SyncParametersClient::set_parameters doesn't take a timeout option. So, if anything goes wrong in the service call (e.g. the server goes down), we will get stuck waiting indefinitely.

> 例如，SyncParametersClient::set_parameters 不采用超时选项。因此，如果服务调用中出现任何问题(例如服务器停机)，我们将无限期地等待。

- https://github.com/ros2/rclcpp/issues/360

- https://github.com/ros2/rclcpp/blob/96ebf59a6045a535730d98fff25e522807c7aa75/rclcpp/src/rclcpp/parameter_client.cpp#L453-L468

**Suggested Action**: Update issue, decide if it can be taken for Foxy or not.

> **建议措施**:更新问题，决定是否可以为 Foxy 采取。

**Notes from 2020-03-23**:

- (tfoote) Seems like adding a timeout is a good idea.

> -(tfoote)添加超时似乎是个好主意。

### Name of AsyncParametersClient inconsistent

AsyncParameter**s**Client uses plural, when filename is singular (and ParameterService is singular):

> 当文件名为单数(ParameterService 为单数)时，AsyncParameter**s**客户端使用复数:

- https://github.com/ros2/rclcpp/blob/7c1721a0b390be8242a6b824489d0bc861f6a0ad/rclcpp/include/rclcpp/parameter_client.hpp#L44

**Suggested Action**: Reconcile class and file name, switch to singular name?

> **建议操作**:协调类和文件名，切换到单数名称？

**Notes from on-line, post 2020-03-23 meeting**:

> **2020-03-23 会议后的在线注释**:

- (tfoote) +1 for homogenizing to singular

> -(tfoote)+1 用于均化为奇异

### `SyncParametersClient::get_parameters` doesn't allow you to detect error cases

E.g. https://github.com/ros2/rclcpp/blob/249b7d80d8f677edcda05052f598de84f4c2181c/rclcpp/src/rclcpp/parameter_client.cpp#L246-L257 returns an empty vector if something goes wrong which is also a valid response.

> 例如。https://github.com/ros2/rclcpp/blob/249b7d80d8f677edcda05052f598de84f4c2181c/rclcpp/src/rclcpp/parameter_client.cpp#L246-如果发生错误，L257返回一个空向量，这也是一个有效的响应。

- https://github.com/ros2/rclcpp/issues/200

- https://github.com/ros2/rclcpp/blob/96ebf59a6045a535730d98fff25e522807c7aa75/rclcpp/src/rclcpp/parameter_client.cpp#L412-L426

**Suggested Action**: Throw an exception to indicate if something went wrong and document other expected conditions of the API.

> **建议的操作**:抛出一个异常以指示是否有问题，并记录 API 的其他预期条件。

**Notes from on-line, post 2020-03-23 meeting**:

> **2020-03-23 会议后的在线注释**:

- (tfoote) An empty list is not a valid response unless you passed in an empty list. The return should have the same length as the request in the same order. Any parameters that are not set should return a ParameterVariant with type PARAMETER_NOT_SET. to indicate that it was polled and determined to not be set. Suggested action improve documentation of the API to clarify a short or incomplete.

> -(tfoote)除非传入空列表，否则空列表不是有效的响应。返回的长度应与相同顺序的请求的长度相同。未设置的任何参数都应返回类型为 PARAMETER_not_set 的 ParameterVariant。以指示它被轮询并被确定为不被设置。建议采取行动改进 API 文件，以澄清简短或不完整的内容。

- (jacobperron) I think throwing an exception is also a valid action, making it clear that an error occurred.

> -(jacobperron)我认为抛出一个异常也是一个有效的操作，可以清楚地表明发生了错误。

- (wjwwood) Using exceptions to indicate an exceptional case (something went wrong) seems reasonable to me too.

> -(wjwwood)使用异常来表示异常情况(出了问题)对我来说似乎也是合理的。

## Clock

### Clock Jump callbacks on System or Steady time?

Currently time jump callbacks are registered via Clock::create_jump_handler(). Jump handlers are only invoked by TimeSource::set_clock(). This is only called if the clock type is RCL_ROS_TIME and ROS time is active.

> 当前，时间跳转回调是通过 Clock::create_jump_handler()注册的。跳转处理程序仅由 TimeSource::set_clock()调用。仅当时钟类型为 RCL_ROS_TIME 且 ROS 时间处于活动状态时，才会调用此命令。

- https://github.com/ros2/rclcpp/issues/528

**Suggested Action**: Document that time jumping is only detected with ROS time, consider a warning.

> **建议措施**:记录仅在 ROS 时间内检测到时间跳跃，请考虑警告。

**Notes from on-line, post 2020-03-23 meeting**:

> **2020-03-23 会议后的在线注释**:

- (tfoote) There should be no jumps in steady time. If there's a big change in system time, it doesn't necessarily mean that time jumped, just that you might have been sleeping for a long time. Most ntp systems adjust the slew rate these days instead of jumping but still that's an external process and I don't know of any APIs to introspect the state of the clock. I'm not sure that we have a way to detect jumps in time for system or steady time. To that end I think that we should be clear that we only provide callbacks when simulation time starts or stops, or simulation time jumps. We should also strongly recommend that operators not actively adjust their system clocks while running ROS nodes.

> -(tfoote)在稳定时间内不应有跳跃。如果系统时间发生了重大变化，并不一定意味着时间跳跃，只是意味着你可能已经睡了很长时间。现在大多数 ntp 系统都会调整转换速率，而不是跳跃，但这仍然是一个外部过程，我不知道有什么 API 可以内省时钟状态。我不确定我们是否有办法检测系统或稳定时间的时间跳跃。为此，我认为我们应该清楚，我们只在模拟时间开始或停止或模拟时间跳跃时提供回调。我们还强烈建议运营商在运行 ROS 节点时不要主动调整系统时钟。

- (jacobperron) I agree with Tully, if we don't have a way to detect system time jumps then I think we should just document that this only works with ROS time. In addition to documentation, we could log an info or warning message if the user registers jump callback with steady or system time, but it may be unnecessarily noisy.

> -(jacobperron)我同意 Tully 的观点，如果我们没有检测系统时间跳跃的方法，那么我认为我们应该记录这只适用于 ROS 时间。除了文档之外，如果用户在稳定或系统时间内注册跳转回调，我们还可以记录一条信息或警告消息，但这可能会产生不必要的噪音。

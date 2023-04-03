##

这段代码是 ROS2（Robot Operating System 2）项目中的 `rclcpp` 库的一部分。`rclcpp` 是 ROS2 的 C++ 客户端库，它提供了用于创建 ROS2 节点、发布和订阅消息等功能的 API。这个文件定义了一个名为 `Context` 的类，它封装了底层的 `rcl`（ROS Client Library）上下文对象，并提供了一些额外的功能。

### 类和函数解释

1. **ContextAlreadyInitialized**：这是一个自定义异常类，当试图初始化一个已经初始化过的上下文时抛出。

2. **WeakContextsWrapper**：这个类没有在这段代码中定义，但它可能与管理多个弱引用上下文对象有关。

3. **ShutdownCallbackHandle**：这个类包含一个指向回调函数的弱引用，该回调函数在上下文关闭时被调用。

4. **OnShutdownCallbackHandle** 和 **PreShutdownCallbackHandle**：这两个类型别名都是 `ShutdownCallbackHandle` 的别名，分别表示在关闭过程中和关闭之前执行的回调。

5. **Context**：这是主要的类，它封装了底层的 `rcl_context_t` 结构，并提供了一些额外的功能。这个类有以下成员函数：

   - `init()`：初始化上下文，接受命令行参数和初始化选项。
   - `is_valid()`：检查上下文是否有效。
   - `get_init_options()`：获取初始化选项。
   - `get_domain_id()`：获取域 ID。
   - `shutdown_reason()`：获取关闭原因。
   - `shutdown()`：关闭上下文，并设置关闭原因。
   - `on_shutdown()` 和 `add_on_shutdown_callback()`：添加在关闭过程中执行的回调函数。
   - `remove_on_shutdown_callback()`：移除在关闭过程中执行的回调函数。
   - `add_pre_shutdown_callback()`：添加在关闭之前执行的回调函数。
   - `remove_pre_shutdown_callback()`：移除在关闭之前执行的回调函数。
   - `get_on_shutdown_callbacks()`：获取所有在关闭过程中执行的回调函数。
   - `get_pre_shutdown_callbacks()`：获取所有在关闭之前执行的回调函数。
   - `get_rcl_context()`：获取底层的 `rcl_context_t` 结构。
   - `sleep_for()`：使当前线程休眠指定的纳秒数。
   - `interrupt_all_sleep_for()`：中断所有正在等待的 `sleep_for()` 调用。
   - `get_sub_context()`：获取一个子上下文，如果不存在则创建一个。

6. **get_contexts()**：这是一个全局函数，它返回一个包含所有活动上下文的向量。

### 调用关系

- 当用户创建一个 ROS2 节点时，首先需要创建一个 `Context` 对象。这个对象负责管理与底层 `rcl` 库的交互。
- 用户可以使用 `init()` 函数初始化上下文，传入命令行参数和初始化选项。这个函数会调用底层的 `rcl_init()` 函数。
- 用户可以使用 `shutdown()` 函数关闭上下文。在关闭过程中，会执行所有注册的 `on_shutdown` 和 `pre_shutdown` 回调函数。
- 用户可以通过 `get_rcl_context()` 函数获取底层的 `rcl_context_t` 结构，以便与其他 `rcl` 库函数交互。

总之，这个文件定义了一个名为 `Context` 的类，它封装了底层的 `rcl` 上下文对象，并提供了一些额外的功能。用户可以使用这个类创建 ROS2 节点、管理回调函数等。

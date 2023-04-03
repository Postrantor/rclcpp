---
Crafted by [Genie](https://marketplace.visualstudio.com/items?itemName=genieai.chatgpt-vscode)
---

## You

下面这段代码是在 ros2 项目中 rclcpp 相关的代码，请联系所有函数之间可能的相互联系，梳理、解释这份文件的功能、含义（中文）。（请给出详细完善的回答，不限制字数）

## Trantor

这段代码是 ROS2 项目中的 `rclcpp` 库的一部分，主要实现了 `GraphListener` 类。`GraphListener` 类用于监听节点图（Node Graph）的变化，例如节点、话题和服务的添加或删除。当图发生变化时，`GraphListener` 将通知相关的节点。

以下是该文件中定义的主要函数及其功能：

1. **GraphListener 构造函数**：接受一个 `shared_ptr<Context>` 参数，用于初始化成员变量 `weak_parent_context_` 和 `rcl_parent_context_`。同时，它还初始化了一个 `interrupt_guard_condition_` 守卫条件。

2. **GraphListener 析构函数**：在析构时调用 `shutdown()` 函数来关闭 `GraphListener`。

3. **init_wait_set()**：初始化 `wait_set_` 结构体，用于存储订阅、守卫条件、定时器等待事件。

4. **start_if_not_started()**：如果 `GraphListener` 尚未启动，则启动它。这个函数会创建一个新线程并运行 `run()` 函数。

5. **run()**：捕获异常并运行 `run_loop()` 函数。

6. **run_loop()**：无限循环，等待守卫条件触发。当守卫条件触发时，通知相关节点图形发生了变化。

7. **has_node()**：检查给定的 `NodeGraphInterface` 是否已经存在于 `node_graph_interfaces_` 列表中。

8. **add_node()**：将给定的 `NodeGraphInterface` 添加到 `node_graph_interfaces_` 列表中。

9. **remove_node()**：从 `node_graph_interfaces_` 列表中删除给定的 `NodeGraphInterface`。

10. **cleanup_wait_set()**：清理 `wait_set_` 结构体。

11. **\_\_shutdown()**：关闭 `GraphListener`，设置 `is_shutdown_` 为 `true`，并等待监听线程结束。如果已启动，则调用 `cleanup_wait_set()` 清理 `wait_set_`。

12. **shutdown()**：调用 `__shutdown()` 函数来关闭 `GraphListener`。

13. **is_shutdown()**：返回 `GraphListener` 是否已关闭。

通过这些函数，`GraphListener` 类可以实时监听节点图的变化，并通知相关节点。这对于 ROS2 中的节点管理和通信非常重要。

## You

请进一步梳理这些函数之间的功能调用关系

## Trantor

以下是这些函数之间的功能调用关系：

1. **GraphListener 构造函数**：在创建 `GraphListener` 对象时调用，初始化成员变量和守卫条件。

2. **GraphListener 析构函数**：在销毁 `GraphListener` 对象时调用，会调用 `shutdown()` 函数。

3. **init_wait_set()**：在 `start_if_not_started()` 函数中调用，用于初始化 `wait_set_` 结构体。

4. **start_if_not_started()**：在需要启动 `GraphListener` 时调用。此函数首先检查 `GraphListener` 是否已启动，如果没有，则调用 `init_wait_set()` 初始化 `wait_set_`，并创建一个新线程运行 `run()` 函数。

5. **run()**：在 `start_if_not_started()` 函数中作为新线程的入口点调用。此函数捕获异常并调用 `run_loop()` 函数。

6. **run_loop()**：在 `run()` 函数中调用，无限循环等待守卫条件触发。当守卫条件触发时，通知相关节点图形发生了变化。

7. **has_node()**：在 `add_node()` 和 `remove_node()` 函数中调用，检查给定的 `NodeGraphInterface` 是否已经存在于 `node_graph_interfaces_` 列表中。

8. **add_node()**：在需要将节点添加到 `GraphListener` 时调用。此函数首先调用 `has_node()` 检查节点是否已存在，如果不存在，则将节点添加到 `node_graph_interfaces_` 列表中。

9. **remove_node()**：在需要从 `GraphListener` 中删除节点时调用。此函数首先调用 `has_node()` 检查节点是否存在，如果存在，则从 `node_graph_interfaces_` 列表中删除该节点。

10. **cleanup_wait_set()**：在 `__shutdown()` 函数中调用，清理 `wait_set_` 结构体。

11. **\_\_shutdown()**：在 `shutdown()` 和 `shutdown(std::nothrow_t)` 函数中调用，关闭 `GraphListener`，设置 `is_shutdown_` 为 `true`，并等待监听线程结束。如果已启动，则调用 `cleanup_wait_set()` 清理 `wait_set_`。

12. **shutdown()**：在 `GraphListener` 析构函数或需要关闭 `GraphListener` 时调用。此函数调用 `__shutdown()` 函数来关闭 `GraphListener`。

13. **is_shutdown()**：在需要检查 `GraphListener` 是否已关闭时调用。

通过这些函数之间的调用关系，我们可以看到 `GraphListener` 类是如何实现对节点图变化的监听和通知相关节点的。在创建 `GraphListener` 对象后，可以使用 `add_node()` 和 `remove_node()` 函数管理节点，并通过 `start_if_not_started()` 启动监听线程。当需要关闭 `GraphListener` 时，可以调用 `shutdown()` 函数。

## You

请通过 plantuml 的格式给出这些函数的用例图，来表明调用关系

## Trantor

以下是使用 PlantUML 格式表示的这些函数之间的调用关系用例图：

```plantuml
@startuml

actor User

User --> GraphListener: 创建对象
User --> GraphListener: 销毁对象
User --> GraphListener: 添加节点
User --> GraphListener: 删除节点
User --> GraphListener: 启动监听
User --> GraphListener: 关闭监听
User --> GraphListener: 检查是否已关闭

class GraphListener {
  +GraphListener()
  +~GraphListener()
  +init_wait_set()
  +start_if_not_started()
  +run()
  +run_loop()
  +has_node()
  +add_node()
  +remove_node()
  +cleanup_wait_set()
  +__shutdown()
  +shutdown()
  +is_shutdown()
}

GraphListener --> GraphListener: init_wait_set()
GraphListener --> GraphListener: run()
GraphListener --> GraphListener: run_loop()
GraphListener --> GraphListener: has_node()
GraphListener --> GraphListener: cleanup_wait_set()
GraphListener --> GraphListener: __shutdown()

@enduml
```

将上述 PlantUML 代码粘贴到 [PlantUML 在线编辑器](http://www.plantuml.com/plantuml/uml) 中，即可生成用例图。这个用例图展示了用户与 `GraphListener` 类中函数之间的交互以及函数之间的调用关系。

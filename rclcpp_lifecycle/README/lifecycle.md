## 可以系统介绍一下 ros2 中 rclcpp_lifecycle 组件吗

`rclcpp_lifecycle` 是 ROS2(Robot Operating System 2)中的一个库，用于实现生命周期节点(Lifecycle nodes)。生命周期节点是一种特殊类型的节点，它**具有预定义的状态和转换**，使得开发者能够更好地管理节点的生命周期。这**有助于提高系统的健壮性和可维护性**。

生命周期节点基于以下五个核心状态：

1. **未配置(Unconfigured)**： 节点刚创建时的初始状态。在这个状态下，节点还没有完成配置。
2. **不活动(Inactive)**： 节点已配置，但尚未执行任何处理。例如，节点可能已分配和初始化资源，但尚未启动处理数据的功能。
3. **活动(Active)**： 节点正在执行其主要功能，如处理数据和响应事件。
4. **暂停(Deactivating)**： 节点正在从活动状态转换为不活动状态。在这个过程中，节点可能需要停止处理数据并释放某些资源。
5. **清除(Cleaning up)**： 节点正在撤销配置并释放所有资源。在完成清理后，节点将返回到未配置状态。

**生命周期节点之间的状态转换是通过生命周期管理器(Lifecycle Manager)进行的**。例如，可以通过生命周期管理器将节点从“不活动”状态转换为“活动”状态。

> [!NOTE]
> 这里提到是要通过 lifecycle manager 实现生命周期的管理！

要在 ROS2 中实现生命周期节点，可以使用 `rclcpp_lifecycle` 组件。首先，需要包含相关头文件：

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
```

然后，创建一个继承自 `rclcpp_lifecycle::LifecycleNode` 的类：

```cpp
class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 构造函数中的代码
  }

  // 实现生命周期回调函数
};
```

接下来，为生命周期状态实现相应的回调函数，例如：

```cpp
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_configure(const rclcpp_lifecycle::State & state)
{
  // 配置节点时的代码
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_activate(const rclcpp_lifecycle::State & state)
{
  // 激活节点时的代码
}

// 其他生命周期回调函数...
```

最后，将回调函数与生命周期节点关联：

```cpp
this->declare_parameter("param_name");
this->add_on_configure_transition_callback(std::bind(&MyLifecycleNode::on_configure, this, std::placeholders::_1));
this->add_on_activate_transition_callback(std::bind(&MyLifecycleNode::on_activate, this, std::placeholders::_1));
// 添加其他生命周期回调函数...
```

现在，已经创建了一个具有生命周期管理功能的 ROS2 节点。在运行节点时，可以

- 使用 `ros2 lifecycle` 命令来手动控制节点的状态转换，或者
- **在代码中使用生命周期管理器实现自动状态转换**。

## 如何使用生命周期管理器控制节点的状态转换？

要**使用生命周期管理器(Lifecycle Manager)控制生命周期节点的状态转换，你可以通过客户端 API 发送生命周期转换请求**。在 ROS2 中，可以使用 `rclcpp::LifecycleNode` 的 `rclcpp::LifecycleNode::trigger_transition` 方法将生命周期节点从一个状态转换到另一个状态。

以下是一个简单的示例，展示了如何**创建一个客户端节点(manager?)来控制生命周期节点的状态转换**：

首先，包含必要的头文件：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
```

创建一个客户端节点类，继承自 `rclcpp::Node`：

```cpp
class LifecycleController : public rclcpp::Node
{
public:
  LifecycleController() : Node("lifecycle_controller")
  {
    // 构造函数中的代码
  }

  // 控制生命周期节点的函数
};
```

然后，在构造函数中**实例化一个生命周期客户端，用于与生命周期节点通信**：

```cpp
lifecycle_client_ = std::make_shared<rclcpp_lifecycle::LifecycleServiceClient>(this, "my_lifecycle_node");
```

接下来，实现一个控制生命周期节点状态转换的函数，例如：

```cpp
void start_lifecycle_node()
{
  // 等待生命周期服务可用
  lifecycle_client_->wait_for_service();

  // 将节点配置为 Inactive 状态
  //? 不太清楚
  auto configure_request = std::make_shared<rclcpp_lifecycle::srv::ChangeState::Request>();
  configure_request->transition.id = rclcpp_lifecycle::Transition::TRANSITION_CONFIGURE;
  auto configure_future = lifecycle_client_->async_send_request(configure_request);

  // 等待配置完成
  if (rclcpp::spin_until_future_complete(this, configure_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure the lifecycle node.");
    return;
  }

  // 将节点激活为 Active 状态
  auto activate_request = std::make_shared<rclcpp_lifecycle::srv::ChangeState::Request>();
  activate_request->transition.id = rclcpp_lifecycle::Transition::TRANSITION_ACTIVATE;
  auto activate_future = lifecycle_client_->async_send_request(activate_request);

  // 等待激活完成
  if (rclcpp::spin_until_future_complete(this, activate_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate the lifecycle node.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Lifecycle node has been successfully activated.");
}
```

最后，在主函数中创建控制器节点并运行：

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<LifecycleController>();
  // 控制生命周期节点
  controller->start_lifecycle_node();
  rclcpp::shutdown();
  return 0;
}
```

此示例中的 `start_lifecycle_node` 函数将生命周期节点从 Unconfigured 状态配置为 Inactive 状态，然后激活为 Active 状态。你可以根据需要调整此函数，以便控制生命周期节点的其他状态转换。

## 如何将生命周期节点从 Active 状态转换为 Inactive 状态？

要将生命周期节点从 Active 状态转换为 Inactive 状态，你可以向生命周期节点发送一个 `TRANSITION_DEACTIVATE` 转换请求。以下是一个示例，展示了如何在生命周期控制器中实现这个状态转换：

首先，确保你已经包含了必要的头文件，并创建了一个继承自 `rclcpp::Node` 的控制器类。在这个类中，**实例化一个生命周期客户端，用于与生命周期节点通信**，如前面的示例所示。

接下来，实现一个将生命周期节点从 Active 状态转换为 Inactive 状态的函数：

```cpp
void deactivate_lifecycle_node()
{
  // 等待生命周期服务可用
  lifecycle_client_->wait_for_service();

  // 将节点从 Active 状态转换为 Inactive 状态
  auto deactivate_request = std::make_shared<rclcpp_lifecycle::srv::ChangeState::Request>();
  deactivate_request->transition.id = rclcpp_lifecycle::Transition::TRANSITION_DEACTIVATE;
  auto deactivate_future = lifecycle_client_->async_send_request(deactivate_request);

  // 等待去激活完成
  if (rclcpp::spin_until_future_complete(this, deactivate_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to deactivate the lifecycle node.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Lifecycle node has been successfully deactivated.");
}
```

在主函数中创建控制器节点，并运行你的 `deactivate_lifecycle_node()` 函数：

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<LifecycleController>();
  // 控制生命周期节点
  // 注意：确保你的生命周期节点处于 Active 状态，否则去激活操作将失败
  controller->deactivate_lifecycle_node();
  rclcpp::shutdown();
  return 0;
}
```

运行这个示例，生命周期控制器将尝试将生命周期节点从 Active 状态转换为 Inactive 状态。如果生命周期节点已经处于 Active 状态，那么这个操作将成功。如果节点不在 Active 状态，去激活操作将失败。**为了确保转换成功，可以在尝试去激活之前先激活节点，或者检查节点当前的状态**。

## 如何检查生命周期节点的当前状态？

要检查生命周期节点的当前状态，可以使用 `rclcpp_lifecycle::LifecycleServiceClient` 的 `get_state` 方法。以下是一个示例，展示了如何在生命周期控制器中实现这个功能：

首先，确保你已经包含了必要的头文件，并创建了一个继承自 `rclcpp::Node` 的控制器类。在这个类中，实例化一个生命周期客户端，用于与生命周期节点通信，如前面的示例所示。

接下来，实现一个查询生命周期节点当前状态的函数：

```cpp
void query_lifecycle_node_state()
{
  // 等待生命周期服务可用
  lifecycle_client_->wait_for_service();

  // 查询生命周期节点的当前状态
  auto get_state_request = std::make_shared<rclcpp_lifecycle::srv::GetState::Request>();
  auto get_state_future = lifecycle_client_->async_send_request(get_state_request);

  // 等待查询结果
  if (rclcpp::spin_until_future_complete(this, get_state_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get the lifecycle node's current state.");
    return;
  }

  // 打印生命周期节点的当前状态
  auto state = get_state_future.get()->current_state;
  RCLCPP_INFO(this->get_logger(), "Lifecycle node is in state: %s (%d)", state.label.c_str(), state.id);
}
```

在主函数中创建控制器节点，并运行你的 `query_lifecycle_node_state()` 函数：

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<LifecycleController>();
  // 查询生命周期节点的当前状态
  controller->query_lifecycle_node_state();
  rclcpp::shutdown();
  return 0;
}
```

运行这个示例，生命周期控制器将查询生命周期节点的当前状态，并将其打印到控制台。你**可以根据需要在代码中使用这个状态信息**，例如，在尝试激活或去激活节点之前检查节点的当前状态。

## 如何激活生命周期节点？

要激活生命周期节点，你需要向生命周期节点**发送一个 `TRANSITION_ACTIVATE` 转换请求**。以下是一个示例，展示了如何在生命周期控制器中实现激活生命周期节点：

首先，确保你已经包含了必要的头文件，并创建了一个继承自 `rclcpp::Node` 的控制器类。在这个类中，实例化一个生命周期客户端，用于与生命周期节点通信，如前面的示例所示。

接下来，实现一个激活生命周期节点的函数：

```cpp
void activate_lifecycle_node()
{
  // 等待生命周期服务可用
  lifecycle_client_->wait_for_service();

  // 将节点从 Inactive 状态转换为 Active 状态
  auto activate_request = std::make_shared<rclcpp_lifecycle::srv::ChangeState::Request>();
  activate_request->transition.id = rclcpp_lifecycle::Transition::TRANSITION_ACTIVATE;
  auto activate_future = lifecycle_client_->async_send_request(activate_request);

  // 等待激活完成
  if (rclcpp::spin_until_future_complete(this, activate_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate the lifecycle node.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Lifecycle node has been successfully activated.");
}
```

在主函数中创建控制器节点，并运行你的 `activate_lifecycle_node()` 函数：

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<LifecycleController>();
  // 激活生命周期节点
  // 注意：确保你的生命周期节点处于 Inactive 状态，否则激活操作将失败
  controller->activate_lifecycle_node();
  rclcpp::shutdown();
  return 0;
}
```

运行这个示例，生命周期控制器将尝试将生命周期节点从 Inactive 状态激活为 Active 状态。如果生命周期节点已经处于 Inactive 状态，那么这个操作将成功。如果节点不在 Inactive 状态，激活操作将失败。为了确保转换成功，可以在尝试激活之前先配置节点，或者检查节点当前的状态。

## 如何在生命周期节点中实现配置？

在生命周期节点中实现配置，需要在你的节点类中覆盖 `on_configure` 回调。`on_configure` 回调在节点从 `unconfigured` 状态转移到 `inactive` 状态时被调用。以下是一个示例，展示了如何在生命周期节点中实现配置：

1. 首先，确保你已经包含了必要的头文件，并创建了一个继承自 `rclcpp_lifecycle::LifecycleNode` 的节点类。

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 构造函数中的初始化代码
  }

  // 其他成员和方法
};
```

2. 在你的节点类中，覆盖 `on_configure` 方法。在这个方法中，添加你的配置代码，例如加载参数、初始化服务、初始化发布器和订阅器等。

```cpp
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 构造函数中的初始化代码
  }

  // 覆盖 on_configure 回调
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "Configuring the lifecycle node...");

    // 在这里实现配置代码，例如加载参数、初始化服务、初始化发布器和订阅器等

    RCLCPP_INFO(this->get_logger(), "Lifecycle node has been configured.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 其他成员和方法
};
```

3. 在主函数中，创建一个 `rclcpp_lifecycle::LifecycleNode` 类型的节点，并运行生命周期管理器：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "my_lifecycle_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  //? 再详细问一下，这里的manager
  // 创建生命周期节点
  auto lifecycle_node = std::make_shared<MyLifecycleNode>();
  // 创建生命周期管理器
  auto lifecycle_manager = std::make_shared<rclcpp_lifecycle::LifecycleNodeInterface>(lifecycle_node);
  // 运行生命周期管理器
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_manager->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
```

现在，在生命周期节点从 `unconfigured` 状态转移到 `inactive` 状态时，`on_configure` 回调将被调用，你的配置代码将被执行。你可以通过发送 `TRANSITION_CONFIGURE` 转换请求来触发这个过程。

## 如何在生命周期节点中实现启动？停止？清理？

在生命周期节点中实现启动、停止和清理，需要分别覆盖 `on_activate`、`on_deactivate` 和 `on_cleanup` 回调。以下是一个示例，展示了如何在生命周期节点中实现这些操作：

1. 首先，确保你已经包含了必要的头文件，并创建了一个继承自 `rclcpp_lifecycle::LifecycleNode` 的节点类。

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 构造函数中的初始化代码
  }

  // 其他成员和方法
};
```

2. 在你的节点类中，分别覆盖 `on_activate`、`on_deactivate` 和 `on_cleanup` 方法。在这些方法中，添加你的启动、停止和清理代码。

```cpp
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 构造函数中的初始化代码
  }

  // 覆盖 on_activate 回调
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "Activating the lifecycle node...");

    // 在这里实现启动代码，例如激活发布器和订阅器等

    RCLCPP_INFO(this->get_logger(), "Lifecycle node has been activated.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 覆盖 on_deactivate 回调
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating the lifecycle node...");

    // 在这里实现停止代码，例如去激活发布器和订阅器等

    RCLCPP_INFO(this->get_logger(), "Lifecycle node has been deactivated.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 覆盖 on_cleanup 回调
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up the lifecycle node...");

    // 在这里实现清理代码，例如释放资源、销毁服务、销毁发布器和订阅器等

    RCLCPP_INFO(this->get_logger(), "Lifecycle node has been cleaned up.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 其他成员和方法
};
```

3. 在主函数中，创建一个 `rclcpp_lifecycle::LifecycleNode` 类型的节点，并运行生命周期管理器：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "my_lifecycle_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 创建生命周期节点
  auto lifecycle_node = std::make_shared<MyLifecycleNode>();
  // 创建生命周期管理器
  auto lifecycle_manager = std::make_shared<rclcpp_lifecycle::LifecycleNodeInterface>(lifecycle_node);
  // 运行生命周期管理器
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_manager->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
```

现在，当生命周期节点分别从 `inactive` 到 `active`，`active` 到 `inactive` 和 `inactive` 到 `unconfigured` 状态时，`on_activate`、`on_deactivate` 和 `on_cleanup` 回调将被调用，你的启动、停止和清理代码将被执行。通过发送相应的转换请求(`TRANSITION_ACTIVATE`、`TRANSITION_DEACTIVATE`和 `TRANSITION_CLEANUP`)，可以触发这些过程。

## 如何使用生命周期管理器控制节点的状态转换？

要使用生命周期管理器控制节点的状态转换，你需要**创建一个生命周期管理器并将其连接到生命周期节点**。然后，你可以使用生命周期管理器提供的方法来触发状态转换。以下是一个示例，展示了如何使用生命周期管理器控制节点的状态转换：

1. 首先，确保你已经包含了必要的头文件，并创建了一个继承自 `rclcpp_lifecycle::LifecycleNode` 的节点类。这个类应该包含了你的生命周期节点实现。

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 构造函数中的初始化代码
  }

  // 其他成员和方法
};
```

2. 在主函数中，创建一个 `rclcpp_lifecycle::LifecycleNode` 类型的节点，并将其添加到生命周期管理器：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "my_lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_node_interface.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 创建生命周期节点
  auto lifecycle_node = std::make_shared<MyLifecycleNode>();
  // 创建生命周期管理器
  auto lifecycle_manager = std::make_shared<rclcpp_lifecycle::LifecycleNodeInterface>(lifecycle_node);
  // 运行生命周期管理器
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_manager->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
```

3. 使用生命周期管理器的方法来触发节点的状态转换。例如，你可以通过调用 `configure()`、`activate()`、`deactivate()` 和 `cleanup()` 方法来触发相应的状态转换。以下是一个示例，展示了如何使用生命周期管理器控制节点的状态转换：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "my_lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_node_interface.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 创建生命周期节点
  auto lifecycle_node = std::make_shared<MyLifecycleNode>();

  // 创建生命周期管理器
  auto lifecycle_manager = std::make_shared<rclcpp_lifecycle::LifecycleNodeInterface>(lifecycle_node);

  // 配置节点
  auto configure_result = lifecycle_manager->configure();
  if (configure_result.success)
  {
    RCLCPP_INFO(lifecycle_node->get_logger(), "Node successfully configured.");
  }
  else
  {
    RCLCPP_ERROR(lifecycle_node->get_logger(), "Node configuration failed.");
    rclcpp::shutdown();
    return 1;
  }

  // 激活节点
  auto activate_result = lifecycle_manager->activate();
  if (activate_result.success)
  {
    RCLCPP_INFO(lifecycle_node->get_logger(), "Node successfully activated.");
  }
  else
  {
    RCLCPP_ERROR(lifecycle_node->get_logger(), "Node activation failed.");
    rclcpp::shutdown();
    return 1;
  }

  // 运行生命周期管理器
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_manager->get_node_base_interface());
  executor.spin();

  // 取消激活节点
  auto deactivate_result = lifecycle_manager->deactivate();
  if (deactivate_result.success)
  {
    RCLCPP_INFO(lifecycle_node->get_logger(), "Node successfully deactivated.");
  }
  else
  {
    RCLCPP_ERROR(lifecycle_node->get_logger(), "Node deactivation failed.");
    rclcpp::shutdown();
    return 1;
  }

  // 清理节点
  auto cleanup_result = lifecycle_manager->cleanup();
  if (cleanup_result.success)
  {
    RCLCPP_INFO(lifecycle_node->get_logger(), "Node successfully cleaned up.");
  }
  else
  {
    RCLCPP_ERROR(lifecycle_node->get_logger(), "Node cleanup failed.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
```

这个示例展示了如何使用生命周期管理器来控制节点的状态转换。在这个示例中，我们**首先配置节点，然后激活节点。在执行期间，节点会保持激活状态。当我们需要停止节点时，我们先将节点取消激活，然后进行清理**。

请注意，这个示例仅用于演示目的。在实际应用中，你可能**需要根据特定的应用逻辑或外部事件来触发节点的状态转换**。例如，你可以**使用一个服务或动作服务器来接收状态转换请求，并使用生命周期管理器来控制节点的状态转换**。

## 如何使用服务或动作服务器来触发节点的状态转换？

要使用服务或动作服务器触发节点的状态转换，你**需要为生命周期状态转换创建一个服务或动作**，然后**在回调中调用生命周期管理器的方法来触发状态转换**。以下是一个使用服务触发状态转换的示例：

1. 首先，创建一个用于触发状态转换的服务。例如，我们可以定义一个名为 `TriggerTransition` 的服务，它接收一个整数参数 `transition_id`，用于表示要触发的状态转换。

```yaml
# trigger_transition.srv
int32 transition_id
---
bool success
string message
```

2. 在你的生命周期节点类中，添加一个服务成员，并在构造函数中创建服务：

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "your_package/srv/trigger_transition.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 创建用于触发状态转换的服务
    trigger_transition_service_ = this->create_service<your_package::srv::TriggerTransition>(
      "trigger_transition",
      std::bind(&MyLifecycleNode::trigger_transition_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  // 其他成员和方法

private:
  void trigger_transition_callback(
    const std::shared_ptr<your_package::srv::TriggerTransition::Request> request,
    std::shared_ptr<your_package::srv::TriggerTransition::Response> response);

  rclcpp::Service<your_package::srv::TriggerTransition>::SharedPtr trigger_transition_service_;
};
```

3. **在回调函数中，根据 `transition_id` 调用生命周期管理器的方法来触发相应的状态转换**。例如，你可以根据以下映射来触发状态转换：

- `TRANSITION_CONFIGURE`: 1
- `TRANSITION_ACTIVATE`: 2
- `TRANSITION_DEACTIVATE`: 3
- `TRANSITION_CLEANUP`: 4

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "your_package/srv/trigger_transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node_interface.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 创建用于触发状态转换的服务
    trigger_transition_service_ = this->create_service<your_package::srv::TriggerTransition>(
      "trigger_transition",
      std::bind(&MyLifecycleNode::trigger_transition_callback, this, std::placeholders::_1, std::placeholders::_2));
    lifecycle_manager_ = std::make_shared<rclcpp_lifecycle::LifecycleNodeInterface>(shared_from_this());
  }

  // 其他成员和方法

private:
  void trigger_transition_callback(
    const std::shared_ptr<your_package::srv::TriggerTransition::Request> request,
    std::shared_ptr<your_package::srv::TriggerTransition::Response> response)
  {
    rclcpp_lifecycle::LifecycleNodeInterface::CallbackReturn ret;

    switch (request->transition_id)
    {
      case 1:
        ret = lifecycle_manager_->configure();
        break;
      case 2:
        ret = lifecycle_manager_->activate();
        break;
      case 3:
        ret = lifecycle_manager_->deactivate();
        break;
      case 4:
        ret = lifecycle_manager_->cleanup();
        break;
      default:
        response->success = false;
        response->message = "Invalid transition_id";
        return;
    }

    response->success = (ret == rclcpp_lifecycle::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    response->message = response->success ? "Transition successful" : "Transition failed";
  }

  rclcpp::Service<your_package::srv::TriggerTransition>::SharedPtr trigger_transition_service_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNodeInterface> lifecycle_manager_;
};
```

现在，你可以通过调用 `trigger_transition` 服务来触发节点的状态转换。例如，你可以使用 `ros2 service call` 命令来触发状态转换：

```
ros2 service call /my_lifecycle_node/trigger_transition your_package/srv/TriggerTransition "{transition_id: 2}"
```

使用动作服务器触发状态转换的过程类似。你需要定义一个动作，然后在动作服务器的回调中调用生命周期管理器的方法来触发状态转换。以下是一个使用动作服务器触发状态转换的示例：

1. 首先，创建一个用于触发状态转换的动作。例如，我们可以定义一个名为 `TriggerTransition` 的动作，它接收一个整数参数 `transition_id`，用于表示要触发的状态转换。

```yaml
# trigger_transition.action
int32 transition_id
---
bool success
string message
---
```

2. 在你的生命周期节点类中，添加一个动作服务器成员，并在构造函数中创建动作服务器：

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "your_package/action/trigger_transition.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 创建用于触发状态转换的动作服务器
    trigger_transition_action_server_ = rclcpp_action::create_server<your_package::action::TriggerTransition>(
      this, "trigger_transition",
      std::bind(&MyLifecycleNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MyLifecycleNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&MyLifecycleNode::handle_accepted, this, std::placeholders::_1));
  }

  // 其他成员和方法

private:
  rclcpp_action::Server<your_package::action::TriggerTransition>::SharedPtr trigger_transition_action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const your_package::action::TriggerTransition::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<your_package::action::TriggerTransition>> goal_handle);

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<your_package::action::TriggerTransition>> goal_handle);
};
```

3. 在动作服务器的回调函数中，根据 `transition_id` 调用生命周期管理器的方法来触发相应的状态转换。

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "your_package/action/trigger_transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node_interface.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 创建用于触发状态转换的动作服务器
    trigger_transition_action_server_ = rclcpp_action::create_server<your_package::action::TriggerTransition>(
      this, "trigger_transition",
      std::bind(&MyLifecycleNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MyLifecycleNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&MyLifecycleNode::handle_accepted, this, std::placeholders::_1));
    lifecycle_manager_ = std::make_shared<rclcpp_lifecycle::LifecycleNodeInterface>(shared_from_this());
  }

  // 其他成员和方法

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const your_package::action::TriggerTransition::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received transition request with ID: %d", goal->transition_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<your_package::action::TriggerTransition>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<your_package::action::TriggerTransition>> goal_handle)
  {
    std::shared_ptr<your_package::action::TriggerTransition::Result> result =
      std::make_shared<your_package::action::TriggerTransition::Result>();

    rclcpp_lifecycle::LifecycleNodeInterface::CallbackReturn ret;

    switch (goal_handle->get_goal()->transition_id)
    {
      case 1:
        ret = lifecycle_manager_->configure();
        break;
      case 2:
        ret = lifecycle_manager_->activate();
        break;
      case 3:
        ret = lifecycle_manager_->deactivate();
        break;
      case 4:
        ret = lifecycle_manager_->cleanup();
        break;
      default:
        result->success = false;
        result->message = "Invalid transition_id";
        goal_handle->abort(result);
        return;
    }

    result->success = (ret == rclcpp_lifecycle::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    result->message = result->success ? "Transition successful" : "Transition failed";

    if (result->success)
    {
      goal_handle->succeed(result);
    }
    else
    {
      goal_handle->abort(result);
    }
  }

  rclcpp_action::Server<your_package::action::TriggerTransition>::SharedPtr trigger_transition_action_server_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNodeInterface> lifecycle_manager_;
};
```

现在，你可以通过调用 `trigger_transition` 动作来触发节点的状态转换。例如，你可以使用 `ros2 action send_goal` 命令来触发状态转换：

```
ros2 action send_goal /my_lifecycle_node/trigger_transition your_package/action/TriggerTransition "{transition_id: 2}"
```

这就是如何使用服务或动作服务器来触发节点的状态转换。请注意，你可能需要根据你的具体需求进一步完善和调整这些示例。

## 如何自定义节点的状态转换行为？

要自定义节点的状态转换行为，您可以在生命周期节点类中覆盖每个转换的默认回调方法。ROS 2 生命周期节点提供了以下可以覆盖的方法：

1. `on_configure`：在从 `unconfigured` 状态转换到 `inactive` 状态时调用。
2. `on_activate`：在从 `inactive` 状态转换到 `active` 状态时调用。
3. `on_deactivate`：在从 `active` 状态转换到 `inactive` 状态时调用。
4. `on_cleanup`：在从 `inactive` 状态转换到 `unconfigured` 状态时调用。
5. `on_shutdown`：在节点关闭时调用。
6. `on_error`：在节点遇到错误时调用。

> [!NOTE]
> 是不是依据 rcl_lifecycle 中给出的转换，应该还可以扩充很多的内容。

要自定义行为，您可以在从 `rclcpp_lifecycle::LifecycleNode` 派生的自定义生命周期节点类中覆盖这些方法。以下是一个示例：

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
  }

private:
  // 覆盖 on_configure 方法
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "自定义 on_configure 被调用");
    // 在此处添加自定义行为
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 同样，您可以根据需要覆盖其他方法(如 on_activate、on_deactivate 等)
};
```

在覆盖的方法中，您可以添加自定义行为，例如初始化资源、设置发布者或订阅者或进行清理。在这些方法中返回 `SUCCESS` 或 `FAILURE` 非常重要，具体取决于操作是否成功。

要使用此自定义生命周期节点，您需要在 `main` 函数中创建 `MyLifecycleNode` 类的实例：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "my_lifecycle_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto my_lifecycle_node = std::make_shared<MyLifecycleNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(my_lifecycle_node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
```

现在，您的节点将在状态转换期间执行覆盖方法中指定的自定义行为。

## 您能否提供一个覆盖 on_activate 方法的示例？

当然可以，以下是一个覆盖 `on_activate` 方法的示例。在这个示例中，我们假设您的节点有一个发布者。当节点从 `inactive` 状态转换到 `active` 状态时，将启动发布者。

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 创建一个不活动的发布者
    publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10,
                    rclcpp::PublisherOptions(),
                    rclcpp::IntraProcessSetting::AUTO);
  }

private:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Custom on_activate called");

    // 激活发布者
    publisher_->on_activate();

    // 添加其他自定义行为(例如启动定时器或初始化数据)

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 你还可以根据需要覆盖其他方法(如 on_deactivate、on_configure 等)

  // 定义一个发布者
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
};
```

在这个示例中，我们首先在构造函数中创建一个不活动的发布者。然后，我们覆盖了 `on_activate` 方法，当这个方法被调用时，我们激活了先前创建的发布者。您还可以在 `on_activate` 方法中添加其他自定义行为，例如启动定时器或初始化数据。

请注意，为了使此示例正常工作，您需要在 `CMakeLists.txt` 文件中链接相应的消息包(在本例中为 `std_msgs`)并将其包含在包的依赖关系中。

## 您能否提供一个覆盖 on_deactivate 方法的示例？

当然可以。以下是一个覆盖 `on_deactivate` 方法的示例。在这个示例中，我们假设您的节点有一个发布者。当节点从 `active` 状态转换到 `inactive` 状态时，将停用发布者。

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 创建一个不活动的发布者
    publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10,
                    rclcpp::PublisherOptions(),
                    rclcpp::IntraProcessSetting::AUTO);
  }

private:
  // 覆盖 on_deactivate 方法
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Custom on_deactivate called");

    // 停用发布者
    publisher_->on_deactivate();

    // 添加其他自定义行为(例如停止定时器或清除数据)

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 你还可以根据需要覆盖其他方法(如 on_activate、on_configure 等)

  // 定义一个发布者
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
};
```

在这个示例中，我们首先在构造函数中创建一个不活动的发布者。然后，我们覆盖了 `on_deactivate` 方法，当这个方法被调用时，我们停用了先前创建的发布者。您还可以在 `on_deactivate` 方法中添加其他自定义行为，例如停止定时器或清除数据。

请注意，为了使此示例正常工作，您需要在 `CMakeLists.txt` 文件中链接相应的消息包(在本例中为 `std_msgs`)并将其包含在包的依赖关系中。

## 如何在自定义生命周期节点类中添加发布者或订阅者？

在自定义生命周期节点类中添加发布者或订阅者，您需要使用 `rclcpp_lifecycle::LifecyclePublisher` 和 `rclcpp_lifecycle::LifecycleSubscriber` 类。这些类提供了与生命周期节点一起使用的发布者和订阅者的实现。

以下示例展示了如何在自定义生命周期节点类中添加一个发布者和一个订阅者：

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 创建一个不活动的发布者
    publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10,
                    rclcpp::PublisherOptions(),
                    rclcpp::IntraProcessSetting::AUTO);

    // 创建一个不活动的订阅者
    subscriber_ = this->create_subscription<std_msgs::msg::String>("my_topic", 10,
                      std::bind(&MyLifecycleNode::on_message, this, std::placeholders::_1),
                      rclcpp::SubscriptionOptions(),
                      rclcpp::IntraProcessSetting::AUTO);
  }

private:
  // 定义一个发布者
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;

  // 定义一个订阅者
  rclcpp_lifecycle::LifecycleSubscription<std_msgs::msg::String>::SharedPtr subscriber_;

  // 订阅者回调函数
  void on_message(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received message: %s", msg->data.c_str());
  }
};
```

在这个示例中，我们在构造函数中创建了一个不活动的发布者和一个不活动的订阅者。注意，我们使用了 `rclcpp_lifecycle::LifecyclePublisher` 和 `rclcpp_lifecycle::LifecycleSubscriber` 类来替换 `rclcpp` 中的标准 `Publisher` 和 `Subscriber` 类。

要激活和停用发布者和订阅者，您需要在 `on_activate` 和 `on_deactivate` 生命周期回调方法中调用它们的 `on_activate()` 和 `on_deactivate()` 方法。例如：

```cpp
private:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Custom on_activate called");

    // 激活发布者和订阅者
    publisher_->on_activate();
    subscriber_->on_activate();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Custom on_deactivate called");

    // 停用发布者和订阅者
    publisher_->on_deactivate();
    subscriber_->on_deactivate();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
```

请注意，为了使此示例正常工作，您需要在 `CMakeLists.txt` 文件中链接相应的消息包(在本例中为 `std_msgs`)并将其包含在包的依赖关系中。

## 如何在生命周期节点类中添加其他类型的 ROS 组件？

在生命周期节点类中**添加其他类型的 ROS 组件(如定时器、服务、客户端等)的过程与添加发布者和订阅者类似**。您需要使用生命周期节点兼容的组件。

以定时器为例，要在生命周期节点中添加定时器，您需要使用 `rclcpp_lifecycle::LifecycleNode::create_wall_timer` 方法。这是一个创建定时器的示例：

> [!NOTE]
> 这里给出了 timer\_ 中的一个 api `/*manual_start=*/true`
> 但是实际上并没有这个，但是如果要是是对 timer 使用 lifecycle 倒确实可以增加这个 api
> 可能实际上并不用，但是这个思路是可以的，直接提供给用户这样的操作

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 创建一个定时器，但在构造函数中不启动它
    auto timer_callback = std::bind(&MyLifecycleNode::on_timer, this);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), timer_callback, /*manual_start=*/true);
  }

private:
  // 定义一个定时器
  rclcpp::TimerBase::SharedPtr timer_;

  // 定时器回调函数
  void on_timer()
  {
    RCLCPP_INFO(get_logger(), "Timer callback triggered");
  }

  // 覆盖 on_activate 方法以启动定时器
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Custom on_activate called");

    // 启动定时器
    timer_->reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 覆盖 on_deactivate 方法以停止定时器
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Custom on_deactivate called");

    // 停止定时器
    timer_->cancel();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};
```

在这个示例中，我们在构造函数中创建了一个定时器，但未启动它(通过将 `manual_start` 参数设置为 `true`)。然后，我们覆盖了 `on_activate` 和 `on_deactivate` 生命周期回调方法，以分别启动和停止定时器。

---

对于服务**和客户端**，您需要使用 `rclcpp_lifecycle::LifecycleNode::create_service` 和 `rclcpp_lifecycle::LifecycleNode::create_client` 方法。下面是一个使用生命周期服务和生命周期客户端的示例：

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 创建一个生命周期服务
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&MyLifecycleNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));

    // 创建一个生命周期客户端
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  }

private:
  // 定义一个生命周期服务
  rclcpp_lifecycle::LifecycleService<example_interfaces::srv::AddTwoInts>::SharedPtr service_;

  // 定义一个生命周期客户端
  rclcpp_lifecycle::LifecycleClient<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

  // 服务回调函数
  void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    (void) request_header;
    response->sum = request->a + request->b;
    RCLCPP_INFO(get_logger(), "Incoming request: a: %ld, b: %ld", request->a, request->b);
  }
};
```

在这个示例中，我们在构造函数中创建了一个生命周期服务和一个生命周期客户端。请注意，**生命周期服务和生命周期客户端会在节点激活和停用时自动启用和禁用**。因此，您不需要在 `on_activate` 和 `on_deactivate` 生命周期回调方法中显式启用和禁用它们。

> [!NOTE]
> 虽然仅供参考，但是不清楚是为什么？

请注意，为了使此示例正常工作，您需要在 `CMakeLists.txt` 文件中链接相应的服务包(在本例中为 `example_interfaces`)并将其包含在包的依赖关系中。

## 如何在生命周期节点中启动和停止定时器？

在生命周期节点中启动和停止定时器，您需要在 `on_activate()` 和 `on_deactivate()` 生命周期回调方法中分别调用定时器的 `reset()` 和 `cancel()` 方法。

下面是一个在生命周期节点中使用定时器的示例：

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 创建一个定时器，但在构造函数中不启动它
    auto timer_callback = std::bind(&MyLifecycleNode::on_timer, this);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), timer_callback, /*manual_start=*/true);
  }

private:
  // 定义一个定时器
  rclcpp::TimerBase::SharedPtr timer_;

  // 定时器回调函数
  void on_timer()
  {
    RCLCPP_INFO(get_logger(), "Timer callback triggered");
  }

  // 覆盖 on_activate 方法以启动定时器
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Custom on_activate called");

    // 启动定时器
    timer_->reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 覆盖 on_deactivate 方法以停止定时器
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Custom on_deactivate called");

    // 停止定时器
    timer_->cancel();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};
```

在这个示例中，我们在构造函数中创建了一个定时器，但未启动它(通过将 `manual_start` 参数设置为 `true`)。然后，我们覆盖了 `on_activate` 和 `on_deactivate` 生命周期回调方法，以分别启动和停止定时器。

当然，您可以根据需要在其他生命周期回调方法中启动或停止定时器。例如，您可以在 `on_cleanup()` 中停止定时器，或者在 `on_configure()` 中启动定时器。

## 如何在自定义生命周期节点类中进行清理？

在自定义生命周期节点类中进行清理，您需要**覆盖 `on_cleanup()` 生命周期回调方法。在此方法中，您可以释放资源、停止定时器、关闭服务等**。下面是一个在生命周期节点中进行清理的示例：

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node")
  {
    // 在构造函数中创建发布者和定时器
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback = std::bind(&MyLifecycleNode::on_timer, this);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), timer_callback, /*manual_start=*/true);
  }

private:
  // 定义一个发布者
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;

  // 定义一个定时器
  rclcpp::TimerBase::SharedPtr timer_;

  // 定时器回调函数
  void on_timer()
  {
    auto message = std::make_unique<std_msgs::msg::String>();
    message->data = "Hello, world!";
    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message->data.c_str());
    publisher_->publish(std::move(message));
  }

  // 覆盖 on_activate 方法以启动定时器
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Custom on_activate called");

    // 启动定时器
    timer_->reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 覆盖 on_deactivate 方法以停止定时器
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Custom on_deactivate called");

    // 停止定时器
    timer_->cancel();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // 覆盖 on_cleanup 方法以进行清理
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Custom on_cleanup called");

    // 停止定时器
    timer_->cancel();

    // 释放发布者资源
    publisher_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};
```

在这个示例中，我们覆盖了 `on_cleanup()` 方法，并在其中停止了定时器，并释放了发布者资源。您可以根据需要在 `on_cleanup()` 方法中添加其他清理操作。例如，关闭服务、客户端、订阅者等。

## 如何在节点的状态转换期间执行特定的应用逻辑？

## 如何在节点的状态转换期间处理外部事件？

## 如何使用生命周期管理器将节点从 Inactive 状态转换为 Active 状态？

## 生命周期节点有哪些优点？

## 如何在 ROS2 中创建一个生命周期管理器？

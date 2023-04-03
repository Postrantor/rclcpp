##

这段代码是 ROS2（Robot Operating System 2）中的 `rclcpp` 库的一部分，用于实现异步和同步参数客户端。参数客户端允许用户从远程节点获取、设置、删除和描述参数。

### AsyncParametersClient 类

`AsyncParametersClient` 类提供了与远程节点进行异步通信的方法。它的构造函数接收以下参数：

- node_base_interface：节点基本接口的共享指针
- node_topics_interface：节点主题接口的共享指针
- node_graph_interface：节点图形接口的共享指针
- node_services_interface：节点服务接口的共享指针
- remote_node_name：远程节点名称
- qos_profile：QoS（Quality of Service）配置文件
- group：回调组的共享指针

在构造函数中，根据传入的接口和选项创建了五个客户端，分别用于处理以下服务：

1. get*parameters_client*：获取参数值
2. get*parameter_types_client*：获取参数类型
3. set*parameters_client*：设置参数值
4. set*parameters_atomically_client*：原子地设置参数值
5. list*parameters_client*：列出参数

`AsyncParametersClient` 类还提供了以下方法：

- get_parameters：获取指定名称的参数值
- describe_parameters：获取指定名称的参数描述符
- get_parameter_types：获取指定名称的参数类型
- set_parameters：设置参数值
- set_parameters_atomically：原子地设置参数值
- delete_parameters：删除指定名称的参数
- load_parameters：从 YAML 文件或参数映射中加载参数
- list_parameters：列出具有指定前缀的参数
- service_is_ready：检查所有服务是否准备就绪
- wait_for_service_nanoseconds：等待所有服务在指定的超时时间内准备就绪

### SyncParametersClient 类

`SyncParametersClient` 类提供了与远程节点进行同步通信的方法。它使用 `AsyncParametersClient` 的实例来执行同步操作。这个类提供了以下方法：

- get_parameters：获取指定名称的参数值
- has_parameter：检查是否存在指定名称的参数
- describe_parameters：获取指定名称的参数描述符
- get_parameter_types：获取指定名称的参数类型
- set_parameters：设置参数值
- delete_parameters：删除指定名称的参数
- load_parameters：从 YAML 文件加载参数
- set_parameters_atomically：原子地设置参数值
- list_parameters：列出具有指定前缀的参数

这些方法与 `AsyncParametersClient` 中的方法相似，但它们是同步的，会阻塞调用线程直到操作完成或超时。

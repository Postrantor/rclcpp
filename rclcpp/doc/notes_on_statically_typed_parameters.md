# Notes on statically typed parameters

## Introduction

Until ROS 2 Foxy, all parameters could change type anytime, except if the user installed a parameter callback to reject a change.

> 在 ROS 2 Foxy 之前，所有参数都可以随时更改类型，除非用户安装了参数回调以拒绝更改。

This could generate confusing errors, for example:

> 这可能会产生令人困惑的错误，例如:

```
    $ ros2 run demo_nodes_py listener &
    $ ros2 param set /listener use_sim_time not_a_boolean
    [ERROR] [1614712713.233733147] [listener]: use_sim_time parameter set to something besides a bool
    Set parameter successful
    $ ros2 param get /listener use_sim_time
    String value is: not_a_boolean

```

For most use cases, having static parameter types is enough.

> 对于大多数用例，拥有静态参数类型就足够了。

This article documents some of the decisions that were made when implementing static parameter types enforcement in:

> 本文记录了在实现静态参数类型强制时所做的一些决定:

- https://github.com/ros2/rclcpp/pull/1522
- https://github.com/ros2/rclpy/pull/683

## Allowing dynamically typed parameters

There might be some scenarios where dynamic typing is desired, so a `dynamic_typing` field was added to the [parameter descriptor](https://github.com/ros2/rcl_interfaces/blob/09b5ed93a733adb9deddc47f9a4a8c6e9f584667/rcl_interfaces/msg/ParameterDescriptor.msg#L25).

> 在某些情况下，可能需要动态键入，因此在[parameterdescriptor]中添加了“dynamic_typeing”字段(https://github.com/ros2/rcl_interfaces/blob/09b5ed93a733adb9deddc47f9a4a8c6e9f584667/rcl_interfaces/msg/ParameterDescriptor.msg#L25).

It defaults to `false`.

> 它默认为“false”。

For example:

> 例如:

```cpp
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;

    node->declare_parameter("dynamically_typed_parameter", rclcpp::ParameterValue{}, descriptor);

```

```py
    rcl_interfaces.msg.ParameterDescriptor descriptor;
    descriptor.dynamic_typing = True;

    node.declare_parameter("dynamically_typed_parameter", None, descriptor);

```

## How is the parameter type specified?

The parameter type will be inferred from the default value provided when declaring it.

> 参数类型将从声明时提供的默认值中推断出来。

## Statically typed parameters when allowing undeclared parameters

When undeclared parameters are allowed and a parameter is set without a previous declaration, the parameter will be dynamically typed.

> 如果允许未声明的参数，并且在没有先前声明的情况下设置参数，则该参数将被动态类型化。

This is consistent with other allowed behaviors when undeclared parameters are allowed, e.g. trying to get an undeclared parameter returns "NOT_SET".

> 当允许未声明的参数时，这与其他允许的行为是一致的，例如，尝试获取未声明参数返回“NOT_SET”。

Parameter declarations will remain special and dynamic or static typing will be used based on the parameter descriptor (default to static).

> 参数声明将保持特殊性，将根据参数描述符（默认为静态）使用动态或静态类型。

## Declaring a parameter without a default value

There might be cases were a default value does not make sense and the user must always provide an override.

> 在某些情况下，默认值不合理，用户必须始终提供覆盖。

In those cases, the parameter type can be specified explicitly:

> 在这些情况下，可以显式指定参数类型:

```cpp
    // method signature
    template<typename T>
    Node::declare_parameter<T>(std::string name, rcl_interfaces::msg::ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor{});
    // or alternatively
    Node::declare_parameter(std::string name, rclcpp::ParameterType type, rcl_interfaces::msg::ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor{});

    // examples
    node->declare_parameter<int64_t>("my_integer_parameter");  // declare an integer parameter
    node->declare_parameter("another_integer_parameter", rclcpp::ParameterType::PARAMETER_INTEGER);  // another way to do the same

```

```py
    # method signature
    Node.declare_parameter(name: str, param_type: rclpy.Parameter.Type, descriptor: rcl_interfaces.msg.ParameterDescriptor = rcl_interfaces.msg.ParameterDescriptor())

    # example
    node.declare_parameter('my_integer_parameter', rclpy.Parameter.Type.INTEGER);  # declare an integer parameter

```

If the parameter may be unused, but when used requires a parameter override, then you could conditionally declare it:

> 如果参数可能未使用，但使用时需要参数重写，则可以有条件地声明它:

```cpp
    auto mode = node->declare_parameter("mode", "modeA");  // "mode" parameter is an string
    if (mode == "modeB") {
        node->declare_parameter<int64_t>("param_needed_when_mode_b");  // when "modeB", the user must provide "param_needed_when_mode_b"
    }

```

## Other migration notes

Declaring a parameter with only a name is deprecated:

> 不推荐只使用名称声明参数:

```cpp
    node->declare_parameter("my_param");  // this generates a build warning

```

```py

node.declare_parameter("my_param");  # this generates a python user warning

> node.declare_parameter（“my_param”）；#这将生成一个python用户警告

```

Before, you could initialize a parameter with the "NOT SET" value (in cpp `rclcpp::ParameterValue{}`, in python `None`).

> 之前，您可以使用“NOT SET”值初始化参数（在 cpp`rcrcpp::ParameterValue｛｝`中，在 python`None`中）。

Now this will throw an exception in both cases:

> 现在，这将在两种情况下引发异常:

```cpp
    node->declare_parameter("my_param", rclcpp::ParameterValue{});  // not valid, will throw exception

```

```py
    node.declare_parameter("my_param", None);  # not valid, will raise error

```

## Possible improvements

### Easier way to declare dynamically typed parameters

Declaring a dynamically typed parameter in `rclcpp` could be considered to be a bit verbose:

> 在“rclcpp”中声明动态类型参数可能会被认为有点冗长:

```cpp
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;

    node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);

```

the following ways could be supported to make it simpler:

> 可以支持以下方式使其更简单:

```cpp
    node->declare_parameter(name, rclcpp::PARAMETER_DYNAMIC);
    node->declare_parameter(name, default_value, rclcpp::PARAMETER_DYNAMIC);

```

or alternatively:

> 或者可选地:

```cpp
    node->declare_parameter(name, default_value, rclcpp::ParameterDescriptor{}.dynamic_typing());

```

For `rclpy`, there's already a short way to do it:

> 对于“rcppy”，已经有了一个简单的方法:

```py
    node.declare_parameter(name, default_value, rclpy.ParameterDescriptor(dynamic_typing=true));

```

# Proposed node parameters callback Design

## Introduction:

The original requirement came in **gazebo_ros_pkgs** for setting individual wheel slip parameters based on global wheel slip value [link to original issue](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1365).

> 原始要求来自**gazebo_ros_pkgs**，用于基于全局车轮滑移值设置单个车轮滑移参数[链接至原始问题](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1365).

The main requirement is to set one or more parameters after another parameter is set successfully.

> 主要要求是在成功设置另一个参数后设置一个或多个参数。

Additionally, it would be nice if users could be notified locally (via a callback) when parameters have been set successfully (i.e. post validation).

> 此外，如果参数设置成功（即验证后），用户可以在本地（通过回调）得到通知，这将是很好的。

Related discussion can be found in [#609](https://github.com/ros2/rclcpp/issues/609) [#1789](https://github.com/ros2/rclcpp/pull/1789)

> 相关讨论可在[#609]中找到(https://github.com/ros2/rclcpp/issues/609) [#1789](https://github.com/ros2/rclcpp/pull/1789)

With the current parameters API, the `add_on_set_parameters_callback` is intended for validation of parameter values before they are set, it should **not** cause any side-effects.

> 对于当前的参数 API，`add_on_set_parameters_callback`用于在设置参数值之前对其进行验证，它应该**不会**引起任何副作用。

There is also the `ParameterEventHandler` that publishes changes to node parameters on `/parameter_events` topic for external nodes to see. Though the node could subscribe to the `/parameter_events` topic to be notified of changes to its own parameters, it is less than ideal since there is a delay caused by waiting for an executor to process the callback.

> 还有`ParameterEventHandler`，它发布对`/parameter_events`主题上节点参数的更改，供外部节点查看。尽管节点可以订阅`/parameter_events`主题，以获得其自身参数更改的通知，但这并不理想，因为等待执行器处理回调会导致延迟。

We propose adding a `PostSetParametersCallbackHandle` for successful parameter set similar to `OnSetParametersCallbackHandle` for parameter validation. Also, we propose adding a `PreSetParametersCallbackHandle` useful for modifying list of parameters being set.

> 我们建议为成功的参数集添加一个`PostSetParametersCallbackHandle`，类似于为参数验证添加`OnSetParametersCallbackHandle`。此外，我们建议添加一个`PreSetParametersCallbackHandle`，用于修改要设置的参数列表。

The validation callback is often abused to trigger side effects in the code, for instance updating class attributes even before a parameter has been set successfully. Instead of relying on the `/parameter_events` topic to be notified of parameter changes, users can register a callback with a new API, `add_post_set_parameters_callback`.

> 验证回调经常被滥用以触发代码中的副作用，例如，甚至在成功设置参数之前就更新类属性。用户可以使用新 API`add_post_set_parameters_callback`注册回调，而不是依靠`/parameter_events`主题来通知参数更改。

It is possible to use the proposed `add_post_set_parameters_callback` for setting additional parameters, but this might result in infinite recursion and does not allow those additional parameters to be set atomically with the original parameter(s) changed.

> 可以使用建议的`add_post_set_parameters_callback`设置附加参数，但这可能会导致无限递归，并且不允许在原始参数更改的情况下自动设置这些附加参数。

To workaround these issues, we propose adding a "pre set" callback type that can be registered with `add_pre_set_parameters_callback`, which will be triggered before the validation callbacks and can be used to modify the parameter list.

> 为了解决这些问题，我们建议添加一个`预设置`回调类型，该类型可以使用`add_pre_set_parameters_callback`注册，该类型将在验证回调之前触发，并可用于修改参数列表。

![Desgin API](https://github.com/ros2/rclcpp/blob/deepanshu/local-param-changed-callback-support/rclcpp/doc/param_callback_design.png?raw=true)

## Alternatives

- Users could call `set_parameter` while processing a message from the `/parameter_events` topic, however, there is extra overhead in having to create subscription (as noted earlier).

> -用户可以在处理来自`/parameter_events`主题的消息时调用`set_parameter`，但是，创建订阅会产生额外的开销（如前所述）。

- Users could call `set_parameter` inside the "on set" parameters callback, however it is not well-defined how side-effects should handle cases where parameter validation fails.

> -用户可以在`on set`参数回调中调用`set_parameter`，但副作用如何处理参数验证失败的情况尚不明确。

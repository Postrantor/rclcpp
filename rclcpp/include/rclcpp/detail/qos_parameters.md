---
> [!note]
> 感觉是找到了，这个就是 ros_to_dds？
> 这里是 rclcpp 层，直接对接的是 rmw 层的设置
...

##

```cpp
#include <algorithm>
#include <array>
#include <functional>
#include <initializer_list>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/node_interfaces/get_node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/qos_overriding_options.hpp"
#include "rcpputils/pointer_traits.hpp"
#include "rmw/qos_string_conversions.h"
```

##

```cpp
namespace rclcpp {
namespace detail {

struct PublisherQosParametersTraits {
  static constexpr const char *entity_type() { return "publisher"; }

  static constexpr auto allowed_policies() {
    return std::array<::rclcpp::QosPolicyKind, 9>{
    };
  }
};

struct SubscriptionQosParametersTraits {
  static constexpr const char *entity_type() { return "subscription"; }

  static constexpr auto allowed_policies() {
    return std::array<::rclcpp::QosPolicyKind, 8>{
    };
  }
};
```

这段代码位于 `ros2` 项目的 `rclcpp` 组件中，文件名为 `qos_parameters.hpp`。在这个文件中，定义了两个结构体：`PublisherQosParametersTraits` 和 `SubscriptionQosParametersTraits`，它们分别表示发布者和订阅者的 QoS（Quality of Service，服务质量）参数特征。

### PublisherQosParametersTraits

`PublisherQosParametersTraits` 结构体表示发布者的 QoS 参数特征。它包含以下成员函数：

1. `entity_type()`：返回一个字符串常量，表示实体类型为 "publisher"。
2. `allowed_policies()`：返回一个 `std::array` 类型的对象，其中包含允许的 QoS 策略种类。当前数组为空，但可以根据需要添加相关策略。

### SubscriptionQosParametersTraits

`SubscriptionQosParametersTraits` 结构体表示订阅者的 QoS 参数特征。它包含以下成员函数：

1. `entity_type()`：返回一个字符串常量，表示实体类型为 "subscription"。
2. `allowed_policies()`：返回一个 `std::array` 类型的对象，其中包含允许的 QoS 策略种类。当前数组为空，但可以根据需要添加相关策略。

这两个结构体主要用于描述发布者和订阅者的 QoS 参数特征，以便在实现发布者和订阅者时，可以根据这些特征来设置相应的 QoS 策略。虽然当前 `allowed_policies()` 函数返回的数组为空，但在实际使用中，可以根据需要添加不同的 QoS 策略种类，以满足不同场景下的服务质量要求。

---

在实际的 ROS 2 节点中，`PublisherQosParametersTraits` 和 `SubscriptionQosParametersTraits` 结构体主要用于在创建发布者和订阅者时设置 QoS 参数。当你创建一个发布者或订阅者时，这些函数会被调用。

以下是一个简单的示例，说明如何在 ROS 2 节点中创建发布者和订阅者：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      std::bind(&MyNode::on_message, this, std::placeholders::_1));
  }

private:
  void on_message(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}
```

在上述示例中，我们创建了一个名为 `MyNode` 的 ROS 2 节点。在节点的构造函数中，我们使用 `create_publisher` 和 `create_subscription` 函数创建了一个发布者和一个订阅者。这两个函数在内部会调用 `PublisherQosParametersTraits` 和 `SubscriptionQosParametersTraits` 结构体的成员函数，以设置相应的 QoS 参数。

需要注意的是，在实际使用中，你可能不会直接调用这些结构体的成员函数。它们主要用于 `rclcpp` 库内部实现，以确保在创建发布者和订阅者时正确设置 QoS 参数。然而，了解这些结构体及其成员函数如何工作有助于更好地理解 ROS 2 的 QoS 设置机制。

---

是的，如果您需要在 ROS 2 中引入其他暂时未添加的 QoS 设置，如 `ownership` 和 `ownership_strength`，您需要对 `qos_parameters.hpp` 文件中的相关函数内容进行修改。具体来说，您需要执行以下步骤：

1. 在 `qos_parameters.hpp` 文件中，为 `PublisherQosParametersTraits` 和/或 `SubscriptionQosParametersTraits` 结构体添加新的 QoS 参数成员变量。

```cpp
struct PublisherQosParametersTraits
{
  // ...
  rmw_qos_ownership_kind_t ownership;
  int32_t ownership_strength;
};
```

2. 修改 `declare_parameters` 函数以声明新的 QoS 参数。

```cpp
template<typename NodeT>
static void declare_parameters(
  NodeT * node,
  const std::string & topic_name,
  const PublisherQosParametersTraits & traits)
{
  // ...
  node->declare_parameter(
    prefix + ".ownership",
    rclcpp::ParameterValue(static_cast<int>(traits.ownership)),
    descriptor);
  node->declare_parameter(
    prefix + ".ownership_strength",
    rclcpp::ParameterValue(traits.ownership_strength),
    descriptor);
}
```

3. 修改 `get_parameters` 函数以获取新的 QoS 参数值。

```cpp
template<typename NodeT>
static void get_parameters(
  NodeT * node,
  const std::string & topic_name,
  PublisherQosParametersTraits & traits)
{
  // ...
  int ownership;
  node->get_parameter(prefix + ".ownership", ownership);
  traits.ownership = static_cast<rmw_qos_ownership_kind_t>(ownership);
  node->get_parameter(prefix + ".ownership_strength", traits.ownership_strength);
}
```

4. 修改 `apply_parameters` 函数以将新的 QoS 参数值应用到 `rclcpp::QoS` 对象。

```cpp
template<typename NodeT>
static void apply_parameters(
  const PublisherQosParametersTraits & traits,
  rclcpp::QoS & qos)
{
  // ...
  qos.ownership(traits.ownership);
  qos.ownership_strength(traits.ownership_strength);
}
```

请注意，这些更改可能需要您对 ROS 2 的底层实现进行修改，以确保新引入的 QoS 设置能够在运行时生效。此外，在使用新的 QoS 设置之前，请确保已经了解了它们的含义和适用场景，以避免可能出现的问题。

##

```cpp
inline ::rclcpp::ParameterValue get_default_qos_param_value(
  rclcpp::QosPolicyKind policy,
  const rclcpp::QoS &qos);

inline void apply_qos_override(
  rclcpp::QosPolicyKind policy,
  rclcpp::ParameterValue value,
  rclcpp::QoS &qos);

inline rclcpp::ParameterValue declare_parameter_or_get(
    rclcpp::node_interfaces::NodeParametersInterface &parameters_interface,
    const std::string &param_name,
    rclcpp::ParameterValue param_value,
    rcl_interfaces::msg::ParameterDescriptor descriptor) {
  try {
    return parameters_interface.declare_parameter(param_name, param_value, descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    return parameters_interface.get_parameter(param_name).get_parameter_value();
  }
}

#ifdef DOXYGEN_ONLY
template <typename NodeT, typename EntityQosParametersTraits>
rclcpp::QoS declare_qos_parameters(
  const ::rclcpp::QosOverridingOptions &options,
  NodeT &node,
  const std::string &topic_name,
  const ::rclcpp::QoS &default_qos,
  EntityQosParametersTraits);

#else
```

这个文件（qos_parameters.hpp）包含以下几个函数：

1. `get_default_qos_param_value`：获取默认的 QoS 参数值。该函数接受一个 QoS 策略（`rclcpp::QosPolicyKind`）和一个 QoS 对象（`rclcpp::QoS`），然后返回相应的默认参数值。

2. `apply_qos_override`：应用 QoS 参数覆盖。该函数接受一个 QoS 策略（`rclcpp::QosPolicyKind`）、一个参数值（`rclcpp::ParameterValue`）和一个 QoS 对象（`rclcpp::QoS`）。它会根据给定的策略和参数值修改 QoS 对象。

3. `declare_parameter_or_get`：声明参数或获取已存在的参数。该函数接受一个参数接口（`rclcpp::node_interfaces::NodeParametersInterface`）、一个参数名（`std::string`）、一个参数值（`rclcpp::ParameterValue`）和一个参数描述符（`rcl_interfaces::msg::ParameterDescriptor`）。如果参数尚未声明，它会声明参数并返回给定的参数值；如果参数已经声明，它会返回现有的参数值。

4. `declare_qos_parameters`：声明 QoS 参数。这是一个模板函数，接受以下参数：

   - 一个 QoS 覆盖选项对象（`::rclcpp::QosOverridingOptions`）
   - 一个节点对象（`NodeT`）
   - 一个主题名称（`std::string`）
   - 一个默认的 QoS 对象（`::rclcpp::QoS`）
   - 一个实体 QoS 参数特征对象（`EntityQosParametersTraits`）

   这个函数用于声明和调整 QoS 参数，以便在节点之间进行通信时满足特定的服务质量要求。

总之，这个文件（qos_parameters.hpp）主要用于处理 ROS2 项目中 rclcpp 的 QoS 参数设置和调整。通过这些函数，用户可以根据需要自定义节点之间的通信行为，以满足不同场景下的性能要求。

##

```cpp
template <typename NodeT, typename EntityQosParametersTraits>
std::enable_if_t<
    (rclcpp::node_interfaces::has_node_parameters_interface<
         decltype(std::declval<typename rcpputils::remove_pointer<NodeT>::type>())>::value ||
     std::is_same<typename std::decay_t<NodeT>,
                  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr>::value),
    rclcpp::QoS>
declare_qos_parameters(
  const ::rclcpp::QosOverridingOptions &options,  ///< [in] QoS 覆盖选项。(QoS overriding options.)
  NodeT &node,   ///< [inout] 节点引用。(A reference to the node.)
  const std::string &topic_name,     ///< [in] 主题名称。(Topic name.)
  const ::rclcpp::QoS &default_qos,  ///< [in] 默认 QoS。(Default QoS.)
  EntityQosParametersTraits)  ///< [in] QoS 参数特征。(QoS parameter traits.)
{
  auto &parameters_interface = *rclcpp::node_interfaces::get_node_parameters_interface(node);

  std::string param_prefix;
  const auto &id = options.get_id();
  {
    std::ostringstream oss{"qos_overrides.", std::ios::ate};
    oss << topic_name << "." << EntityQosParametersTraits::entity_type();
    if (!id.empty()) {
      oss << "_" << id;
    }
    oss << ".";
    param_prefix = oss.str();
  }

  std::string param_description_suffix;
  {
    std::ostringstream oss{"} for ", std::ios::ate};
    oss << EntityQosParametersTraits::entity_type() << " {" << topic_name << "}";
    if (!id.empty()) {
      oss << " with id {" << id << "}";
    }
    param_description_suffix = oss.str();
  }

  rclcpp::QoS qos = default_qos;

  for (auto policy : EntityQosParametersTraits::allowed_policies()) {
    if (std::count(options.get_policy_kinds().begin(), options.get_policy_kinds().end(), policy)) {
      std::ostringstream param_name{param_prefix, std::ios::ate};
      param_name << qos_policy_kind_to_cstr(policy);

      std::ostringstream param_desciption{"qos policy {", std::ios::ate};
      param_desciption << qos_policy_kind_to_cstr(policy) << param_description_suffix;

      rcl_interfaces::msg::ParameterDescriptor descriptor{};
      descriptor.description = param_desciption.str();
      descriptor.read_only = true;

      auto value = declare_parameter_or_get(parameters_interface, param_name.str(),
                                            get_default_qos_param_value(policy, qos), descriptor);

      ::rclcpp::detail::apply_qos_override(policy, value, qos);
    }
  }

  const auto &validation_callback = options.get_validation_callback();
  if (validation_callback) {
    auto result = validation_callback(qos);
    if (!result.successful) {
      throw rclcpp::exceptions::InvalidQosOverridesException{"validation callback failed: " +
                                                             result.reason};
    }
  }

  return qos;
}

template <typename NodeT, typename EntityQosParametersTraits>
std::enable_if_t<
    !(rclcpp::node_interfaces::has_node_parameters_interface<
          decltype(std::declval<typename rcpputils::remove_pointer<NodeT>::type>())>::value ||
      std::is_same<typename std::decay_t<NodeT>,
                   rclcpp::node_interfaces::NodeParametersInterface::SharedPtr>::value),
    rclcpp::QoS>
declare_qos_parameters(const ::rclcpp::QosOverridingOptions &options,
                       NodeT &node,
                       const std::string &entity_name,
                       const ::rclcpp::QoS &default_qos,
                       EntityQosParametersTraits) {
  if (options.get_policy_kinds().size()) {
    std::runtime_error exc{
        "passed non-default qos overriding options without providing a parameters interface"};

    throw exc;
  }

  return default_qos;
}
#endif
```

1. 第一个 `declare_qos_parameters` 函数：

   这个函数用于声明和获取与实体相关的 QoS 参数。它接受以下参数：

   - `options`：QoS 覆盖选项，用于指定哪些 QoS 策略需要覆盖。
   - `node`：节点引用，用于访问节点的参数接口。
   - `topic_name`：主题名称，用于构建参数名称前缀。
   - `default_qos`：默认 QoS，如果没有覆盖选项，则使用此 QoS。
   - `EntityQosParametersTraits`：QoS 参数特征，包括实体类型（发布者或订阅者）以及允许的 QoS 策略。

   函数首先获取节点的参数接口，并构建参数名称前缀和描述后缀。然后，遍历允许的 QoS 策略，检查是否有覆盖选项。如果有覆盖选项，函数会声明相应的参数并获取其值，然后将这些值应用到 QoS 对象上。最后，如果提供了验证回调，函数会使用该回调验证生成的 QoS，并在验证失败时抛出异常。

2. 第二个 `declare_qos_parameters` 函数：

   这个函数是第一个函数的重载版本，用于处理没有参数接口的节点。它接受与第一个函数相同的参数。当传入的节点没有参数接口时，这个函数将被调用。如果提供了非默认的 QoS 覆盖选项，函数会抛出运行时异常。否则，它将直接返回默认 QoS。

总之，这两个 `declare_qos_parameters` 函数用于处理 QoS 参数的声明和获取，以便在 ROS2 项目中根据需要覆盖和应用 QoS 策略。

##

```cpp
#define RCLCPP_DETAIL_APPLY_QOS_OVERRIDE_FROM_PARAMETER_STRING(kind_lower, kind_upper,           \
                                                               parameter_value, rclcpp_qos)      \
  do {                                                                                           \
    auto policy_string = (parameter_value).get<std::string>();                                   \
    auto policy_value = rmw_qos_##kind_lower##_policy_from_str(policy_string.c_str());           \
    if (RMW_QOS_POLICY_##kind_upper##_UNKNOWN == policy_value) {                                 \
      throw std::invalid_argument{"unknown QoS policy " #kind_lower " value: " + policy_string}; \
    }                                                                                            \
    ((rclcpp_qos).kind_lower)(policy_value);                                                     \
  } while (0)

inline void apply_qos_override(rclcpp::QosPolicyKind policy,
                               rclcpp::ParameterValue value,
                               rclcpp::QoS &qos) {
  switch (policy) {
    case QosPolicyKind::AvoidRosNamespaceConventions:
      qos.avoid_ros_namespace_conventions(value.get<bool>());
      break;
    case QosPolicyKind::Deadline:
      qos.deadline(::rclcpp::Duration::from_nanoseconds(value.get<int64_t>()));
      break;
    case QosPolicyKind::Durability:
      RCLCPP_DETAIL_APPLY_QOS_OVERRIDE_FROM_PARAMETER_STRING(durability, DURABILITY, value, qos);
      break;
    case QosPolicyKind::History:
      RCLCPP_DETAIL_APPLY_QOS_OVERRIDE_FROM_PARAMETER_STRING(history, HISTORY, value, qos);
      break;
    case QosPolicyKind::Depth:
      qos.get_rmw_qos_profile().depth = static_cast<size_t>(value.get<int64_t>());
      break;
    case QosPolicyKind::Lifespan:
      qos.lifespan(::rclcpp::Duration::from_nanoseconds(value.get<int64_t>()));
      break;
    case QosPolicyKind::Liveliness:
      RCLCPP_DETAIL_APPLY_QOS_OVERRIDE_FROM_PARAMETER_STRING(liveliness, LIVELINESS, value, qos);
      break;
    case QosPolicyKind::LivelinessLeaseDuration:
      qos.liveliness_lease_duration(::rclcpp::Duration::from_nanoseconds(value.get<int64_t>()));
      break;
    case QosPolicyKind::Reliability:
      RCLCPP_DETAIL_APPLY_QOS_OVERRIDE_FROM_PARAMETER_STRING(reliability, RELIABILITY, value, qos);
      break;
    default:
      throw std::invalid_argument{"unknown QosPolicyKind"};
  }
}
```

这段代码位于 ROS2 项目中的 rclcpp 库，与 QoS（Quality of Service，服务质量）参数相关。QoS 参数用于配置通信层的各种策略，以满足不同应用场景的需求。在这个文件中，主要定义了一个宏（`RCLCPP_DETAIL_APPLY_QOS_OVERRIDE_FROM_PARAMETER_STRING`）和一个内联函数（`apply_qos_override`），用于根据给定的参数值更新 QoS 对象的策略。

### RCLCPP_DETAIL_APPLY_QOS_OVERRIDE_FROM_PARAMETER_STRING 宏

这个宏接受四个参数：`kind_lower`、`kind_upper`、`parameter_value` 和 `rclcpp_qos`。`kind_lower` 和 `kind_upper` 分别表示 QoS 策略的小写和大写形式。`parameter_value` 是一个 `rclcpp::ParameterValue` 对象，包含了策略的字符串值。`rclcpp_qos` 是一个 `rclcpp::QoS` 对象，用于存储更新后的 QoS 策略。

宏的主要作用是将策略字符串转换为对应的枚举值，并将其应用到 QoS 对象上。如果策略字符串无法识别，它会抛出一个 `std::invalid_argument` 异常。

### apply_qos_override 函数

这个内联函数接受三个参数：`policy`、`value` 和 `qos`。`policy` 是一个 `rclcpp::QosPolicyKind` 枚举值，表示要更新的 QoS 策略。`value` 是一个 `rclcpp::ParameterValue` 对象，包含了策略的值。`qos` 是一个 `rclcpp::QoS` 对象，用于存储更新后的 QoS 策略。

函数的主要作用是根据给定的策略和值，更新 QoS 对象的相应策略。它使用了一个 `switch` 语句来处理不同的策略类型，并调用相应的 QoS 对象的方法进行更新。对于需要将字符串值转换为枚举值的策略（如 Durability、History、Liveliness 和 Reliability），函数会使用前面定义的宏来完成转换和更新操作。

如果遇到未知的 QosPolicyKind，函数会抛出一个 `std::invalid_argument` 异常。

总之，这个文件的主要功能是提供一种灵活的方式来根据参数值更新 QoS 对象的策略，以便在运行时根据需要调整通信层的行为。

##

```cpp
inline int64_t rmw_duration_to_int64_t(rmw_time_t rmw_duration) {
  return ::rclcpp::Duration(static_cast<int32_t>(rmw_duration.sec),
                            static_cast<uint32_t>(rmw_duration.nsec))
      .nanoseconds();
}

inline const char *check_if_stringified_policy_is_null(const char *policy_value_stringified,
                                                       QosPolicyKind kind) {
  if (!policy_value_stringified) {
    std::ostringstream oss{"unknown value for policy kind {", std::ios::ate};
    oss << kind << "}";
    throw std::invalid_argument{oss.str()};
  }
  return policy_value_stringified;
}

inline ::rclcpp::ParameterValue get_default_qos_param_value(rclcpp::QosPolicyKind kind,
                                                            const rclcpp::QoS &qos) {
  using ParameterValue = ::rclcpp::ParameterValue;
  const auto &rmw_qos = qos.get_rmw_qos_profile();
  switch (kind) {
    case QosPolicyKind::AvoidRosNamespaceConventions:
      return ParameterValue(rmw_qos.avoid_ros_namespace_conventions);
    case QosPolicyKind::Deadline:
      return ParameterValue(rmw_duration_to_int64_t(rmw_qos.deadline));
    case QosPolicyKind::Durability:
      return ParameterValue(check_if_stringified_policy_is_null(
          rmw_qos_durability_policy_to_str(rmw_qos.durability), kind));
    case QosPolicyKind::History:
      return ParameterValue(check_if_stringified_policy_is_null(
          rmw_qos_history_policy_to_str(rmw_qos.history), kind));
    case QosPolicyKind::Depth:
      return ParameterValue(static_cast<int64_t>(rmw_qos.depth));
    case QosPolicyKind::Lifespan:
      return ParameterValue(rmw_duration_to_int64_t(rmw_qos.lifespan));
    case QosPolicyKind::Liveliness:
      return ParameterValue(check_if_stringified_policy_is_null(
          rmw_qos_liveliness_policy_to_str(rmw_qos.liveliness), kind));
    case QosPolicyKind::LivelinessLeaseDuration:
      return ParameterValue(rmw_duration_to_int64_t(rmw_qos.liveliness_lease_duration));
    case QosPolicyKind::Reliability:
      return ParameterValue(check_if_stringified_policy_is_null(
          rmw_qos_reliability_policy_to_str(rmw_qos.reliability), kind));
    default:
      throw std::invalid_argument{"unknown QoS policy kind"};
  }
}
```

这段代码是 ROS2 项目中的 `rclcpp` 库相关的一部分，主要涉及到 QoS（Quality of Service，服务质量）参数的处理。在 ROS2 中，QoS 参数用于控制节点间通信的各种属性，如可靠性、延迟等。这个文件（`qos_parameters.hpp`）定义了一些内联函数，用于处理 QoS 参数。

1. `rmw_duration_to_int64_t` 函数：将 `rmw_time_t` 类型的持续时间转换为 `int64_t` 类型的纳秒值。输入参数是一个 `rmw_time_t` 类型的结构体，包含秒和纳秒两个成员。函数通过创建一个 `rclcpp::Duration` 对象来完成转换，并返回纳秒值。

2. `check_if_stringified_policy_is_null` 函数：检查给定的策略字符串是否为空。如果为空，则抛出一个 `std::invalid_argument` 异常。输入参数包括策略值的字符串表示和策略类型（`QosPolicyKind` 枚举值）。该函数主要用于确保策略字符串有效。

3. `get_default_qos_param_value` 函数：根据给定的 QoS 策略类型（`rclcpp::QosPolicyKind`）和 QoS 配置对象（`rclcpp::QoS`），获取默认的 QoS 参数值。函数首先从 QoS 对象中获取对应的 RMW（ROS Middleware）QoS 配置，然后根据策略类型使用 `switch` 语句选择适当的处理方式。对于每种策略类型，函数返回一个 `rclcpp::ParameterValue` 对象，其中包含相应的默认值。

这些函数主要用于处理 QoS 参数，确保它们在 ROS2 系统中正确传递和使用。

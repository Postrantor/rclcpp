// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__TYPE_ADAPTER_HPP_
#define RCLCPP__TYPE_ADAPTER_HPP_

#include <type_traits>

namespace rclcpp {

/// 模板结构用于将自定义类型适配为ROS类型。
/// Template structure used to adapt custom, user-defined types to ROS types.
/**
 * 将自定义类型适配为ROS类型，使得自定义类型可以在ROS中发布和订阅。
 * Adapting a custom, user-defined type to a ROS type allows that custom type
 * to be used when publishing and subscribing in ROS.
 *
 * 为了将自定义类型适配为ROS类型，用户必须为自定义类型创建此结构的模板特化。
 * In order to adapt a custom type to a ROS type, the user must create a
 * template specialization of this structure for the custom type.
 * 在该特化中，他们必须：
 * In that specialization they must:
 *
 *   - 将 `is_specialized` 更改为 `std::true_type`，
 *   - change `is_specialized` to `std::true_type`,
 *   - 使用 `using custom_type = ...` 指定自定义类型，
 *   - specify the custom type with `using custom_type = ...`,
 *   - 使用 `using ros_message_type = ...` 指定ROS类型，
 *   - specify the ROS type with `using ros_message_type = ...`,
 *   - 提供具有以下签名的静态转换函数：
 *   - provide static convert functions with the signatures:
 *     - static void convert_to_ros(const custom_type &, ros_message_type &)
 *     - static void convert_to_custom(const ros_message_type &, custom_type &)
 *
 * 转换函数必须将一种类型转换为另一种类型。
 * The convert functions must convert from one type to the other.
 *
 * 例如，这是一个将 `std::string` 适配为 `std_msgs::msg::String` ROS消息类型的理论示例：
 * For example, here is a theoretical example for adapting `std::string` to the
 * `std_msgs::msg::String` ROS message type:
 *
 *     template<>
 *     struct rclcpp::TypeAdapter<std::string, std_msgs::msg::String>
 *     {
//  *       使用 is_specialized = std::true_type;
 *       using is_specialized = std::true_type;
//  *       使用 custom_type = std::string;
 *       using custom_type = std::string;
//  *       使用 ros_message_type = std_msgs::msg::String;
 *       using ros_message_type = std_msgs::msg::String;
 *
 *       static
 *       void
 *       convert_to_ros_message(
 *         const custom_type & source,
 *         ros_message_type & destination)
 *       {
 *         destination.data = source;
 *       }
 *
 *       static
 *       void
 *       convert_to_custom(
 *         const ros_message_type & source,
 *         custom_type & destination)
 *       {
 *         destination = source.data;
 *       }
 *     };
 *
 * 然后，可以在创建发布者或订阅者时使用适配器，例如：
 * The adapter can then be used when creating a publisher or subscription,
 * e.g.:
 *
//  *     使用 MyAdaptedType = TypeAdapter<std::string, std_msgs::msg::String>;
 *     auto pub = node->create_publisher<MyAdaptedType>("topic", 10);
 *     auto sub = node->create_subscription<MyAdaptedType>(
 *       "topic",
 *       10,
 *       [](const std::string & msg) {...});
 *
 * 您还可以使用 adapt_type::as 元函数更具声明性地进行操作，这些元函数在阅读时稍微不那么模糊：
 * You can also be more declarative by using the adapt_type::as metafunctions,
 * which are a bit less ambiguous to read:
 *
//  *     使用 AdaptedType = rclcpp::adapt_type<std::string>::as<std_msgs::msg::String>;
 *     auto pub = node->create_publisher<AdaptedType>(...);
 *
 * 如果您愿意，您可以将自定义类型与单个ROS消息类型关联，
 * 允许您在创建实体时更简洁，例如：
 * If you wish, you may associate a custom type with a single ROS message type,
 * allowing you to be a bit more brief when creating entities, e.g.:
 *
 *     // 首先，您必须声明关联，这类似于如何通过执行 `using std::vector;`
来避免在C++中使用命名空间。
 *     // First you must declare the association, this is similar to how you
 *     // would avoid using the namespace in C++ by doing `using std::vector;`.
 *     RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(std::string, std_msgs::msg::String);
 *
 *     // 然后，您可以仅使用自定义类型创建事物，并根据之前的语句暗示ROS
 *     // 消息类型。
 *     // Then you can create things with just the custom type, and the ROS
 *     // message type is implied based on the previous statement.
 *     auto pub = node->create_publisher<std::string>(...);
 */
template <typename CustomType, typename ROSMessageType = void, class Enable = void>
struct TypeAdapter {
  // 使用 is_specialized = std::false_type;
  using is_specialized = std::false_type;
  // 使用 custom_type = CustomType;
  using custom_type = CustomType;
  // 在这种情况下，仅给出了CustomType，或者没有特化。
  // 为前一种情况将ros_message_type分配给CustomType。
  // In this case, the CustomType is the only thing given, or there is no specialization.
  // Assign ros_message_type to CustomType for the former case.
  // 使用 ros_message_type = CustomType;
  using ros_message_type = CustomType;
};

/// 辅助模板以确定类型是否为TypeAdapter，false特化。
/// Helper template to determine if a type is a TypeAdapter, false specialization.
template <typename T>
struct is_type_adapter : std::false_type {};

/// 辅助模板以确定类型是否为TypeAdapter，true特化。
/// Helper template to determine if a type is a TypeAdapter, true specialization.
template <typename... Ts>
struct is_type_adapter<TypeAdapter<Ts...>> : std::true_type {};

/// TypeAdapter的身份特化。
/// Identity specialization for TypeAdapter.
template <typename T>
struct TypeAdapter<T, void, std::enable_if_t<is_type_adapter<T>::value>> : T {};

namespace detail {

template <typename CustomType, typename ROSMessageType>
struct assert_type_pair_is_specialized_type_adapter {
  // 使用 type_adapter = TypeAdapter<CustomType, ROSMessageType>;
  using type_adapter = TypeAdapter<CustomType, ROSMessageType>;
  static_assert(
      type_adapter::is_specialized::value,
      "No type adapter for this custom type/ros message type pair");
};

}  // namespace detail

/// 模板元函数，可以使正在适配的类型明确。
/// Template metafunction that can make the type being adapted explicit.
template <typename CustomType>
struct adapt_type {
  template <typename ROSMessageType>
  // 使用 as = typename ::rclcpp::detail::
  assert_type_pair_is_specialized_type_adapter<CustomType, ROSMessageType>::type_adapter;
};

/// 隐式类型适配器用作仅使用自定义类型创建某些内容的简写方式。
/// Implicit type adapter used as a short hand way to create something with just the custom type.
/**
 * 当与RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE()结合使用时，在创建发布者或订阅者时只需使用自定义类型。
 * This is used when creating a publisher or subscription using just the custom
 * type in conjunction with RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE().
 * 例如：
 * For example:
 *
 *     #include "type_adapter_for_std_string_to_std_msgs_String.hpp"
 *
 *     RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(std::string, std_msgs::msg::String);
 *
 *     int main(...) {
 *       // ...
 *       auto pub = node->create_publisher<std::string>(...);
 *     }
 *
 * \sa TypeAdapter 更多示例。
 * \sa TypeAdapter for more examples.
 */
template <typename CustomType>
struct ImplicitTypeAdapter {
  // 使用 is_specialized = std::false_type;
  using is_specialized = std::false_type;
};

/// TypeAdapter的ImplicitTypeAdapter特化。
/// Specialization of TypeAdapter for ImplicitTypeAdapter.
/**
 * 这允许这样的事情：
 *
 *    RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(std::string, std_msgs::msg::String);
 *    auto pub = node->create_publisher<std::string>("topic", 10);
 *
 */
template <typename T>
struct TypeAdapter<T, void, std::enable_if_t<ImplicitTypeAdapter<T>::is_specialized::value>>
    : ImplicitTypeAdapter<T> {};

/// 将给定的自定义类型/ROS消息类型对隐式分配给自定义类型。
/// Assigns the custom type implicitly to the given custom type/ros message type pair.
/**
 * 注意：此宏需要在根命名空间中使用。
 * 我们不能使用 ::rclcpp 来防止这种情况，因为GCC如何解释
 * 语法，请参阅：https://stackoverflow.com/a/2781537
 *
 * \sa TypeAdapter
 * \sa ImplicitTypeAdapter
 */
#define RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(CustomType, ROSMessageType)          \
  template <>                                                                             \
  struct rclcpp::ImplicitTypeAdapter<CustomType>                                          \
      : public rclcpp::TypeAdapter<CustomType, ROSMessageType> {                          \
    static_assert(                                                                        \
        is_specialized::value,                                                            \
        "Cannot use custom type as ros type when there is no TypeAdapter for that pair"); \
  }

}  // namespace rclcpp

#endif  // RCLCPP__TYPE_ADAPTER_HPP_

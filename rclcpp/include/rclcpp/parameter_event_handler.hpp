// Copyright 2019 Intel Corporation
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

#ifndef RCLCPP__PARAMETER_EVENT_HANDLER_HPP_
#define RCLCPP__PARAMETER_EVENT_HANDLER_HPP_

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/node_interfaces/get_node_base_interface.hpp"
#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

/**
 * @brief 参数回调句柄结构体 (Parameter Callback Handle Structure)
 */
struct ParameterCallbackHandle {
  // 定义智能指针类型 (Define smart pointer types)
  RCLCPP_SMART_PTR_DEFINITIONS(ParameterCallbackHandle)

  /**
   * @brief 参数回调类型定义 (Parameter callback type definition)
   * @param[in] parameter rclcpp::Parameter 类型的参数 (parameter of type rclcpp::Parameter)
   */
  using ParameterCallbackType = std::function<void(const rclcpp::Parameter &)>;

  std::string parameter_name;  ///< 参数名称 (Parameter name)
  std::string node_name;       ///< 节点名称 (Node name)

  /**
   * @brief 回调函数，用于处理参数变化 (Callback function to handle parameter changes)
   */
  ParameterCallbackType callback;
};

/**
 * @brief 参数事件回调句柄结构体 (Parameter Event Callback Handle Structure)
 */
struct ParameterEventCallbackHandle {
  // 定义智能指针类型 (Define smart pointer types)
  RCLCPP_SMART_PTR_DEFINITIONS(ParameterEventCallbackHandle)

  /**
   * @brief 参数事件回调类型定义 (Parameter event callback type definition)
   * @param[in] parameter_event rcl_interfaces::msg::ParameterEvent 类型的参数事件 (parameter event
   * of type rcl_interfaces::msg::ParameterEvent)
   */
  using ParameterEventCallbackType =
      std::function<void(const rcl_interfaces::msg::ParameterEvent &)>;

  /**
   * @brief 回调函数，用于处理参数事件 (Callback function to handle parameter events)
   */
  ParameterEventCallbackType callback;
};

/// 用于“处理”（监控和响应）参数更改的类。
/// A class used to "handle" (monitor and respond to) changes to parameters.
/**
 * ParameterEventHandler 类允许监控节点参数的变化，
 * 包括节点自身的参数或系统中其他节点拥有的参数。
 * 可以设置多个参数回调，并在指定的参数发生变化时调用它们。
 *
 * 第一步是实例化一个 ParameterEventHandler，提供一个 ROS 节点以创建所需的订阅：
 *
 * The first step is to instantiate a ParameterEventHandler, providing a ROS node to use
 * to create any required subscriptions:
 *
 *   auto param_handler = std::make_shared<rclcpp::ParameterEventHandler>(node);
 *
 * 接下来，您可以将回调添加到 add_parameter_callback 方法，如下所示：
 *
 * Next, you can supply a callback to the add_parameter_callback method, as follows:
 *
 *   auto cb1 = [&node](const rclcpp::Parameter & p) {
 *      RCLCPP_INFO(
 *        node->get_logger(),
 *        "cb1: Received an update to parameter \"%s\" of type %s: \"%ld\"",
 *        p.get_name().c_str(),
 *        p.get_type_name().c_str(),
 *        p.as_int());
 *   };
 *   auto handle1 = param_handler->add_parameter_callback("an_int_param", cb1);
 *
 * 在这种情况下，我们没有提供节点名称（第三个，可选的参数），因此默认值将是监视与
 * 在 ParameterEventHandler 构造函数中提供的 ROS 节点关联的 "an_int_param" 参数的更改。
 * 回调（在这种情况下是 lambda 函数）只需打印出参数的值。
 *
 * In this case, we didn't supply a node name (the third, optional, parameter) so the
 * default will be to monitor for changes to the "an_int_param" parameter associated with
 * the ROS node supplied in the ParameterEventHandler constructor.
 * The callback, a lambda function in this case, simply prints out the value of the parameter.
 *
 * 注意：必须捕获 add_parameter_callback 返回的对象，否则回调将立即被取消注册。
 *
 * Note: the object returned from add_parameter_callback must be captured or the callback will
 * be immediately unregistered.
 *
 * 您还可以通过向 add_parameter_callback 提供节点名称来监视其他节点中的参数更改：
 *
 * You may also monitor for changes to parameters in other nodes by supplying the node
 * name to add_parameter_callback:
 *
 *   auto cb2 = [&node](const rclcpp::Parameter & p) {
 *       RCLCPP_INFO(
 *         node->get_logger(),
 *         "cb2: Received an update to parameter \"%s\" of type: %s: \"%s\"",
 *         p.get_name().c_str(),
 *         p.get_type_name().c_str(),
 *         p.as_string().c_str());
 *     };
 *   auto handle2 = param_handler->add_parameter_callback(
 *     "some_remote_param_name", cb2, "some_remote_node_name");
 *
 * 在这种情况下，当远程节点 "some_remote_node_name" 上的 "some_remote_param_name" 发生变化时，
 * 将调用回调。
 *
 * In this case, the callback will be invoked whenever "some_remote_param_name" changes
 * on remote node "some_remote_node_name".
 *
 * 若要删除参数回调，请重置回调句柄智能指针或调用
 * remove_parameter_callback，传递 add_parameter_callback 返回的句柄：
 *
 * To remove a parameter callback, reset the callback handle smart pointer or call
 * remove_parameter_callback, passing the handle returned from add_parameter_callback:
 *
 *   param_handler->remove_parameter_callback(handle2);
 *
 * 您还可以使用 add_parameter_event_callback 监视*所有*参数更改。
 * 在这种情况下，只要系统中的任何参数发生变化，回调就会被调用。
 * 您可能对这些参数更改中的一部分感兴趣，因此在回调中，最好使用正则表达式
 * 对感兴趣的节点名称或命名空间进行筛选。
 * 例如：
 *
 * You can also monitor for *all* parameter changes, using add_parameter_event_callback.
 * In this case, the callback will be invoked whenever any parameter changes in the system.
 * You are likely interested in a subset of these parameter changes, so in the callback it
 * is convenient to use a regular expression on the node names or namespaces of interest.
 * For example:
 *
 *   auto cb3 =
 *     [fqn, remote_param_name, &node](const rcl_interfaces::msg::ParameterEvent & event) {
 *       // Look for any updates to parameters in "/a_namespace" as well as any parameter changes
 *       // to our own node ("this_node")
 *       std::regex re("(/a_namespace/.*)|(/this_node)");
 *       if (regex_match(event.node, re)) {
 *         // Now that we know the event matches the regular expression we scanned for, we can
 *         // use 'get_parameter_from_event' to get a specific parameter name that we're looking for
 *         rclcpp::Parameter p;
 *         if (rclcpp::ParameterEventsSubscriber::get_parameter_from_event(
 *             event, p, remote_param_name, fqn))
 *         {
 *           RCLCPP_INFO(
 *             node->get_logger(),
 *             "cb3: Received an update to parameter \"%s\" of type: %s: \"%s\"",
 *             p.get_name().c_str(),
 *             p.get_type_name().c_str(),
 *             p.as_string().c_str());
 *         }
 *
 *         // You can also use 'get_parameter*s*_from_event' to enumerate all changes that came
 *         // in on this event
 *         auto params = rclcpp::ParameterEventsSubscriber::get_parameters_from_event(event);
 *         for (auto & p : params) {
 *           RCLCPP_INFO(
 *             node->get_logger(),
 *             "cb3: Received an update to parameter \"%s\" of type: %s: \"%s\"",
 *             p.get_name().c_str(),
 *             p.get_type_name().c_str(),
 *             p.value_to_string().c_str());
 *         }
 *       }
 *     };
 *   auto handle3 = param_handler->add_parameter_event_callback(cb3);
 *
 * 对于参数回调和参数事件回调，当添加多个回调时，按照后进先出（LIFO）顺序调用回调。
 *
 * For both parameter callbacks and parameter event callbacks, when multiple callbacks are added,
 * the callbacks are invoked last-in, first-called order (LIFO).
 *
 * 注意：必须捕获 add_parameter_event_callback 返回的回调句柄，否则回调将立即被取消注册。
 *
 * Note: the callback handle returned from add_parameter_event_callback must be captured or
 * the callback will immediately be unregistered.
 *
 * 若要删除参数事件回调，请重置回调智能指针或使用：
 *
 * To remove a parameter event callback, reset the callback smart pointer or use:
 *
 *   param_handler->remove_event_parameter_callback(handle3);
 */

class ParameterEventHandler {
public:
  /// 构造一个参数事件监视器。 (Construct a parameter events monitor.)
  /**
   * \param[in] node 用于创建所需订阅者的节点。 (The node to use to create any required
   * subscribers.) \param[in] qos 用于任何订阅的QoS设置。 (The QoS settings to use for any
   * subscriptions.)
   */
  template <typename NodeT>
  explicit ParameterEventHandler(
      NodeT node,
      const rclcpp::QoS &qos =
          rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events)))
      : node_base_(rclcpp::node_interfaces::get_node_base_interface(node)) {
    // 获取节点主题接口。 (Get the node topics interface.)
    auto node_topics = rclcpp::node_interfaces::get_node_topics_interface(node);

    // 创建回调共享指针。 (Create shared pointer for callbacks.)
    callbacks_ = std::make_shared<Callbacks>();

    // 创建参数事件订阅。 (Create the parameter event subscription.)
    event_subscription_ = rclcpp::create_subscription<rcl_interfaces::msg::ParameterEvent>(
        node_topics, "/parameter_events", qos,
        [callbacks = callbacks_](const rcl_interfaces::msg::ParameterEvent &event) {
          // 调用事件回调。 (Invoke the event callback.)
          callbacks->event_callback(event);
        });
  }

  // 定义参数事件回调类型。 (Define the parameter event callback type.)
  using ParameterEventCallbackType = ParameterEventCallbackHandle::ParameterEventCallbackType;

  /// 设置所有参数事件的回调。 (Set a callback for all parameter events.)
  /**
   * 可以多次调用此函数以设置多个参数事件回调。 (This function may be called multiple times to set
   * multiple parameter event callbacks.)
   *
   * 注意：如果未捕获返回的回调句柄智能指针，则立即取消注册回调。编译器应生成警告以发出此警告。
   * (Note: if the returned callback handle smart pointer is not captured, the callback is
   * immediatedly unregistered. A compiler warning should be generated to warn of this.)
   *
   * \param[in] callback 参数更新时要调用的函数回调。 (Function callback to be invoked on parameter
   * updates.) \returns 用于引用回调的句柄。 (A handle used to refer to the callback.)
   */
  RCLCPP_PUBLIC
  RCUTILS_WARN_UNUSED
  ParameterEventCallbackHandle::SharedPtr add_parameter_event_callback(
      ParameterEventCallbackType callback);

  /// 删除使用 add_parameter_event_callback 注册的参数事件回调。 (Remove parameter event callback
  /// registered with add_parameter_event_callback.)
  /**
   * \param[in] callback_handle 要删除的回调的句柄。 (Handle of the callback to remove.)
   */
  RCLCPP_PUBLIC
  void remove_parameter_event_callback(ParameterEventCallbackHandle::SharedPtr callback_handle);

  // 定义参数回调类型 (Define Parameter Callback Type)
  using ParameterCallbackType = ParameterCallbackHandle::ParameterCallbackType;

  /// 为指定参数添加回调。 (Add a callback for a specified parameter.)
  /**
   * 如果未提供 node_name，则默认为当前节点。 (If a node_name is not provided, defaults to the
   * current node.)
   *
   * 注意：如果未捕获返回的回调句柄智能指针，则立即取消注册回调。编译器应生成警告以警告此问题。
   * (Note: if the returned callback handle smart pointer is not captured, the callback is
   * immediately unregistered. A compiler warning should be generated to warn of this.)
   *
   * \param[in] parameter_name 要监视的参数名称。 (Name of parameter to monitor.)
   * \param[in] callback 参数更新时要调用的函数回调。 (Function callback to be invoked upon
   * parameter update.) \param[in] node_name 托管参数的节点名称。 (Name of node which hosts the
   * parameter.) \returns 用于引用回调的句柄。 (A handle used to refer to the callback.)
   */
  RCLCPP_PUBLIC
  RCUTILS_WARN_UNUSED
  ParameterCallbackHandle::SharedPtr add_parameter_callback(
      const std::string &parameter_name,
      ParameterCallbackType callback,
      const std::string &node_name = "");

  /// 删除使用 add_parameter_callback 注册的参数回调。 (Remove a parameter callback registered with
  /// add_parameter_callback.)
  /**
   * 从回调句柄检查参数名称和节点名称。在映射中的 {parameter_name, node_name}
   * 的回调句柄列表中删除回调句柄。如果句柄不存在或已被删除，将抛出错误。 (The parameter name and
   * node name are inspected from the callback handle. The callback handle is erased from the list
   * of callback handles on the {parameter_name, node_name} in the map. An error is thrown if the
   * handle does not exist and/or was already removed.)
   *
   * \param[in] callback_handle 要删除的回调的句柄。 (Handle of the callback to remove.)
   */
  RCLCPP_PUBLIC
  void remove_parameter_callback(ParameterCallbackHandle::SharedPtr callback_handle);

  /// 从参数事件中获取 rclcpp::Parameter。 (Get an rclcpp::Parameter from a parameter event.)
  /**
   * 如果未提供 node_name，则默认为当前节点。 (If a node_name is not provided, defaults to the
   * current node.)
   *
   * \param[in] event 要检查的事件消息。 (Event msg to be inspected.)
   * \param[out] parameter 要分配的 rclcpp::Parameter 引用。 (Reference to rclcpp::Parameter to be
   * assigned.) \param[in] parameter_name 参数名称。 (Name of parameter.) \param[in] node_name
   * 托管参数的节点名称。 (Name of node which hosts the parameter.) \returns
   * 如果请求的参数名称和节点位于事件中，则设置输出参数并返回 true。否则，返回 false。 (Output
   * parameter is set with requested parameter info and returns true if requested parameter name and
   * node is in event. Otherwise, returns false.)
   */
  RCLCPP_PUBLIC
  static bool get_parameter_from_event(
      const rcl_interfaces::msg::ParameterEvent &event,
      rclcpp::Parameter &parameter,
      const std::string &parameter_name,
      const std::string &node_name = "");

  /// 从参数事件中获取一个 rclcpp::Parameter (Get an rclcpp::Parameter from parameter event)
  /**
   * 如果未提供 node_name，则默认为当前节点。(If a node_name is not provided, defaults to the
   * current node.)
   *
   * 用户有责任检查返回的参数是否已正确分配。(The user is responsible for checking if the returned
   * parameter has been properly assigned.)
   * 默认情况下，如果请求的参数在事件中未找到，则返回的参数具有 rclcpp::PARAMETER_NOT_SET
   * 类型的参数值。(By default, if the requested parameter is not found in the event, the returned
   * parameter has a parameter value of type rclcpp::PARAMETER_NOT_SET.)
   *
   * \param[in] event 要检查的事件消息。(Event msg to be inspected.)
   * \param[in] parameter_name 参数名称。(Name of parameter.)
   * \param[in] node_name 托管参数的节点名称。(Name of the node which hosts the parameter.)
   * \returns 事件中产生的 rclcpp::Parameter。(The resultant rclcpp::Parameter from the event.)
   * \throws std::runtime_error 如果输入的节点名称与参数事件中的节点名称不匹配。(if the input node
   * name doesn't match the node name in the parameter event.)
   */
  RCLCPP_PUBLIC
  static rclcpp::Parameter get_parameter_from_event(
      const rcl_interfaces::msg::ParameterEvent &event,
      const std::string &parameter_name,
      const std::string &node_name = "");

  /// 从参数事件中获取所有 rclcpp::Parameter 值 (Get all rclcpp::Parameter values from a parameter
  /// event)
  /**
   * \param[in] event 要检查的事件消息。(Event msg to be inspected.)
   * \returns 事件中的一组 rclcpp::Parameter 值。(A vector of rclcpp::Parameter values from the
   * event.)
   */
  RCLCPP_PUBLIC
  static std::vector<rclcpp::Parameter> get_parameters_from_event(
      const rcl_interfaces::msg::ParameterEvent &event);

  // 使用 CallbacksContainerType 类型定义回调容器 (Using CallbacksContainerType to define a
  // container for callbacks)
  using CallbacksContainerType = std::list<ParameterCallbackHandle::WeakPtr>;

protected:
  // *INDENT-OFF* Uncrustify 不处理缩进的公共/私有标签
  // *INDENT-OFF* Uncrustify doesn't handle indented public/private labels

  // 为 std::unordered_map 中所需的字符串对哈希函数
  // Hash function for string pair required in std::unordered_map
  // 参考：https://stackoverflow.com/questions/35985960/c-why-is-boosthash-combine-the-best-way-to-combine-hash-values
  // See:
  // https://stackoverflow.com/questions/35985960/c-why-is-boosthash-combine-the-best-way-to-combine-hash-values
  class StringPairHash {
  public:
    // 内联模板函数，用于组合哈希值
    // Inline template function for combining hash values
    template <typename T>
    inline void hash_combine(std::size_t &seed, const T &v) const {
      std::hash<T> hasher;
      seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    // 重载 () 运算符，用于计算字符串对的哈希值
    // Overload the () operator to calculate the hash value of a string pair
    inline size_t operator()(const std::pair<std::string, std::string> &s) const {
      size_t seed = 0;
      hash_combine(seed, s.first);
      hash_combine(seed, s.second);
      return seed;
    }
  };
  // *INDENT-ON*
  // *INDENT-ON*

  struct Callbacks {
    // 递归互斥量，用于保护参数回调容器
    // Recursive mutex for protecting the parameter callbacks container
    std::recursive_mutex mutex_;

    // 注册参数的映射容器
    // Map container for registered parameters
    std::unordered_map<std::pair<std::string, std::string>, CallbacksContainerType, StringPairHash>
        parameter_callbacks_;

    // 参数事件回调的弱指针列表
    // List of weak pointers to parameter event callbacks
    std::list<ParameterEventCallbackHandle::WeakPtr> event_callbacks_;

    /// Callback for parameter events subscriptions.
    RCLCPP_PUBLIC
    void event_callback(const rcl_interfaces::msg::ParameterEvent &event);
  };

  // 定义一个共享指针类型的成员变量，用于存储回调函数对象
  // Define a shared pointer type member variable for storing callback function objects
  std::shared_ptr<Callbacks> callbacks_;

  // 用于解析节点路径的实用函数
  // Utility function for resolving node path
  std::string resolve_path(const std::string &path);

  // 用于基本功能的节点接口
  // Node interface used for base functionality
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_;

  // 定义一个共享指针类型的订阅者，用于订阅参数事件消息
  // Define a shared pointer type subscriber for subscribing to parameter event messages
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr event_subscription_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_EVENT_HANDLER_HPP_

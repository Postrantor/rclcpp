// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/node_interfaces/node_parameters.hpp"

#include <rcl_yaml_param_parser/parser.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "../detail/resolve_parameter_overrides.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/qos_profiles.h"

using rclcpp::node_interfaces::NodeParameters;

/**
 * @brief 从参数重写中自动声明参数的本地性能。
 * @param parameter_overrides 参数重写映射
 * @param has_parameter 用于检查参数是否存在的函数
 * @param declare_parameter 用于声明参数的函数
 *
 * @details 该函数使用给定的参数重写映射，如果参数未被声明，则将其自动声明。
 *
 * @brief Local perform for automatically declaring parameters from overrides.
 * @param parameter_overrides Map of parameter overrides
 * @param has_parameter Function to check if a parameter exists
 * @param declare_parameter Function to declare a parameter
 *
 * @details This function takes the provided map of parameter overrides and automatically declares
 * them if they haven't been declared yet.
 */
RCLCPP_LOCAL
void local_perform_automatically_declare_parameters_from_overrides(
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
    std::function<bool(const std::string &)> has_parameter,
    std::function<void(
        const std::string &,
        const rclcpp::ParameterValue &,
        const rcl_interfaces::msg::ParameterDescriptor &,
        bool)> declare_parameter) {
  // 创建参数描述符对象，用于描述参数的属性
  // Create a ParameterDescriptor object for describing the properties of the parameter
  rcl_interfaces::msg::ParameterDescriptor descriptor;

  // 设置参数描述符的动态类型属性为 true，表示参数支持动态类型
  // Set the dynamic_typing attribute of the parameter descriptor to true, indicating that the
  // parameter supports dynamic types
  descriptor.dynamic_typing = true;

  // 遍历参数重写映射中的每个键值对
  // Iterate through each key-value pair in the parameter overrides map
  for (const auto &pair : parameter_overrides) {
    // 检查参数是否已经存在，如果不存在，则声明参数
    // Check if the parameter already exists, if not, declare the parameter
    if (!has_parameter(pair.first)) {
      // 使用给定的参数名、参数值和参数描述符声明参数
      // Declare the parameter using the provided parameter name, parameter value, and parameter
      // descriptor
      declare_parameter(pair.first, pair.second, descriptor, true);
    }
  }
}

/**
 * @brief 构造函数，初始化节点参数对象（Constructor, initializing the node parameters object）
 *
 * @param node_base 节点基本接口的共享指针（Shared pointer of the node base interface）
 * @param node_logging 节点日志接口的共享指针（Shared pointer of the node logging interface）
 * @param node_topics 节点主题接口的共享指针（Shared pointer of the node topics interface）
 * @param node_services 节点服务接口的共享指针（Shared pointer of the node services interface）
 * @param node_clock 节点时钟接口的共享指针（Shared pointer of the node clock interface）
 * @param parameter_overrides 参数重载列表（List of parameter overrides）
 * @param start_parameter_services 是否启动参数服务（Whether to start parameter services）
 * @param start_parameter_event_publisher 是否启动参数事件发布器（Whether to start parameter event
 * publisher）
 * @param parameter_event_qos 参数事件QoS配置（Parameter event QoS configuration）
 * @param parameter_event_publisher_options 参数事件发布器选项（Parameter event publisher options）
 * @param allow_undeclared_parameters 是否允许未声明的参数（Whether to allow undeclared parameters）
 * @param automatically_declare_parameters_from_overrides 是否自动从重载中声明参数（Whether to
 * automatically declare parameters from overrides）
 */
NodeParameters::NodeParameters(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    const std::vector<rclcpp::Parameter> &parameter_overrides,
    bool start_parameter_services,
    bool start_parameter_event_publisher,
    const rclcpp::QoS &parameter_event_qos,
    const rclcpp::PublisherOptionsBase &parameter_event_publisher_options,
    bool allow_undeclared_parameters,
    bool automatically_declare_parameters_from_overrides)
    : allow_undeclared_(allow_undeclared_parameters),
      events_publisher_(nullptr),
      node_logging_(node_logging),
      node_clock_(node_clock) {
  // 使用rcl_interfaces::msg::ParameterEvent消息类型和分配器创建发布器选项
  // (Create publisher options with rcl_interfaces::msg::ParameterEvent message type and allocator)
  using MessageT = rcl_interfaces::msg::ParameterEvent;
  using PublisherT = rclcpp::Publisher<MessageT>;
  using AllocatorT = std::allocator<void>;
  // TODO(wjwwood): expose this allocator through the Parameter interface.
  rclcpp::PublisherOptionsWithAllocator<AllocatorT> publisher_options(
      parameter_event_publisher_options);
  publisher_options.allocator = std::make_shared<AllocatorT>();

  // 如果需要，启动参数服务（Start parameter services if needed）
  if (start_parameter_services) {
    parameter_service_ = std::make_shared<ParameterService>(node_base, node_services, this);
  }

  // 如果需要，启动参数事件发布器（Start parameter event publisher if needed）
  if (start_parameter_event_publisher) {
    // TODO(ivanpauno): Qos of the `/parameters_event` topic should be somehow overridable.
    events_publisher_ = rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(
        node_topics, "/parameter_events", parameter_event_qos, publisher_options);
  }

  // 获取节点选项（Get the node options）
  const rcl_node_t *node = node_base->get_rcl_node_handle();
  if (nullptr == node) {
    throw std::runtime_error("Need valid node handle in NodeParameters");
  }
  const rcl_node_options_t *options = rcl_node_get_options(node);
  if (nullptr == options) {
    throw std::runtime_error("Need valid node options in NodeParameters");
  }

  // 如果使用全局参数，获取全局参数（Get global arguments if using them）
  const rcl_arguments_t *global_args = nullptr;
  if (options->use_global_arguments) {
    auto context_ptr = node_base->get_context()->get_rcl_context();
    global_args = &(context_ptr->global_arguments);
  }
  combined_name_ = node_base->get_fully_qualified_name();

  // 解析参数重载（Resolve parameter overrides）
  parameter_overrides_ = rclcpp::detail::resolve_parameter_overrides(
      combined_name_, parameter_overrides, &options->arguments, global_args);

  // 如果需要，从重载中自动声明未明确声明的参数
  // (Automatically declare parameters from overrides if needed and not explicitly declared)
  if (automatically_declare_parameters_from_overrides) {
    using namespace std::placeholders;
    local_perform_automatically_declare_parameters_from_overrides(
        this->get_parameter_overrides(), std::bind(&NodeParameters::has_parameter, this, _1),
        [this](
            const std::string &name, const rclcpp::ParameterValue &default_value,
            const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
            bool ignore_override) {
          NodeParameters::declare_parameter(
              name, default_value, parameter_descriptor, ignore_override);
        });
  }
}

/*!
 * \brief 自动从覆盖参数中声明参数 (Automatically declare parameters from overrides)
 */
void NodeParameters::perform_automatically_declare_parameters_from_overrides() {
  // 本地函数执行自动声明参数操作，传入参数覆盖值、检查参数是否存在的回调和声明参数的回调
  // (Call the local function to perform automatic declaration of parameters, passing in parameter
  // overrides, callback for checking if a parameter exists, and callback for declaring the
  // parameter)
  local_perform_automatically_declare_parameters_from_overrides(
      this->get_parameter_overrides(),
      [this](const std::string &name) {
        return this->has_parameter(name);
      },  // 检查参数是否存在的回调 (Callback for checking if a parameter exists)
      [this](
          const std::string &name, const rclcpp::ParameterValue &default_value,
          const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
          bool ignore_override) {
        // 声明参数的回调 (Callback for declaring the parameter)
        this->declare_parameter(name, default_value, parameter_descriptor, ignore_override);
      });
}

/*!
 * \brief 节点参数析构函数 (NodeParameters destructor)
 */
NodeParameters::~NodeParameters() {}

/*!
 * \brief 检查参数是否存在，不加锁 (Check if a parameter exists without locking)
 * \param[in] parameters 参数列表 (The list of parameters)
 * \param[in] name 参数名称 (The parameter name)
 * \return 是否存在该参数 (Whether the parameter exists)
 */
RCLCPP_LOCAL
bool __lockless_has_parameter(
    const std::map<std::string, rclcpp::node_interfaces::ParameterInfo> &parameters,
    const std::string &name) {
  // 查找参数并判断是否存在 (Find the parameter and check if it exists)
  return parameters.find(name) != parameters.end();
}

/*!
 * \brief 检查两个双精度浮点数是否相等，允许一定误差 (Check if two double-precision floating-point
 * numbers are equal, allowing for some error) \param[in] x 第一个浮点数 (The first floating-point
 * number) \param[in] y 第二个浮点数 (The second floating-point number) \param[in] ulp
 * 允许的误差倍数 (Allowed error multiple) \return 是否相等 (Whether they are equal)
 */
// 参考：https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
// (Reference: https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon)
RCLCPP_LOCAL
bool __are_doubles_equal(double x, double y, double ulp = 100.0) {
  // 根据给定的误差倍数比较两个浮点数是否相等 (Compare the two floating-point numbers for equality
  // based on the given error multiple)
  return std::abs(x - y) <= std::numeric_limits<double>::epsilon() * std::abs(x + y) * ulp;
}

/*!
 * \brief 格式化范围原因字符串 (Format range reason string)
 * \param[in] name 参数名称 (The parameter name)
 * \param[in] range_type 范围类型 (The range type)
 * \return 格式化后的范围原因字符串 (Formatted range reason string)
 */
static std::string format_range_reason(const std::string &name, const char *range_type) {
  // 使用字符串流拼接范围原因字符串 (Concatenate the range reason string using a string stream)
  std::ostringstream ss;
  ss << "Parameter {" << name << "} doesn't comply with " << range_type << " range.";
  return ss.str();
}

/**
 * @brief 检查参数值是否在给定的范围内 (Check if the parameter value is within the given range)
 *
 * @param descriptor 参数描述符 (Parameter Descriptor)
 * @param value 参数值 (Parameter Value)
 * @return rcl_interfaces::msg::SetParametersResult 设置参数结果 (Set Parameters Result)
 */
RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult __check_parameter_value_in_range(
    const rcl_interfaces::msg::ParameterDescriptor &descriptor,
    const rclcpp::ParameterValue &value) {
  // 初始化设置参数结果对象 (Initialize the SetParametersResult object)
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 判断参数类型为整数且整数范围非空 (Check if the parameter type is integer and the integer range
  // is not empty)
  if (!descriptor.integer_range.empty() && value.get_type() == rclcpp::PARAMETER_INTEGER) {
    int64_t v = value.get<int64_t>();  // 获取整数参数值 (Get the integer parameter value)
    auto integer_range = descriptor.integer_range.at(0);  // 获取整数范围 (Get the integer range)

    // 如果参数值等于范围边界，则返回成功结果 (If the parameter value is equal to the range
    // boundary, return a successful result)
    if (v == integer_range.from_value || v == integer_range.to_value) {
      return result;
    }

    // 如果参数值不在范围内，设置结果为失败并给出原因 (If the parameter value is out of range, set
    // the result to failure and provide a reason)
    if ((v < integer_range.from_value) || (v > integer_range.to_value)) {
      result.successful = false;
      result.reason = format_range_reason(descriptor.name, "integer");
      return result;
    }

    // 如果步长为0，返回成功结果 (If the step is 0, return a successful result)
    if (integer_range.step == 0) {
      return result;
    }

    // 如果参数值满足步长条件，返回成功结果 (If the parameter value meets the step condition, return
    // a successful result)
    if (((v - integer_range.from_value) % integer_range.step) == 0) {
      return result;
    }

    // 否则设置结果为失败并给出原因 (Otherwise, set the result to failure and provide a reason)
    result.successful = false;
    result.reason = format_range_reason(descriptor.name, "integer");
    return result;
  }

  // 判断参数类型为浮点数且浮点数范围非空 (Check if the parameter type is floating point and the
  // floating point range is not empty)
  if (!descriptor.floating_point_range.empty() && value.get_type() == rclcpp::PARAMETER_DOUBLE) {
    double v = value.get<double>();  // 获取浮点数参数值 (Get the floating point parameter value)
    auto fp_range =
        descriptor.floating_point_range.at(0);  // 获取浮点数范围 (Get the floating point range)

    // 如果参数值等于范围边界，则返回成功结果 (If the parameter value is equal to the range
    // boundary, return a successful result)
    if (__are_doubles_equal(v, fp_range.from_value) || __are_doubles_equal(v, fp_range.to_value)) {
      return result;
    }

    // 如果参数值不在范围内，设置结果为失败并给出原因 (If the parameter value is out of range, set
    // the result to failure and provide a reason)
    if ((v < fp_range.from_value) || (v > fp_range.to_value)) {
      result.successful = false;
      result.reason = format_range_reason(descriptor.name, "floating point");
      return result;
    }

    // 如果步长为0，返回成功结果 (If the step is 0, return a successful result)
    if (fp_range.step == 0.0) {
      return result;
    }

    // 计算参数值与范围起始值的差值除以步长的四舍五入值 (Calculate the rounded value of the
    // difference between the parameter value and the range start value divided by the step)
    double rounded_div = std::round((v - fp_range.from_value) / fp_range.step);

    // 如果参数值满足步长条件，返回成功结果 (If the parameter value meets the step condition, return
    // a successful result)
    if (__are_doubles_equal(v, fp_range.from_value + rounded_div * fp_range.step)) {
      return result;
    }

    // 否则设置结果为失败并给出原因 (Otherwise, set the result to failure and provide a reason)
    result.successful = false;
    result.reason = format_range_reason(descriptor.name, "floating point");
    return result;
  }

  // 如果不满足整数和浮点数范围条件，直接返回成功结果 (If the integer and floating point range
  // conditions are not met, return a successful result directly)
  return result;
}

/**
 * @brief 格式化类型原因的错误信息（Format the error message for wrong parameter type）
 *
 * @param[in] name 参数名称（Parameter name）
 * @param[in] old_type 旧的参数类型（Old parameter type）
 * @param[in] new_type 新的参数类型（New parameter type）
 * @return std::string 格式化后的错误信息（Formatted error message）
 */
static std::string format_type_reason(
    const std::string &name, const std::string &old_type, const std::string &new_type) {
  std::ostringstream ss;
  // 警告：后续条件依赖于此消息以 "Wrong parameter type" 开头，
  // 如果修改此处，请检查 `declare_parameter` 函数！
  // WARN: A condition later depends on this message starting with "Wrong parameter type",
  // check `declare_parameter` if you modify this!
  ss << "Wrong parameter type, parameter {" << name << "} is of type {" << old_type
     << "}, setting it to {" << new_type << "} is not allowed.";
  return ss.str();
}

/**
 * @brief 检查参数设置是否有效（Check if the parameter settings are valid）
 *
 * @param[in] parameter_infos 参数信息映射（Map of parameter information）
 * @param[in] parameters 要检查的参数集合（Collection of parameters to check）
 * @param[in] allow_undeclared 是否允许未声明的参数（Whether undeclared parameters are allowed）
 * @return rcl_interfaces::msg::SetParametersResult 设置参数结果（Set parameters result）
 */
RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult __check_parameters(
    std::map<std::string, rclcpp::node_interfaces::ParameterInfo> &parameter_infos,
    const std::vector<rclcpp::Parameter> &parameters,
    bool allow_undeclared) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 遍历参数向量（Iterate through the parameter vector）
  for (const rclcpp::Parameter &parameter : parameters) {
    std::string name = parameter.get_name();
    rcl_interfaces::msg::ParameterDescriptor descriptor;

    // 检查是否允许未声明的参数（Check if undeclared parameters are allowed）
    if (allow_undeclared) {
      auto it = parameter_infos.find(name);
      if (it != parameter_infos.cend()) {
        descriptor = it->second.descriptor;
      } else {
        // 隐式声明的参数是动态类型的！
        // implicitly declared parameters are dynamically typed!
        descriptor.dynamic_typing = true;
      }
    } else {
      descriptor = parameter_infos[name].descriptor;
    }

    // 如果描述符名称为空，则使用参数名称（If the descriptor name is empty, use the parameter name）
    if (descriptor.name.empty()) {
      descriptor.name = name;
    }

    // 获取新类型和指定类型（Get the new type and specified type）
    const auto new_type = parameter.get_type();
    const auto specified_type = static_cast<rclcpp::ParameterType>(descriptor.type);

    // 检查动态类型或指定类型是否与新类型相等（Check if dynamic typing or specified type is equal to
    // the new type）
    result.successful = descriptor.dynamic_typing || specified_type == new_type;

    // 如果结果不成功，设置原因并返回结果（If the result is not successful, set the reason and
    // return the result）
    if (!result.successful) {
      result.reason =
          format_type_reason(name, rclcpp::to_string(specified_type), rclcpp::to_string(new_type));
      return result;
    }

    // 检查参数值是否在范围内（Check if the parameter value is in range）
    result = __check_parameter_value_in_range(descriptor, parameter.get_parameter_value());

    // 如果结果不成功，返回结果（If the result is not successful, return the result）
    if (!result.successful) {
      return result;
    }
  }

  // 返回成功的结果（Return a successful result）
  return result;
}

using PreSetParametersCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::PreSetParametersCallbackType;
using PreSetParametersCallbackHandle = rclcpp::node_interfaces::PreSetParametersCallbackHandle;
using PreSetCallbacksHandleContainer =
    rclcpp::node_interfaces::NodeParameters::PreSetCallbacksHandleContainer;

using OnSetParametersCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType;
using OnSetParametersCallbackHandle = rclcpp::node_interfaces::OnSetParametersCallbackHandle;
using OnSetCallbacksHandleContainer =
    rclcpp::node_interfaces::NodeParameters::OnSetCallbacksHandleContainer;

using PostSetParametersCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::PostSetParametersCallbackType;
using PostSetParametersCallbackHandle = rclcpp::node_interfaces::PostSetParametersCallbackHandle;
using PostSetCallbacksHandleContainer =
    rclcpp::node_interfaces::NodeParameters::PostSetCallbacksHandleContainer;

/**
 * @brief 对给定的参数列表调用预设参数回调。(Call pre-set parameter callbacks for the given list of
 * parameters.)
 *
 * @param[in,out] parameters 参数列表，将被传递给回调函数。(List of parameters that will be passed
 * to the callbacks.)
 * @param[in,out] callback_container 预设参数回调的容器。(Container for pre-set parameter
 * callbacks.)
 */
RCLCPP_LOCAL
void __call_pre_set_parameters_callbacks(
    std::vector<rclcpp::Parameter> &parameters,
    PreSetCallbacksHandleContainer &callback_container) {
  // 如果回调容器为空，则直接返回。(If the callback container is empty, return directly.)
  if (callback_container.empty()) {
    return;
  }

  // 迭代回调容器。(Iterate through the callback container.)
  auto it = callback_container.begin();
  while (it != callback_container.end()) {
    // 尝试获取共享指针。(Try to get a shared pointer.)
    auto shared_handle = it->lock();

    // 如果共享指针非空，则调用回调函数。(If the shared pointer is not null, call the callback
    // function.)
    if (nullptr != shared_handle) {
      shared_handle->callback(parameters);
      it++;
    } else {
      // 否则删除失效的回调句柄。(Otherwise, remove the invalid callback handle.)
      it = callback_container.erase(it);
    }
  }
}

/**
 * @brief 调用设置参数回调并返回结果。(Call on-set parameter callbacks and return the result.)
 *
 * @param[in] parameters 参数列表，将被传递给回调函数。(List of parameters that will be passed to
 * the callbacks.)
 * @param[in,out] callback_container 设置参数回调的容器。(Container for on-set parameter callbacks.)
 * @return 返回设置参数结果。(Return the set parameters result.)
 */
RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult __call_on_set_parameters_callbacks(
    const std::vector<rclcpp::Parameter> &parameters,
    OnSetCallbacksHandleContainer &callback_container) {
  // 初始化结果。(Initialize the result.)
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 迭代回调容器。(Iterate through the callback container.)
  auto it = callback_container.begin();
  while (it != callback_container.end()) {
    // 尝试获取共享指针。(Try to get a shared pointer.)
    auto shared_handle = it->lock();

    // 如果共享指针非空，则调用回调函数并更新结果。(If the shared pointer is not null, call the
    // callback function and update the result.)
    if (nullptr != shared_handle) {
      result = shared_handle->callback(parameters);
      if (!result.successful) {
        return result;
      }
      it++;
    } else {
      // 否则删除失效的回调句柄。(Otherwise, remove the invalid callback handle.)
      it = callback_container.erase(it);
    }
  }

  // 返回结果。(Return the result.)
  return result;
}

RCLCPP_LOCAL
/**
 * @brief 调用设置参数后的回调函数
 * @param parameters 参数列表
 * @param callback_container 回调函数容器
 *
 * @note Call post-set parameter callbacks.
 * @param[in] parameters A vector of parameters.
 * @param[in,out] callback_container A container with the callbacks to be called.
 */
void __call_post_set_parameters_callbacks(
    const std::vector<rclcpp::Parameter> &parameters,
    PostSetCallbacksHandleContainer &callback_container) {
  // 如果回调容器为空，不执行任何操作
  // If the callback container is empty, do nothing.
  if (callback_container.empty()) {
    return;
  }

  auto it = callback_container.begin();
  while (it != callback_container.end()) {
    auto shared_handle = it->lock();
    // 如果共享句柄不为空，则调用回调函数
    // If the shared handle is not null, call the callback.
    if (nullptr != shared_handle) {
      shared_handle->callback(parameters);
      it++;
    } else {
      // 否则，从容器中删除该句柄
      // Otherwise, erase the handle from the container.
      it = callback_container.erase(it);
    }
  }
}

RCLCPP_LOCAL
/**
 * @brief 原子地设置参数并调用回调函数
 * @param parameters 参数列表
 * @param parameter_infos 参数信息映射表
 * @param on_set_callback_container 设置参数时的回调函数容器
 * @param post_set_callback_container 设置参数后的回调函数容器
 * @param allow_undeclared 是否允许未声明的参数，默认为false
 * @return 返回设置参数结果
 *
 * @note Set parameters atomically and call the callbacks.
 * @param[in] parameters A vector of parameters to be set.
 * @param[in,out] parameter_infos A map containing the parameter information.
 * @param[in,out] on_set_callback_container A container with the on-set parameter callbacks.
 * @param[in,out] post_set_callback_container A container with the post-set parameter callbacks.
 * @param[in] allow_undeclared A flag indicating whether undeclared parameters are allowed (default
 * is false).
 * @return The result of setting the parameters.
 */
rcl_interfaces::msg::SetParametersResult __set_parameters_atomically_common(
    const std::vector<rclcpp::Parameter> &parameters,
    std::map<std::string, rclcpp::node_interfaces::ParameterInfo> &parameter_infos,
    OnSetCallbacksHandleContainer &on_set_callback_container,
    PostSetCallbacksHandleContainer &post_set_callback_container,
    bool allow_undeclared = false) {
  // 检查设置的值是否符合描述符
  // Check if the value being set complies with the descriptor.
  rcl_interfaces::msg::SetParametersResult result =
      __check_parameters(parameter_infos, parameters, allow_undeclared);
  if (!result.successful) {
    return result;
  }
  // 调用用户回调，检查新值是否允许设置
  // Call the user callbacks to see if the new value(s) are allowed.
  result = __call_on_set_parameters_callbacks(parameters, on_set_callback_container);
  if (!result.successful) {
    return result;
  }
  // 如果接受新值，则实际设置参数值
  // If accepted, actually set the values.
  if (result.successful) {
    for (size_t i = 0; i < parameters.size(); ++i) {
      const std::string &name = parameters[i].get_name();
      parameter_infos[name].descriptor.name = parameters[i].get_name();
      parameter_infos[name].descriptor.type = parameters[i].get_type();
      parameter_infos[name].value = parameters[i].get_parameter_value();
    }
    // 调用用户设置参数后的回调函数
    // Call the user post-set parameter callback.
    __call_post_set_parameters_callbacks(parameters, post_set_callback_container);
  }

  // 返回设置参数结果
  // Return the result of setting the parameters.
  return result;
}

/**
 * @brief 声明参数的通用函数，处理默认值、覆盖和回调。 (Common function for declaring parameters,
 * handling default values, overrides, and callbacks.)
 *
 * @param name 参数名称 (Parameter name)
 * @param default_value 默认值 (Default value)
 * @param parameter_descriptor 参数描述符 (Parameter descriptor)
 * @param parameters_out 输出参数信息 (Output parameter information)
 * @param overrides 覆盖参数值的映射 (Mapping of overridden parameter values)
 * @param on_set_callback_container 设置回调容器 (On set callback container)
 * @param post_set_callback_container 设置后回调容器 (Post set callback container)
 * @param parameter_event_out 输出参数事件 (Output parameter event)
 * @param ignore_override 是否忽略覆盖值 (Whether to ignore the override value)
 * @return rcl_interfaces::msg::SetParametersResult 设置参数结果 (Set parameters result)
 */
RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult __declare_parameter_common(
    const std::string &name,
    const rclcpp::ParameterValue &default_value,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
    std::map<std::string, rclcpp::node_interfaces::ParameterInfo> &parameters_out,
    const std::map<std::string, rclcpp::ParameterValue> &overrides,
    OnSetCallbacksHandleContainer &on_set_callback_container,
    PostSetCallbacksHandleContainer &post_set_callback_container,
    rcl_interfaces::msg::ParameterEvent *parameter_event_out,
    bool ignore_override = false) {
  using rclcpp::node_interfaces::ParameterInfo;
  // 初始化一个名为name的参数信息映射 (Initialize a parameter info mapping named "name")
  std::map<std::string, ParameterInfo> parameter_infos{{name, ParameterInfo()}};
  // 设置参数描述符 (Set the parameter descriptor)
  parameter_infos.at(name).descriptor = parameter_descriptor;

  // 如果覆盖值可用，则使用覆盖值，否则使用默认值。 (Use the value from the overrides if available,
  // otherwise use the default.)
  const rclcpp::ParameterValue *initial_value = &default_value;
  auto overrides_it = overrides.find(name);
  if (!ignore_override && overrides_it != overrides.end()) {
    initial_value = &overrides_it->second;
  }

  // 如果没有初始值，则跳过初始化 (If there is no initial value, then skip initialization)
  if (initial_value->get_type() == rclcpp::PARAMETER_NOT_SET) {
    // 将声明的参数添加到存储中（无值）(Add declared parameters to storage (without a value))
    parameter_infos[name].descriptor.name = name;
    if (parameter_descriptor.dynamic_typing) {
      parameter_infos[name].descriptor.type = rclcpp::PARAMETER_NOT_SET;
    } else {
      parameter_infos[name].descriptor.type = parameter_descriptor.type;
    }
    parameters_out[name] = parameter_infos.at(name);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // 使用用户的回调检查初始值是否可以设置。 (Check with the user's callbacks to see if the initial
  // value can be set.)
  std::vector<rclcpp::Parameter> parameter_wrappers{rclcpp::Parameter(name, *initial_value)};
  // 此函数还处理默认值与初始值。 (This function also takes care of default vs initial value.)
  auto result = __set_parameters_atomically_common(
      parameter_wrappers, parameter_infos, on_set_callback_container, post_set_callback_container);

  if (!result.successful) {
    return result;
  }

  // 将声明的参数添加到存储中。 (Add declared parameters to storage.)
  parameters_out[name] = parameter_infos.at(name);

  // 如果有效，则扩展给定的参数事件。 (Extend the given parameter event, if valid.)
  if (parameter_event_out) {
    parameter_event_out->new_parameters.push_back(parameter_wrappers[0].to_parameter_msg());
  }

  return result;
}

/**
 * @brief 声明参数的辅助函数 (Helper function for declaring a parameter)
 *
 * @param[in] name 参数名称 (Parameter name)
 * @param[in] type 参数类型 (Parameter type)
 * @param[in] default_value 默认值 (Default value)
 * @param[in] parameter_descriptor 参数描述符 (Parameter descriptor)
 * @param[in] ignore_override 是否忽略覆盖 (Whether to ignore overrides)
 * @param[in,out] parameters 参数映射 (Map of parameters)
 * @param[in] overrides 覆盖映射 (Map of overrides)
 * @param[in,out] on_set_callback_container OnSet回调容器 (OnSet callback container)
 * @param[in,out] post_set_callback_container PostSet回调容器 (PostSet callback container)
 * @param[in] events_publisher 参数事件发布器 (Parameter event publisher)
 * @param[in] combined_name 组合名称 (Combined name)
 * @param[in] node_clock 节点时钟接口 (Node clock interface)
 * @return 返回声明参数的结果 (Returns the result of declaring the parameter)
 */
static const rclcpp::ParameterValue &declare_parameter_helper(
    const std::string &name,
    rclcpp::ParameterType type,
    const rclcpp::ParameterValue &default_value,
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor,
    bool ignore_override,
    std::map<std::string, rclcpp::node_interfaces::ParameterInfo> &parameters,
    const std::map<std::string, rclcpp::ParameterValue> &overrides,
    OnSetCallbacksHandleContainer &on_set_callback_container,
    PostSetCallbacksHandleContainer &post_set_callback_container,
    rclcpp::Publisher<rcl_interfaces::msg::ParameterEvent> *events_publisher,
    const std::string &combined_name,
    rclcpp::node_interfaces::NodeClockInterface &node_clock) {
  // TODO(sloretz) 参数名称验证 (Parameter name validation)
  if (name.empty()) {
    throw rclcpp::exceptions::InvalidParametersException("parameter name must not be empty");
  }

  // 如果此参数已经声明且不同，则报错 (Error if this parameter has already been declared and is
  // different)
  if (__lockless_has_parameter(parameters, name)) {
    throw rclcpp::exceptions::ParameterAlreadyDeclaredException(
        "parameter '" + name + "' has already been declared");
  }

  // 如果参数不是动态类型，设置参数类型 (If the parameter is not dynamically typed, set the
  // parameter type)
  if (!parameter_descriptor.dynamic_typing) {
    if (rclcpp::PARAMETER_NOT_SET == type) {
      type = default_value.get_type();
    }
    if (rclcpp::PARAMETER_NOT_SET == type) {
      throw rclcpp::exceptions::InvalidParameterTypeException{
          name, "cannot declare a statically typed parameter with an uninitialized value"};
    }
    parameter_descriptor.type = static_cast<uint8_t>(type);
  }

  rcl_interfaces::msg::ParameterEvent parameter_event;
  auto result = __declare_parameter_common(
      name, default_value, parameter_descriptor, parameters, overrides, on_set_callback_container,
      post_set_callback_container, &parameter_event, ignore_override);

  // 如果设置失败，则抛出异常 (If it failed to be set, then throw an exception)
  if (!result.successful) {
    constexpr const char type_error_msg_start[] = "Wrong parameter type";
    if (0u == std::strncmp(
                  result.reason.c_str(), type_error_msg_start, sizeof(type_error_msg_start) - 1)) {
      // TODO(ivanpauno):
      // 重构逻辑，以便我们不需要上面的`strncmp`，并且可以更优雅地在两个异常之间进行检测 (Refactor
      // the logic so we don't need the above `strncmp` and we can detect between both exceptions
      // more elegantly)
      throw rclcpp::exceptions::InvalidParameterTypeException(name, result.reason);
    }
    throw rclcpp::exceptions::InvalidParameterValueException(
        "parameter '" + name + "' could not be set: " + result.reason);
  }

  // 如果events_publisher_不是nullptr，则发布事件。这可能是在构造函数中禁用的 (Publish if
  // events_publisher_ is not nullptr, which may be if disabled in the constructor)
  if (nullptr != events_publisher) {
    parameter_event.node = combined_name;
    parameter_event.stamp = node_clock.get_clock()->now();
    events_publisher->publish(parameter_event);
  }

  return parameters.at(name).value;
}

/*!
 * \brief 声明参数（Declare a parameter）
 * \param[in] name 参数名称（Parameter name）
 * \param[in] default_value 默认值（Default value）
 * \param[in] parameter_descriptor 参数描述符（Parameter descriptor）
 * \param[in] ignore_override 是否忽略覆盖（Whether to ignore override）
 * \return 返回声明的参数值（Return the declared parameter value）
 */
const rclcpp::ParameterValue &NodeParameters::declare_parameter(
    const std::string &name,
    const rclcpp::ParameterValue &default_value,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
    bool ignore_override) {
  std::lock_guard<std::recursive_mutex> lock(
      mutex_);  // 锁定互斥量，防止多线程同时访问（Lock the mutex to prevent multiple threads from
                // accessing simultaneously）
  ParameterMutationRecursionGuard guard(
      parameter_modification_enabled_);  // 递归保护（Recursion protection）

  return declare_parameter_helper(  // 调用辅助函数进行参数声明（Call the helper function for
                                    // parameter declaration）
      name, rclcpp::PARAMETER_NOT_SET, default_value, parameter_descriptor, ignore_override,
      parameters_, parameter_overrides_, on_set_parameters_callback_container_,
      post_set_parameters_callback_container_, events_publisher_.get(), combined_name_,
      *node_clock_);
}

/*!
 * \brief 声明参数（Declare a parameter）
 * \param[in] name 参数名称（Parameter name）
 * \param[in] type 参数类型（Parameter type）
 * \param[in] parameter_descriptor 参数描述符（Parameter descriptor）
 * \param[in] ignore_override 是否忽略覆盖（Whether to ignore override）
 * \return 返回声明的参数值（Return the declared parameter value）
 */
const rclcpp::ParameterValue &NodeParameters::declare_parameter(
    const std::string &name,
    rclcpp::ParameterType type,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
    bool ignore_override) {
  std::lock_guard<std::recursive_mutex> lock(
      mutex_);  // 锁定互斥量，防止多线程同时访问（Lock the mutex to prevent multiple threads from
                // accessing simultaneously）
  ParameterMutationRecursionGuard guard(
      parameter_modification_enabled_);  // 递归保护（Recursion protection）

  if (rclcpp::PARAMETER_NOT_SET ==
      type) {  // 检查参数类型是否有效（Check if the parameter type is valid）
    throw std::invalid_argument{
        // 抛出异常（Throw exception）
        "declare_parameter(): the provided parameter type cannot be rclcpp::PARAMETER_NOT_SET"};
  }

  if (parameter_descriptor.dynamic_typing ==
      true) {  // 检查动态类型是否有效（Check if dynamic typing is valid）
    throw std::invalid_argument{
        // 抛出异常（Throw exception）
        "declare_parameter(): cannot declare parameter of specific type and pass descriptor"
        "with `dynamic_typing=true`"};
  }

  return declare_parameter_helper(  // 调用辅助函数进行参数声明（Call the helper function for
                                    // parameter declaration）
      name, type, rclcpp::ParameterValue{}, parameter_descriptor, ignore_override, parameters_,
      parameter_overrides_, on_set_parameters_callback_container_,
      post_set_parameters_callback_container_, events_publisher_.get(), combined_name_,
      *node_clock_);
}

/**
 * @brief 取消声明参数 (Undeclare a parameter)
 *
 * @param[in] name 参数名称 (Parameter name)
 *
 * @throws rclcpp::exceptions::ParameterNotDeclaredException 如果参数尚未声明 (If the parameter has
 * not been declared yet)
 * @throws rclcpp::exceptions::ParameterImmutableException 如果参数是只读的 (If the parameter is
 * read-only)
 * @throws rclcpp::exceptions::InvalidParameterTypeException 如果参数是静态类型的 (If the parameter
 * is statically typed)
 */
void NodeParameters::undeclare_parameter(const std::string &name) {
  // 对互斥体进行加锁，确保线程安全 (Lock the mutex to ensure thread safety)
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // 创建一个递归防护对象，防止递归修改参数 (Create a recursion guard object to prevent recursive
  // modification of parameters)
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  // 查找参数信息 (Find the parameter information)
  auto parameter_info = parameters_.find(name);
  // 如果参数不存在，则抛出异常 (If the parameter does not exist, throw an exception)
  if (parameter_info == parameters_.end()) {
    throw rclcpp::exceptions::ParameterNotDeclaredException(
        "cannot undeclare parameter '" + name + "' which has not yet been declared");
  }

  // 如果参数是只读的，则抛出异常 (If the parameter is read-only, throw an exception)
  if (parameter_info->second.descriptor.read_only) {
    throw rclcpp::exceptions::ParameterImmutableException(
        "cannot undeclare parameter '" + name + "' because it is read-only");
  }
  // 如果参数是静态类型的，则抛出异常 (If the parameter is statically typed, throw an exception)
  if (!parameter_info->second.descriptor.dynamic_typing) {
    throw rclcpp::exceptions::InvalidParameterTypeException{
        name, "cannot undeclare an statically typed parameter"};
  }

  // 从参数列表中删除参数 (Remove the parameter from the parameter list)
  parameters_.erase(parameter_info);
}

/**
 * @brief 检查是否存在参数 (Check if a parameter exists)
 *
 * @param[in] name 参数名称 (Parameter name)
 * @return bool 如果参数存在则返回 true，否则返回 false (Return true if the parameter exists,
 * otherwise return false)
 */
bool NodeParameters::has_parameter(const std::string &name) const {
  // 对互斥体进行加锁，确保线程安全 (Lock the mutex to ensure thread safety)
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // 使用无锁函数检查参数是否存在 (Use the lockless function to check if the parameter exists)
  return __lockless_has_parameter(parameters_, name);
}

/**
 * @brief 设置一组参数 (Set a group of parameters)
 *
 * @param[in] parameters 要设置的参数列表 (The list of parameters to set)
 * @return std::vector<rcl_interfaces::msg::SetParametersResult> 设置结果列表 (List of set results)
 */
std::vector<rcl_interfaces::msg::SetParametersResult> NodeParameters::set_parameters(
    const std::vector<rclcpp::Parameter> &parameters) {
  // 初始化结果列表 (Initialize result list)
  std::vector<rcl_interfaces::msg::SetParametersResult> results;
  results.reserve(parameters.size());

  // 遍历参数列表，并逐个设置参数 (Iterate through the parameter list and set the parameters one by
  // one)
  for (const auto &p : parameters) {
    auto result = set_parameters_atomically({{p}});
    results.push_back(result);
  }

  // 返回结果列表 (Return the result list)
  return results;
}

/**
 * @brief 根据名称在参数列表中查找参数 (Find a parameter by name in the parameter list)
 *
 * @tparam ParameterVectorType 参数列表类型 (Parameter list type)
 * @param[in] parameters 参数列表 (Parameter list)
 * @param[in] name 要查找的参数名称 (The name of the parameter to find)
 * @return auto 返回找到的参数，如果未找到，则返回参数列表的 end() 迭代器 (Return the found
 * parameter, if not found, return the end() iterator of the parameter list)
 */
template <typename ParameterVectorType>
auto __find_parameter_by_name(ParameterVectorType &parameters, const std::string &name) {
  // 使用 lambda 表达式在参数列表中查找指定名称的参数 (Use a lambda expression to find a parameter
  // with the specified name in the parameter list)
  return std::find_if(parameters.begin(), parameters.end(), [&](auto parameter) {
    return parameter.get_name() == name;
  });
}

/**
 * @brief Set a list of parameters atomically.
 *
 * @param[in] parameters A vector of rclcpp::Parameter objects to be set atomically.
 * @return rcl_interfaces::msg::SetParametersResult The result of setting the parameters.
 */
rcl_interfaces::msg::SetParametersResult NodeParameters::set_parameters_atomically(
    const std::vector<rclcpp::Parameter> &parameters) {
  // 对递归互斥体进行加锁，确保线程安全
  // Lock the recursive mutex to ensure thread safety
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // 创建参数修改递归防护对象
  // Create a ParameterMutationRecursionGuard object to prevent recursion in parameter modification
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  // 初始化结果对象
  // Initialize the result object
  rcl_interfaces::msg::SetParametersResult result;

  // 调用用户注册的预设置参数回调
  // 这个回调可以对原始参数列表进行更改
  // 同时检查更改后的参数列表是否为空，如果为空则返回
  // Call any user registered pre-set parameter callbacks
  // This callback can make changes to the original parameters list
  // Also check if the changed parameter list is empty or not, if empty return
  std::vector<rclcpp::Parameter> parameters_after_pre_set_callback(parameters);
  __call_pre_set_parameters_callbacks(
      parameters_after_pre_set_callback, pre_set_parameters_callback_container_);

  if (parameters_after_pre_set_callback.empty()) {
    result.successful = false;
    result.reason =
        "parameter list cannot be empty, this might be due to "
        "pre_set_parameters_callback modifying the original parameters list.";
    return result;
  }

  // 检查参数是否只读，或者参数是否没有声明
  // 如果没有声明，请在稍后允许未声明参数时跟踪它们以便声明它们，
  // 如果不允许，则失败。
  // Check if any of the parameters are read-only, or if any parameters are not
  // declared.
  // If not declared, keep track of them in order to declare them later, when
  // undeclared parameters are allowed, and if they're not allowed, fail.
  std::vector<const rclcpp::Parameter *> parameters_to_be_declared;
  for (const auto &parameter : parameters_after_pre_set_callback) {
    const std::string &name = parameter.get_name();

    // 确保参数名称有效
    // Check to make sure the parameter name is valid.
    if (name.empty()) {
      throw rclcpp::exceptions::InvalidParametersException("parameter name must not be empty");
    }

    // 检查参数是否已声明
    // Check to see if it is declared.
    auto parameter_info = parameters_.find(name);
    if (parameter_info == parameters_.end()) {
      // 如果没有检查到未声明的参数是否允许...
      // If not check to see if undeclared paramaters are allowed, ...
      if (allow_undeclared_) {
        // 如果是这样，则为用户隐式标记要声明的参数
        // If so, mark the parameter to be declared for the user implicitly.
        parameters_to_be_declared.push_back(&parameter);
        // 继续进行，因为它不能是只读的，而且因为声明将
        // 隐式设置参数和仅用于设置的parameter_infos。
        // continue as it cannot be read-only, and because the declare will
        // implicitly set the parameter and parameter_infos is for setting only.
        continue;
      } else {
        // 如果不是，则按照文档抛出异常
        // If not, then throw the exception as documented.
        throw rclcpp::exceptions::ParameterNotDeclaredException(
            "parameter '" + name + "' cannot be set because it was not declared");
      }
    }

    // 检查参数是否只读
    // Check to see if it is read-only.
    if (parameter_info->second.descriptor.read_only) {
      result.successful = false;
      result.reason = "parameter '" + name + "' cannot be set because it is read-only";
      return result;
    }
  }

  // 在一个临时的“暂存区”中声明参数，以防其中一个声明失败。
  // 我们将使用暂存更改作为“原子设置”的输入。
  // 我们在这里明确避免调用用户回调，以便可以一次调用所有其他要设置的参数（已声明的参数）。
  // Declare parameters into a temporary "staging area", in case one of the declares fail.
  // We will use the staged changes as input to the "set atomically" action.
  // We explicitly avoid calling the user callbacks here, so that it may be called once, with
  // all the other parameters to be set (already declared parameters).
  std::map<std::string, rclcpp::node_interfaces::ParameterInfo> staged_parameter_changes;
  rcl_interfaces::msg::ParameterEvent parameter_event_msg;
  parameter_event_msg.node = combined_name_;
  OnSetCallbacksHandleContainer empty_on_set_callback_container;
  PostSetCallbacksHandleContainer empty_post_set_callback_container;

  // 隐式声明使用动态类型描述符
  // Implicit declare uses dynamic type descriptor.
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  for (auto parameter_to_be_declared : parameters_to_be_declared) {
    // 这不应该抛出异常，因为我们验证了名称并检查了参数是否已经声明。
    // This should not throw, because we validated the name and checked that
    // the parameter was not already declared.
    result = __declare_parameter_common(
        parameter_to_be_declared->get_name(), parameter_to_be_declared->get_parameter_value(),
        descriptor, staged_parameter_changes, parameter_overrides_,
        // Only call callbacks once below
        empty_on_set_callback_container,    // callback_container is explicitly empty
        empty_post_set_callback_container,  // callback_container is explicitly empty
        &parameter_event_msg, true);
    if (!result.successful) {
      // Declare failed, return knowing that nothing was changed because the
      // staged changes were not applied.
      return result;
    }
  }

  // 如果有隐式声明的参数，那么我们可能需要复制输入参数，
  // 然后分配在声明后选择的值（可能受到初始参数值的影响）。
  // If there were implicitly declared parameters, then we may need to copy the input parameters
  // and then assign the value that was selected after the declare (could be affected by the
  // initial parameter values).
  const std::vector<rclcpp::Parameter> *parameters_to_be_set = &parameters_after_pre_set_callback;
  std::vector<rclcpp::Parameter> parameters_copy;
  if (0 != staged_parameter_changes.size()) {  // If there were any implicitly declared parameters.
    bool any_initial_values_used = false;
    for (const auto &staged_parameter_change : staged_parameter_changes) {
      auto it = __find_parameter_by_name(
          parameters_after_pre_set_callback, staged_parameter_change.first);
      if (it->get_parameter_value() != staged_parameter_change.second.value) {
        // 在这种情况下，暂存参数的值与用户输入的值不同，
        // 因此我们需要在设置之前进行更新。
        // In this case, the value of the staged parameter differs from the
        // input from the user, and therefore we need to update things before setting.
        any_initial_values_used = true;
        // 不再需要进一步搜索，因为至少有一个初始值需要使用。
        // No need to search further since at least one initial value needs to be used.
        break;
      }
    }
    if (any_initial_values_used) {
      parameters_copy = parameters_after_pre_set_callback;
      for (const auto &staged_parameter_change : staged_parameter_changes) {
        auto it = __find_parameter_by_name(parameters_copy, staged_parameter_change.first);
        *it = Parameter(staged_parameter_change.first, staged_parameter_change.second.value);
      }
      parameters_to_be_set = &parameters_copy;
    }
  }

  // 收集将其类型更改为rclcpp::PARAMETER_NOT_SET的参数，
  // 以便稍后可以隐式取消声明它们。
  // Collect parameters who will have had their type changed to
  // rclcpp::PARAMETER_NOT_SET so they can later be implicitly undeclared.
  std::vector<const rclcpp::Parameter *> parameters_to_be_undeclared;
  for (const auto &parameter : *parameters_to_be_set) {
    if (rclcpp::PARAMETER_NOT_SET == parameter.get_type()) {
      auto it = parameters_.find(parameter.get_name());
      if (it != parameters_.end() && rclcpp::PARAMETER_NOT_SET != it->second.value.get_type()) {
        if (!it->second.descriptor.dynamic_typing) {
          result.reason = "cannot undeclare an statically typed parameter";
          result.successful = false;
          return result;
        }
        parameters_to_be_undeclared.push_back(&parameter);
      }
    }
  }

  // 设置所有参数，包括上面隐式声明的参数。
  // Set all of the parameters including the ones declared implicitly above.
  result = __set_parameters_atomically_common(
      // either the original parameters given by the user, or ones updated with initial values
      *parameters_to_be_set,
      // they are actually set on the official parameter storage
      parameters_,
      // These callbacks are called once. When a callback returns an unsuccessful result,
      // the remaining aren't called
      on_set_parameters_callback_container_, post_set_parameters_callback_container_,
      allow_undeclared_);  // allow undeclared

  // 如果不成功，则在此停止。
  // If not successful, then stop here.
  if (!result.successful) {
    return result;
  }

  // 如果成功，则从隐式声明参数的参数信息中更新参数信息。
  // If successful, update the parameter information from the implicitly declared parameter
  // information.
  for (const auto &kv_pair : staged_parameter_changes) {
    // 假设：由于上面的 "set"，参数已经存在于 parameters_ 中
    // assumption: the parameter is already present in parameters_ due to the above "set"
    assert(__lockless_has_parameter(parameters_, kv_pair.first));
    // 假设：parameters_ 中的值与 declare 后的值相同
    // assumption: the value in parameters_ is the same as the value resulting from the declare
    assert(parameters_[kv_pair.first].value == kv_pair.second.value);
    // 这个赋值不应该改变名称、类型或值，但可能会改变 ParameterInfo 的其他内容。
    // This assignment should not change the name, type, or value, but may change other things from
    // the ParameterInfo.
    parameters_[kv_pair.first] = kv_pair.second;
  }

  // 取消声明需要取消声明的参数。
  // Undeclare parameters that need to be.
  for (auto parameter_to_undeclare : parameters_to_be_undeclared) {
    auto it = parameters_.find(parameter_to_undeclare->get_name());
    // 假设：要取消声明的参数应该在参数信息映射中
    // assumption: the parameter to be undeclared should be in the parameter infos map
    assert(it != parameters_.end());
    if (it != parameters_.end()) {
      // 更新参数事件消息并删除它。
      // Update the parameter event message and remove it.
      parameter_event_msg.deleted_parameters.push_back(
          rclcpp::Parameter(it->first, it->second.value).to_parameter_msg());
      parameters_.erase(it);
    }
  }

  // 仅更新已设置但未声明或取消声明的参数的参数事件消息。
  // Update the parameter event message for any parameters which were only set, and not either
  // declared or undeclared.
  for (const auto &parameter : *parameters_to_be_set) {
    if (staged_parameter_changes.find(parameter.get_name()) != staged_parameter_changes.end()) {
      // 此参数已声明。
      // This parameter was declared.
      continue;
    }
    auto it = std::find_if(
        parameters_to_be_undeclared.begin(), parameters_to_be_undeclared.end(),
        [&parameter](const auto &p) { return p->get_name() == parameter.get_name(); });
    if (it != parameters_to_be_undeclared.end()) {
      // 此参数已取消声明（删除）。
      // This parameter was undeclared (deleted).
      continue;
    }
    // 该参数既未声明也未取消声明。
    // This parameter was neither declared nor undeclared.
    parameter_event_msg.changed_parameters.push_back(parameter.to_parameter_msg());
  }

  // 如果 events_publisher_ 不是 nullptr，则发布，这可能是在构造函数中禁用的。
  // Publish if events_publisher_ is not nullptr, which may be if disabled in the constructor.
  if (nullptr != events_publisher_) {
    parameter_event_msg.stamp = node_clock_->get_clock()->now();
    events_publisher_->publish(parameter_event_msg);
  }
  return result;
}

/**
 * @brief 获取指定名称的参数列表
 * @param names 参数名称列表
 * @return std::vector<rclcpp::Parameter> 参数对象列表
 *
 * @brief Get the parameters with the specified names
 * @param names A vector of parameter names
 * @return std::vector<rclcpp::Parameter> A vector of Parameter objects
 */
std::vector<rclcpp::Parameter> NodeParameters::get_parameters(
    const std::vector<std::string> &names) const {
  // 创建一个结果向量，用于存储参数对象
  // Create a results vector to store the Parameter objects
  std::vector<rclcpp::Parameter> results;
  results.reserve(names.size());

  // 使用互斥锁保护共享资源
  // Use a lock guard to protect shared resources
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // 遍历参数名称列表
  // Iterate through the list of parameter names
  for (auto &name : names) {
    // 将获取到的参数添加到结果向量中
    // Add the retrieved parameter to the results vector
    results.emplace_back(this->get_parameter(name));
  }

  // 返回参数对象列表
  // Return the vector of Parameter objects
  return results;
}

/**
 * @brief 获取指定名称的参数
 * @param name 参数名称
 * @return rclcpp::Parameter 参数对象
 *
 * @brief Get the parameter with the specified name
 * @param name The parameter name
 * @return rclcpp::Parameter The Parameter object
 */
rclcpp::Parameter NodeParameters::get_parameter(const std::string &name) const {
  // 使用互斥锁保护共享资源
  // Use a lock guard to protect shared resources
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // 查找指定名称的参数
  // Find the parameter with the specified name
  auto param_iter = parameters_.find(name);

  // 如果找到了参数，并且参数类型不是未设置或者允许动态类型
  // If the parameter is found and its type is not unset or dynamic typing is allowed
  if (parameters_.end() != param_iter) {
    if (param_iter->second.value.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET ||
        param_iter->second.descriptor.dynamic_typing) {
      return rclcpp::Parameter{name, param_iter->second.value};
    }
    throw rclcpp::exceptions::ParameterUninitializedException(name);
  } else if (this->allow_undeclared_) {
    return rclcpp::Parameter{name};
  } else {
    throw rclcpp::exceptions::ParameterNotDeclaredException(name);
  }
}

/**
 * @brief 获取指定名称的参数，如果存在则返回 true
 * @param name 参数名称
 * @param parameter 参数对象的引用，用于存储获取到的参数
 * @return bool 参数是否存在
 *
 * @brief Get the parameter with the specified name, return true if it exists
 * @param name The parameter name
 * @param parameter A reference to a Parameter object to store the retrieved parameter
 * @return bool Whether the parameter exists or not
 */
bool NodeParameters::get_parameter(const std::string &name, rclcpp::Parameter &parameter) const {
  // 使用互斥锁保护共享资源
  // Use a lock guard to protect shared resources
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // 查找指定名称的参数
  // Find the parameter with the specified name
  auto param_iter = parameters_.find(name);

  // 如果找到了参数，并且参数类型不是未设置
  // If the parameter is found and its type is not unset
  if (parameters_.end() != param_iter &&
      param_iter->second.value.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    parameter = {name, param_iter->second.value};
    return true;
  } else {
    return false;
  }
}

/**
 * @brief 根据前缀获取参数列表
 * @param prefix 参数名称前缀
 * @param parameters 用于存储获取到的参数的映射表
 * @return bool 是否找到了任何匹配的参数
 *
 * @brief Get the parameters with the specified prefix
 * @param prefix The parameter name prefix
 * @param parameters A map to store the retrieved parameters
 * @return bool Whether any matching parameters were found or not
 */
bool NodeParameters::get_parameters_by_prefix(
    const std::string &prefix, std::map<std::string, rclcpp::Parameter> &parameters) const {
  // 使用互斥锁保护共享资源
  // Use a lock guard to protect shared resources
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // 如果前缀不为空，则在前缀后添加一个点
  // If the prefix is not empty, add a dot after the prefix
  std::string prefix_with_dot = prefix.empty() ? prefix : prefix + ".";
  bool ret = false;

  // 遍历所有参数
  // Iterate through all the parameters
  for (const auto &param : parameters_) {
    // 如果参数名称以指定前缀开头，并且长度大于前缀长度
    // If the parameter name starts with the specified prefix and its length is greater than the
    // prefix length
    if (param.first.find(prefix_with_dot) == 0 && param.first.length() > prefix_with_dot.length()) {
      // 找到一个匹配的参数
      // Found a matching parameter
      parameters[param.first.substr(prefix_with_dot.length())] = rclcpp::Parameter(param.second);
      ret = true;
    }
  }

  // 返回是否找到了任何匹配的参数
  // Return whether any matching parameters were found or not
  return ret;
}

/**
 * @brief 描述参数列表的信息 (Describe the parameter list information)
 *
 * @param names 参数名列表 (List of parameter names)
 * @return std::vector<rcl_interfaces::msg::ParameterDescriptor> 返回参数描述符列表 (Return a list
 * of parameter descriptors)
 */
std::vector<rcl_interfaces::msg::ParameterDescriptor> NodeParameters::describe_parameters(
    const std::vector<std::string> &names) const {
  // 对互斥量进行加锁，保证线程安全 (Lock the mutex to ensure thread safety)
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // 创建结果向量，用于存储参数描述符 (Create a result vector to store parameter descriptors)
  std::vector<rcl_interfaces::msg::ParameterDescriptor> results;

  // 预分配结果向量的大小，以提高性能 (Pre-allocate the size of the result vector for better
  // performance)
  results.reserve(names.size());

  // 遍历输入的参数名列表 (Iterate through the input parameter name list)
  for (const auto &name : names) {
    // 在参数映射中查找参数名 (Find the parameter name in the parameter map)
    auto it = parameters_.find(name);

    // 如果找到了参数名，则将其对应的参数描述符添加到结果向量中 (If the parameter name is found, add
    // its corresponding parameter descriptor to the result vector)
    if (it != parameters_.cend()) {
      results.push_back(it->second.descriptor);
    } else if (allow_undeclared_) {
      // 参数未找到，但允许未声明参数，因此返回空描述符 (Parameter not found, but undeclared
      // parameters allowed, so return an empty descriptor)
      rcl_interfaces::msg::ParameterDescriptor default_description;
      default_description.name = name;
      results.push_back(default_description);
    } else {
      // 参数未找到且不允许未声明参数时，抛出异常 (Throw exception when parameter not found and
      // undeclared parameters not allowed)
      throw rclcpp::exceptions::ParameterNotDeclaredException(name);
    }
  }

  // 如果结果向量的大小与输入参数名列表的大小不同，则抛出运行时错误 (If the size of the result
  // vector is different from the size of the input parameter name list, throw a runtime error)
  if (results.size() != names.size()) {
    throw std::runtime_error("results and names unexpectedly different sizes");
  }

  // 返回参数描述符列表 (Return the list of parameter descriptors)
  return results;
}

/**
 * @brief 获取参数类型列表 (Get the list of parameter types)
 *
 * @param names 参数名列表 (List of parameter names)
 * @return std::vector<uint8_t> 返回参数类型列表 (Return a list of parameter types)
 */
std::vector<uint8_t> NodeParameters::get_parameter_types(
    const std::vector<std::string> &names) const {
  // 对互斥量进行加锁，保证线程安全 (Lock the mutex to ensure thread safety)
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // 创建结果向量，用于存储参数类型 (Create a result vector to store parameter types)
  std::vector<uint8_t> results;

  // 预分配结果向量的大小，以提高性能 (Pre-allocate the size of the result vector for better
  // performance)
  results.reserve(names.size());

  // 遍历输入的参数名列表 (Iterate through the input parameter name list)
  for (const auto &name : names) {
    // 在参数映射中查找参数名 (Find the parameter name in the parameter map)
    auto it = parameters_.find(name);

    // 如果找到了参数名，则将其对应的参数类型添加到结果向量中 (If the parameter name is found, add
    // its corresponding parameter type to the result vector)
    if (it != parameters_.cend()) {
      results.push_back(it->second.value.get_type());
    } else if (allow_undeclared_) {
      // 参数未找到，但允许未声明参数，因此返回未设置的参数类型 (Parameter not found, but undeclared
      // parameters allowed, so return not set parameter type)
      results.push_back(rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
    } else {
      // 参数未找到且不允许未声明参数时，抛出异常 (Throw exception when parameter not found and
      // undeclared parameters not allowed)
      throw rclcpp::exceptions::ParameterNotDeclaredException(name);
    }
  }

  // 如果结果向量的大小与输入参数名列表的大小不同，则抛出运行时错误 (If the size of the result
  // vector is different from the size of the input parameter name list, throw a runtime error)
  if (results.size() != names.size()) {
    throw std::runtime_error("results and names unexpectedly different sizes");
  }

  // 返回参数类型列表 (Return the list of parameter types)
  return results;
}

/**
 * @brief 列出参数列表 (List the parameters)
 *
 * @param prefixes 参数名前缀 (Parameter name prefixes)
 * @param depth 参数层次深度 (Parameter hierarchy depth)
 * @return rcl_interfaces::msg::ListParametersResult 返回包含参数名称和前缀的结果 (Return a result
 * containing parameter names and prefixes)
 */
rcl_interfaces::msg::ListParametersResult NodeParameters::list_parameters(
    const std::vector<std::string> &prefixes, uint64_t depth) const {
  // 用递归互斥锁保护数据 (Protect data with recursive mutex lock)
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // 创建 ListParametersResult 结果对象 (Create ListParametersResult result object)
  rcl_interfaces::msg::ListParametersResult result;

  // TODO(mikaelarguedas) 定义与"/"不同的参数分隔符以避免歧义 (Define parameter separator different
  // from "/" to avoid ambiguity) 暂时使用 "." (Using "." for now)
  const char *separator = ".";

  // 遍历参数键值对 (Iterate through parameter key-value pairs)
  for (auto &kv : parameters_) {
    // 判断是否获取所有参数 (Determine whether to get all parameters)
    bool get_all =
        (prefixes.size() == 0) &&
        ((depth == rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE) ||
         (static_cast<uint64_t>(std::count(kv.first.begin(), kv.first.end(), *separator)) < depth));

    // 判断前缀是否匹配 (Check if prefix matches)
    bool prefix_matches = std::any_of(
        prefixes.cbegin(), prefixes.cend(), [&kv, &depth, &separator](const std::string &prefix) {
          if (kv.first == prefix) {
            return true;
          } else if (kv.first.find(prefix + separator) == 0) {
            size_t length = prefix.length();
            std::string substr = kv.first.substr(length);
            // 转换为无符号整数以避免警告 (Cast as unsigned integer to avoid warning)
            return (depth == rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE) ||
                   (static_cast<uint64_t>(std::count(substr.begin(), substr.end(), *separator)) <
                    depth);
          }
          return false;
        });

    // 如果获取所有参数或前缀匹配 (If get all parameters or prefix matches)
    if (get_all || prefix_matches) {
      result.names.push_back(kv.first);
      size_t last_separator = kv.first.find_last_of(separator);
      if (std::string::npos != last_separator) {
        std::string prefix = kv.first.substr(0, last_separator);
        if (std::find(result.prefixes.cbegin(), result.prefixes.cend(), prefix) ==
            result.prefixes.cend()) {
          result.prefixes.push_back(prefix);
        }
      }
    }
  }

  // 返回结果 (Return the result)
  return result;
}

/**
 * @brief 删除预设参数回调 (Remove pre-set parameters callback)
 *
 * @param handle 预设参数回调句柄 (PreSetParametersCallbackHandle pointer)
 */
void NodeParameters::remove_pre_set_parameters_callback(
    const PreSetParametersCallbackHandle *const handle) {
  // 对互斥量进行加锁，防止多线程同时修改 (Lock the mutex to prevent multiple threads from modifying
  // at the same time)
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // 创建参数修改递归保护对象 (Create a parameter modification recursion guard object)
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  // 在回调容器中查找指定的回调句柄 (Find the specified callback handle in the callback container)
  auto it = std::find_if(
      pre_set_parameters_callback_container_.begin(), pre_set_parameters_callback_container_.end(),
      [handle](const auto &weak_handle) { return handle == weak_handle.lock().get(); });
  // 如果找到了指定的回调句柄，则删除它 (If the specified callback handle is found, delete it)
  if (it != pre_set_parameters_callback_container_.end()) {
    pre_set_parameters_callback_container_.erase(it);
  } else {
    // 否则抛出运行时错误 (Otherwise, throw a runtime error)
    throw std::runtime_error("Pre set parameter callback doesn't exist");
  }
}

/**
 * @brief 删除设置参数回调 (Remove on-set parameters callback)
 *
 * @param handle 设置参数回调句柄 (OnSetParametersCallbackHandle pointer)
 */
void NodeParameters::remove_on_set_parameters_callback(
    const OnSetParametersCallbackHandle *const handle) {
  // 对互斥量进行加锁，防止多线程同时修改 (Lock the mutex to prevent multiple threads from modifying
  // at the same time)
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // 创建参数修改递归保护对象 (Create a parameter modification recursion guard object)
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  // 在回调容器中查找指定的回调句柄 (Find the specified callback handle in the callback container)
  auto it = std::find_if(
      on_set_parameters_callback_container_.begin(), on_set_parameters_callback_container_.end(),
      [handle](const auto &weak_handle) { return handle == weak_handle.lock().get(); });
  // 如果找到了指定的回调句柄，则删除它 (If the specified callback handle is found, delete it)
  if (it != on_set_parameters_callback_container_.end()) {
    on_set_parameters_callback_container_.erase(it);
  } else {
    // 否则抛出运行时错误 (Otherwise, throw a runtime error)
    throw std::runtime_error("On set parameter callback doesn't exist");
  }
}

/**
 * @brief 删除后设置参数回调 (Remove post-set parameters callback)
 *
 * @param handle 后设置参数回调句柄 (PostSetParametersCallbackHandle pointer)
 */
void NodeParameters::remove_post_set_parameters_callback(
    const PostSetParametersCallbackHandle *const handle) {
  // 对互斥量进行加锁，防止多线程同时修改 (Lock the mutex to prevent multiple threads from modifying
  // at the same time)
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // 创建参数修改递归保护对象 (Create a parameter modification recursion guard object)
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  // 在回调容器中查找指定的回调句柄 (Find the specified callback handle in the callback container)
  auto it = std::find_if(
      post_set_parameters_callback_container_.begin(),
      post_set_parameters_callback_container_.end(),
      [handle](const auto &weak_handle) { return handle == weak_handle.lock().get(); });
  // 如果找到了指定的回调句柄，则删除它 (If the specified callback handle is found, delete it)
  if (it != post_set_parameters_callback_container_.end()) {
    post_set_parameters_callback_container_.erase(it);
  } else {
    // 否则抛出运行时错误 (Otherwise, throw a runtime error)
    throw std::runtime_error("Post set parameter callback doesn't exist");
  }
}

/**
 * @brief 添加预设参数回调函数（Add a pre-set parameters callback function）
 *
 * @param[in] callback 预设参数回调函数类型的参数（Parameter of PreSetParametersCallbackType）
 * @return PreSetParametersCallbackHandle::SharedPtr 返回一个预设参数回调句柄的共享指针（Return a
 * shared pointer to a pre-set parameters callback handle）
 */
PreSetParametersCallbackHandle::SharedPtr NodeParameters::add_pre_set_parameters_callback(
    PreSetParametersCallbackType callback) {
  // 对互斥锁进行上锁，保护共享资源（Lock the mutex to protect shared resources）
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // 创建递归防护对象，防止参数修改的递归调用（Create a recursion guard object to prevent recursive
  // calls to parameter modification）
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  // 创建预设参数回调句柄的共享指针（Create a shared pointer to the pre-set parameters callback
  // handle）
  auto handle = std::make_shared<PreSetParametersCallbackHandle>();
  // 设置回调函数（Set the callback function）
  handle->callback = callback;
  // 最后注册的回调函数将首先执行（The last registered callback is executed first）
  pre_set_parameters_callback_container_.emplace_front(handle);
  return handle;
}

/**
 * @brief 添加设置参数回调函数（Add an on-set parameters callback function）
 *
 * @param[in] callback 设置参数回调函数类型的参数（Parameter of OnSetParametersCallbackType）
 * @return OnSetParametersCallbackHandle::SharedPtr 返回一个设置参数回调句柄的共享指针（Return a
 * shared pointer to an on-set parameters callback handle）
 */
OnSetParametersCallbackHandle::SharedPtr NodeParameters::add_on_set_parameters_callback(
    OnSetParametersCallbackType callback) {
  // 对互斥锁进行上锁，保护共享资源（Lock the mutex to protect shared resources）
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // 创建递归防护对象，防止参数修改的递归调用（Create a recursion guard object to prevent recursive
  // calls to parameter modification）
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  // 创建设置参数回调句柄的共享指针（Create a shared pointer to the on-set parameters callback
  // handle）
  auto handle = std::make_shared<OnSetParametersCallbackHandle>();
  // 设置回调函数（Set the callback function）
  handle->callback = callback;
  // 最后注册的回调函数将首先执行（The last registered callback is executed first）
  on_set_parameters_callback_container_.emplace_front(handle);
  return handle;
}

/**
 * @brief 添加设置参数之后的回调函数（Add a post-set parameters callback function）
 *
 * @param[in] callback 设置参数之后回调函数类型的参数（Parameter of PostSetParametersCallbackType）
 * @return PostSetParametersCallbackHandle::SharedPtr 返回一个设置参数之后回调句柄的共享指针（Return
 * a shared pointer to a post-set parameters callback handle）
 */
PostSetParametersCallbackHandle::SharedPtr NodeParameters::add_post_set_parameters_callback(
    PostSetParametersCallbackType callback) {
  // 对互斥锁进行上锁，保护共享资源（Lock the mutex to protect shared resources）
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // 创建递归防护对象，防止参数修改的递归调用（Create a recursion guard object to prevent recursive
  // calls to parameter modification）
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  // 创建设置参数之后回调句柄的共享指针（Create a shared pointer to the post-set parameters callback
  // handle）
  auto handle = std::make_shared<PostSetParametersCallbackHandle>();
  // 设置回调函数（Set the callback function）
  handle->callback = callback;
  // 最后注册的回调函数将首先执行（The last registered callback is executed first）
  post_set_parameters_callback_container_.emplace_front(handle);
  return handle;
}

/**
 * @brief 获取参数覆盖值（Get parameter overrides）
 *
 * @return const std::map<std::string, rclcpp::ParameterValue>&
 * 返回一个字符串和参数值映射的引用（Return a reference to a map of strings and parameter values）
 */
const std::map<std::string, rclcpp::ParameterValue> &NodeParameters::get_parameter_overrides()
    const {
  return parameter_overrides_;
}

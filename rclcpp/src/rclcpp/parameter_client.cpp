// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/parameter_client.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "./parameter_service_names.hpp"

using rclcpp::AsyncParametersClient;
using rclcpp::SyncParametersClient;

/**
 * @brief 构造异步参数客户端 (Constructs an asynchronous parameters client)
 *
 * @param node_base_interface 节点基本接口共享指针 (Shared pointer to the node base interface)
 * @param node_topics_interface 节点主题接口共享指针 (Shared pointer to the node topics interface)
 * @param node_graph_interface 节点图形接口共享指针 (Shared pointer to the node graph interface)
 * @param node_services_interface 节点服务接口共享指针 (Shared pointer to the node services
 * interface)
 * @param remote_node_name 远程节点名称 (Remote node name)
 * @param qos_profile 服务质量配置文件 (Quality of service profile)
 * @param group 回调组共享指针 (Shared pointer to the callback group)
 */
AsyncParametersClient::AsyncParametersClient(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
    const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
    const std::string& remote_node_name,
    const rclcpp::QoS& qos_profile,
    rclcpp::CallbackGroup::SharedPtr group)
    : node_topics_interface_(
          node_topics_interface)  // 初始化节点主题接口 (Initialize the node topics interface)
{
  // 判断远程节点名是否为空 (Check if the remote node name is empty)
  if (remote_node_name != "") {
    remote_node_name_ = remote_node_name;  // 设置远程节点名称 (Set the remote node name)
  } else {
    // 获取完全限定名称 (Get the fully qualified name)
    remote_node_name_ = node_base_interface->get_fully_qualified_name();
  }

  // 获取默认的客户端选项 (Get default client options)
  rcl_client_options_t options = rcl_client_get_default_options();
  // 设置服务质量配置 (Set the quality of service profile)
  options.qos = qos_profile.get_rmw_qos_profile();

  // 定义客户端类型别名 (Define client type aliases)
  using rclcpp::Client;
  using rclcpp::ClientBase;

  // 创建获取参数的客户端 (Create a get parameters client)
  get_parameters_client_ = Client<rcl_interfaces::srv::GetParameters>::make_shared(
      node_base_interface.get(), node_graph_interface,
      remote_node_name_ + "/" + parameter_service_names::get_parameters, options);
  auto get_parameters_base = std::dynamic_pointer_cast<ClientBase>(get_parameters_client_);
  // 将客户端添加到节点服务接口中 (Add the client to the node services interface)
  node_services_interface->add_client(get_parameters_base, group);

  // 创建获取参数类型的客户端 (Create a get parameter types client)
  get_parameter_types_client_ = Client<rcl_interfaces::srv::GetParameterTypes>::make_shared(
      node_base_interface.get(), node_graph_interface,
      remote_node_name_ + "/" + parameter_service_names::get_parameter_types, options);
  auto get_parameter_types_base =
      std::dynamic_pointer_cast<ClientBase>(get_parameter_types_client_);
  // 将客户端添加到节点服务接口中 (Add the client to the node services interface)
  node_services_interface->add_client(get_parameter_types_base, group);

  // 创建设置参数的客户端 (Create a set parameters client)
  set_parameters_client_ = Client<rcl_interfaces::srv::SetParameters>::make_shared(
      node_base_interface.get(), node_graph_interface,
      remote_node_name_ + "/" + parameter_service_names::set_parameters, options);
  auto set_parameters_base = std::dynamic_pointer_cast<ClientBase>(set_parameters_client_);
  // 将客户端添加到节点服务接口中 (Add the client to the node services interface)
  node_services_interface->add_client(set_parameters_base, group);

  // 创建原子性设置参数的客户端 (Create a set parameters atomically client)
  set_parameters_atomically_client_ =
      Client<rcl_interfaces::srv::SetParametersAtomically>::make_shared(
          node_base_interface.get(), node_graph_interface,
          remote_node_name_ + "/" + parameter_service_names::set_parameters_atomically, options);
  auto set_parameters_atomically_base =
      std::dynamic_pointer_cast<ClientBase>(set_parameters_atomically_client_);
  // 将客户端添加到节点服务接口中 (Add the client to the node services interface)
  node_services_interface->add_client(set_parameters_atomically_base, group);

  // 创建列出参数的客户端 (Create a list parameters client)
  list_parameters_client_ = Client<rcl_interfaces::srv::ListParameters>::make_shared(
      node_base_interface.get(), node_graph_interface,
      remote_node_name_ + "/" + parameter_service_names::list_parameters, options);
  auto list_parameters_base = std::dynamic_pointer_cast<ClientBase>(list_parameters_client_);
  // 将客户端添加到节点服务接口中 (Add the client to the node services interface)
  node_services_interface->add_client(list_parameters_base, group);

  // 创建描述参数的客户端 (Create a describe parameters client)
  describe_parameters_client_ = Client<rcl_interfaces::srv::DescribeParameters>::make_shared(
      node_base_interface.get(), node_graph_interface,
      remote_node_name_ + "/" + parameter_service_names::describe_parameters, options);
  auto describe_parameters_base =
      std::dynamic_pointer_cast<ClientBase>(describe_parameters_client_);
  // 将客户端添加到节点服务接口中 (Add the client to the node services interface)
  node_services_interface->add_client(describe_parameters_base, group);
}

/**
 * @brief 获取参数列表
 * @param names 参数名称列表
 * @param callback 回调函数，当获取到参数后执行
 * @return 返回一个 std::shared_future 对象，用于获取异步操作的结果
 *
 * @brief Get the list of parameters
 * @param names The list of parameter names
 * @param callback The callback function to be executed when the parameters are retrieved
 * @return Returns a std::shared_future object for obtaining the result of the asynchronous
 * operation
 */
std::shared_future<std::vector<rclcpp::Parameter>> AsyncParametersClient::get_parameters(
    const std::vector<std::string>& names,
    std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback) {
  // 创建一个 promise 对象，并获取对应的 future 对象
  // Create a promise object and get the corresponding future object
  auto promise_result = std::make_shared<std::promise<std::vector<rclcpp::Parameter>>>();
  auto future_result = promise_result->get_future().share();

  // 创建一个 GetParameters 请求对象并设置请求参数名
  // Create a GetParameters request object and set the requested parameter names
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = names;

  // 发送异步请求，并处理回调
  // Send an asynchronous request and handle the callback
  get_parameters_client_->async_send_request(
      request, [request, promise_result, future_result,
                callback](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture cb_f) {
        // 初始化参数向量
        // Initialize the parameter vector
        std::vector<rclcpp::Parameter> parameters;
        auto& pvalues = cb_f.get()->values;

        // 遍历返回的参数值，将其转换为 rclcpp::Parameter 对象
        // Iterate through the returned parameter values and convert them to rclcpp::Parameter
        // objects
        for (auto& pvalue : pvalues) {
          auto i = static_cast<size_t>(&pvalue - &pvalues[0]);
          rcl_interfaces::msg::Parameter parameter;
          parameter.name = request->names[i];
          parameter.value = pvalue;
          parameters.push_back(rclcpp::Parameter::from_parameter_msg(parameter));
        }

        // 设置 promise 的值
        // Set the value of the promise
        promise_result->set_value(parameters);
        // 如果回调不为空，则执行回调
        // If the callback is not null, execute the callback
        if (callback != nullptr) {
          callback(future_result);
        }
      });

  return future_result;
}

/**
 * @brief 描述参数列表
 * @param names 参数名称列表
 * @param callback 回调函数，当描述参数后执行
 * @return 返回一个 std::shared_future 对象，用于获取异步操作的结果
 *
 * @brief Describe the list of parameters
 * @param names The list of parameter names
 * @param callback The callback function to be executed when the parameters are described
 * @return Returns a std::shared_future object for obtaining the result of the asynchronous
 * operation
 */
std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>>
AsyncParametersClient::describe_parameters(
    const std::vector<std::string>& names,
    std::function<void(std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>>)>
        callback) {
  // 创建一个 promise 对象，并获取对应的 future 对象
  // Create a promise object and get the corresponding future object
  auto promise_result =
      std::make_shared<std::promise<std::vector<rcl_interfaces::msg::ParameterDescriptor>>>();
  auto future_result = promise_result->get_future().share();

  // 创建一个 DescribeParameters 请求对象并设置请求参数名
  // Create a DescribeParameters request object and set the requested parameter names
  auto request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();
  request->names = names;

  // 发送异步请求，并处理回调
  // Send an asynchronous request and handle the callback
  describe_parameters_client_->async_send_request(
      request, [promise_result, future_result, callback](
                   rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedFuture cb_f) {
        // 设置 promise 的值
        // Set the value of the promise
        promise_result->set_value(cb_f.get()->descriptors);
        // 如果回调不为空，则执行回调
        // If the callback is not null, execute the callback
        if (callback != nullptr) {
          callback(future_result);
        }
      });

  return future_result;
}

/**
 * @brief 获取参数类型列表
 * @param names 参数名称列表
 * @param callback 回调函数，当获取到参数类型后执行
 * @return 返回一个 std::shared_future 对象，用于获取异步操作的结果
 *
 * @brief Get the list of parameter types
 * @param names The list of parameter names
 * @param callback The callback function to be executed when the parameter types are retrieved
 * @return Returns a std::shared_future object for obtaining the result of the asynchronous
 * operation
 */
std::shared_future<std::vector<rclcpp::ParameterType>> AsyncParametersClient::get_parameter_types(
    const std::vector<std::string>& names,
    std::function<void(std::shared_future<std::vector<rclcpp::ParameterType>>)> callback) {
  // 创建一个 promise 对象，并获取对应的 future 对象
  // Create a promise object and get the corresponding future object
  auto promise_result = std::make_shared<std::promise<std::vector<rclcpp::ParameterType>>>();
  auto future_result = promise_result->get_future().share();

  // 创建一个 GetParameterTypes 请求对象并设置请求参数名
  // Create a GetParameterTypes request object and set the requested parameter names
  auto request = std::make_shared<rcl_interfaces::srv::GetParameterTypes::Request>();
  request->names = names;

  // 发送异步请求，并处理回调
  // Send an asynchronous request and handle the callback
  get_parameter_types_client_->async_send_request(
      request, [promise_result, future_result, callback](
                   rclcpp::Client<rcl_interfaces::srv::GetParameterTypes>::SharedFuture cb_f) {
        // 初始化参数类型向量
        // Initialize the parameter type vector
        std::vector<rclcpp::ParameterType> types;
        auto& pts = cb_f.get()->types;
        // 遍历返回的参数类型，将其转换为 rclcpp::ParameterType 对象
        // Iterate through the returned parameter types and convert them to rclcpp::ParameterType
        // objects
        for (auto& pt : pts) {
          types.push_back(static_cast<rclcpp::ParameterType>(pt));
        }
        // 设置 promise 的值
        // Set the value of the promise
        promise_result->set_value(types);
        // 如果回调不为空，则执行回调
        // If the callback is not null, execute the callback
        if (callback != nullptr) {
          callback(future_result);
        }
      });

  return future_result;
}

/**
 * @brief 设置参数
 * @param parameters 参数列表
 * @param callback 回调函数
 * @return std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
 *
 * @brief Set parameters
 * @param parameters A list of parameters
 * @param callback Callback function
 * @return std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
 */
std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
AsyncParametersClient::set_parameters(
    const std::vector<rclcpp::Parameter>& parameters,
    std::function<void(std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>)>
        callback) {
  // 创建一个承诺结果实例
  // Create a promise result instance
  auto promise_result =
      std::make_shared<std::promise<std::vector<rcl_interfaces::msg::SetParametersResult>>>();

  // 获取承诺的未来结果
  // Get the future result of the promise
  auto future_result = promise_result->get_future().share();

  // 创建一个设置参数请求实例
  // Create a set parameters request instance
  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

  // 将输入的参数转换为参数消息并插入到请求中
  // Transform input parameters to parameter messages and insert into the request
  std::transform(
      parameters.begin(), parameters.end(), std::back_inserter(request->parameters),
      [](rclcpp::Parameter p) { return p.to_parameter_msg(); });

  // 异步发送设置参数请求
  // Asynchronously send the set parameters request
  set_parameters_client_->async_send_request(
      request, [promise_result, future_result,
                callback](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture cb_f) {
        // 设置承诺的值
        // Set the value of the promise
        promise_result->set_value(cb_f.get()->results);

        // 如果回调函数不为空，则执行回调函数
        // If the callback function is not null, execute the callback function
        if (callback != nullptr) {
          callback(future_result);
        }
      });

  // 返回未来结果
  // Return the future result
  return future_result;
}

/**
 * @brief 原子地设置参数
 * @param parameters 参数列表
 * @param callback 回调函数
 * @return std::shared_future<rcl_interfaces::msg::SetParametersResult>
 *
 * @brief Set parameters atomically
 * @param parameters A list of parameters
 * @param callback Callback function
 * @return std::shared_future<rcl_interfaces::msg::SetParametersResult>
 */
std::shared_future<rcl_interfaces::msg::SetParametersResult>
AsyncParametersClient::set_parameters_atomically(
    const std::vector<rclcpp::Parameter>& parameters,
    std::function<void(std::shared_future<rcl_interfaces::msg::SetParametersResult>)> callback) {
  // 创建一个承诺结果实例
  // Create a promise result instance
  auto promise_result = std::make_shared<std::promise<rcl_interfaces::msg::SetParametersResult>>();

  // 获取承诺的未来结果
  // Get the future result of the promise
  auto future_result = promise_result->get_future().share();

  // 创建一个原子设置参数请求实例
  // Create an atomic set parameters request instance
  auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();

  // 将输入的参数转换为参数消息并插入到请求中
  // Transform input parameters to parameter messages and insert into the request
  std::transform(
      parameters.begin(), parameters.end(), std::back_inserter(request->parameters),
      [](rclcpp::Parameter p) { return p.to_parameter_msg(); });

  // 异步发送原子设置参数请求
  // Asynchronously send the atomic set parameters request
  set_parameters_atomically_client_->async_send_request(
      request,
      [promise_result, future_result,
       callback](rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture cb_f) {
        // 设置承诺的值
        // Set the value of the promise
        promise_result->set_value(cb_f.get()->result);

        // 如果回调函数不为空，则执行回调函数
        // If the callback function is not null, execute the callback function
        if (callback != nullptr) {
          callback(future_result);
        }
      });

  // 返回未来结果
  // Return the future result
  return future_result;
}

/**
 * @brief 删除参数
 * @param parameters_names 参数名列表
 * @return std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
 *
 * @brief Delete parameters
 * @param parameters_names A list of parameter names
 * @return std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
 */
std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
AsyncParametersClient::delete_parameters(const std::vector<std::string>& parameters_names) {
  // 创建一个参数列表
  // Create a list of parameters
  std::vector<rclcpp::Parameter> parameters;

  // 遍历参数名列表，将参数名添加到参数列表中
  // Iterate through the parameter names list and add parameter names to the parameters list
  for (const std::string& name : parameters_names) {
    parameters.push_back(rclcpp::Parameter(name));
  }

  // 设置参数并获取未来结果
  // Set parameters and get future result
  auto future_result = set_parameters(parameters);

  // 返回未来结果
  // Return the future result
  return future_result;
}

/**
 * @brief 从 YAML 文件加载参数
 * @param yaml_filename YAML 文件名
 * @return std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
 *
 * @brief Load parameters from a YAML file
 * @param yaml_filename The YAML filename
 * @return std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
 */
std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
AsyncParametersClient::load_parameters(const std::string& yaml_filename) {
  // 从 YAML 文件中读取参数映射
  // Read parameter map from the YAML file
  rclcpp::ParameterMap parameter_map =
      rclcpp::parameter_map_from_yaml_file(yaml_filename, remote_node_name_.c_str());

  // 查找远程节点名对应的参数
  // Find the parameters corresponding to the remote node name
  auto iter = parameter_map.find(remote_node_name_);

  // 如果没有找到有效参数，则抛出异常
  // If no valid parameters are found, throw an exception
  if (iter == parameter_map.end() || iter->second.size() == 0) {
    throw rclcpp::exceptions::InvalidParametersException("No valid parameter");
  }

  // 设置参数并获取未来结果
  // Set parameters and get future result
  auto future_result = set_parameters(iter->second);

  // 返回未来结果
  // Return the future result
  return future_result;
}

/**
 * @brief 从参数映射中加载参数
 * @param parameter_map 参数映射
 * @return std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
 *
 * @brief Load parameters from a parameter map
 * @param parameter_map The parameter map
 * @return std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
 */
std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
AsyncParametersClient::load_parameters(const rclcpp::ParameterMap& parameter_map) {
  // 从参数映射中获取参数列表
  // Get the list of parameters from the parameter map
  std::vector<rclcpp::Parameter> parameters =
      rclcpp::parameters_from_map(parameter_map, remote_node_name_.c_str());

  // 如果没有找到有效参数，则抛出异常
  // If no valid parameters are found, throw an exception
  if (parameters.size() == 0) {
    throw rclcpp::exceptions::InvalidParametersException("No valid parameter");
  }

  // 设置参数并获取未来结果
  // Set parameters and get future result
  auto future_result = set_parameters(parameters);

  // 返回未来结果
  // Return the future result
  return future_result;
}

/**
 * @brief List parameters with the specified prefixes and depth.
 *
 * @param[in] prefixes 指定的参数名前缀列表 (A list of parameter name prefixes)
 * @param[in] depth 参数树的搜索深度 (The depth of search in the parameter tree)
 * @param[in] callback 当异步请求完成时调用的回调函数 (The callback function to call when the
 * asynchronous request is completed)
 * @return 返回一个共享的 future，包含 ListParametersResult 类型的结果 (Returns a shared future
 * containing the ListParametersResult type result)
 */
std::shared_future<rcl_interfaces::msg::ListParametersResult>
AsyncParametersClient::list_parameters(
    const std::vector<std::string>& prefixes,
    uint64_t depth,
    std::function<void(std::shared_future<rcl_interfaces::msg::ListParametersResult>)> callback) {
  // 创建一个 promise 用于存储结果 (Create a promise to store the result)
  auto promise_result = std::make_shared<std::promise<rcl_interfaces::msg::ListParametersResult>>();
  // 从 promise 中获取一个共享的 future (Get a shared future from the promise)
  auto future_result = promise_result->get_future().share();

  // 创建一个 ListParameters 服务请求 (Create a ListParameters service request)
  auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
  request->prefixes = prefixes;
  request->depth = depth;

  // 发送异步请求，并在完成时设置 promise 的值和调用回调函数 (Send an asynchronous request, and set
  // the value of the promise and call the callback function when completed)
  list_parameters_client_->async_send_request(
      request, [promise_result, future_result,
                callback](rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture cb_f) {
        promise_result->set_value(cb_f.get()->result);
        if (callback != nullptr) {
          callback(future_result);
        }
      });

  return future_result;
}

/**
 * @brief 检查所有参数服务是否已就绪 (Check if all parameter services are ready)
 *
 * @return 如果所有参数服务已就绪，则返回 true，否则返回 false (Returns true if all parameter
 * services are ready, false otherwise)
 */
bool AsyncParametersClient::service_is_ready() const {
  return get_parameters_client_->service_is_ready() &&
         get_parameter_types_client_->service_is_ready() &&
         set_parameters_client_->service_is_ready() &&
         list_parameters_client_->service_is_ready() &&
         describe_parameters_client_->service_is_ready();
}

/**
 * @brief 等待所有参数服务在指定超时时间内就绪 (Wait for all parameter services to be ready within
 * the specified timeout)
 *
 * @param[in] timeout 等待的超时时间 (The timeout time to wait)
 * @return 如果所有参数服务在超时时间内就绪，则返回 true，否则返回 false (Returns true if all
 * parameter services are ready within the timeout, false otherwise)
 */
bool AsyncParametersClient::wait_for_service_nanoseconds(std::chrono::nanoseconds timeout) {
  // 定义一个包含所有客户端的向量 (Define a vector containing all clients)
  const std::vector<std::shared_ptr<rclcpp::ClientBase>> clients = {
      get_parameters_client_, get_parameter_types_client_, set_parameters_client_,
      list_parameters_client_, describe_parameters_client_};
  for (auto& client : clients) {
    auto stamp = std::chrono::steady_clock::now();
    if (!client->wait_for_service(timeout)) {
      return false;
    }
    if (timeout > std::chrono::nanoseconds::zero()) {
      timeout -= std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - stamp);
      if (timeout < std::chrono::nanoseconds::zero()) {
        timeout = std::chrono::nanoseconds::zero();
      }
    }
  }
  return true;
}

/**
 * @brief 获取指定参数名称的参数值 (Get the parameter values for the specified parameter names)
 *
 * @param[in] parameter_names 指定的参数名称列表 (The list of specified parameter names)
 * @param[in] timeout 等待结果的超时时间 (The timeout time to wait for the result)
 * @return 返回一个包含获取到的参数值的向量 (Returns a vector containing the obtained parameter
 * values)
 */
std::vector<rclcpp::Parameter> SyncParametersClient::get_parameters(
    const std::vector<std::string>& parameter_names, std::chrono::nanoseconds timeout) {
  auto f = async_parameters_client_->get_parameters(parameter_names);
  using rclcpp::executors::spin_node_until_future_complete;
  if (spin_node_until_future_complete(*executor_, node_base_interface_, f, timeout) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return f.get();
  }
  // 如果不成功，返回一个空向量 (Return an empty vector if unsuccessful)
  return std::vector<rclcpp::Parameter>();
}

/**
 * @brief 检查是否存在具有指定参数名称的参数 (Check if there is a parameter with the specified
 * parameter name)
 *
 * @param[in] parameter_name 指定的参数名称 (The specified parameter name)
 * @return 如果存在具有指定参数名称的参数，则返回 true，否则返回 false (Returns true if there is a
 * parameter with the specified parameter name, false otherwise)
 */
bool SyncParametersClient::has_parameter(const std::string& parameter_name) {
  std::vector<std::string> names;
  names.push_back(parameter_name);
  auto vars = list_parameters(names, 1);
  return vars.names.size() > 0;
}

/**
 * @brief 获取参数描述
 * @param parameter_names 参数名列表
 * @param timeout 超时时间
 * @return 参数描述向量
 *
 * @brief Get parameter descriptions
 * @param parameter_names List of parameter names
 * @param timeout Timeout duration
 * @return Vector of parameter descriptors
 */
std::vector<rcl_interfaces::msg::ParameterDescriptor> SyncParametersClient::describe_parameters(
    const std::vector<std::string>& parameter_names, std::chrono::nanoseconds timeout) {
  // 调用异步客户端的 describe_parameters 方法
  // Call the describe_parameters method of the asynchronous client
  auto f = async_parameters_client_->describe_parameters(parameter_names);

  // 使用 spin_node_until_future_complete 方法等待结果
  // Use the spin_node_until_future_complete method to wait for the result
  using rclcpp::executors::spin_node_until_future_complete;
  rclcpp::FutureReturnCode future =
      spin_node_until_future_complete(*executor_, node_base_interface_, f, timeout);
  if (future == rclcpp::FutureReturnCode::SUCCESS) {
    return f.get();
  }
  return std::vector<rcl_interfaces::msg::ParameterDescriptor>();
}

/**
 * @brief 获取参数类型
 * @param parameter_names 参数名列表
 * @param timeout 超时时间
 * @return 参数类型向量
 *
 * @brief Get parameter types
 * @param parameter_names List of parameter names
 * @param timeout Timeout duration
 * @return Vector of parameter types
 */
std::vector<rclcpp::ParameterType> SyncParametersClient::get_parameter_types(
    const std::vector<std::string>& parameter_names, std::chrono::nanoseconds timeout) {
  // 调用异步客户端的 get_parameter_types 方法
  // Call the get_parameter_types method of the asynchronous client
  auto f = async_parameters_client_->get_parameter_types(parameter_names);

  using rclcpp::executors::spin_node_until_future_complete;
  if (spin_node_until_future_complete(*executor_, node_base_interface_, f, timeout) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return f.get();
  }
  return std::vector<rclcpp::ParameterType>();
}

/**
 * @brief 设置参数
 * @param parameters 参数列表
 * @param timeout 超时时间
 * @return 设置结果向量
 *
 * @brief Set parameters
 * @param parameters List of parameters
 * @param timeout Timeout duration
 * @return Vector of set parameter results
 */
std::vector<rcl_interfaces::msg::SetParametersResult> SyncParametersClient::set_parameters(
    const std::vector<rclcpp::Parameter>& parameters, std::chrono::nanoseconds timeout) {
  // 调用异步客户端的 set_parameters 方法
  // Call the set_parameters method of the asynchronous client
  auto f = async_parameters_client_->set_parameters(parameters);

  using rclcpp::executors::spin_node_until_future_complete;
  if (spin_node_until_future_complete(*executor_, node_base_interface_, f, timeout) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return f.get();
  }
  return std::vector<rcl_interfaces::msg::SetParametersResult>();
}

/**
 * @brief 删除参数
 * @param parameters_names 参数名列表
 * @param timeout 超时时间
 * @return 删除结果向量
 *
 * @brief Delete parameters
 * @param parameters_names List of parameter names
 * @param timeout Timeout duration
 * @return Vector of delete parameter results
 */
std::vector<rcl_interfaces::msg::SetParametersResult> SyncParametersClient::delete_parameters(
    const std::vector<std::string>& parameters_names, std::chrono::nanoseconds timeout) {
  // 调用异步客户端的 delete_parameters 方法
  // Call the delete_parameters method of the asynchronous client
  auto f = async_parameters_client_->delete_parameters(parameters_names);

  using rclcpp::executors::spin_node_until_future_complete;
  if (spin_node_until_future_complete(*executor_, node_base_interface_, f, timeout) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    return f.get();
  }
  return std::vector<rcl_interfaces::msg::SetParametersResult>();
}

/**
 * @brief 从指定的YAML文件中加载参数，并同步到节点上。
 *        Load parameters from the specified YAML file and synchronize them to the node.
 *
 * @param yaml_filename YAML文件名。The name of the YAML file.
 * @param timeout 超时时间。Timeout duration.
 * @return 返回一个包含SetParametersResult的向量。Returns a vector containing SetParametersResult.
 */
std::vector<rcl_interfaces::msg::SetParametersResult> SyncParametersClient::load_parameters(
    const std::string& yaml_filename, std::chrono::nanoseconds timeout) {
  // 异步加载参数。Asynchronously load parameters.
  auto f = async_parameters_client_->load_parameters(yaml_filename);

  // 使用rclcpp::executors::spin_node_until_future_complete等待future完成。Wait for the future to
  // complete using rclcpp::executors::spin_node_until_future_complete.
  if (spin_node_until_future_complete(*executor_, node_base_interface_, f, timeout) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    // 如果成功，返回future的结果。If successful, return the result of the future.
    return f.get();
  }
  // 否则，返回一个空的SetParametersResult向量。Otherwise, return an empty vector of
  // SetParametersResult.
  return std::vector<rcl_interfaces::msg::SetParametersResult>();
}

/**
 * @brief 原子性地设置一组参数。
 *        Set a group of parameters atomically.
 *
 * @param parameters 参数向量。A vector of parameters.
 * @param timeout 超时时间。Timeout duration.
 * @return 返回一个SetParametersResult对象。Returns a SetParametersResult object.
 * @throws std::runtime_error 如果无法获取设置参数服务调用结果。If unable to get the result of the
 * set parameters service call.
 */
rcl_interfaces::msg::SetParametersResult SyncParametersClient::set_parameters_atomically(
    const std::vector<rclcpp::Parameter>& parameters, std::chrono::nanoseconds timeout) {
  // 异步原子性地设置参数。Asynchronously set parameters atomically.
  auto f = async_parameters_client_->set_parameters_atomically(parameters);

  // 使用rclcpp::executors::spin_node_until_future_complete等待future完成。Wait for the future to
  // complete using rclcpp::executors::spin_node_until_future_complete.
  if (spin_node_until_future_complete(*executor_, node_base_interface_, f, timeout) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    // 如果成功，返回future的结果。If successful, return the result of the future.
    return f.get();
  }

  // 抛出运行时错误。Throw a runtime error.
  throw std::runtime_error("Unable to get result of set parameters service call.");
}

/**
 * @brief 根据指定的参数前缀和深度列出参数。
 *        List parameters based on the specified parameter prefixes and depth.
 *
 * @param parameter_prefixes 参数前缀向量。A vector of parameter prefixes.
 * @param depth 列表参数的层次深度。The depth of listing parameters.
 * @param timeout 超时时间。Timeout duration.
 * @return 返回一个ListParametersResult对象。Returns a ListParametersResult object.
 * @throws std::runtime_error 如果无法获取列出参数服务调用结果。If unable to get the result of the
 * list parameters service call.
 */
rcl_interfaces::msg::ListParametersResult SyncParametersClient::list_parameters(
    const std::vector<std::string>& parameter_prefixes,
    uint64_t depth,
    std::chrono::nanoseconds timeout) {
  // 异步列出参数。Asynchronously list parameters.
  auto f = async_parameters_client_->list_parameters(parameter_prefixes, depth);

  // 使用rclcpp::executors::spin_node_until_future_complete等待future完成。Wait for the future to
  // complete using rclcpp::executors::spin_node_until_future_complete.
  if (spin_node_until_future_complete(*executor_, node_base_interface_, f, timeout) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    // 如果成功，返回future的结果。If successful, return the result of the future.
    return f.get();
  }

  // 抛出运行时错误。Throw a runtime error.
  throw std::runtime_error("Unable to get result of list parameters service call.");
}

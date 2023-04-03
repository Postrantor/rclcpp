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

#include "rclcpp/parameter_event_handler.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcpputils/join.hpp"

namespace rclcpp {

/**
 * @brief 添加参数事件回调 (Add a parameter event callback)
 *
 * @param[in] callback 参数事件回调类型 (ParameterEventCallbackType)
 * @return ParameterEventCallbackHandle::SharedPtr
 */
ParameterEventCallbackHandle::SharedPtr ParameterEventHandler::add_parameter_event_callback(
    ParameterEventCallbackType callback) {
  // 对互斥锁加锁，保护共享资源 (Lock the mutex to protect shared resources)
  std::lock_guard<std::recursive_mutex> lock(callbacks_->mutex_);

  // 创建一个参数事件回调句柄 (Create a ParameterEventCallbackHandle)
  auto handle = std::make_shared<ParameterEventCallbackHandle>();

  // 设置回调函数 (Set the callback function)
  handle->callback = callback;

  // 将回调句柄添加到事件回调列表的前面 (Add the callback handle to the front of the event_callbacks
  // list)
  callbacks_->event_callbacks_.emplace_front(handle);

  return handle;
}

/**
 * @brief 删除参数事件回调 (Remove a parameter event callback)
 *
 * @param[in] callback_handle 参数事件回调句柄 (ParameterEventCallbackHandle::SharedPtr)
 */
void ParameterEventHandler::remove_parameter_event_callback(
    ParameterEventCallbackHandle::SharedPtr callback_handle) {
  // 对互斥锁加锁，保护共享资源 (Lock the mutex to protect shared resources)
  std::lock_guard<std::recursive_mutex> lock(callbacks_->mutex_);

  // 查找指定的回调句柄 (Find the specified callback handle)
  auto it = std::find_if(
      callbacks_->event_callbacks_.begin(), callbacks_->event_callbacks_.end(),
      [callback_handle](const auto& weak_handle) {
        return callback_handle.get() == weak_handle.lock().get();
      });

  // 如果找到了回调句柄，删除它 (If the callback handle is found, remove it)
  if (it != callbacks_->event_callbacks_.end()) {
    callbacks_->event_callbacks_.erase(it);
  } else {
    throw std::runtime_error("Callback doesn't exist");
  }
}

/**
 * @brief 添加参数回调 (Add a parameter callback)
 *
 * @param[in] parameter_name 参数名称 (Parameter name)
 * @param[in] callback 参数回调类型 (ParameterCallbackType)
 * @param[in] node_name 节点名称 (Node name)
 * @return ParameterCallbackHandle::SharedPtr
 */
ParameterCallbackHandle::SharedPtr ParameterEventHandler::add_parameter_callback(
    const std::string& parameter_name,
    ParameterCallbackType callback,
    const std::string& node_name) {
  // 对互斥锁加锁，保护共享资源 (Lock the mutex to protect shared resources)
  std::lock_guard<std::recursive_mutex> lock(callbacks_->mutex_);

  // 解析节点路径 (Resolve the node path)
  auto full_node_name = resolve_path(node_name);

  // 创建一个参数回调句柄 (Create a ParameterCallbackHandle)
  auto handle = std::make_shared<ParameterCallbackHandle>();

  // 设置回调函数、参数名称和节点名称 (Set the callback function, parameter name, and node name)
  handle->callback = callback;
  handle->parameter_name = parameter_name;
  handle->node_name = full_node_name;

  // 将回调句柄添加到参数回调列表的前面 (Add the callback handle to the front of the
  // parameter_callbacks list) 最后注册的回调将首先执行 (The last registered callback will be
  // executed first)
  callbacks_->parameter_callbacks_[{parameter_name, full_node_name}].emplace_front(handle);

  return handle;
}

/**
 * @brief 删除参数回调 (Remove a parameter callback)
 *
 * @param[in] callback_handle 参数回调句柄 (ParameterCallbackHandle::SharedPtr)
 */
void ParameterEventHandler::remove_parameter_callback(
    ParameterCallbackHandle::SharedPtr callback_handle) {
  // 对互斥锁加锁，保护共享资源 (Lock the mutex to protect shared resources)
  std::lock_guard<std::recursive_mutex> lock(callbacks_->mutex_);

  // 获取回调句柄 (Get the callback handle)
  auto handle = callback_handle.get();

  // 获取回调容器 (Get the callback container)
  auto& container = callbacks_->parameter_callbacks_[{handle->parameter_name, handle->node_name}];

  // 查找指定的回调句柄 (Find the specified callback handle)
  auto it = std::find_if(container.begin(), container.end(), [handle](const auto& weak_handle) {
    return handle == weak_handle.lock().get();
  });

  // 如果找到了回调句柄，删除它 (If the callback handle is found, remove it)
  if (it != container.end()) {
    container.erase(it);
    if (container.empty()) {
      callbacks_->parameter_callbacks_.erase({handle->parameter_name, handle->node_name});
    }
  } else {
    throw std::runtime_error("Callback doesn't exist");
  }
}

/**
 * @brief 从事件中获取参数 (Get parameter from event)
 *
 * @param[in] event 参数事件 (ParameterEvent)
 * @param[out] parameter 输出参数 (Output parameter)
 * @param[in] parameter_name 参数名称 (Parameter name)
 * @param[in] node_name 节点名称 (Node name)
 * @return bool 是否获取成功 (Whether the parameter is obtained successfully)
 */
bool ParameterEventHandler::get_parameter_from_event(
    const rcl_interfaces::msg::ParameterEvent& event,
    rclcpp::Parameter& parameter,
    const std::string& parameter_name,
    const std::string& node_name) {
  // 检查事件节点是否与指定节点匹配 (Check if the event node matches the specified node)
  if (event.node != node_name) {
    return false;
  }

  // 遍历新参数列表，检查参数名称是否匹配 (Iterate through the new_parameters list, checking if the
  // parameter name matches)
  for (auto& new_parameter : event.new_parameters) {
    if (new_parameter.name == parameter_name) {
      parameter = rclcpp::Parameter::from_parameter_msg(new_parameter);
      return true;
    }
  }

  // 遍历已更改的参数列表，检查参数名称是否匹配 (Iterate through the changed_parameters list,
  // checking if the parameter name matches)
  for (auto& changed_parameter : event.changed_parameters) {
    if (changed_parameter.name == parameter_name) {
      parameter = rclcpp::Parameter::from_parameter_msg(changed_parameter);
      return true;
    }
  }

  return false;
}

/**
 * @brief 从参数事件中获取指定参数
 * @param event 参数事件
 * @param parameter_name 参数名
 * @param node_name 节点名
 * @return rclcpp::Parameter 获取到的参数
 *
 * @brief Get the specified parameter from the parameter event
 * @param event The parameter event
 * @param parameter_name The name of the parameter
 * @param node_name The name of the node
 * @return rclcpp::Parameter The obtained parameter
 */
rclcpp::Parameter ParameterEventHandler::get_parameter_from_event(
    const rcl_interfaces::msg::ParameterEvent& event,
    const std::string& parameter_name,
    const std::string& node_name) {
  // 初始化一个 rclcpp::Parameter 对象
  // Initialize an rclcpp::Parameter object
  rclcpp::Parameter p;

  // 如果没有从事件中获取到指定参数，则根据节点名称返回相应结果
  // If the specified parameter is not obtained from the event, return the corresponding result
  // based on the node name
  if (!get_parameter_from_event(event, p, parameter_name, node_name)) {
    // 如果节点名匹配，返回一个未设置的参数对象
    // If the node name matches, return an unset parameter object
    if (event.node == node_name) {
      return rclcpp::Parameter(parameter_name, rclcpp::PARAMETER_NOT_SET);
    } else {
      // 如果节点名不匹配，抛出运行时异常
      // If the node name does not match, throw a runtime exception
      throw std::runtime_error(
          "The node name '" + node_name + "' of parameter '" + parameter_name +
          +"' doesn't match the node name '" + event.node + "' in parameter event");
    }
  }

  // 返回获取到的参数对象
  // Return the obtained parameter object
  return p;
}

/**
 * @brief 从参数事件中获取所有参数
 * @param event 参数事件
 * @return std::vector<rclcpp::Parameter> 获取到的参数列表
 *
 * @brief Get all parameters from the parameter event
 * @param event The parameter event
 * @return std::vector<rclcpp::Parameter> The list of obtained parameters
 */
std::vector<rclcpp::Parameter> ParameterEventHandler::get_parameters_from_event(
    const rcl_interfaces::msg::ParameterEvent& event) {
  // 初始化一个参数列表
  // Initialize a parameter list
  std::vector<rclcpp::Parameter> params;

  // 遍历新参数并添加到参数列表中
  // Iterate through new parameters and add them to the parameter list
  for (auto& new_parameter : event.new_parameters) {
    params.push_back(rclcpp::Parameter::from_parameter_msg(new_parameter));
  }

  // 遍历已更改的参数并添加到参数列表中
  // Iterate through changed parameters and add them to the parameter list
  for (auto& changed_parameter : event.changed_parameters) {
    params.push_back(rclcpp::Parameter::from_parameter_msg(changed_parameter));
  }

  // 返回参数列表
  // Return the parameter list
  return params;
}

/**
 * @brief 参数事件回调函数
 * @param event 参数事件
 *
 * @brief Parameter event callback function
 * @param event The parameter event
 */
void ParameterEventHandler::Callbacks::event_callback(
    const rcl_interfaces::msg::ParameterEvent& event) {
  // 为了线程安全，使用互斥锁
  // Use a mutex for thread safety
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // 遍历参数回调列表并执行相应的回调函数
  // Iterate through the parameter callback list and execute the corresponding callback functions
  for (auto it = parameter_callbacks_.begin(); it != parameter_callbacks_.end(); ++it) {
    rclcpp::Parameter p;
    if (get_parameter_from_event(event, p, it->first.first, it->first.second)) {
      for (auto cb = it->second.begin(); cb != it->second.end(); ++cb) {
        auto shared_handle = cb->lock();
        if (nullptr != shared_handle) {
          shared_handle->callback(p);
        } else {
          cb = it->second.erase(cb);
        }
      }
    }
  }

  // 遍历事件回调列表并执行相应的回调函数
  // Iterate through the event callback list and execute the corresponding callback functions
  for (auto event_cb = event_callbacks_.begin(); event_cb != event_callbacks_.end(); ++event_cb) {
    auto shared_event_handle = event_cb->lock();
    if (nullptr != shared_event_handle) {
      shared_event_handle->callback(event);
    } else {
      event_cb = event_callbacks_.erase(event_cb);
    }
  }
}

/**
 * @brief 解析路径
 * @param path 路径字符串
 * @return std::string 解析后的完整路径
 *
 * @brief Resolve path
 * @param path The path string
 * @return std::string The resolved full path
 */
std::string ParameterEventHandler::resolve_path(const std::string& path) {
  // 初始化一个完整路径字符串
  // Initialize a full path string
  std::string full_path;

  // 如果路径为空，则使用节点的完全限定名称
  // If the path is empty, use the fully qualified name of the node
  if (path == "") {
    full_path = node_base_->get_fully_qualified_name();
  } else {
    // 如果路径不为空，根据路径开始字符是否为 '/' 来拼接命名空间和路径
    // If the path is not empty, concatenate the namespace and path based on whether the starting
    // character of the path is '/'
    full_path = path;
    if (*path.begin() != '/') {
      auto ns = node_base_->get_namespace();
      const std::vector<std::string> paths{ns, path};
      full_path = (ns == std::string("/")) ? ns + path : rcpputils::join(paths, "/");
    }
  }

  // 返回解析后的完整路径
  // Return the resolved full path
  return full_path;
}

}  // namespace rclcpp

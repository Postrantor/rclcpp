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

#include "rclcpp/memory_strategy.hpp"

#include <memory>

using rclcpp::memory_strategy::MemoryStrategy;

/**
 * @brief 获取具有给定订阅者句柄的订阅 (Get the subscription with the given subscriber handle)
 *
 * @param[in] subscriber_handle 订阅者句柄 (Subscriber handle)
 * @param[in] weak_groups_to_nodes 弱回调组到节点映射 (Weak callback group to node map)
 * @return 订阅指针，如果找不到则为 nullptr (Subscription pointer, nullptr if not found)
 */
rclcpp::SubscriptionBase::SharedPtr MemoryStrategy::get_subscription_by_handle(
    const std::shared_ptr<const rcl_subscription_t>& subscriber_handle,
    const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) {
  // 遍历弱回调组到节点映射 (Iterate through the weak callback group to node map)
  for (const auto& pair : weak_groups_to_nodes) {
    // 锁定回调组 (Lock the callback group)
    auto group = pair.first.lock();
    // 如果回调组不存在，则跳过此次循环 (If the callback group doesn't exist, skip this iteration)
    if (!group) {
      continue;
    }
    // 查找匹配的订阅 (Find the matching subscription)
    auto match_subscription = group->find_subscription_ptrs_if(
        [&subscriber_handle](const rclcpp::SubscriptionBase::SharedPtr& subscription) -> bool {
          return subscription->get_subscription_handle() == subscriber_handle;
        });
    // 如果找到匹配的订阅，则返回该订阅 (If a matching subscription is found, return it)
    if (match_subscription) {
      return match_subscription;
    }
  }
  // 如果没有找到匹配的订阅，则返回 nullptr (If no matching subscription is found, return nullptr)
  return nullptr;
}

/**
 * @brief 获取具有给定服务句柄的服务 (Get the service with the given service handle)
 *
 * @param[in] service_handle 服务句柄 (Service handle)
 * @param[in] weak_groups_to_nodes 弱回调组到节点映射 (Weak callback group to node map)
 * @return 服务指针，如果找不到则为 nullptr (Service pointer, nullptr if not found)
 */
rclcpp::ServiceBase::SharedPtr MemoryStrategy::get_service_by_handle(
    const std::shared_ptr<const rcl_service_t>& service_handle,
    const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) {
  // 遍历弱回调组到节点映射 (Iterate through the weak callback group to node map)
  for (const auto& pair : weak_groups_to_nodes) {
    // 锁定回调组 (Lock the callback group)
    auto group = pair.first.lock();
    // 如果回调组不存在，则跳过此次循环 (If the callback group doesn't exist, skip this iteration)
    if (!group) {
      continue;
    }
    // 查找匹配的服务 (Find the matching service)
    auto service_ref = group->find_service_ptrs_if(
        [&service_handle](const rclcpp::ServiceBase::SharedPtr& service) -> bool {
          return service->get_service_handle() == service_handle;
        });
    // 如果找到匹配的服务，则返回该服务 (If a matching service is found, return it)
    if (service_ref) {
      return service_ref;
    }
  }
  // 如果没有找到匹配的服务，则返回 nullptr (If no matching service is found, return nullptr)
  return nullptr;
}

/**
 * @brief 获取具有给定客户端句柄的客户端 (Get the client with the given client handle)
 *
 * @param[in] client_handle 客户端句柄 (Client handle)
 * @param[in] weak_groups_to_nodes 弱回调组到节点映射 (Weak callback group to node map)
 * @return 客户端指针，如果找不到则为 nullptr (Client pointer, nullptr if not found)
 */
rclcpp::ClientBase::SharedPtr MemoryStrategy::get_client_by_handle(
    const std::shared_ptr<const rcl_client_t>& client_handle,
    const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) {
  // 遍历弱回调组到节点映射 (Iterate through the weak callback group to node map)
  for (const auto& pair : weak_groups_to_nodes) {
    // 锁定回调组 (Lock the callback group)
    auto group = pair.first.lock();
    // 如果回调组不存在，则跳过此次循环 (If the callback group doesn't exist, skip this iteration)
    if (!group) {
      continue;
    }
    // 查找匹配的客户端 (Find the matching client)
    auto client_ref = group->find_client_ptrs_if(
        [&client_handle](const rclcpp::ClientBase::SharedPtr& client) -> bool {
          return client->get_client_handle() == client_handle;
        });
    // 如果找到匹配的客户端，则返回该客户端 (If a matching client is found, return it)
    if (client_ref) {
      return client_ref;
    }
  }
  // 如果没有找到匹配的客户端，则返回 nullptr (If no matching client is found, return nullptr)
  return nullptr;
}

/**
 * @brief 获取具有给定计时器句柄的计时器 (Get the timer with the given timer handle)
 *
 * @param[in] timer_handle 计时器句柄 (Timer handle)
 * @param[in] weak_groups_to_nodes 弱回调组到节点映射 (Weak callback group to node map)
 * @return 计时器指针，如果找不到则为 nullptr (Timer pointer, nullptr if not found)
 */
rclcpp::TimerBase::SharedPtr MemoryStrategy::get_timer_by_handle(
    const std::shared_ptr<const rcl_timer_t>& timer_handle,
    const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) {
  // 遍历弱回调组到节点映射 (Iterate through the weak callback group to node map)
  for (const auto& pair : weak_groups_to_nodes) {
    // 锁定回调组 (Lock the callback group)
    auto group = pair.first.lock();
    // 如果回调组不存在，则跳过此次循环 (If the callback group doesn't exist, skip this iteration)
    if (!group) {
      continue;
    }
    // 查找匹配的计时器 (Find the matching timer)
    auto timer_ref = group->find_timer_ptrs_if(
        [&timer_handle](const rclcpp::TimerBase::SharedPtr& timer) -> bool {
          return timer->get_timer_handle() == timer_handle;
        });
    // 如果找到匹配的计时器，则返回该计时器 (If a matching timer is found, return it)
    if (timer_ref) {
      return timer_ref;
    }
  }
  // 如果没有找到匹配的计时器，则返回 nullptr (If no matching timer is found, return nullptr)
  return nullptr;
}

/**
 * @brief 根据回调组获取节点 (Get the node by the callback group)
 *
 * @param[in] group 回调组指针 (Callback group pointer)
 * @param[in] weak_groups_to_nodes 弱回调组到节点映射 (Weak callback group to node map)
 * @return 节点指针，如果找不到则为 nullptr (Node pointer, nullptr if not found)
 */
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MemoryStrategy::get_node_by_group(
    const rclcpp::CallbackGroup::SharedPtr& group,
    const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) {
  // 如果回调组不存在，则返回 nullptr (If the callback group doesn't exist, return nullptr)
  if (!group) {
    return nullptr;
  }

  // 创建弱回调组指针 (Create a weak callback group pointer)
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr(group);
  // 在弱回调组到节点映射中查找节点 (Find the node in the weak callback group to node map)
  const auto finder = weak_groups_to_nodes.find(weak_group_ptr);
  // 如果找到节点，则锁定并返回节点指针 (If the node is found, lock and return the node pointer)
  if (finder != weak_groups_to_nodes.end()) {
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr = finder->second.lock();
    return node_ptr;
  }
  // 如果没有找到节点，则返回 nullptr (If no node is found, return nullptr)
  return nullptr;
}

/**
 * @brief 获取包含给定订阅的回调组 (Get the callback group containing the given subscription)
 * @param subscription 给定的订阅 (The given subscription)
 * @param weak_groups_to_nodes 回调组到节点的映射 (Mapping of callback groups to nodes)
 * @return 包含订阅的回调组智能指针，如果找不到则返回 nullptr (SharedPtr to the callback group
 * containing the subscription, or nullptr if not found)
 */
rclcpp::CallbackGroup::SharedPtr MemoryStrategy::get_group_by_subscription(
    const rclcpp::SubscriptionBase::SharedPtr& subscription,
    const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) {
  // 遍历回调组到节点的映射 (Iterate through the mapping of callback groups to nodes)
  for (const auto& pair : weak_groups_to_nodes) {
    // 尝试获取回调组和节点的智能指针 (Try to get the shared pointers of the callback group and the
    // node)
    auto group = pair.first.lock();
    auto node = pair.second.lock();

    // 如果回调组或节点无效，则跳过此次循环 (If the callback group or the node is invalid, skip this
    // iteration)
    if (!group || !node) {
      continue;
    }

    // 查找匹配的订阅 (Look for the matching subscription)
    auto match_sub = group->find_subscription_ptrs_if(
        [&subscription](const rclcpp::SubscriptionBase::SharedPtr& sub) -> bool {
          return sub == subscription;
        });

    // 如果找到匹配的订阅，返回对应的回调组 (If a matching subscription is found, return the
    // corresponding callback group)
    if (match_sub) {
      return group;
    }
  }

  // 如果没有找到匹配的回调组，返回 nullptr (Return nullptr if no matching callback group is found)
  return nullptr;
}

/**
 * @brief 获取包含给定服务的回调组 (Get the callback group containing the given service)
 * @param service 给定的服务 (The given service)
 * @param weak_groups_to_nodes 回调组到节点的映射 (Mapping of callback groups to nodes)
 * @return 包含服务的回调组智能指针，如果找不到则返回 nullptr (SharedPtr to the callback group
 * containing the service, or nullptr if not found)
 */
rclcpp::CallbackGroup::SharedPtr MemoryStrategy::get_group_by_service(
    const rclcpp::ServiceBase::SharedPtr& service,
    const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) {
  // 遍历回调组到节点的映射 (Iterate through the mapping of callback groups to nodes)
  for (const auto& pair : weak_groups_to_nodes) {
    // 尝试获取回调组和节点的智能指针 (Try to get the shared pointers of the callback group and the
    // node)
    auto group = pair.first.lock();
    auto node = pair.second.lock();

    // 如果回调组或节点无效，则跳过此次循环 (If the callback group or the node is invalid, skip this
    // iteration)
    if (!group || !node) {
      continue;
    }

    // 查找匹配的服务 (Look for the matching service)
    auto service_ref = group->find_service_ptrs_if(
        [&service](const rclcpp::ServiceBase::SharedPtr& serv) -> bool { return serv == service; });

    // 如果找到匹配的服务，返回对应的回调组 (If a matching service is found, return the
    // corresponding callback group)
    if (service_ref) {
      return group;
    }
  }

  // 如果没有找到匹配的回调组，返回 nullptr (Return nullptr if no matching callback group is found)
  return nullptr;
}

/**
 * @brief 获取包含给定客户端的回调组 (Get the callback group containing the given client)
 * @param client 给定的客户端 (The given client)
 * @param weak_groups_to_nodes 回调组到节点的映射 (Mapping of callback groups to nodes)
 * @return 包含客户端的回调组智能指针，如果找不到则返回 nullptr (SharedPtr to the callback group
 * containing the client, or nullptr if not found)
 */
rclcpp::CallbackGroup::SharedPtr MemoryStrategy::get_group_by_client(
    const rclcpp::ClientBase::SharedPtr& client,
    const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) {
  // 遍历回调组到节点的映射 (Iterate through the mapping of callback groups to nodes)
  for (const auto& pair : weak_groups_to_nodes) {
    // 尝试获取回调组和节点的智能指针 (Try to get the shared pointers of the callback group and the
    // node)
    auto group = pair.first.lock();
    auto node = pair.second.lock();

    // 如果回调组或节点无效，则跳过此次循环 (If the callback group or the node is invalid, skip this
    // iteration)
    if (!group || !node) {
      continue;
    }

    // 查找匹配的客户端 (Look for the matching client)
    auto client_ref = group->find_client_ptrs_if(
        [&client](const rclcpp::ClientBase::SharedPtr& cli) -> bool { return cli == client; });

    // 如果找到匹配的客户端，返回对应的回调组 (If a matching client is found, return the
    // corresponding callback group)
    if (client_ref) {
      return group;
    }
  }

  // 如果没有找到匹配的回调组，返回 nullptr (Return nullptr if no matching callback group is found)
  return nullptr;
}

/**
 * @brief 获取包含给定计时器的回调组 (Get the callback group containing the given timer)
 * @param timer 给定的计时器 (The given timer)
 * @param weak_groups_to_nodes 回调组到节点的映射 (Mapping of callback groups to nodes)
 * @return 包含计时器的回调组智能指针，如果找不到则返回 nullptr (SharedPtr to the callback group
 * containing the timer, or nullptr if not found)
 */
rclcpp::CallbackGroup::SharedPtr MemoryStrategy::get_group_by_timer(
    const rclcpp::TimerBase::SharedPtr& timer,
    const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) {
  // 遍历回调组到节点的映射 (Iterate through the mapping of callback groups to nodes)
  for (const auto& pair : weak_groups_to_nodes) {
    // 尝试获取回调组和节点的智能指针 (Try to get the shared pointers of the callback group and the
    // node)
    auto group = pair.first.lock();
    auto node = pair.second.lock();

    // 如果回调组或节点无效，则跳过此次循环 (If the callback group or the node is invalid, skip this
    // iteration)
    if (!group || !node) {
      continue;
    }

    // 查找匹配的计时器 (Look for the matching timer)
    auto timer_ref = group->find_timer_ptrs_if(
        [&timer](const rclcpp::TimerBase::SharedPtr& time) -> bool { return time == timer; });

    // 如果找到匹配的计时器，返回对应的回调组 (If a matching timer is found, return the
    // corresponding callback group)
    if (timer_ref) {
      return group;
    }
  }

  // 如果没有找到匹配的回调组，返回 nullptr (Return nullptr if no matching callback group is found)
  return nullptr;
}

/**
 * @brief 获取包含给定等待对象的回调组 (Get the callback group containing the given waitable object)
 * @param waitable 给定的等待对象 (The given waitable object)
 * @param weak_groups_to_nodes 回调组到节点的映射 (Mapping of callback groups to nodes)
 * @return 包含等待对象的回调组智能指针，如果找不到则返回 nullptr (SharedPtr to the callback group
 * containing the waitable object, or nullptr if not found)
 */
rclcpp::CallbackGroup::SharedPtr MemoryStrategy::get_group_by_waitable(
    const rclcpp::Waitable::SharedPtr& waitable,
    const WeakCallbackGroupsToNodesMap& weak_groups_to_nodes) {
  // 遍历回调组到节点的映射 (Iterate through the mapping of callback groups to nodes)
  for (const auto& pair : weak_groups_to_nodes) {
    // 尝试获取回调组和节点的智能指针 (Try to get the shared pointers of the callback group and the
    // node)
    auto group = pair.first.lock();
    auto node = pair.second.lock();

    // 如果回调组或节点无效，则跳过此次循环 (If the callback group or the node is invalid, skip this
    // iteration)
    if (!group || !node) {
      continue;
    }

    // 查找匹配的等待对象 (Look for the matching waitable object)
    auto waitable_ref = group->find_waitable_ptrs_if(
        [&waitable](const rclcpp::Waitable::SharedPtr& group_waitable) -> bool {
          return group_waitable == waitable;
        });

    // 如果找到匹配的等待对象，返回对应的回调组 (If a matching waitable object is found, return the
    // corresponding callback group)
    if (waitable_ref) {
      return group;
    }
  }

  // 如果没有找到匹配的回调组，返回 nullptr (Return nullptr if no matching callback group is found)
  return nullptr;
}

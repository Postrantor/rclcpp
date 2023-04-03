// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/executors/static_executor_entities_collector.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"
#include "rclcpp/executors/static_single_threaded_executor.hpp"
#include "rclcpp/memory_strategy.hpp"

using rclcpp::executors::StaticExecutorEntitiesCollector;

/// Destructor for StaticExecutorEntitiesCollector.
StaticExecutorEntitiesCollector::~StaticExecutorEntitiesCollector() {
  // 遍历 weak_groups_associated_with_executor_to_nodes_ 中的所有回调组，将它们与执行器解除关联。
  // Iterate through all callback groups in weak_groups_associated_with_executor_to_nodes_,
  // disassociate them with the executor.
  for (const auto &pair : weak_groups_associated_with_executor_to_nodes_) {
    auto group = pair.first.lock();
    if (group) {
      std::atomic_bool &has_executor = group->get_associated_with_executor_atomic();
      has_executor.store(false);
    }
  }
  // 遍历 weak_groups_to_nodes_associated_with_executor_ 中的所有回调组，将它们与执行器解除关联。
  // Iterate through all callback groups in weak_groups_to_nodes_associated_with_executor_,
  // disassociate them with the executor.
  for (const auto &pair : weak_groups_to_nodes_associated_with_executor_) {
    auto group = pair.first.lock();
    if (group) {
      std::atomic_bool &has_executor = group->get_associated_with_executor_atomic();
      has_executor.store(false);
    }
  }
  // 解除所有节点与执行器的关联。
  // Disassociate all nodes with the executor.
  for (const auto &weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node) {
      std::atomic_bool &has_executor = node->get_associated_with_executor_atomic();
      has_executor.store(false);
    }
  }
  // 清空各个容器。
  // Clear the various containers.
  weak_groups_associated_with_executor_to_nodes_.clear();
  weak_groups_to_nodes_associated_with_executor_.clear();
  exec_list_.clear();
  weak_nodes_.clear();
  weak_nodes_to_guard_conditions_.clear();
}

/// Initialize the StaticExecutorEntitiesCollector.
/**
 * @param[in] p_wait_set The pointer to the rcl_wait_set_t structure.
 * @param[in] memory_strategy The shared pointer to the MemoryStrategy object.
 */
void StaticExecutorEntitiesCollector::init(
    rcl_wait_set_t *p_wait_set,
    rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy) {
  // 初始化空的可执行列表。
  // Initialize an empty executable list.
  exec_list_ = rclcpp::experimental::ExecutableList();
  // 获取执行器的 wait_set_ 指针。
  // Get the executor's wait_set_ pointer.
  p_wait_set_ = p_wait_set;
  // 获取执行器的内存策略指针。
  // Get the executor's memory strategy pointer.
  if (memory_strategy == nullptr) {
    throw std::runtime_error("Received NULL memory strategy in executor waitable.");
  }
  memory_strategy_ = memory_strategy;

  // 获取内存策略和可执行列表。准备 wait_set_。
  // Get the memory strategy and executable list. Prepare the wait_set_.
  std::shared_ptr<void> shared_ptr;
  execute(shared_ptr);

  // 实体收集器现在已初始化。
  // The entities collector is now initialized.
  initialized_ = true;
}

/// Finalize the StaticExecutorEntitiesCollector.
void StaticExecutorEntitiesCollector::fini() {
  memory_strategy_->clear_handles();
  exec_list_.clear();
}

/// Take data from the StaticExecutorEntitiesCollector.
/**
 * @return A shared pointer to void.
 */
std::shared_ptr<void> StaticExecutorEntitiesCollector::take_data() { return nullptr; }

/// Execute the StaticExecutorEntitiesCollector.
/**
 * @param[in] data The shared pointer to void.
 */
void StaticExecutorEntitiesCollector::execute(std::shared_ptr<void> &data) {
  (void)data;
  // 使用来自 weak_nodes_ 的实体填充内存策略。
  // Fill the memory strategy with entities coming from weak_nodes_.
  fill_memory_strategy();
  // 使用来自 weak_nodes_ 的实体填充 exec_list_（与内存策略相同）。
  // Fill the exec_list_ with entities coming from weak_nodes_ (same as memory strategy).
  fill_executable_list();
  // 根据内存策略句柄调整 wait_set_ 大小（rcl_wait_set_resize）。
  // Resize the wait_set_ based on memory_strategy handles (rcl_wait_set_resize).
  prepare_wait_set();
  // 将新节点的 guard conditions 添加到映射中。
  // Add new nodes' guard conditions to the map.
  std::lock_guard<std::mutex> guard{new_nodes_mutex_};
  for (const auto &weak_node : new_nodes_) {
    if (auto node_ptr = weak_node.lock()) {
      const auto &gc = node_ptr->get_notify_guard_condition();
      weak_nodes_to_guard_conditions_[node_ptr] = &gc;
    }
  }
  new_nodes_.clear();
}

/// Fill the memory strategy with entities.
void StaticExecutorEntitiesCollector::fill_memory_strategy() {
  memory_strategy_->clear_handles();
  bool has_invalid_weak_groups_or_nodes =
      memory_strategy_->collect_entities(weak_groups_to_nodes_associated_with_executor_);
  // 如果检测到无效节点，请清理它们。
  // Clean up any invalid nodes, if they were detected.
  if (has_invalid_weak_groups_or_nodes) {
    std::vector<rclcpp::CallbackGroup::WeakPtr> invalid_group_ptrs;
    for (const auto &pair : weak_groups_to_nodes_associated_with_executor_) {
      auto &weak_group_ptr = pair.first;
      auto &weak_node_ptr = pair.second;
      if (weak_group_ptr.expired() || weak_node_ptr.expired()) {
        invalid_group_ptrs.push_back(weak_group_ptr);
      }
    }
    std::for_each(
        invalid_group_ptrs.begin(), invalid_group_ptrs.end(),
        [this](rclcpp::CallbackGroup::WeakPtr group_ptr) {
          weak_groups_to_nodes_associated_with_executor_.erase(group_ptr);
        });
  }
  has_invalid_weak_groups_or_nodes =
      memory_strategy_->collect_entities(weak_groups_associated_with_executor_to_nodes_);
  // 如果检测到无效节点，请清理它们。
  // Clean up any invalid nodes, if they were detected.
  if (has_invalid_weak_groups_or_nodes) {
    std::vector<rclcpp::CallbackGroup::WeakPtr> invalid_group_ptrs;
    for (const auto &pair : weak_groups_associated_with_executor_to_nodes_) {
      auto &weak_group_ptr = pair.first;
      const auto &weak_node_ptr = pair.second;
      if (weak_group_ptr.expired() || weak_node_ptr.expired()) {
        invalid_group_ptrs.push_back(weak_group_ptr);
      }
    }
    std::for_each(
        invalid_group_ptrs.begin(), invalid_group_ptrs.end(),
        [this](rclcpp::CallbackGroup::WeakPtr group_ptr) {
          weak_groups_associated_with_executor_to_nodes_.erase(group_ptr);
        });
  }

  // 将静态执行器 waitable 添加到内存策略中。
  // Add the static executor waitable to the memory strategy.
  memory_strategy_->add_waitable_handle(this->shared_from_this());
}

/*!
 * \brief 填充可执行实体列表。
 *
 * Fill the executable list with entities.
 */
void StaticExecutorEntitiesCollector::fill_executable_list() {
  // 清空 exec_list_
  // Clear the exec_list_
  exec_list_.clear();

  // 将与 executor 关联的节点的回调组添加到 exec_list_ 中
  // Add callback groups from nodes associated with the executor to the exec_list_
  add_callback_groups_from_nodes_associated_to_executor();

  // 使用 weak_groups_associated_with_executor_to_nodes_ 填充 exec_list_
  // Fill the exec_list_ using weak_groups_associated_with_executor_to_nodes_
  fill_executable_list_from_map(weak_groups_associated_with_executor_to_nodes_);

  // 使用 weak_groups_to_nodes_associated_with_executor_ 填充 exec_list_
  // Fill the exec_list_ using weak_groups_to_nodes_associated_with_executor_
  fill_executable_list_from_map(weak_groups_to_nodes_associated_with_executor_);

  // 将 executor 的 waitable 添加到可执行列表中
  // Add the executor's waitable to the executable list
  exec_list_.add_waitable(shared_from_this());
}

/*!
 * \brief 从映射中填充可执行实体列表。
 * \param weak_groups_to_nodes 映射，包含弱指针回调组和节点。
 *
 * Fill the executable list from a map.
 * \param weak_groups_to_nodes Map containing weak pointers of callback groups and nodes.
 */
void StaticExecutorEntitiesCollector::fill_executable_list_from_map(
    const rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap
        &weak_groups_to_nodes) {
  // 遍历 weak_groups_to_nodes
  // Iterate through weak_groups_to_nodes
  for (const auto &pair : weak_groups_to_nodes) {
    auto group = pair.first.lock();
    auto node = pair.second.lock();

    // 如果节点、组不存在或组不能被获取，跳过此次循环
    // If the node or group does not exist or the group cannot be taken from, skip this iteration
    if (!node || !group || !group->can_be_taken_from().load()) {
      continue;
    }

    // 添加定时器到 exec_list_
    // Add timers to the exec_list_
    group->find_timer_ptrs_if([this](const rclcpp::TimerBase::SharedPtr &timer) {
      if (timer) {
        exec_list_.add_timer(timer);
      }
      return false;
    });

    // 添加订阅到 exec_list_
    // Add subscriptions to the exec_list_
    group->find_subscription_ptrs_if(
        [this](const rclcpp::SubscriptionBase::SharedPtr &subscription) {
          if (subscription) {
            exec_list_.add_subscription(subscription);
          }
          return false;
        });

    // 添加服务到 exec_list_
    // Add services to the exec_list_
    group->find_service_ptrs_if([this](const rclcpp::ServiceBase::SharedPtr &service) {
      if (service) {
        exec_list_.add_service(service);
      }
      return false;
    });

    // 添加客户端到 exec_list_
    // Add clients to the exec_list_
    group->find_client_ptrs_if([this](const rclcpp::ClientBase::SharedPtr &client) {
      if (client) {
        exec_list_.add_client(client);
      }
      return false;
    });

    // 添加 waitable 到 exec_list_
    // Add waitables to the exec_list_
    group->find_waitable_ptrs_if([this](const rclcpp::Waitable::SharedPtr &waitable) {
      if (waitable) {
        exec_list_.add_waitable(waitable);
      }
      return false;
    });
  }
}

/*!
 * \brief 准备等待集。
 *
 * Prepare the wait set.
 */
void StaticExecutorEntitiesCollector::prepare_wait_set() {
  // 清空等待集
  // Clear the wait set
  if (rcl_wait_set_clear(p_wait_set_) != RCL_RET_OK) {
    throw std::runtime_error("Couldn't clear wait set");
  }

  // 调整等待集的大小，考虑到 waitables 的大小
  // Resize the wait set, accounting for the size of waitables
  rcl_ret_t ret = rcl_wait_set_resize(
      p_wait_set_, memory_strategy_->number_of_ready_subscriptions(),
      memory_strategy_->number_of_guard_conditions(), memory_strategy_->number_of_ready_timers(),
      memory_strategy_->number_of_ready_clients(), memory_strategy_->number_of_ready_services(),
      memory_strategy_->number_of_ready_events());

  if (RCL_RET_OK != ret) {
    throw std::runtime_error(
        std::string("Couldn't resize the wait set: ") + rcl_get_error_string().str);
  }
}

/*!
 * \brief 刷新等待集。
 * \param timeout 等待超时时间。
 *
 * Refresh the wait set.
 * \param timeout The waiting timeout duration.
 */
void StaticExecutorEntitiesCollector::refresh_wait_set(std::chrono::nanoseconds timeout) {
  // 清空等待集（将所有 wait_set_ 实体设置为 '0'，但保留 wait_set_ 中的实体数量）
  // Clear the wait set (set all wait_set_ entities to '0', but keep the number of entities in
  // wait_set_)
  if (rcl_wait_set_clear(p_wait_set_) != RCL_RET_OK) {
    throw std::runtime_error("Couldn't clear wait set");
  }

  // 将句柄添加到等待集中
  // Add handles to the wait set
  if (!memory_strategy_->add_handles_to_wait_set(p_wait_set_)) {
    throw std::runtime_error("Couldn't fill wait set");
  }

  // 等待实体准备就绪
  // Wait for entities to be ready
  rcl_ret_t status =
      rcl_wait(p_wait_set_, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count());

  if (status == RCL_RET_WAIT_SET_EMPTY) {
    RCUTILS_LOG_WARN_NAMED(
        "rclcpp", "empty wait set received in rcl_wait(). This should never happen.");
  } else if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(status, "rcl_wait() failed");
  }
}

/**
 * @brief 添加实体到等待集 (Add entities to the wait set)
 *
 * @param wait_set 等待集指针 (Pointer to the wait set)
 */
void StaticExecutorEntitiesCollector::add_to_wait_set(rcl_wait_set_t *wait_set) {
  // 将可等待的守护条件（每个已注册节点一个）添加到等待集中 (Add waitable guard conditions (one for
  // each registered node) into the wait set.)
  for (const auto &pair : weak_nodes_to_guard_conditions_) {
    auto &gc = pair.second;
    detail::add_guard_condition_to_rcl_wait_set(*wait_set, *gc);
  }
}

/**
 * @brief 获取已准备好的守护条件数量 (Get the number of ready guard conditions)
 *
 * @return 已准备好的守护条件数量 (Number of ready guard conditions)
 */
size_t StaticExecutorEntitiesCollector::get_number_of_ready_guard_conditions() {
  std::lock_guard<std::mutex> guard{new_nodes_mutex_};
  return weak_nodes_to_guard_conditions_.size() + new_nodes_.size();
}

/**
 * @brief 添加节点 (Add a node)
 *
 * @param node_ptr 节点共享指针 (Shared pointer to the node)
 * @return 是否为新节点 (Whether it is a new node)
 */
bool StaticExecutorEntitiesCollector::add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr) {
  bool is_new_node = false;
  // 如果节点已经有执行器 (If the node already has an executor)
  std::atomic_bool &has_executor = node_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Node has already been added to an executor.");
  }
  node_ptr->for_each_callback_group(
      [this, node_ptr, &is_new_node](rclcpp::CallbackGroup::SharedPtr group_ptr) {
        if (!group_ptr->get_associated_with_executor_atomic().load() &&
            group_ptr->automatically_add_to_executor_with_node()) {
          is_new_node =
              (add_callback_group(
                   group_ptr, node_ptr, weak_groups_to_nodes_associated_with_executor_) ||
               is_new_node);
        }
      });
  weak_nodes_.push_back(node_ptr);
  return is_new_node;
}

/**
 * @brief 添加回调组 (Add a callback group)
 *
 * @param group_ptr 回调组共享指针 (Shared pointer to the callback group)
 * @param node_ptr 节点共享指针 (Shared pointer to the node)
 * @param weak_groups_to_nodes 弱回调组到节点映射 (Weak callback groups to nodes map)
 * @return 是否为新节点 (Whether it is a new node)
 */
bool StaticExecutorEntitiesCollector::add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap &weak_groups_to_nodes) {
  // 如果回调组已经有执行器 (If the callback_group already has an executor)
  std::atomic_bool &has_executor = group_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Callback group has already been added to an executor.");
  }
  bool is_new_node = !has_node(node_ptr, weak_groups_associated_with_executor_to_nodes_) &&
                     !has_node(node_ptr, weak_groups_to_nodes_associated_with_executor_);
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;
  auto insert_info = weak_groups_to_nodes.insert(std::make_pair(weak_group_ptr, node_ptr));
  bool was_inserted = insert_info.second;
  if (!was_inserted) {
    throw std::runtime_error("Callback group was already added to executor.");
  }
  if (is_new_node) {
    std::lock_guard<std::mutex> guard{new_nodes_mutex_};
    new_nodes_.push_back(node_ptr);
    return true;
  }
  return false;
}

/**
 * @brief 添加回调组 (Add a callback group)
 *
 * @param group_ptr 回调组共享指针 (Shared pointer to the callback group)
 * @param node_ptr 节点共享指针 (Shared pointer to the node)
 * @return 是否为新节点 (Whether it is a new node)
 */
bool StaticExecutorEntitiesCollector::add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr) {
  return add_callback_group(group_ptr, node_ptr, weak_groups_associated_with_executor_to_nodes_);
}

/**
 * @brief 移除回调组 (Remove a callback group)
 *
 * @param group_ptr 回调组共享指针 (Shared pointer to the callback group)
 * @return 是否成功移除 (Whether removed successfully)
 */
bool StaticExecutorEntitiesCollector::remove_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr) {
  return this->remove_callback_group_from_map(
      group_ptr, weak_groups_associated_with_executor_to_nodes_);
}

/**
 * @brief 从映射中移除回调组 (Remove a callback group from the map)
 *
 * @param group_ptr 回调组共享指针 (Shared pointer to the callback group)
 * @param weak_groups_to_nodes 弱回调组到节点映射 (Weak callback groups to nodes map)
 * @return 是否成功移除 (Whether removed successfully)
 */
bool StaticExecutorEntitiesCollector::remove_callback_group_from_map(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap &weak_groups_to_nodes) {
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr;
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr = group_ptr;
  auto iter = weak_groups_to_nodes.find(weak_group_ptr);
  if (iter != weak_groups_to_nodes.end()) {
    node_ptr = iter->second.lock();
    if (node_ptr == nullptr) {
      throw std::runtime_error("Node must not be deleted before its callback group(s).");
    }
    weak_groups_to_nodes.erase(iter);
  } else {
    throw std::runtime_error("Callback group needs to be associated with executor.");
  }
  // 如果节点被匹配并移除，中断等待 (If the node was matched and removed, interrupt waiting.)
  if (!has_node(node_ptr, weak_groups_associated_with_executor_to_nodes_) &&
      !has_node(node_ptr, weak_groups_to_nodes_associated_with_executor_)) {
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr node_weak_ptr(node_ptr);
    weak_nodes_to_guard_conditions_.erase(node_weak_ptr);
    return true;
  }
  return false;
}

/*!
 * \brief 删除节点
 * \param node_ptr 要删除的节点的共享指针
 * \return 如果成功删除节点，则返回 true，否则返回 false
 *
 * \brief Remove a node
 * \param node_ptr Shared pointer of the node to be removed
 * \return Returns true if the node is successfully removed, otherwise returns false
 */
bool StaticExecutorEntitiesCollector::remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr) {
  // 检查节点是否与执行器关联
  // Check if the node is associated with the executor
  if (!node_ptr->get_associated_with_executor_atomic().load()) {
    return false;
  }
  bool node_found = false;
  auto node_it = weak_nodes_.begin();
  // 遍历弱节点列表，查找要删除的节点
  // Iterate through the weak nodes list to find the node to be removed
  while (node_it != weak_nodes_.end()) {
    bool matched = (node_it->lock() == node_ptr);
    if (matched) {
      weak_nodes_.erase(node_it);
      node_found = true;
      break;
    }
    ++node_it;
  }
  if (!node_found) {
    return false;
  }
  std::vector<rclcpp::CallbackGroup::SharedPtr> found_group_ptrs;
  // 查找与要删除的节点关联的回调组
  // Find callback groups associated with the node to be removed
  std::for_each(
      weak_groups_to_nodes_associated_with_executor_.begin(),
      weak_groups_to_nodes_associated_with_executor_.end(),
      [&found_group_ptrs, node_ptr](
          std::pair<
              rclcpp::CallbackGroup::WeakPtr, rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>
              key_value_pair) {
        auto &weak_node_ptr = key_value_pair.second;
        auto shared_node_ptr = weak_node_ptr.lock();
        auto group_ptr = key_value_pair.first.lock();
        if (shared_node_ptr == node_ptr) {
          found_group_ptrs.push_back(group_ptr);
        }
      });
  // 从映射中移除找到的回调组
  // Remove the found callback groups from the map
  std::for_each(
      found_group_ptrs.begin(), found_group_ptrs.end(),
      [this](rclcpp::CallbackGroup::SharedPtr group_ptr) {
        this->remove_callback_group_from_map(
            group_ptr, weak_groups_to_nodes_associated_with_executor_);
      });
  // 将节点与执行器关联设置为 false
  // Set the association of the node with the executor to false
  std::atomic_bool &has_executor = node_ptr->get_associated_with_executor_atomic();
  has_executor.store(false);
  return true;
}

/*!
 * \brief 检查实体是否准备好
 * \param p_wait_set 等待集指针
 * \return 如果实体已准备好，则返回 true，否则返回 false
 *
 * \brief Check if entities are ready
 * \param p_wait_set Pointer to the wait set
 * \return Returns true if the entities are ready, otherwise returns false
 */
bool StaticExecutorEntitiesCollector::is_ready(rcl_wait_set_t *p_wait_set) {
  // 检查等待集合的守护条件，以获取添加/删除节点的实体
  // Check wait_set guard_conditions for added/removed entities to/from a node
  for (size_t i = 0; i < p_wait_set->size_of_guard_conditions; ++i) {
    if (p_wait_set->guard_conditions[i] != NULL) {
      auto found_guard_condition = std::find_if(
          weak_nodes_to_guard_conditions_.begin(), weak_nodes_to_guard_conditions_.end(),
          [&](std::pair<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr, const GuardCondition *>
                  pair) -> bool {
            const rcl_guard_condition_t &rcl_gc = pair.second->get_rcl_guard_condition();
            return &rcl_gc == p_wait_set->guard_conditions[i];
          });
      // 如果找到守护条件，则返回 true
      // If the guard condition is found, return true
      if (found_guard_condition != weak_nodes_to_guard_conditions_.end()) {
        return true;
      }
    }
  }
  // 触发的守护条件都不属于已注册的节点
  // None of the guard conditions triggered belong to a registered node
  return false;
}

/**
 * @brief 检查给定的 weak_groups_to_nodes 映射中是否有 node_ptr 作为其条目值。
 *        Check if the given weak_groups_to_nodes map has node_ptr as the value in any of its entry.
 *
 * @param[in] node_ptr 要查找的节点智能指针。The shared pointer of the node to search for.
 * @param[in] weak_groups_to_nodes 要在其中查找节点的映射。The map in which to search for the node.
 * @return 如果找到节点，则返回 true，否则返回 false。Return true if the node is found, false
 * otherwise.
 */
bool StaticExecutorEntitiesCollector::has_node(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
    const rclcpp::memory_strategy::MemoryStrategy::WeakCallbackGroupsToNodesMap
        &weak_groups_to_nodes) const {
  // 使用 std::find_if 查找与 node_ptr 匹配的节点，并检查是否找到了该节点。
  // Use std::find_if to find a node that matches node_ptr and check if it's found.
  return std::find_if(
             weak_groups_to_nodes.begin(), weak_groups_to_nodes.end(),
             [&](const WeakCallbackGroupsToNodesMap::value_type &other) -> bool {
               auto other_ptr = other.second.lock();
               return other_ptr == node_ptr;
             }) != weak_groups_to_nodes.end();
}

/**
 * @brief 从与执行器关联的节点添加回调组。
 *        Add callback groups from nodes associated with the executor.
 */
void StaticExecutorEntitiesCollector::add_callback_groups_from_nodes_associated_to_executor() {
  // 遍历所有弱节点。
  // Iterate through all weak nodes.
  for (const auto &weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node) {
      // 对与节点关联的每个回调组执行以下操作。
      // Perform the following operation for each callback group associated with the node.
      node->for_each_callback_group(
          [this, node](rclcpp::CallbackGroup::SharedPtr shared_group_ptr) {
            // 如果回调组自动添加到与节点关联的执行器，并且尚未与执行器关联，则添加回调组。
            // Add the callback group if it's automatically added to the executor associated with
            // the node and not yet associated with the executor.
            if (shared_group_ptr->automatically_add_to_executor_with_node() &&
                !shared_group_ptr->get_associated_with_executor_atomic().load()) {
              add_callback_group(
                  shared_group_ptr, node, weak_groups_to_nodes_associated_with_executor_);
            }
          });
    }
  }
}

/**
 * @brief 获取所有回调组（手动和自动添加）。
 *        Get all callback groups (manually and automatically added).
 *
 * @return 包含所有回调组的弱指针的向量。A vector of weak pointers containing all callback groups.
 */
std::vector<rclcpp::CallbackGroup::WeakPtr>
StaticExecutorEntitiesCollector::get_all_callback_groups() {
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  // 添加手动添加的回调组。
  // Add manually added callback groups.
  for (const auto &group_node_ptr : weak_groups_associated_with_executor_to_nodes_) {
    groups.push_back(group_node_ptr.first);
  }
  // 添加自动从节点添加的回调组。
  // Add automatically added callback groups from nodes.
  for (const auto &group_node_ptr : weak_groups_to_nodes_associated_with_executor_) {
    groups.push_back(group_node_ptr.first);
  }
  return groups;
}

/**
 * @brief 获取手动添加的回调组。
 *        Get manually added callback groups.
 *
 * @return 包含手动添加的回调组的弱指针的向量。A vector of weak pointers containing manually added
 * callback groups.
 */
std::vector<rclcpp::CallbackGroup::WeakPtr>
StaticExecutorEntitiesCollector::get_manually_added_callback_groups() {
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  // 添加手动添加的回调组。
  // Add manually added callback groups.
  for (const auto &group_node_ptr : weak_groups_associated_with_executor_to_nodes_) {
    groups.push_back(group_node_ptr.first);
  }
  return groups;
}

/**
 * @brief 从节点获取自动添加的回调组。
 *        Get automatically added callback groups from nodes.
 *
 * @return 包含从节点自动添加的回调组的弱指针的向量。A vector of weak pointers containing
 * automatically added callback groups from nodes.
 */
std::vector<rclcpp::CallbackGroup::WeakPtr>
StaticExecutorEntitiesCollector::get_automatically_added_callback_groups_from_nodes() {
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  // 添加自动从节点添加的回调组。
  // Add automatically added callback groups from nodes.
  for (const auto &group_node_ptr : weak_groups_to_nodes_associated_with_executor_) {
    groups.push_back(group_node_ptr.first);
  }
  return groups;
}

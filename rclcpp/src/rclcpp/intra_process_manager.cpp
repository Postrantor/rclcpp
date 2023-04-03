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

#include "rclcpp/experimental/intra_process_manager.hpp"

#include <atomic>
#include <memory>
#include <mutex>

namespace rclcpp {
namespace experimental {

// 静态原子变量，用于生成唯一ID
// Static atomic variable, used for generating unique IDs
static std::atomic<uint64_t> _next_unique_id{1};

// 构造函数
// Constructor
IntraProcessManager::IntraProcessManager() {}

// 析构函数
// Destructor
IntraProcessManager::~IntraProcessManager() {}

/**
 * @brief 添加发布者
 * @param publisher 要添加的发布者共享指针
 * @return 返回分配给发布者的唯一ID
 *
 * @brief Add a publisher
 * @param publisher Shared pointer of the publisher to be added
 * @return Returns the unique ID assigned to the publisher
 */
uint64_t IntraProcessManager::add_publisher(rclcpp::PublisherBase::SharedPtr publisher) {
  // 获取互斥锁
  // Acquire the mutex lock
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  // 获取下一个唯一ID
  // Get the next unique ID
  uint64_t pub_id = IntraProcessManager::get_next_unique_id();

  // 将发布者添加到发布者列表中
  // Add the publisher to the publishers list
  publishers_[pub_id] = publisher;

  // 初始化此发布者的订阅存储
  // Initialize the subscriptions storage for this publisher
  pub_to_subs_[pub_id] = SplittedSubscriptions();

  // 为发布者ID创建条目，并使用已经存在的订阅填充
  // Create an entry for the publisher ID and populate with already existing subscriptions
  for (auto& pair : subscriptions_) {
    auto subscription = pair.second.lock();
    if (!subscription) {
      continue;
    }
    if (can_communicate(publisher, subscription)) {
      uint64_t sub_id = pair.first;
      insert_sub_id_for_pub(sub_id, pub_id, subscription->use_take_shared_method());
    }
  }

  return pub_id;
}

/**
 * @brief 添加订阅
 * @param subscription 要添加的订阅共享指针
 * @return 返回分配给订阅的唯一ID
 *
 * @brief Add a subscription
 * @param subscription Shared pointer of the subscription to be added
 * @return Returns the unique ID assigned to the subscription
 */
uint64_t IntraProcessManager::add_subscription(
    SubscriptionIntraProcessBase::SharedPtr subscription) {
  // 获取互斥锁
  // Acquire the mutex lock
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  // 获取下一个唯一ID
  // Get the next unique ID
  uint64_t sub_id = IntraProcessManager::get_next_unique_id();

  // 将订阅添加到订阅列表中
  // Add the subscription to the subscriptions list
  subscriptions_[sub_id] = subscription;

  // 将订阅ID添加到所有可匹配的发布者中
  // Add the subscription ID to all the matchable publishers
  for (auto& pair : publishers_) {
    auto publisher = pair.second.lock();
    if (!publisher) {
      continue;
    }
    if (can_communicate(publisher, subscription)) {
      uint64_t pub_id = pair.first;
      insert_sub_id_for_pub(sub_id, pub_id, subscription->use_take_shared_method());
    }
  }

  return sub_id;
}

/**
 * @brief 删除订阅者 (Remove a subscription)
 *
 * @param[in] intra_process_subscription_id 内部进程订阅者ID (Intra-process subscription ID)
 */
void IntraProcessManager::remove_subscription(uint64_t intra_process_subscription_id) {
  // 获取互斥锁 (Acquire the mutex lock)
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  // 从订阅者列表中删除指定的订阅者 (Remove the specified subscription from the subscriptions list)
  subscriptions_.erase(intra_process_subscription_id);

  // 遍历发布者到订阅者的映射 (Iterate through the publisher to subscriber mapping)
  for (auto& pair : pub_to_subs_) {
    // 删除共享订阅者 (Remove shared subscriptions)
    pair.second.take_shared_subscriptions.erase(
        std::remove(
            pair.second.take_shared_subscriptions.begin(),
            pair.second.take_shared_subscriptions.end(), intra_process_subscription_id),
        pair.second.take_shared_subscriptions.end());

    // 删除独占订阅者 (Remove exclusive subscriptions)
    pair.second.take_ownership_subscriptions.erase(
        std::remove(
            pair.second.take_ownership_subscriptions.begin(),
            pair.second.take_ownership_subscriptions.end(), intra_process_subscription_id),
        pair.second.take_ownership_subscriptions.end());
  }
}

/**
 * @brief 删除发布者 (Remove a publisher)
 *
 * @param[in] intra_process_publisher_id 内部进程发布者ID (Intra-process publisher ID)
 */
void IntraProcessManager::remove_publisher(uint64_t intra_process_publisher_id) {
  // 获取互斥锁 (Acquire the mutex lock)
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  // 从发布者列表中删除指定的发布者 (Remove the specified publisher from the publishers list)
  publishers_.erase(intra_process_publisher_id);
  // 从发布者到订阅者的映射中删除指定的发布者 (Remove the specified publisher from the publisher to
  // subscriber mapping)
  pub_to_subs_.erase(intra_process_publisher_id);
}

/**
 * @brief 检查给定的ID是否与任何发布者匹配 (Check if the given ID matches any publishers)
 *
 * @param[in] id 要检查的ID (The ID to check)
 * @return 如果匹配任何发布者，则返回true，否则返回false (Returns true if it matches any publishers,
 * false otherwise)
 */
bool IntraProcessManager::matches_any_publishers(const rmw_gid_t* id) const {
  // 获取共享锁 (Acquire the shared lock)
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);

  // 遍历发布者列表 (Iterate through the publishers list)
  for (auto& publisher_pair : publishers_) {
    auto publisher = publisher_pair.second.lock();
    if (!publisher) {
      continue;
    }
    // 如果找到匹配的发布者，返回true (If a matching publisher is found, return true)
    if (*publisher.get() == id) {
      return true;
    }
  }
  // 如果没有找到匹配的发布者，返回false (Return false if no matching publisher is found)
  return false;
}

/**
 * @brief 获取指定发布者的订阅者数量 (Get the number of subscribers for the specified publisher)
 *
 * @param[in] intra_process_publisher_id 内部进程发布者ID (Intra-process publisher ID)
 * @return 订阅者数量 (Number of subscribers)
 */
size_t IntraProcessManager::get_subscription_count(uint64_t intra_process_publisher_id) const {
  // 获取共享锁 (Acquire the shared lock)
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);

  // 查找指定的发布者 (Find the specified publisher)
  auto publisher_it = pub_to_subs_.find(intra_process_publisher_id);
  if (publisher_it == pub_to_subs_.end()) {
    // 发布者无效或不存在 (Publisher is invalid or no longer exists)
    RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Calling get_subscription_count for invalid or no longer existing publisher id");
    return 0;
  }

  // 计算共享订阅者和独占订阅者的总数 (Calculate the total number of shared and exclusive
  // subscribers)
  auto count = publisher_it->second.take_shared_subscriptions.size() +
               publisher_it->second.take_ownership_subscriptions.size();

  return count;
}

/**
 * @brief 获取内部进程订阅对象 (Get the intra-process subscription object)
 *
 * @param[in] intra_process_subscription_id 内部进程订阅ID (Intra-process subscription ID)
 * @return SubscriptionIntraProcessBase::SharedPtr 返回内部进程订阅共享指针 (Return the shared
 * pointer of the intra-process subscription)
 */
SubscriptionIntraProcessBase::SharedPtr IntraProcessManager::get_subscription_intra_process(
    uint64_t intra_process_subscription_id) {
  // 对互斥量进行共享锁定 (Shared lock on the mutex)
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);

  // 查找订阅ID (Find the subscription ID)
  auto subscription_it = subscriptions_.find(intra_process_subscription_id);
  if (subscription_it == subscriptions_.end()) {
    // 如果未找到，返回空指针 (If not found, return nullptr)
    return nullptr;
  } else {
    // 如果找到，尝试获取订阅对象的弱引用 (If found, try to get the weak reference of the
    // subscription object)
    auto subscription = subscription_it->second.lock();
    if (subscription) {
      // 如果成功获取弱引用，返回订阅对象 (If successfully obtained the weak reference, return the
      // subscription object)
      return subscription;
    } else {
      // 否则，从订阅列表中删除该订阅ID，并返回空指针 (Otherwise, remove the subscription ID from
      // the subscription list and return nullptr)
      subscriptions_.erase(subscription_it);
      return nullptr;
    }
  }
}

/**
 * @brief 获取下一个唯一ID (Get the next unique ID)
 *
 * @return uint64_t 返回下一个唯一ID (Return the next unique ID)
 */
uint64_t IntraProcessManager::get_next_unique_id() {
  // 获取并递增下一个唯一ID (Get and increment the next unique ID)
  auto next_id = _next_unique_id.fetch_add(1, std::memory_order_relaxed);
  // 检查翻转 (Check for rollover)
  if (0 == next_id) {
    // 抛出溢出错误 (Throw an overflow error)
    throw std::overflow_error(
        "exhausted the unique id's for publishers and subscribers in this process "
        "(congratulations your computer is either extremely fast or extremely old)");
  }
  return next_id;
}

/**
 * @brief 为发布者插入订阅ID (Insert subscription ID for publisher)
 *
 * @param[in] sub_id 订阅ID (Subscription ID)
 * @param[in] pub_id 发布者ID (Publisher ID)
 * @param[in] use_take_shared_method 是否使用共享方法 (Whether to use the shared method)
 */
void IntraProcessManager::insert_sub_id_for_pub(
    uint64_t sub_id, uint64_t pub_id, bool use_take_shared_method) {
  if (use_take_shared_method) {
    // 使用共享方法，将订阅ID添加到 take_shared_subscriptions 列表中 (Use the shared method, add the
    // subscription ID to the take_shared_subscriptions list)
    pub_to_subs_[pub_id].take_shared_subscriptions.push_back(sub_id);
  } else {
    // 使用所有权方法，将订阅ID添加到 take_ownership_subscriptions 列表中 (Use the ownership method,
    // add the subscription ID to the take_ownership_subscriptions list)
    pub_to_subs_[pub_id].take_ownership_subscriptions.push_back(sub_id);
  }
}

/**
 * @brief 检查发布者和订阅者是否可以通信 (Check if the publisher and subscriber can communicate)
 *
 * @param[in] pub 发布者共享指针 (Publisher shared pointer)
 * @param[in] sub 订阅者共享指针 (Subscriber shared pointer)
 * @return bool 返回是否可以通信 (Return whether they can communicate)
 */
bool IntraProcessManager::can_communicate(
    rclcpp::PublisherBase::SharedPtr pub,
    rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr sub) const {
  // 发布者和订阅者必须在同一个主题上 (Publisher and subscriber must be on the same topic)
  if (strcmp(pub->get_topic_name(), sub->get_topic_name()) != 0) {
    return false;
  }

  // 检查QoS兼容性 (Check QoS compatibility)
  auto check_result = rclcpp::qos_check_compatible(pub->get_actual_qos(), sub->get_actual_qos());
  if (check_result.compatibility == rclcpp::QoSCompatibility::Error) {
    return false;
  }

  return true;
}

}  // namespace experimental
}  // namespace rclcpp

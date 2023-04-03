// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXPERIMENTAL__INTRA_PROCESS_MANAGER_HPP_
#define RCLCPP__EXPERIMENTAL__INTRA_PROCESS_MANAGER_HPP_

#include <rmw/types.h>

#include <iterator>
#include <memory>
#include <shared_mutex>
#include <stdexcept>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/experimental/ros_message_intra_process_buffer.hpp"
#include "rclcpp/experimental/subscription_intra_process.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "rclcpp/experimental/subscription_intra_process_buffer.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/type_adapter.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp {

namespace experimental {

/// 这个类执行节点之间的内部进程通信。 (This class performs intra process communication between
/// nodes.)
/**
 * 在创建发布者和订阅者时，将使用此类。
 * rclcpp::Context 拥有此类的单例实例，rclcpp::Node 可以使用关联的 Context 获取此类的实例。
 * 没有共同 Context 的节点不会交换内部进程消息，因为它们无法共享对此类的相同实例的访问。
 * (This class is used in the creation of publishers and subscriptions.
 * A singleton instance of this class is owned by a rclcpp::Context and a
 * rclcpp::Node can use an associated Context to get an instance of this class.
 * Nodes which do not have a common Context will not exchange intra process
 * messages because they do not share access to the same instance of this class.)
 *
 * 当 Node 创建订阅时，它还可以创建一个辅助类（称为
 * SubscriptionIntraProcess），用于接收内部进程消息。 它可以在此类中注册。 它还分配了一个 ID，该 ID
 * 在此进程中的所有发布者和订阅者之间是唯一的，并且与订阅相关联。 (When a Node creates a
 * subscription, it can also create a helper class, called SubscriptionIntraProcess, meant to
 * receive intra process messages. It can be registered with this class. It is also allocated an id
 * which is unique among all publishers and subscriptions in this process and that is associated to
 * the subscription.)
 *
 * 当 Node 创建发布者时，与订阅一样，辅助类可以在此类中注册。
 * 这是发布内部进程消息所必需的。
 * 它还分配了一个 ID，该 ID 在此进程中的所有发布者和订阅者之间是唯一的，并且与发布者相关联。
 * (When a Node creates a publisher, as with subscriptions, a helper class can
 * be registered with this class.
 * This is required in order to publish intra-process messages.
 * It is also allocated an id which is unique among all publishers
 * and subscriptions in this process and that is associated to the publisher.)
 *
 * 当注册发布者或订阅者时，此类检查它将与哪些其他订阅者或发布者进行通信，
 * 即它们具有相同的主题和兼容的 QoS。
 * (When a publisher or a subscription are registered, this class checks to see
 * which other subscriptions or publishers it will communicate with,
 * i.e. they have the same topic and compatible QoS.)
 *
 * 当用户发布消息时，如果发布者上启用了内部进程通信，则将消息提供给此类。
 * 使用发布者 ID，为消息选择一组接收者。
 * 对于列表中的每个订阅，此类都会将消息（无论是共享所有权还是制作副本）存储在与订阅助手类关联的缓冲区中。
 * (When the user publishes a message, if intra-process communication is enabled
 * on the publisher, the message is given to this class.
 * Using the publisher id, a list of recipients for the message is selected.
 * For each subscription in the list, this class stores the message, whether
 * sharing ownership or making a copy, in a buffer associated with the
 * subscription helper class.)
 *
 * 订阅助手类包含一个缓冲区，其中存储了已发布的内部进程消息，直到它们从订阅中获取。
 * 根据存储在缓冲区中的数据类型，订阅助手类可以请求消息的共享或独占所有权。
 * (The subscription helper class contains a buffer where published
 * intra-process messages are stored until they are taken from the subscription.
 * Depending on the data type stored in the buffer, the subscription helper
 * class can request either shared or exclusive ownership on the message.)
 *
 * 因此，当发布内部进程消息时，此类知道有多少内部进程订阅需要它以及有多少需要所有权。
 * 这些信息使得此类能够通过执行所需的最少数量的消息副本来高效运行。
 * (Thus, when an intra-process message is published, this class knows how many
 * intra-process subscriptions needs it and how many require ownership.
 * This information allows this class to operate efficiently by performing the
 * fewest number of copies of the message required.)
 *
 * 此类既不可复制构造也不可复制赋值。
 * (This class is neither CopyConstructable nor CopyAssignable.)
 */
class IntraProcessManager {
private:
  // 禁用复制构造和复制赋值。 (Disable copy constructor and copy assignment.)
  RCLCPP_DISABLE_COPY(IntraProcessManager)

public:
  // 定义智能指针类型 IntraProcessManager（定义 IntraProcessManager 的智能指针类型）
  // Define smart pointer type for IntraProcessManager
  RCLCPP_SMART_PTR_DEFINITIONS(IntraProcessManager)

  // 公共构造函数（创建 IntraProcessManager 实例）
  // Public constructor (create an instance of IntraProcessManager)
  RCLCPP_PUBLIC
  IntraProcessManager();

  // 公共虚析构函数（销毁 IntraProcessManager 实例）
  // Public virtual destructor (destroy an instance of IntraProcessManager)
  RCLCPP_PUBLIC
  virtual ~IntraProcessManager();

  /// 使用管理器注册订阅，并返回订阅的唯一 ID。
  /// Register a subscription with the manager, returns subscriptions unique id.
  /**
   * 此方法将订阅的内部进程对象与其包装订阅的信息（即主题名称和 QoS）一起存储。
   * This method stores the subscription intra process object, together with
   * the information of its wrapped subscription (i.e. topic name and QoS).
   *
   * 另外，此方法为订阅生成一个唯一的内部进程 ID。
   * In addition this generates a unique intra process id for the subscription.
   *
   * \param subscription 要注册的 SubscriptionIntraProcess。
   * \param subscription the SubscriptionIntraProcess to register.
   * \return 无符号 64 位整数，表示订阅的唯一 ID。
   * \return an unsigned 64-bit integer which is the subscription's unique id.
   */
  RCLCPP_PUBLIC
  uint64_t add_subscription(
      rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr subscription);

  /// 使用订阅的唯一 ID 取消注册订阅。
  /// Unregister a subscription using the subscription's unique id.
  /**
   * 此方法不分配内存。
   * This method does not allocate memory.
   *
   * \param intra_process_subscription_id 要删除的订阅的 ID。
   * \param intra_process_subscription_id id of the subscription to remove.
   */
  RCLCPP_PUBLIC
  void remove_subscription(uint64_t intra_process_subscription_id);

  /// 注册一个发布者到管理器，返回发布者的唯一ID。
  /// Register a publisher with the manager, returns the publisher unique id.
  /**
   * 此方法将发布者的内部进程对象及其封装的发布者的信息（如主题名称和QoS）一起存储。
   * This method stores the publisher intra process object, together with
   * the information of its wrapped publisher (i.e. topic name and QoS).
   *
   * 另外，此方法还会为发布者生成一个唯一的内部进程ID。
   * In addition this generates a unique intra process id for the publisher.
   *
   * \param publisher 需要注册到管理器的发布者。
   * \param publisher publisher to be registered with the manager.
   * \return 一个无符号的64位整数，表示发布者的唯一ID。
   * \return an unsigned 64-bit integer which is the publisher's unique id.
   */
  RCLCPP_PUBLIC
  uint64_t add_publisher(rclcpp::PublisherBase::SharedPtr publisher);

  /// 使用发布者的唯一ID取消注册发布者。
  /// Unregister a publisher using the publisher's unique id.
  /**
   * 此方法不分配内存。
   * This method does not allocate memory.
   *
   * \param intra_process_publisher_id 要删除的发布者的ID。
   * \param intra_process_publisher_id id of the publisher to remove.
   */
  RCLCPP_PUBLIC
  void remove_publisher(uint64_t intra_process_publisher_id);

  /// 发布一个以 unique pointer 传递的内部进程消息。
  /// Publishes an intra-process message, passed as a unique pointer.
  /**
   * 这是发布内部进程的两种方法之一。
   * This is one of the two methods for publishing intra-process.
   *
   * 使用内部进程发布者 ID，获得一份收件人列表。
   * Using the intra-process publisher id, a list of recipients is obtained.
   * 根据是否需要所有权，该列表被分为两半。
   * This list is split in half, depending whether they require ownership or not.
   *
   * 这个特定的方法将 unique pointer 作为输入。
   * This particular method takes a unique pointer as input.
   * 该指针可以提升为 shared pointer，并传递给所有不需要所有权的订阅。
   * The pointer can be promoted to a shared pointer and passed to all the subscriptions
   * that do not require ownership.
   * 对于需要所有权的订阅，除最后一个（可以转移所有权）外，消息将为它们全部复制。
   * In case of subscriptions requiring ownership, the message will be copied for all of
   * them except the last one, when ownership can be transferred.
   *
   * 与使用 shared pointer 的方法相比，此方法可以节省一次额外的复制。
   * This method can save an additional copy compared to the shared pointer one.
   *
   * 如果未找到发布者 ID 或给 add_publisher 的发布者 shared_ptr 超出范围，
   * 则此方法可能会引发异常。
   * This method can throw an exception if the publisher id is not found or
   * if the publisher shared_ptr given to add_publisher has gone out of scope.
   *
   * 此方法确实分配内存。
   * This method does allocate memory.
   *
   * \param intra_process_publisher_id 此消息的发布者 ID。
   * \param intra_process_publisher_id the id of the publisher of this message.
   * \param message 被存储的消息。
   * \param message the message that is being stored.
   * \param allocator 缓冲消息时的分配器。
   * \param allocator for allocations when buffering messages.
   */
  template <
      typename MessageT,
      typename ROSMessageType,
      typename Alloc,
      typename Deleter = std::default_delete<MessageT>>
  void do_intra_process_publish(
      uint64_t intra_process_publisher_id,
      std::unique_ptr<MessageT, Deleter> message,
      typename allocator::AllocRebind<MessageT, Alloc>::allocator_type& allocator) {
    using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
    using MessageAllocatorT = typename MessageAllocTraits::allocator_type;

    // 使用共享锁保护互斥体
    // Use a shared lock to protect the mutex
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    // 查找发布者 ID
    // Find the publisher id
    auto publisher_it = pub_to_subs_.find(intra_process_publisher_id);
    if (publisher_it == pub_to_subs_.end()) {
      // 发布者无效或不存在
      // Publisher is either invalid or no longer exists.
      RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"),
          "Calling do_intra_process_publish for invalid or no longer existing publisher id");
      return;
    }
    const auto& sub_ids = publisher_it->second;

    // 如果没有订阅需要所有权
    // If none of the subscriptions require ownership
    if (sub_ids.take_ownership_subscriptions.empty()) {
      // 提升指针为 shared pointer
      // Promote the pointer to a shared pointer
      std::shared_ptr<MessageT> msg = std::move(message);

      // 将消息添加到不需要所有权的缓冲区
      // Add the message to the buffers that don't require ownership
      this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
          msg, sub_ids.take_shared_subscriptions);
    } else if (
        !sub_ids.take_ownership_subscriptions.empty() &&  // NOLINT
        sub_ids.take_shared_subscriptions.size() <= 1) {
      // 最多有一个缓冲区不需要所有权。
      // There is at maximum 1 buffer that does not require ownership.
      // 因此，这种情况等同于所有缓冲区都需要所有权
      // So this case is equivalent to all the buffers requiring ownership

      // 合并两个 ID 向量为一个唯一向量
      // Merge the two vector of ids into a unique one
      std::vector<uint64_t> concatenated_vector(sub_ids.take_shared_subscriptions);
      concatenated_vector.insert(
          concatenated_vector.end(), sub_ids.take_ownership_subscriptions.begin(),
          sub_ids.take_ownership_subscriptions.end());
      this->template add_owned_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
          std::move(message), concatenated_vector, allocator);
    } else if (
        !sub_ids.take_ownership_subscriptions.empty() &&  // NOLINT
        sub_ids.take_shared_subscriptions.size() > 1) {
      // 从消息构造一个新的 shared pointer
      // Construct a new shared pointer from the message
      // 用于不需要所有权的缓冲区
      // for the buffers that do not require ownership
      auto shared_msg = std::allocate_shared<MessageT, MessageAllocatorT>(allocator, *message);

      // 将消息添加到不需要所有权和需要所有权的缓冲区
      // Add the message to the buffers that don't require ownership and those that do
      this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
          shared_msg, sub_ids.take_shared_subscriptions);
      this->template add_owned_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
          std::move(message), sub_ids.take_ownership_subscriptions, allocator);
    }
  }

  /**
   * @brief 发布并返回共享内存中的消息（Publish and return a shared message in intra-process
   * communication）
   *
   * @tparam MessageT 消息类型（Message type）
   * @tparam ROSMessageType ROS 消息类型（ROS message type）
   * @tparam Alloc 分配器类型（Allocator type）
   * @tparam Deleter 删除器类型（Deleter type）
   * @param[in] intra_process_publisher_id 内部进程发布者 ID（Intra-process publisher ID）
   * @param[in] message 要发布的消息（The message to be published）
   * @param[in, out] allocator 分配器实例（Allocator instance）
   * @return std::shared_ptr<const MessageT> 共享指针，指向发布的消息（Shared pointer to the
   * published message）
   */
  template <
      typename MessageT,
      typename ROSMessageType,
      typename Alloc,
      typename Deleter = std::default_delete<MessageT>>
  std::shared_ptr<const MessageT> do_intra_process_publish_and_return_shared(
      uint64_t intra_process_publisher_id,
      std::unique_ptr<MessageT, Deleter> message,
      typename allocator::AllocRebind<MessageT, Alloc>::allocator_type& allocator) {
    // 使用分配器重新绑定消息类型（Using allocator to rebind message type）
    using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
    using MessageAllocatorT = typename MessageAllocTraits::allocator_type;

    // 对互斥体加共享锁（Acquire a shared lock on the mutex）
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    // 查找发布者 ID 对应的订阅者集合（Find the set of subscribers associated with the publisher
    // ID）
    auto publisher_it = pub_to_subs_.find(intra_process_publisher_id);
    if (publisher_it == pub_to_subs_.end()) {
      // 发布者无效或不存在（Publisher is invalid or no longer exists）
      RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"),
          "Calling do_intra_process_publish for invalid or no longer existing publisher id");
      return nullptr;
    }
    const auto& sub_ids = publisher_it->second;

    // 如果没有拥有所有权的订阅者（If there are no owning subscribers）
    if (sub_ids.take_ownership_subscriptions.empty()) {
      // 只需将消息转换为共享指针（Just convert the message to a shared pointer）
      std::shared_ptr<MessageT> shared_msg = std::move(message);
      if (!sub_ids.take_shared_subscriptions.empty()) {
        this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
            shared_msg, sub_ids.take_shared_subscriptions);
      }
      return shared_msg;
    } else {
      // 为不需要所有权的缓冲区和返回值构造一个新的共享指针（Construct a new shared pointer for
      // buffers that do not require ownership and to return）
      auto shared_msg = std::allocate_shared<MessageT, MessageAllocatorT>(allocator, *message);

      if (!sub_ids.take_shared_subscriptions.empty()) {
        this->template add_shared_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
            shared_msg, sub_ids.take_shared_subscriptions);
      }
      if (!sub_ids.take_ownership_subscriptions.empty()) {
        this->template add_owned_msg_to_buffers<MessageT, Alloc, Deleter, ROSMessageType>(
            std::move(message), sub_ids.take_ownership_subscriptions, allocator);
      }
      return shared_msg;
    }
  }

  /// \brief 返回给定的 rmw_gid_t 是否与任何已存储的发布者匹配 (Return true if the given rmw_gid_t
  /// matches any stored Publishers) \param id 一个指向 rmw_gid_t 类型的指针，表示要检查的发布者 ID
  /// (A pointer to a rmw_gid_t, representing the publisher ID to check) \return 如果给定的
  /// rmw_gid_t 与任何已存储的发布者匹配，则返回 true，否则返回 false (Returns true if the given
  /// rmw_gid_t matches any stored Publishers, otherwise returns false)
  RCLCPP_PUBLIC
  bool matches_any_publishers(const rmw_gid_t* id) const;

  /// \brief 返回与给定发布者 ID 匹配的内部进程订阅的数量 (Return the number of intraprocess
  /// subscriptions that are matched with a given publisher id) \param intra_process_publisher_id
  /// 要检查的内部进程发布者 ID (The intra-process publisher ID to check) \return 与给定发布者 ID
  /// 匹配的内部进程订阅的数量 (The number of intraprocess subscriptions that are matched with the
  /// given publisher id)
  RCLCPP_PUBLIC
  size_t get_subscription_count(uint64_t intra_process_publisher_id) const;

  /// \brief 获取具有给定内部进程订阅 ID 的内部进程订阅 (Get the intraprocess subscription with the
  /// given intra-process subscription ID) \param intra_process_subscription_id 要获取的内部进程订阅
  /// ID (The intra-process subscription ID to get) \return 返回一个指向
  /// rclcpp::experimental::SubscriptionIntraProcessBase 类型的共享指针，表示获取到的内部进程订阅
  /// (Returns a shared pointer to a rclcpp::experimental::SubscriptionIntraProcessBase,
  /// representing the obtained intraprocess subscription)
  RCLCPP_PUBLIC
  rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr get_subscription_intra_process(
      uint64_t intra_process_subscription_id);

private:
  // 定义一个名为 SplittedSubscriptions 的结构体 (Define a struct called SplittedSubscriptions)
  struct SplittedSubscriptions {
    // 用于存储共享订阅的 ID 的向量 (A vector for storing shared subscription IDs)
    std::vector<uint64_t> take_shared_subscriptions;
    // 用于存储拥有订阅的 ID 的向量 (A vector for storing ownership subscription IDs)
    std::vector<uint64_t> take_ownership_subscriptions;
  };

  using SubscriptionMap =
      std::unordered_map<uint64_t, rclcpp::experimental::SubscriptionIntraProcessBase::WeakPtr>;

  using PublisherMap = std::unordered_map<uint64_t, rclcpp::PublisherBase::WeakPtr>;

  using PublisherToSubscriptionIdsMap = std::unordered_map<uint64_t, SplittedSubscriptions>;

  /// \brief 获取下一个唯一的ID
  /// \return 下一个唯一的ID
  /// \brief Get the next unique ID
  /// \return The next unique ID
  RCLCPP_PUBLIC
  static uint64_t get_next_unique_id();

  /// \brief 为给定的发布者ID和订阅者ID插入关联关系
  /// \param[in] sub_id 订阅者ID
  /// \param[in] pub_id 发布者ID
  /// \param[in] use_take_shared_method 是否使用共享方法
  /// \brief Insert a relation for the given publisher ID and subscriber ID
  /// \param[in] sub_id Subscriber ID
  /// \param[in] pub_id Publisher ID
  /// \param[in] use_take_shared_method Whether to use the shared method or not
  RCLCPP_PUBLIC
  void insert_sub_id_for_pub(uint64_t sub_id, uint64_t pub_id, bool use_take_shared_method);

  /// \brief 检查给定的发布者和订阅者是否可以进行通信
  /// \param[in] pub 发布者对象的共享指针
  /// \param[in] sub 订阅者对象的共享指针
  /// \return 如果可以通信，则为true；否则为false
  /// \brief Check if the given publisher and subscriber can communicate with each other
  /// \param[in] pub Shared pointer to the publisher object
  /// \param[in] sub Shared pointer to the subscriber object
  /// \return True if they can communicate; false otherwise
  RCLCPP_PUBLIC
  bool can_communicate(
      rclcpp::PublisherBase::SharedPtr pub,
      rclcpp::experimental::SubscriptionIntraProcessBase::SharedPtr sub) const;

  /// \brief 将共享消息添加到缓冲区
  /// \tparam MessageT 消息类型
  /// \tparam Alloc 分配器类型
  /// \tparam Deleter 删除器类型
  /// \tparam ROSMessageType ROS消息类型
  /// \param[in] message 要添加的共享消息
  /// \param[in] subscription_ids 订阅者ID列表
  /// \brief Add shared message to buffers
  /// \tparam MessageT Message type
  /// \tparam Alloc Allocator type
  /// \tparam Deleter Deleter type
  /// \tparam ROSMessageType ROS message type
  /// \param[in] message Shared message to be added
  /// \param[in] subscription_ids List of subscriber IDs
  template <typename MessageT, typename Alloc, typename Deleter, typename ROSMessageType>
  void add_shared_msg_to_buffers(
      std::shared_ptr<const MessageT> message, std::vector<uint64_t> subscription_ids) {
    // 定义ROS消息类型分配器和删除器
    // Define ROS message type allocator and deleter
    using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<ROSMessageType, Alloc>;
    using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
    using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;

    // 定义发布类型分配器和删除器
    // Define published type allocator and deleter
    using PublishedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;
    using PublishedTypeAllocatorTraits = allocator::AllocRebind<PublishedType, Alloc>;
    using PublishedTypeAllocator = typename PublishedTypeAllocatorTraits::allocator_type;
    using PublishedTypeDeleter = allocator::Deleter<PublishedTypeAllocator, PublishedType>;

    // 遍历订阅者ID列表
    // Iterate through the list of subscriber IDs
    for (auto id : subscription_ids) {
      // 查找订阅者
      // Find the subscriber
      auto subscription_it = subscriptions_.find(id);
      if (subscription_it == subscriptions_.end()) {
        // 如果未找到订阅者，抛出运行时错误
        // If subscriber not found, throw a runtime error
        throw std::runtime_error("subscription has unexpectedly gone out of scope");
      }
      auto subscription_base = subscription_it->second.lock();
      if (subscription_base == nullptr) {
        // 如果订阅者基类为空，从订阅者列表中删除该ID
        // If subscriber base is null, remove the ID from the subscriber list
        subscriptions_.erase(id);
        continue;
      }

      // 尝试将订阅者基类转换为适当的类型
      // Try to cast the subscriber base to the appropriate type
      auto subscription =
          std::dynamic_pointer_cast<rclcpp::experimental::SubscriptionIntraProcessBuffer<
              PublishedType, PublishedTypeAllocator, PublishedTypeDeleter, ROSMessageType>>(
              subscription_base);
      if (subscription != nullptr) {
        // 如果成功转换，为订阅者提供进程内数据
        // If cast successful, provide intra-process data to the subscriber
        subscription->provide_intra_process_data(message);
        continue;
      }

      // 尝试将订阅者基类转换为ROS消息类型的订阅者
      // Try to cast the subscriber base to the ROS message type subscriber
      auto ros_message_subscription =
          std::dynamic_pointer_cast<rclcpp::experimental::SubscriptionROSMsgIntraProcessBuffer<
              ROSMessageType, ROSMessageTypeAllocator, ROSMessageTypeDeleter>>(subscription_base);
      if (nullptr == ros_message_subscription) {
        // 如果转换失败，抛出运行时错误
        // If cast fails, throw a runtime error
        throw std::runtime_error(
            "failed to dynamic cast SubscriptionIntraProcessBase to "
            "SubscriptionIntraProcessBuffer<MessageT, Alloc, Deleter>, or to "
            "SubscriptionROSMsgIntraProcessBuffer<ROSMessageType,ROSMessageTypeAllocator,"
            "ROSMessageTypeDeleter> which can happen when the publisher and "
            "subscription use different allocator types, which is not supported");
      }

      // 根据消息类型提供适当的进程内消息
      // Provide appropriate intra-process message based on the message type
      if constexpr (rclcpp::TypeAdapter<MessageT>::is_specialized::value) {
        ROSMessageType ros_msg;
        rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(*message, ros_msg);
        ros_message_subscription->provide_intra_process_message(
            std::make_shared<ROSMessageType>(ros_msg));
      } else {
        if constexpr (std::is_same<MessageT, ROSMessageType>::value) {
          ros_message_subscription->provide_intra_process_message(message);
        } else {
          if constexpr (std::is_same<
                            typename rclcpp::TypeAdapter<
                                MessageT, ROSMessageType>::ros_message_type,
                            ROSMessageType>::value) {
            ROSMessageType ros_msg;
            rclcpp::TypeAdapter<MessageT, ROSMessageType>::convert_to_ros_message(
                *message, ros_msg);
            ros_message_subscription->provide_intra_process_message(
                std::make_shared<ROSMessageType>(ros_msg));
          }
        }
      }
    }
  }

  /**
   * @brief 将拥有的消息添加到缓冲区 (Add owned messages to buffers)
   *
   * @tparam MessageT 消息类型 (Message type)
   * @tparam Alloc 分配器类型 (Allocator type)
   * @tparam Deleter 删除器类型 (Deleter type)
   * @tparam ROSMessageType ROS 消息类型 (ROS message type)
   * @param[in] message 唯一指针，指向要添加到缓冲区的消息 (Unique pointer to the message to be
   * added to the buffers)
   * @param[in] subscription_ids 订阅 ID 的向量 (Vector of subscription IDs)
   * @param[in, out] allocator 分配器引用，用于内存管理 (Reference to allocator for memory
   * management)
   */
  template <typename MessageT, typename Alloc, typename Deleter, typename ROSMessageType>
  void add_owned_msg_to_buffers(
      std::unique_ptr<MessageT, Deleter> message,
      std::vector<uint64_t> subscription_ids,
      typename allocator::AllocRebind<MessageT, Alloc>::allocator_type& allocator) {
    // 定义 MessageT 类型的分配器特性 (Define allocator traits for MessageT type)
    using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
    // 定义 MessageT 类型的唯一指针 (Define unique pointer for MessageT type)
    using MessageUniquePtr = std::unique_ptr<MessageT, Deleter>;

    // 定义 ROSMessageType 类型的分配器特性 (Define allocator traits for ROSMessageType type)
    using ROSMessageTypeAllocatorTraits = allocator::AllocRebind<ROSMessageType, Alloc>;
    // 定义 ROSMessageType 类型的分配器 (Define allocator for ROSMessageType type)
    using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
    // 定义 ROSMessageType 类型的删除器 (Define deleter for ROSMessageType type)
    using ROSMessageTypeDeleter = allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;

    // 定义发布类型 (Define published type)
    using PublishedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;
    // 定义发布类型的分配器特性 (Define allocator traits for PublishedType)
    using PublishedTypeAllocatorTraits = allocator::AllocRebind<PublishedType, Alloc>;
    // 定义发布类型的分配器 (Define allocator for PublishedType)
    using PublishedTypeAllocator = typename PublishedTypeAllocatorTraits::allocator_type;
    // 定义发布类型的删除器 (Define deleter for PublishedType)
    using PublishedTypeDeleter = allocator::Deleter<PublishedTypeAllocator, PublishedType>;

    // 遍历订阅 ID (Iterate through subscription IDs)
    for (auto it = subscription_ids.begin(); it != subscription_ids.end(); it++) {
      // 查找订阅 (Find subscription)
      auto subscription_it = subscriptions_.find(*it);
      // 如果找不到订阅，抛出异常 (If subscription not found, throw exception)
      if (subscription_it == subscriptions_.end()) {
        throw std::runtime_error("subscription has unexpectedly gone out of scope");
      }
      // 获取订阅基类的弱指针 (Get weak pointer to subscription base)
      auto subscription_base = subscription_it->second.lock();
      // 如果订阅基类为空，从订阅集中删除它并继续循环 (If subscription base is null, erase it from
      // subscriptions and continue loop)
      if (subscription_base == nullptr) {
        subscriptions_.erase(subscription_it);
        continue;
      }

      // 尝试将订阅基类动态转换为适当的类型 (Try to dynamically cast subscription base to
      // appropriate type)
      auto subscription =
          std::dynamic_pointer_cast<rclcpp::experimental::SubscriptionIntraProcessBuffer<
              PublishedType, PublishedTypeAllocator, PublishedTypeDeleter, ROSMessageType>>(
              subscription_base);
      // 如果转换成功 (If cast is successful)
      if (subscription != nullptr) {
        // 如果这是最后一个订阅，放弃所有权 (If this is the last subscription, give up ownership)
        if (std::next(it) == subscription_ids.end()) {
          subscription->provide_intra_process_data(std::move(message));
        } else {
          // 复制消息，因为我们还有其他订阅要服务 (Copy the message since we have additional
          // subscriptions to serve)
          Deleter deleter = message.get_deleter();
          auto ptr = MessageAllocTraits::allocate(allocator, 1);
          MessageAllocTraits::construct(allocator, ptr, *message);

          subscription->provide_intra_process_data(std::move(MessageUniquePtr(ptr, deleter)));
        }

        continue;
      }

      // 尝试将订阅基类动态转换为另一种类型 (Try to dynamically cast subscription base to another
      // type)
      auto ros_message_subscription =
          std::dynamic_pointer_cast<rclcpp::experimental::SubscriptionROSMsgIntraProcessBuffer<
              ROSMessageType, ROSMessageTypeAllocator, ROSMessageTypeDeleter>>(subscription_base);
      // 如果转换失败，抛出异常 (If cast fails, throw exception)
      if (nullptr == ros_message_subscription) {
        throw std::runtime_error(
            "failed to dynamic cast SubscriptionIntraProcessBase to "
            "SubscriptionIntraProcessBuffer<MessageT, Alloc, Deleter>, or to "
            "SubscriptionROSMsgIntraProcessBuffer<ROSMessageType,ROSMessageTypeAllocator,"
            "ROSMessageTypeDeleter> which can happen when the publisher and "
            "subscription use different allocator types, which is not supported");
      }

      // 如果类型适配器特化 (If type adapter is specialized)
      if constexpr (rclcpp::TypeAdapter<MessageT>::is_specialized::value) {
        // 使用分配器构造 ROS 消息 (Construct ROS message with allocator)
        ROSMessageTypeAllocator ros_message_alloc(allocator);
        auto ptr = ros_message_alloc.allocate(1);
        ros_message_alloc.construct(ptr);
        ROSMessageTypeDeleter deleter;
        allocator::set_allocator_for_deleter(&deleter, &allocator);
        // 将 MessageT 转换为 ROS 消息 (Convert MessageT to ROS message)
        rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(*message, *ptr);
        auto ros_msg = std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>(ptr, deleter);
        // 提供内部进程消息 (Provide intra-process message)
        ros_message_subscription->provide_intra_process_message(std::move(ros_msg));
      } else {
        // 如果 MessageT 和 ROSMessageType 相同 (If MessageT and ROSMessageType are the same)
        if constexpr (std::is_same<MessageT, ROSMessageType>::value) {
          // 如果这是最后一个订阅，放弃所有权 (If this is the last subscription, give up ownership)
          if (std::next(it) == subscription_ids.end()) {
            ros_message_subscription->provide_intra_process_message(std::move(message));
          } else {
            // 复制消息，因为我们还有其他订阅要服务 (Copy the message since we have additional
            // subscriptions to serve)
            Deleter deleter = message.get_deleter();
            allocator::set_allocator_for_deleter(&deleter, &allocator);
            auto ptr = MessageAllocTraits::allocate(allocator, 1);
            MessageAllocTraits::construct(allocator, ptr, *message);

            ros_message_subscription->provide_intra_process_message(
                std::move(MessageUniquePtr(ptr, deleter)));
          }
        }
      }
    }
  }

  PublisherToSubscriptionIdsMap pub_to_subs_;
  SubscriptionMap subscriptions_;
  PublisherMap publishers_;

  mutable std::shared_timed_mutex mutex_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__INTRA_PROCESS_MANAGER_HPP_

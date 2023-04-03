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

#ifndef RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BASE_HPP_
#define RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BASE_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>

#include "rcl/wait.h"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/waitable.hpp"
#include "rmw/impl/cpp/demangle.hpp"

namespace rclcpp {
namespace experimental {

/**
 * @class SubscriptionIntraProcessBase
 * @brief 基于 rclcpp::Waitable 的内部进程订阅基类 (Base class for intra-process subscriptions,
 * derived from rclcpp::Waitable)
 *
 * @tparam EntityType 枚举类型，用于指定实体类型（如订阅）(Enum type to specify entity types like
 * Subscription)
 */
class SubscriptionIntraProcessBase : public rclcpp::Waitable {
public:
  // 只使用智能指针别名 (Using smart pointer aliases only)
  RCLCPP_SMART_PTR_ALIASES_ONLY(SubscriptionIntraProcessBase)

  /**
   * @enum EntityType
   * @brief 实体类型枚举 (Enumeration for entity types)
   */
  enum class EntityType : std::size_t {
    Subscription,  ///< 订阅实体类型 (Subscription entity type)
  };

  /**
   * @brief 构造函数，用于创建一个 SubscriptionIntraProcessBase 对象 (Constructor for creating a
   * SubscriptionIntraProcessBase object)
   *
   * @param context 与节点关联的上下文 (The context associated with the node)
   * @param topic_name 订阅的话题名称 (The name of the topic to subscribe to)
   * @param qos_profile 话题的 QoS 配置 (The QoS configuration for the topic)
   */
  RCLCPP_PUBLIC
  SubscriptionIntraProcessBase(
      rclcpp::Context::SharedPtr context,
      const std::string& topic_name,
      const rclcpp::QoS& qos_profile)
      : gc_(context), topic_name_(topic_name), qos_profile_(qos_profile) {}

  /**
   * @brief 虚拟析构函数 (Virtual destructor)
   */
  RCLCPP_PUBLIC
  virtual ~SubscriptionIntraProcessBase() = default;

  /**
   * @brief 获取准备好的 guard conditions 的数量 (Get the number of ready guard conditions)
   *
   * @return 准备好的 guard conditions 的数量 (The number of ready guard conditions)
   */
  RCLCPP_PUBLIC
  size_t get_number_of_ready_guard_conditions() override { return 1; }

  /**
   * @brief 将订阅添加到等待集合中 (Add the subscription to the wait set)
   *
   * @param wait_set 等待集合指针 (Pointer to the wait set)
   */
  RCLCPP_PUBLIC
  void add_to_wait_set(rcl_wait_set_t* wait_set) override;

  /**
   * @brief 判断订阅是否准备好 (Check if the subscription is ready)
   *
   * @param wait_set 等待集合指针 (Pointer to the wait set)
   * @return 订阅是否准备好的布尔值 (Boolean value indicating if the subscription is ready)
   */
  bool is_ready(rcl_wait_set_t* wait_set) override = 0;

  /**
   * @brief 获取数据 (Take data from the subscription)
   *
   * @return 数据的共享指针 (Shared pointer to the data)
   */
  std::shared_ptr<void> take_data() override = 0;

  /**
   * @brief 根据实体 ID 获取数据 (Take data by entity ID)
   *
   * @param id 实体 ID (The entity ID)
   * @return 数据的共享指针 (Shared pointer to the data)
   */
  std::shared_ptr<void> take_data_by_entity_id(size_t id) override {
    (void)id;
    return take_data();
  }

  /**
   * @brief 执行订阅回调 (Execute the subscription callback)
   *
   * @param data 数据的共享指针 (Shared pointer to the data)
   */
  void execute(std::shared_ptr<void>& data) override = 0;

  /**
   * @brief 判断是否使用 take_shared_method (Check if the take_shared_method should be used)
   *
   * @return 是否使用 take_shared_method 的布尔值 (Boolean value indicating if the
   * take_shared_method should be used)
   */
  virtual bool use_take_shared_method() const = 0;

  /**
   * @brief 获取话题名称 (Get the topic name)
   *
   * @return 话题名称的 C 风格字符串 (C-style string of the topic name)
   */
  RCLCPP_PUBLIC
  const char* get_topic_name() const;

  /**
   * @brief 获取实际 QoS 配置 (Get the actual QoS configuration)
   *
   * @return 实际 QoS 配置 (The actual QoS configuration)
   */
  RCLCPP_PUBLIC
  QoS get_actual_qos() const;

  /// 设置一个回调函数，当有新消息到达时被调用。
  /// Set a callback to be called when each new message arrives.
  /**
   * 回调函数接收一个 size_t 类型参数，表示自上次调用此回调函数以来收到的消息数量。
   * The callback receives a size_t which is the number of messages received
   * since the last time this callback was called.
   * 通常这个值是 1，但如果在设置任何回调函数之前已经收到了消息，那么这个值可能大于 1。
   * Normally this is 1, but can be > 1 if messages were received before any
   * callback was set.
   *
   * 回调函数还接收一个 int 类型的标识符参数。
   * The callback also receives an int identifier argument.
   * 这是因为一个 Waitable 可能由多个不同的实体组成，例如订阅、服务等。
   * This is needed because a Waitable may be composed of several distinct entities,
   * such as subscriptions, services, etc.
   * 应用程序应提供一个通用的回调函数，然后由 waitable 转发给它的所有实体。
   * The application should provide a generic callback function that will be then
   * forwarded by the waitable to all of its entities.
   * 在转发之前，将为标识符参数绑定一个不同的值。
   * Before forwarding, a different value for the identifier argument will be
   * bound to the function.
   * 这意味着所提供的回调函数可以根据触发 waitable 准备就绪的实体使用标识符来执行不同的操作。
   * This implies that the provided callback can use the identifier to behave
   * differently depending on which entity triggered the waitable to become ready.
   *
   * 再次调用它将清除之前设置的任何回调函数。
   * Calling it again will clear any previously set callback.
   *
   * 如果回调函数无法调用，将抛出异常。
   * An exception will be thrown if the callback is not callable.
   *
   * 此函数是线程安全的。
   * This function is thread-safe.
   *
   * 如果您希望在回调函数中提供更多信息，如订阅或其他信息，可以使用带捕获的 lambda 或 std::bind。
   * If you want more information available in the callback, like the subscription
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \param[in] callback 当新消息被接收时要调用的函数对象。
   * \param[in] callback functor to be called when a new message is received.
   */
  void set_on_ready_callback(std::function<void(size_t, int)> callback) override {
    // 如果回调为空，则抛出异常
    // Throw an exception if the callback is not callable
    if (!callback) {
      throw std::invalid_argument(
          "The callback passed to set_on_ready_callback "
          "is not callable.");
    }

    // 注意：我们将 int 标识符参数绑定到此 waitable 的实体类型
    // Note: we bind the int identifier argument to this waitable's entity types
    auto new_callback = [callback, this](size_t number_of_events) {
      try {
        // 调用回调函数，传递事件数量和实体类型
        // Invoke the callback with the number of events and entity type
        callback(number_of_events, static_cast<int>(EntityType::Subscription));
      } catch (const std::exception& exception) {
        // 捕获异常并记录错误信息
        // Catch the exception and log the error message
        RCLCPP_ERROR_STREAM(
            // TODO(wjwwood): get this class access to the node logger it is associated with
            rclcpp::get_logger("rclcpp"),
            "rclcpp::SubscriptionIntraProcessBase@"
                << this << " caught " << rmw::impl::cpp::demangle(exception)
                << " exception in user-provided callback for the 'on ready' callback: "
                << exception.what());
      } catch (...) {
        // 捕获未处理的异常并记录错误信息
        // Catch unhandled exceptions and log the error message
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("rclcpp"),
            "rclcpp::SubscriptionIntraProcessBase@"
                << this << " caught unhandled exception in user-provided callback "
                << "for the 'on ready' callback");
      }
    };

    // 使用互斥锁保护回调函数的设置
    // Use a mutex lock to protect the setting of the callback
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    on_new_message_callback_ = new_callback;

    // 如果有未读消息，则调用回调函数
    // If there are unread messages, invoke the callback
    if (unread_count_ > 0) {
      if (qos_profile_.history() == HistoryPolicy::KeepAll) {
        on_new_message_callback_(unread_count_);
      } else {
        // 使用 qos 配置文件的深度作为未读消息数量的上限
        // Use qos profile depth as upper bound for unread_count_
        on_new_message_callback_(std::min(unread_count_, qos_profile_.depth()));
      }
      unread_count_ = 0;
    }
  }

  /// 取消注册新消息的回调函数（如果有）。
  /// Unset the callback registered for new messages, if any.
  void clear_on_ready_callback() override {
    // 使用互斥锁保护回调函数的清除
    // Use a mutex lock to protect the clearing of the callback
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    on_new_message_callback_ = nullptr;
  }

protected:
  // std::recursive_mutex 用于保护回调函数的互斥访问（允许同一线程多次锁定）
  // std::recursive_mutex is used to protect the callback function's mutual access (allowing the
  // same thread to lock multiple times)
  std::recursive_mutex callback_mutex_;

  // on_new_message_callback_ 是一个 std::function，接受 size_t 类型参数，用于处理新消息的回调
  // on_new_message_callback_ is a std::function that takes a size_t type parameter, and it's used
  // for handling new message callbacks
  std::function<void(size_t)> on_new_message_callback_{nullptr};

  // unread_count_ 用于存储未读消息数量
  // unread_count_ is used to store the number of unread messages
  size_t unread_count_{0};

  // rclcpp::GuardCondition 用于触发回调函数
  // rclcpp::GuardCondition is used to trigger the callback function
  rclcpp::GuardCondition gc_;

  // 定义一个纯虚函数 trigger_guard_condition，子类需要实现此函数以触发 GuardCondition
  // Define a pure virtual function trigger_guard_condition, subclasses need to implement this
  // function to trigger GuardCondition
  virtual void trigger_guard_condition() = 0;

  // invoke_on_new_message 函数用于调用 on_new_message_callback_ 处理新消息
  // The invoke_on_new_message function is used to call on_new_message_callback_ to handle new
  // messages
  void invoke_on_new_message() {
    // 使用 std::lock_guard 对象在作用域内自动加锁和解锁，保证线程安全
    // Use the std::lock_guard object to automatically lock and unlock within the scope, ensuring
    // thread safety
    std::lock_guard<std::recursive_mutex> lock(this->callback_mutex_);

    // 判断 on_new_message_callback_ 是否已设置，如果已设置，则调用回调函数处理新消息
    // Check if on_new_message_callback_ has been set, if so, call the callback function to handle
    // the new message
    if (this->on_new_message_callback_) {
      this->on_new_message_callback_(1);
    } else {
      // 如果回调函数未设置，将未读消息数量加一
      // If the callback function is not set, increment the unread message count by one
      this->unread_count_++;
    }
  }

private:
  std::string topic_name_;
  QoS qos_profile_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BASE_HPP_

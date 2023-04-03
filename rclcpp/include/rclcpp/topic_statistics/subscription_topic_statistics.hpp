// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef RCLCPP__TOPIC_STATISTICS__SUBSCRIPTION_TOPIC_STATISTICS_HPP_
#define RCLCPP__TOPIC_STATISTICS__SUBSCRIPTION_TOPIC_STATISTICS_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "libstatistics_collector/collector/generate_statistics_message.hpp"
#include "libstatistics_collector/moving_average_statistics/types.hpp"
#include "libstatistics_collector/topic_statistics_collector/constants.hpp"
#include "libstatistics_collector/topic_statistics_collector/received_message_age.hpp"
#include "libstatistics_collector/topic_statistics_collector/received_message_period.hpp"
#include "rcl/time.h"
#include "rclcpp/publisher.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "statistics_msgs/msg/metrics_message.hpp"

namespace rclcpp {
namespace topic_statistics {

// 默认的发布主题名称
// Default publish topic name
constexpr const char kDefaultPublishTopicName[]{"/statistics"};
// 默认的发布周期（以毫秒为单位）
// Default publishing period (in milliseconds)
constexpr const std::chrono::milliseconds kDefaultPublishingPeriod{std::chrono::seconds(1)};

// 使用 libstatistics_collector 中的函数和类型
// Use functions and types from libstatistics_collector
using libstatistics_collector::collector::GenerateStatisticMessage;
using libstatistics_collector::moving_average_statistics::StatisticData;
using statistics_msgs::msg::MetricsMessage;

/**
 * @brief
 * 用于收集、测量和发布主题统计数据的类。当前支持的订阅者统计数据包括接收到的消息的年龄和接收到的消息的周期。
 * Class used to collect, measure, and publish topic statistics data. Current statistics
 * supported for subscribers are received message age and received message period.
 *
 * @tparam CallbackMessageT 订阅的消息类型
 * @tparam CallbackMessageT the subscribed message type
 */
template <typename CallbackMessageT>
class SubscriptionTopicStatistics {
  // 定义 TopicStatsCollector 类型，用于收集特定消息类型的主题统计信息
  // Define TopicStatsCollector type for collecting topic statistics of specific message type
  using TopicStatsCollector =
      libstatistics_collector::topic_statistics_collector::TopicStatisticsCollector<
          CallbackMessageT>;
  // 定义 ReceivedMessageAge 类型，用于收集接收到的特定消息类型的消息的年龄统计信息
  // Define ReceivedMessageAge type for collecting age statistics of received messages of specific
  // type
  using ReceivedMessageAge =
      libstatistics_collector::topic_statistics_collector::ReceivedMessageAgeCollector<
          CallbackMessageT>;
  // 定义 ReceivedMessagePeriod 类型，用于收集接收到的特定消息类型的消息的周期统计信息
  // Define ReceivedMessagePeriod type for collecting period statistics of received messages of
  // specific type
  using ReceivedMessagePeriod =
      libstatistics_collector::topic_statistics_collector::ReceivedMessagePeriodCollector<
          CallbackMessageT>;

public:
  /// 构造一个 SubscriptionTopicStatistics 对象。
  /// Construct a SubscriptionTopicStatistics object.
  /**
   * 该对象包装了在 libstatistics_collector
   * 中定义的实用程序，用于收集、测量和发布主题统计数据。如果输入的发布器为空，则抛出
   * invalid_argument 异常。 This object wraps utilities, defined in libstatistics_collector, to
   * collect, measure, and publish topic statistics data. This throws an invalid_argument if the
   * input publisher is null.
   *
   * \param node_name 创建此实例的节点名称，以表示主题来源
   * \param node_name the name of the node, which created this instance, in order to denote
   * topic source
   * \param publisher 节点构建的实例，用于发布统计数据。这个类拥有发布器。
   * \param publisher instance constructed by the node in order to publish statistics data.
   * This class owns the publisher.
   * \throws std::invalid_argument 如果发布器指针为 nullptr
   * \throws std::invalid_argument if publisher pointer is nullptr
   */
  SubscriptionTopicStatistics(
      const std::string& node_name,
      rclcpp::Publisher<statistics_msgs::msg::MetricsMessage>::SharedPtr publisher)
      : node_name_(node_name), publisher_(std::move(publisher)) {
    // TODO(dbbonnie): ros-tooling/aws-roadmap/issues/226, received message age

    // 如果发布器指针为空，则抛出异常
    // If the publisher pointer is null, throw an exception
    if (nullptr == publisher_) {
      throw std::invalid_argument("publisher pointer is nullptr");
    }

    // 初始化操作
    // Initialization operations
    bring_up();
  }

  // 析构函数
  // Destructor
  virtual ~SubscriptionTopicStatistics() { tear_down(); }

  /// 处理订阅收到的消息以收集统计信息。
  /// Handle a message received by the subscription to collect statistics.
  /**
   * 该方法获取锁以防止对收集器列表的竞争条件。
   * This method acquires a lock to prevent race conditions to collectors list.
   *
   * \param received_message 订阅收到的消息
   * \param received_message the message received by the subscription
   * \param now_nanoseconds 当前时间（以纳秒为单位）
   * \param now_nanoseconds current time in nanoseconds
   */
  virtual void handle_message(
      const CallbackMessageT& received_message, const rclcpp::Time now_nanoseconds) const {
    // 对互斥体进行加锁，防止竞争条件
    // Lock the mutex to prevent race conditions
    std::lock_guard<std::mutex> lock(mutex_);

    // 遍历订阅者统计信息收集器，并处理接收到的消息
    // Iterate through subscriber statistics collectors and process the received message
    for (const auto& collector : subscriber_statistics_collectors_) {
      collector->OnMessageReceived(received_message, now_nanoseconds.nanoseconds());
    }
  }

  /// 设置用于发布统计消息的定时器。
  /// Set the timer used to publish statistics messages.
  /**
   * \param publisher_timer 发布器定时器，由节点创建
   * \param publisher_timer the timer to fire the publisher, created by the node
   */
  void set_publisher_timer(rclcpp::TimerBase::SharedPtr publisher_timer) {
    // 将传入的定时器设置为成员变量 publisher_timer_
    // Set the incoming timer as the member variable publisher_timer_
    publisher_timer_ = publisher_timer;
  }

  /// 发布一个填充好的 MetricsStatisticsMessage。
  /// Publish a populated MetricsStatisticsMessage.
  /**
   * 此方法获取锁以防止收集器列表的竞争条件。
   * This method acquires a lock to prevent race conditions to collectors list.
   */
  virtual void publish_message_and_reset_measurements() {
    // 创建一个 MetricsMessage 类型的向量 msgs
    // Create a vector of type MetricsMessage called msgs
    std::vector<MetricsMessage> msgs;

    // 获取当前纳秒级的时间作为窗口结束时间
    // Get the current nanoseconds since epoch as the window end time
    rclcpp::Time window_end{get_current_nanoseconds_since_epoch()};

    {
      // 使用互斥锁保护以下代码块
      // Protect the following code block with a mutex lock
      std::lock_guard<std::mutex> lock(mutex_);

      // 遍历 subscriber_statistics_collectors_ 中的每个收集器
      // Iterate through each collector in subscriber_statistics_collectors_
      for (auto& collector : subscriber_statistics_collectors_) {
        // 获取收集器的统计结果
        // Get the statistics results from the collector
        const auto collected_stats = collector->GetStatisticsResults();

        // 清除收集器的当前度量
        // Clear the current measurements of the collector
        collector->ClearCurrentMeasurements();

        // 生成统计消息
        // Generate the statistics message
        auto message = libstatistics_collector::collector::GenerateStatisticMessage(
            node_name_, collector->GetMetricName(), collector->GetMetricUnit(), window_start_,
            window_end, collected_stats);

        // 将生成的消息添加到 msgs 向量中
        // Add the generated message to the msgs vector
        msgs.push_back(message);
      }
    }

    // 遍历 msgs 向量中的每个消息
    // Iterate through each message in the msgs vector
    for (auto& msg : msgs) {
      // 使用发布器发布消息
      // Publish the message using the publisher
      publisher_->publish(msg);
    }

    // 将窗口开始时间设置为窗口结束时间
    // Set the window start time to the window end time
    window_start_ = window_end;
  }

protected:
  /// 返回当前收集到的所有数据的向量。
  /// Return a vector of all the currently collected data.
  /**
   * 该方法获取锁以防止对收集器列表的竞争条件。
   * This method acquires a lock to prevent race conditions to collectors list.
   *
   * \return 包含所有收集到的数据的向量
   * \return a vector of all the collected data
   */
  std::vector<StatisticData> get_current_collector_data() const {
    // 创建一个空的 StatisticData 类型的向量，用于存储收集到的数据。
    // Create an empty vector of type StatisticData to store the collected data.
    std::vector<StatisticData> data;

    // 使用 lock_guard 对象来保护 mutex_，防止多线程访问时出现竞争条件。
    // Use a lock_guard object to protect mutex_, preventing race conditions when accessing in
    // multi-threading.
    std::lock_guard<std::mutex> lock(mutex_);

    // 遍历 subscriber_statistics_collectors_ 中的每个收集器。
    // Iterate through each collector in subscriber_statistics_collectors_.
    for (const auto& collector : subscriber_statistics_collectors_) {
      // 将收集器的统计结果添加到 data 向量中。
      // Add the collector's statistics results to the data vector.
      data.push_back(collector->GetStatisticsResults());
    }

    // 返回包含所有收集到的数据的向量。
    // Return the vector containing all the collected data.
    return data;
  }

private:
  /// 构造并启动所有收集器，并设置 window_start_。
  /// Construct and start all collectors and set window_start_.
  /**
   * 此方法获取锁以防止对收集器列表的竞争条件。
   * This method acquires a lock to prevent race conditions to the collectors list.
   */
  void bring_up() {
    // 创建一个 ReceivedMessageAge 对象，并使用 std::make_unique 进行内存分配。
    // Create a ReceivedMessageAge object and allocate memory using std::make_unique.
    auto received_message_age = std::make_unique<ReceivedMessageAge>();

    // 启动 received_message_age 收集器。
    // Start the received_message_age collector.
    received_message_age->Start();

    // 将 received_message_age 添加到 subscriber_statistics_collectors_ 列表中。
    // Add received_message_age to the subscriber_statistics_collectors_ list.
    subscriber_statistics_collectors_.emplace_back(std::move(received_message_age));

    // 创建一个 ReceivedMessagePeriod 对象，并使用 std::make_unique 进行内存分配。
    // Create a ReceivedMessagePeriod object and allocate memory using std::make_unique.
    auto received_message_period = std::make_unique<ReceivedMessagePeriod>();

    // 启动 received_message_period 收集器。
    // Start the received_message_period collector.
    received_message_period->Start();

    {
      // 获取锁，以防止对收集器列表的竞争条件。
      // Acquire a lock to prevent race conditions to the collectors list.
      std::lock_guard<std::mutex> lock(mutex_);

      // 将 received_message_period 添加到 subscriber_statistics_collectors_ 列表中。
      // Add received_message_period to the subscriber_statistics_collectors_ list.
      subscriber_statistics_collectors_.emplace_back(std::move(received_message_period));
    }

    // 设置 window_start_ 为当前纳秒级时间。
    // Set window_start_ to the current nanoseconds time.
    window_start_ = rclcpp::Time(get_current_nanoseconds_since_epoch());
  }

  /// 停止所有收集器，清除测量数据，停止发布计时器，并重置发布器。
  /// Stop all collectors, clear measurements, stop publishing timer, and reset publisher.
  /**
   * 此方法获取锁以防止对收集器列表的竞争条件。
   * This method acquires a lock to prevent race conditions to the collectors list.
   */
  void tear_down() {
    {
      // 获取锁，以防止对收集器列表的竞争条件。
      // Acquire a lock to prevent race conditions to the collectors list.
      std::lock_guard<std::mutex> lock(mutex_);

      // 遍历 subscriber_statistics_collectors_ 列表中的每个收集器，并停止它们。
      // Iterate through each collector in the subscriber_statistics_collectors_ list and stop them.
      for (auto& collector : subscriber_statistics_collectors_) {
        collector->Stop();
      }

      // 清空 subscriber_statistics_collectors_ 列表。
      // Clear the subscriber_statistics_collectors_ list.
      subscriber_statistics_collectors_.clear();
    }

    // 如果存在 publisher_timer_，则取消并重置它。
    // If publisher_timer_ exists, cancel and reset it.
    if (publisher_timer_) {
      publisher_timer_->cancel();
      publisher_timer_.reset();
    }

    // 重置 publisher_。
    // Reset the publisher_.
    publisher_.reset();
  }

  /// 返回自纪元以来的当前纳秒数（计数）。
  /// Return the current nanoseconds (count) since epoch.
  /**
   * \return 自纪元以来的当前纳秒数（计数）
   * \return the current nanoseconds (count) since epoch
   */
  int64_t get_current_nanoseconds_since_epoch() const {
    // 获取当前系统时间
    // Get the current system time
    const auto now = std::chrono::system_clock::now();

    // 将当前时间转换为纳秒，并返回其计数值
    // Convert the current time to nanoseconds and return its count value
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  }

  // 保护子序列向量的互斥锁
  // Mutex to protect the subsequence vectors
  mutable std::mutex mutex_;

  // 统计收集器的集合
  // Collection of statistics collectors
  std::vector<std::unique_ptr<TopicStatsCollector>> subscriber_statistics_collectors_{};

  // 用于生成要发布的主题统计消息的节点名称
  // Node name used to generate topic statistics messages to be published
  const std::string node_name_;

  // 由节点创建的发布者，用于发布主题统计消息
  // Publisher, created by the node, used to publish topic statistics messages
  rclcpp::Publisher<statistics_msgs::msg::MetricsMessage>::SharedPtr publisher_;

  // 触发发布器的定时器
  // Timer which fires the publisher
  rclcpp::TimerBase::SharedPtr publisher_timer_;

  // 发布的主题统计消息中使用的收集窗口的开始
  // The start of the collection window, used in the published topic statistics message
  rclcpp::Time window_start_;
};
}  // namespace topic_statistics
}  // namespace rclcpp

#endif  // RCLCPP__TOPIC_STATISTICS__SUBSCRIPTION_TOPIC_STATISTICS_HPP_

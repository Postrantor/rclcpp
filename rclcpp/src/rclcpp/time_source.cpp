// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/time_source.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "rclcpp/time.hpp"

namespace rclcpp {

/**
 * @class ClocksState
 * @brief 管理与ROS时间相关的一组时钟的状态 (Manages the state of a group of clocks related to ROS
 * time)
 */
class ClocksState final {
public:
  /**
   * @brief 构造函数 (Constructor)
   */
  ClocksState() : logger_(rclcpp::get_logger("rclcpp")) {}

  /**
   * @brief 内部方法，用于在时钟回调中启用所有时钟的迭代 (An internal method to use in the clock
   * callback that iterates and enables all clocks)
   */
  void enable_ros_time() {
    // 如果已经启用，则无操作 (If already enabled, no-op)
    if (ros_time_active_) {
      return;
    }

    // 本地存储 (Local storage)
    ros_time_active_ = true;

    // 更新所有附加时钟为零或最后记录的时间 (Update all attached clocks to zero or last recorded
    // time)
    std::lock_guard<std::mutex> guard(clock_list_lock_);
    auto time_msg = std::make_shared<builtin_interfaces::msg::Time>();
    if (last_msg_set_) {
      time_msg = std::make_shared<builtin_interfaces::msg::Time>(last_msg_set_->clock);
    }
    for (auto it = associated_clocks_.begin(); it != associated_clocks_.end(); ++it) {
      set_clock(time_msg, true, *it);
    }
  }

  /**
   * @brief 内部方法，用于在时钟回调中禁用所有时钟的迭代 (An internal method to use in the clock
   * callback that iterates and disables all clocks)
   */
  void disable_ros_time() {
    // 如果已经禁用，则无操作 (If already disabled, no-op)
    if (!ros_time_active_) {
      return;
    }

    // 本地存储 (Local storage)
    ros_time_active_ = false;

    // 更新所有附加时钟 (Update all attached clocks)
    std::lock_guard<std::mutex> guard(clock_list_lock_);
    for (auto it = associated_clocks_.begin(); it != associated_clocks_.end(); ++it) {
      auto msg = std::make_shared<builtin_interfaces::msg::Time>();
      set_clock(msg, false, *it);
    }
  }

  /**
   * @brief 检查ROS时间是否激活 (Check if ROS time is active)
   * @return 是否激活 (Whether it's active or not)
   */
  bool is_ros_time_active() const { return ros_time_active_; }

  /**
   * @brief 附加一个时钟 (Attach a clock)
   * @param clock 要附加的时钟 (The clock to attach)
   */
  void attachClock(rclcpp::Clock::SharedPtr clock) {
    {
      std::lock_guard<std::mutex> clock_guard(clock->get_clock_mutex());
      if (clock->get_clock_type() != RCL_ROS_TIME && ros_time_active_) {
        throw std::invalid_argument(
            "ros_time_active_ can't be true while clock is not of RCL_ROS_TIME type");
      }
    }
    std::lock_guard<std::mutex> guard(clock_list_lock_);
    associated_clocks_.push_back(clock);
    // 设置时钟为零，除非有最近接收到的消息 (Set the clock to zero unless there's a recently
    // received message)
    auto time_msg = std::make_shared<builtin_interfaces::msg::Time>();
    if (last_msg_set_) {
      time_msg = std::make_shared<builtin_interfaces::msg::Time>(last_msg_set_->clock);
    }
    set_clock(time_msg, ros_time_active_, clock);
  }

  /**
   * @brief 分离一个时钟 (Detach a clock)
   * @param clock 要分离的时钟 (The clock to detach)
   */
  void detachClock(rclcpp::Clock::SharedPtr clock) {
    std::lock_guard<std::mutex> guard(clock_list_lock_);
    auto result = std::find(associated_clocks_.begin(), associated_clocks_.end(), clock);
    if (result != associated_clocks_.end()) {
      associated_clocks_.erase(result);
    } else {
      RCLCPP_ERROR(logger_, "failed to remove clock");
    }
  }

  /**
   * @brief 在迭代器内部使用的内部辅助函数 (Internal helper function used inside iterators)
   * @param msg 时间消息 (Time message)
   * @param set_ros_time_enabled 是否启用ROS时间 (Whether to enable ROS time or not)
   * @param clock 要设置的时钟 (The clock to set)
   */
  static void set_clock(
      const builtin_interfaces::msg::Time::SharedPtr msg,
      bool set_ros_time_enabled,
      rclcpp::Clock::SharedPtr clock) {
    std::lock_guard<std::mutex> clock_guard(clock->get_clock_mutex());

    if (clock->get_clock_type() == RCL_ROS_TIME) {
      // 执行更改 (Do change)
      if (!set_ros_time_enabled && clock->ros_time_is_active()) {
        auto ret = rcl_disable_ros_time_override(clock->get_clock_handle());
        if (ret != RCL_RET_OK) {
          rclcpp::exceptions::throw_from_rcl_error(
              ret, "Failed to disable ros_time_override_status");
        }
      } else if (set_ros_time_enabled && !clock->ros_time_is_active()) {
        auto ret = rcl_enable_ros_time_override(clock->get_clock_handle());
        if (ret != RCL_RET_OK) {
          rclcpp::exceptions::throw_from_rcl_error(
              ret, "Failed to enable ros_time_override_status");
        }
      }

      auto ret =
          rcl_set_ros_time_override(clock->get_clock_handle(), rclcpp::Time(*msg).nanoseconds());
      if (ret != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to set ros_time_override_status");
      }
    } else if (set_ros_time_enabled) {
      throw std::invalid_argument(
          "set_ros_time_enabled can't be true while clock is not of RCL_ROS_TIME type");
    }
  }

  /**
   * @brief 内部辅助函数 (Internal helper function)
   * @param msg 时间消息 (Time message)
   * @param set_ros_time_enabled 是否启用ROS时间 (Whether to enable ROS time or not)
   */
  void set_all_clocks(
      const builtin_interfaces::msg::Time::SharedPtr msg, bool set_ros_time_enabled) {
    std::lock_guard<std::mutex> guard(clock_list_lock_);
    for (auto it = associated_clocks_.begin(); it != associated_clocks_.end(); ++it) {
      set_clock(msg, set_ros_time_enabled, *it);
    }
  }

  /**
   * @brief 缓存收到的最后一个时钟消息 (Cache the last clock message received)
   * @param msg 时钟消息 (Clock message)
   */
  void cache_last_msg(std::shared_ptr<const rosgraph_msgs::msg::Clock> msg) { last_msg_set_ = msg; }

  /**
   * @brief 检查所有时钟是否为RCL_ROS_TIME类型 (Check if all clocks are of RCL_ROS_TIME type)
   * @return 是/否 (True/False)
   */
  bool are_all_clocks_rcl_ros_time() {
    std::lock_guard<std::mutex> guard(clock_list_lock_);
    for (auto& clock : associated_clocks_) {
      std::lock_guard<std::mutex> clock_guard(clock->get_clock_mutex());
      if (clock->get_clock_type() != RCL_ROS_TIME) {
        return false;
      }
    }
    return true;
  }

private:
  // 存储（并在节点附加时更新）用于记录的日志记录器 (Store (and update on node attach) logger for
  // logging)
  Logger logger_;

  // 保护迭代associated_clocks_字段的锁 (A lock to protect iterating the associated_clocks_ field)
  std::mutex clock_list_lock_;
  // 存储关联时钟引用的向量 (A vector to store references to associated clocks)
  std::vector<rclcpp::Clock::SharedPtr> associated_clocks_;

  // ROS时间有效性的本地存储 (Local storage of validity of ROS time)
  // 添加新时钟时需要这个 (This is needed when new clocks are added)
  bool ros_time_active_{false};
  // 最后设置的消息，以传递给新注册的时钟 (Last set message to be passed to newly registered clocks)
  std::shared_ptr<const rosgraph_msgs::msg::Clock> last_msg_set_;
};

/**
 * @class TimeSource::NodeState
 * @brief 一个封装了节点状态的类，用于管理节点的各种资源和设置。
 *        A class that encapsulates the node state, used for managing various resources and settings
 * of the node.
 */
class TimeSource::NodeState final {
public:
  /**
   * @brief 构造函数，初始化 NodeState 对象。
   *        Constructor, initializes a NodeState object.
   * @param qos rclcpp::QoS 对象，用于设置节点的 Quality of Service 策略。
   *            An rclcpp::QoS object, used for setting the node's Quality of Service policy.
   * @param use_clock_thread 布尔值，表示是否使用时钟线程。
   *                         Boolean value, indicating whether to use a clock thread or not.
   */
  NodeState(const rclcpp::QoS& qos, bool use_clock_thread)
      : use_clock_thread_(use_clock_thread), logger_(rclcpp::get_logger("rclcpp")), qos_(qos) {}

  /**
   * @brief 析构函数，释放 NodeState 对象的资源。
   *        Destructor, releases resources of the NodeState object.
   */
  ~NodeState() {
    // 如果存在任何节点资源，则在析构时分离它们。
    // If any node resources exist, detach them upon destruction.
    if (node_base_ || node_topics_ || node_graph_ || node_services_ || node_logging_ ||
        node_clock_ || node_parameters_) {
      detachNode();
    }
  }

  /**
   * @brief 检查是否使用时钟线程 (Check if a clock thread will be used)
   *
   * @return bool 返回是否使用时钟线程的标志 (Return the flag indicating whether the clock thread is
   * being used)
   */
  bool get_use_clock_thread() { return use_clock_thread_; }

  /**
   * @brief 设置是否使用时钟线程 (Set whether a clock thread will be used)
   *
   * @param use_clock_thread 是否使用时钟线程的标志 (Flag indicating whether to use the clock
   * thread)
   */
  void set_use_clock_thread(bool use_clock_thread) { use_clock_thread_ = use_clock_thread; }

  /**
   * @brief 检查时钟线程是否可连接 (Check if the clock thread is joinable)
   *
   * @return bool 返回时钟线程是否可连接的标志 (Return the flag indicating whether the clock thread
   * is joinable)
   */
  bool clock_thread_is_joinable() { return clock_executor_thread_.joinable(); }

  /**
   * @brief 将节点附加到此时间源 (Attach a node to this time source)
   *
   * @param node_base_interface 节点的基本接口指针 (Pointer to the node's base interface)
   * @param node_topics_interface 节点的话题接口指针 (Pointer to the node's topics interface)
   * @param node_graph_interface 节点的图接口指针 (Pointer to the node's graph interface)
   * @param node_services_interface 节点的服务接口指针 (Pointer to the node's services interface)
   * @param node_logging_interface 节点的日志接口指针 (Pointer to the node's logging interface)
   * @param node_clock_interface 节点的时钟接口指针 (Pointer to the node's clock interface)
   * @param node_parameters_interface 节点的参数接口指针 (Pointer to the node's parameters
   * interface)
   */
  void attachNode(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
      rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface) {
    // 将各个接口附加到节点上 (Attach the various interfaces to the node)
    node_base_ = node_base_interface;
    node_topics_ = node_topics_interface;
    node_graph_ = node_graph_interface;
    node_services_ = node_services_interface;
    node_logging_ = node_logging_interface;
    node_clock_ = node_clock_interface;
    node_parameters_ = node_parameters_interface;

    // TODO(tfoote): 更新 QOS (Update QOS)

    // 获取节点日志记录器 (Get the node logger)
    logger_ = node_logging_->get_logger();

    // 如果节点有初始参数值，可以通过用户在节点构造时或命令行参数给出，这里的默认值为 false
    // (Though this defaults to false, it can be overridden by initial parameter values for the
    // node, which may be given by the user at the node's construction or even by command-line
    // arguments.)
    rclcpp::ParameterValue use_sim_time_param;
    const std::string use_sim_time_name = "use_sim_time";
    if (!node_parameters_->has_parameter(use_sim_time_name)) {
      use_sim_time_param =
          node_parameters_->declare_parameter(use_sim_time_name, rclcpp::ParameterValue(false));
    } else {
      use_sim_time_param = node_parameters_->get_parameter(use_sim_time_name).get_parameter_value();
    }
    // 如果参数类型为 bool，则根据参数值设置时钟状态 (If the parameter type is bool, set the clock
    // state based on the parameter value)
    if (use_sim_time_param.get_type() == rclcpp::PARAMETER_BOOL) {
      if (use_sim_time_param.get<bool>()) {
        parameter_state_ = SET_TRUE;
        clocks_state_.enable_ros_time();
        create_clock_sub();
      }
    } else {
      // 参数类型无效，抛出异常 (Invalid parameter type, throw an exception)
      RCLCPP_ERROR(
          logger_, "Invalid type '%s' for parameter 'use_sim_time', should be 'bool'",
          rclcpp::to_string(use_sim_time_param.get_type()).c_str());
      throw std::invalid_argument("Invalid type for parameter 'use_sim_time', should be 'bool'");
    }

    // 添加参数设置回调 (Add a callback for setting parameters)
    on_set_parameters_callback_ = node_parameters_->add_on_set_parameters_callback(
        std::bind(&TimeSource::NodeState::on_set_parameters, this, std::placeholders::_1));

    // TODO(tfoote) 使用参数接口而不是通过话题订阅事件 (Use parameters interface not subscribe to
    // events via topic ticketed #609) 设置参数事件回调 (Set up the parameter event callback)
    parameter_subscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
        node_topics_, [this](std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event) {
          if (node_base_ != nullptr) {
            this->on_parameter_event(event);
          }
          // 如果 node_base_ 为空指针，则什么都不做，因为这意味着 TimeSource 现在没有附加节点
          // (Do nothing if node_base_ is nullptr because it means the TimeSource is now
          // without an attached node)
        });
  }

  /**
   * @brief 分离已附加的节点 (Detach the attached node)
   */
  void detachNode() {
    // 在清理过程中，首先确保执行器不能调用任何回调 (destroy_clock_sub() *must* be first here, to
    // ensure that the executor can't possibly call any of the callbacks as we are cleaning up.)
    destroy_clock_sub();
    clocks_state_.disable_ros_time();
    if (on_set_parameters_callback_) {
      node_parameters_->remove_on_set_parameters_callback(on_set_parameters_callback_.get());
    }
    on_set_parameters_callback_.reset();
    parameter_subscription_.reset();
    node_base_.reset();
    node_topics_.reset();
    node_graph_.reset();
    node_services_.reset();
    node_logging_.reset();
    node_clock_.reset();
    node_parameters_.reset();
  }

  /**
   * @brief 将时钟附加到时间源 (Attach a clock to the time source)
   *
   * @param clock 要附加的时钟指针 (Pointer to the clock to attach)
   */
  void attachClock(std::shared_ptr<rclcpp::Clock> clock) {
    clocks_state_.attachClock(std::move(clock));
  }

  /**
   * @brief 从时间源分离时钟 (Detach a clock from the time source)
   *
   * @param clock 要分离的时钟指针 (Pointer to the clock to detach)
   */
  void detachClock(std::shared_ptr<rclcpp::Clock> clock) {
    clocks_state_.detachClock(std::move(clock));
  }

private:
  ClocksState clocks_state_;  // 时钟状态对象 (Clock state object)

  // 用于时钟订阅的专用线程 (Dedicated thread for clock subscription)
  bool use_clock_thread_;
  std::thread clock_executor_thread_;

  // 保留节点引用 (Preserve the node reference)
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_{nullptr};
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_{nullptr};
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_{nullptr};
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_{nullptr};
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_{nullptr};
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_{nullptr};
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_{nullptr};

  // 存储（并在节点附加时更新）日志记录器以进行日志记录 (Store (and update on node attach) logger
  // for logging)
  Logger logger_;

  // 时钟订阅的 QoS (QoS of the clock subscription)
  rclcpp::QoS qos_;

  // 时钟回调的订阅 (The subscription for the clock callback)
  using SubscriptionT = rclcpp::Subscription<rosgraph_msgs::msg::Clock>;
  std::shared_ptr<SubscriptionT> clock_subscription_{nullptr};
  std::mutex clock_sub_lock_;
  rclcpp::CallbackGroup::SharedPtr clock_callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr clock_executor_;
  std::promise<void> cancel_clock_executor_promise_;

  /**
   * @brief 时钟回调函数
   * @param msg 消息指针，类型为 std::shared_ptr<const rosgraph_msgs::msg::Clock>
   *
   * @brief Clock callback function
   * @param msg Message pointer, type is std::shared_ptr<const rosgraph_msgs::msg::Clock>
   */
  void clock_cb(std::shared_ptr<const rosgraph_msgs::msg::Clock> msg) {
    // 判断当前时钟状态是否为 ROS 时间激活状态，并且参数状态为 SET_TRUE
    // Check if the current clock state is ROS time active and the parameter state is SET_TRUE
    if (!clocks_state_.is_ros_time_active() && SET_TRUE == this->parameter_state_) {
      clocks_state_.enable_ros_time();
    }
    // 缓存最后一条消息，以防新时钟被附加
    // Cache the last message in case a new clock is attached
    clocks_state_.cache_last_msg(msg);
    auto time_msg = std::make_shared<builtin_interfaces::msg::Time>(msg->clock);

    // 如果参数状态为 SET_TRUE，则设置所有时钟
    // Set all clocks if the parameter state is SET_TRUE
    if (SET_TRUE == this->parameter_state_) {
      clocks_state_.set_all_clocks(time_msg, true);
    }
  }

  /**
   * @brief 创建时钟主题订阅
   *
   * @brief Create the subscription for the clock topic
   */
  void create_clock_sub() {
    std::lock_guard<std::mutex> guard(clock_sub_lock_);
    if (clock_subscription_) {
      // 订阅已创建
      // Subscription already created
      return;
    }

    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions({
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Reliability,
    });

    // 判断是否使用时钟线程
    // Check if using clock thread
    if (use_clock_thread_) {
      clock_callback_group_ =
          node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
      options.callback_group = clock_callback_group_;
      rclcpp::ExecutorOptions exec_options;
      exec_options.context = node_base_->get_context();
      clock_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(exec_options);
      if (!clock_executor_thread_.joinable()) {
        cancel_clock_executor_promise_ = std::promise<void>{};
        clock_executor_thread_ = std::thread([this]() {
          auto future = cancel_clock_executor_promise_.get_future();
          clock_executor_->add_callback_group(clock_callback_group_, node_base_);
          clock_executor_->spin_until_future_complete(future);
        });
      }
    }

    // 创建时钟主题订阅
    // Create clock topic subscription
    clock_subscription_ = rclcpp::create_subscription<rosgraph_msgs::msg::Clock>(
        node_parameters_, node_topics_, "/clock", qos_,
        [this](std::shared_ptr<const rosgraph_msgs::msg::Clock> msg) {
          // 我们使用 node_base_ 作为节点附加的指示
          // 只有在这种情况下才调用 clock_cb 函数
          // We are using node_base_ as an indication if there is a node attached
          // Only call the clock_cb if that is the case
          if (node_base_ != nullptr) {
            clock_cb(msg);
          }
        },
        options);
  }

  /**
   * @brief 销毁时钟主题订阅
   *
   * @brief Destroy the subscription for the clock topic
   */
  void destroy_clock_sub() {
    std::lock_guard<std::mutex> guard(clock_sub_lock_);
    if (clock_executor_thread_.joinable()) {
      cancel_clock_executor_promise_.set_value();
      clock_executor_->cancel();
      clock_executor_thread_.join();
      clock_executor_->remove_callback_group(clock_callback_group_);
    }
    clock_subscription_.reset();
  }

  // 设置参数回调处理句柄
  // On set Parameters callback handle
  node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_{nullptr};

  // 参数事件订阅
  // Parameter Event subscription
  using ParamSubscriptionT = rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>;
  std::shared_ptr<ParamSubscriptionT> parameter_subscription_;

  /**
   * @brief 参数设置回调函数
   * @param parameters 参数向量，类型为 std::vector<rclcpp::Parameter>
   * @return 返回设置参数结果，类型为 rcl_interfaces::msg::SetParametersResult
   *
   * @brief Callback for parameter settings
   * @param parameters Parameter vector, type is std::vector<rclcpp::Parameter>
   * @return Returns the setting parameter result, type is rcl_interfaces::msg::SetParametersResult
   */
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
      const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& param : parameters) {
      if (param.get_name() == "use_sim_time" && param.get_type() == rclcpp::PARAMETER_BOOL) {
        if (param.as_bool() && !(clocks_state_.are_all_clocks_rcl_ros_time())) {
          result.successful = false;
          result.reason =
              "use_sim_time parameter can't be true while clocks are not all of RCL_ROS_TIME type";
          RCLCPP_ERROR(
              logger_,
              "use_sim_time parameter can't be true while clocks are not all of RCL_ROS_TIME type");
        }
      }
    }
    return result;
  }

  /**
   * @brief 回调函数，用于处理参数更新事件
   * @param event 参数更新事件的共享指针
   *
   * Callback function for handling parameter update events.
   * @param event Shared pointer to the parameter update event
   */
  void on_parameter_event(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event) {
    // 过滤其他节点上 'use_sim_time' 参数实例的事件
    // Filter out events on 'use_sim_time' parameter instances in other nodes.
    if (event->node != node_base_->get_fully_qualified_name()) {
      return;
    }
    // 只过滤 'use_sim_time' 被添加或更改的事件
    // Filter for only 'use_sim_time' being added or changed.
    rclcpp::ParameterEventsFilter filter(
        event, {"use_sim_time"},
        {rclcpp::ParameterEventsFilter::EventType::NEW,
         rclcpp::ParameterEventsFilter::EventType::CHANGED});
    for (auto& it : filter.get_events()) {
      // 如果 'use_sim_time' 参数的类型不是布尔值，则报错
      // If the 'use_sim_time' parameter type is not a bool, report an error.
      if (it.second->value.type != ParameterType::PARAMETER_BOOL) {
        RCLCPP_ERROR(logger_, "use_sim_time parameter cannot be set to anything but a bool");
        continue;
      }
      // 根据 'use_sim_time' 参数的布尔值来设置状态，并启用/禁用 ROS 时间
      // Set the state based on the 'use_sim_time' parameter's bool value and enable/disable ROS
      // time.
      if (it.second->value.bool_value) {
        parameter_state_ = SET_TRUE;
        clocks_state_.enable_ros_time();
        create_clock_sub();
      } else {
        parameter_state_ = SET_FALSE;
        destroy_clock_sub();
        clocks_state_.disable_ros_time();
      }
    }
    // 处理 'use_sim_time' 参数被删除的情况
    // Handle the case that use_sim_time was deleted.
    rclcpp::ParameterEventsFilter deleted(
        event, {"use_sim_time"}, {rclcpp::ParameterEventsFilter::EventType::DELETED});
    for (auto& it : deleted.get_events()) {
      (void)it;  // 如果有匹配项，它已经匹配，不需要阅读它
                 // If there is a match, it's already matched, don't bother reading it.
      // 如果参数被删除，则将其标记为未设置，但不更改状态
      // If the parameter is deleted, mark it as unset but don't change state.
      parameter_state_ = UNSET;
    }
  }

  // 用于保存参数状态的枚举类型
  // An enum to hold the parameter state
  enum UseSimTimeParameterState { UNSET, SET_TRUE, SET_FALSE };
  UseSimTimeParameterState parameter_state_;
};

// TimeSource 类构造函数（基于节点）
// TimeSource class constructor (based on node)
TimeSource::TimeSource(
    std::shared_ptr<rclcpp::Node> node, const rclcpp::QoS& qos, bool use_clock_thread)
    : TimeSource(qos, use_clock_thread) {
  attachNode(node);
}

// TimeSource 类构造函数（基于 QoS 和使用时钟线程）
// TimeSource class constructor (based on QoS and using clock thread)
TimeSource::TimeSource(const rclcpp::QoS& qos, bool use_clock_thread)
    : constructed_use_clock_thread_(use_clock_thread), constructed_qos_(qos) {
  node_state_ = std::make_shared<NodeState>(qos, use_clock_thread);
}

// 将节点附加到 TimeSource
// Attach a node to the TimeSource
void TimeSource::attachNode(rclcpp::Node::SharedPtr node) {
  node_state_->set_use_clock_thread(node->get_node_options().use_clock_thread());
  attachNode(
      node->get_node_base_interface(), node->get_node_topics_interface(),
      node->get_node_graph_interface(), node->get_node_services_interface(),
      node->get_node_logging_interface(), node->get_node_clock_interface(),
      node->get_node_parameters_interface());
}

// 将各个接口附加到 TimeSource
// Attach various interfaces to the TimeSource
void TimeSource::attachNode(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface) {
  node_state_->attachNode(
      std::move(node_base_interface), std::move(node_topics_interface),
      std::move(node_graph_interface), std::move(node_services_interface),
      std::move(node_logging_interface), std::move(node_clock_interface),
      std::move(node_parameters_interface));
}

// 从 TimeSource 中分离节点
// Detach a node from the TimeSource
void TimeSource::detachNode() {
  node_state_.reset();
  node_state_ = std::make_shared<NodeState>(constructed_qos_, constructed_use_clock_thread_);
}

// 将时钟附加到 TimeSource
// Attach a clock to the TimeSource
void TimeSource::attachClock(std::shared_ptr<rclcpp::Clock> clock) {
  node_state_->attachClock(std::move(clock));
}

// 从 TimeSource 中分离时钟
// Detach a clock from the TimeSource
void TimeSource::detachClock(std::shared_ptr<rclcpp::Clock> clock) {
  node_state_->detachClock(std::move(clock));
}

// 获取是否使用时钟线程的状态
// Get the state of whether to use the clock thread or not
bool TimeSource::get_use_clock_thread() { return node_state_->get_use_clock_thread(); }

// 设置是否使用时钟线程
// Set whether to use the clock thread or not
void TimeSource::set_use_clock_thread(bool use_clock_thread) {
  node_state_->set_use_clock_thread(use_clock_thread);
}

// 检查时钟线程是否可连接
// Check if the clock thread is joinable
bool TimeSource::clock_thread_is_joinable() { return node_state_->clock_thread_is_joinable(); }

// TimeSource 类析构函数
// TimeSource class destructor
TimeSource::~TimeSource() {}

}  // namespace rclcpp

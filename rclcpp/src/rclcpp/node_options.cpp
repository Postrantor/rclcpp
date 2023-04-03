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

#include "rclcpp/node_options.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/detail/utilities.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"

using rclcpp::exceptions::throw_from_rcl_error;

namespace rclcpp {

namespace detail {
// 定义一个静态函数，用于销毁 rcl_node_options_t 类型的对象
// Define a static function for destroying rcl_node_options_t objects
static void rcl_node_options_t_destructor(rcl_node_options_t *node_options) {
  if (node_options) {
    // 调用 rcl_node_options_fini 函数来释放 node_options 所占用的资源
    // Call rcl_node_options_fini to release the resources occupied by node_options
    rcl_ret_t ret = rcl_node_options_fini(node_options);
    if (RCL_RET_OK != ret) {
      // 如果释放失败，记录错误信息，但不抛出异常（因为可能在析构函数中调用）
      // If the release fails, log the error message but do not throw an exception (as it may be
      // called in the destructor)
      RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"), "failed to finalize rcl node options: %s",
          rcl_get_error_string().str);
      rcl_reset_error();
    }

    // 删除 node_options 对象，并将指针置空
    // Delete the node_options object and set the pointer to nullptr
    delete node_options;
    node_options = nullptr;
  }
}
}  // namespace detail

NodeOptions::NodeOptions(rcl_allocator_t allocator)
    : node_options_(nullptr, detail::rcl_node_options_t_destructor), allocator_(allocator) {}

NodeOptions::NodeOptions(const NodeOptions &other)
    : node_options_(nullptr, detail::rcl_node_options_t_destructor) {
  *this = other;
}

/**
 * @brief 重载等号运算符，用于赋值 NodeOptions 对象。
 * @brief Overload the assignment operator for assigning a NodeOptions object.
 *
 * @param other 另一个 NodeOptions 对象的引用。
 * @param other A reference to another NodeOptions object.
 * @return 返回 *this 的引用。
 * @return Returns a reference to *this.
 */
NodeOptions &NodeOptions::operator=(const NodeOptions &other) {
  // 检查是否为自赋值，如果是，则不执行任何操作。
  // Check for self-assignment, if true, do nothing.
  if (this != &other) {
    // 重置当前对象的 node_options_ 成员。
    // Reset the current object's node_options_ member.
    this->node_options_.reset();

    // 复制 other 对象的 context_ 成员到当前对象。
    // Copy the context_ member from the other object to the current object.
    this->context_ = other.context_;

    // 复制 other 对象的 arguments_ 成员到当前对象。
    // Copy the arguments_ member from the other object to the current object.
    this->arguments_ = other.arguments_;

    // 复制 other 对象的 parameter_overrides_ 成员到当前对象。
    // Copy the parameter_overrides_ member from the other object to the current object.
    this->parameter_overrides_ = other.parameter_overrides_;

    // 复制 other 对象的 use_global_arguments_ 成员到当前对象。
    // Copy the use_global_arguments_ member from the other object to the current object.
    this->use_global_arguments_ = other.use_global_arguments_;

    // 复制 other 对象的 enable_rosout_ 成员到当前对象。
    // Copy the enable_rosout_ member from the other object to the current object.
    this->enable_rosout_ = other.enable_rosout_;

    // 复制 other 对象的 use_intra_process_comms_ 成员到当前对象。
    // Copy the use_intra_process_comms_ member from the other object to the current object.
    this->use_intra_process_comms_ = other.use_intra_process_comms_;

    // 复制 other 对象的 enable_topic_statistics_ 成员到当前对象。
    // Copy the enable_topic_statistics_ member from the other object to the current object.
    this->enable_topic_statistics_ = other.enable_topic_statistics_;

    // 复制 other 对象的 start_parameter_services_ 成员到当前对象。
    // Copy the start_parameter_services_ member from the other object to the current object.
    this->start_parameter_services_ = other.start_parameter_services_;

    // 复制 other 对象的 start_parameter_event_publisher_ 成员到当前对象。
    // Copy the start_parameter_event_publisher_ member from the other object to the current object.
    this->start_parameter_event_publisher_ = other.start_parameter_event_publisher_;

    // 复制 other 对象的 clock_type_ 成员到当前对象。
    // Copy the clock_type_ member from the other object to the current object.
    this->clock_type_ = other.clock_type_;

    // 复制 other 对象的 clock_qos_ 成员到当前对象。
    // Copy the clock_qos_ member from the other object to the current object.
    this->clock_qos_ = other.clock_qos_;

    // 复制 other 对象的 use_clock_thread_ 成员到当前对象。
    // Copy the use_clock_thread_ member from the other object to the current object.
    this->use_clock_thread_ = other.use_clock_thread_;

    // 复制 other 对象的 parameter_event_qos_ 成员到当前对象。
    // Copy the parameter_event_qos_ member from the other object to the current object.
    this->parameter_event_qos_ = other.parameter_event_qos_;

    // 复制 other 对象的 rosout_qos_ 成员到当前对象。
    // Copy the rosout_qos_ member from the other object to the current object.
    this->rosout_qos_ = other.rosout_qos_;

    // 复制 other 对象的 parameter_event_publisher_options_ 成员到当前对象。
    // Copy the parameter_event_publisher_options_ member from the other object to the current
    // object.
    this->parameter_event_publisher_options_ = other.parameter_event_publisher_options_;

    // 复制 other 对象的 allow_undeclared_parameters_ 成员到当前对象。
    // Copy the allow_undeclared_parameters_ member from the other object to the current object.
    this->allow_undeclared_parameters_ = other.allow_undeclared_parameters_;

    // 复制 other 对象的 automatically_declare_parameters_from_overrides_ 成员到当前对象。
    // Copy the automatically_declare_parameters_from_overrides_ member from the other object to the
    // current object.
    this->automatically_declare_parameters_from_overrides_ =
        other.automatically_declare_parameters_from_overrides_;

    // 复制 other 对象的 allocator_ 成员到当前对象。
    // Copy the allocator_ member from the other object to the current object.
    this->allocator_ = other.allocator_;
  }

  // 返回 *this 的引用，以便链式赋值。
  // Return a reference to *this for chain assignment.
  return *this;
}

/**
 * @brief 获取 RCL 节点选项的函数 (Get the RCL node options function)
 *
 * @return 返回 rcl_node_options_t 类型的指针 (Returns a pointer of type rcl_node_options_t)
 */
const rcl_node_options_t *NodeOptions::get_rcl_node_options() const {
  // 如果 node_options_ 为空，则按需创建它 (If node_options_ is nullptr, create it on demand)
  // If node_options_ is nullptr, create it on demand
  if (!node_options_) {
    // 创建一个新的 rcl_node_options_t 对象并赋值给 node_options_ (Create a new rcl_node_options_t
    // object and assign it to node_options_) Create a new rcl_node_options_t object and assign it
    // to node_options_
    node_options_.reset(new rcl_node_options_t);

    // 使用默认的节点选项初始化 node_options_ (Initialize node_options_ with default node options)
    // Initialize node_options_ with default node options
    *node_options_ = rcl_node_get_default_options();

    // 设置分配器、全局参数、启用 rosout 和 rosout_qos (Set allocator, global arguments, enable
    // rosout and rosout_qos) Set allocator, global arguments, enable rosout and rosout_qos
    node_options_->allocator = this->allocator_;
    node_options_->use_global_arguments = this->use_global_arguments_;
    node_options_->enable_rosout = this->enable_rosout_;
    node_options_->rosout_qos = this->rosout_qos_.get_rmw_qos_profile();

    int c_argc = 0;
    std::unique_ptr<const char *[]> c_argv;

    // 如果参数列表不为空 (If the argument list is not empty)
    // If the argument list is not empty
    if (!this->arguments_.empty()) {
      // 检查参数数量是否超过 int 类型的最大值 (Check if the number of arguments exceeds the maximum
      // value of the int type) Check if the number of arguments exceeds the maximum value of the
      // int type
      if (this->arguments_.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "Too many args");
      }

      // 转换参数列表大小为 int 类型并赋值给 c_argc (Convert the size of the argument list to int
      // type and assign it to c_argc) Convert the size of the argument list to int type and assign
      // it to c_argc
      c_argc = static_cast<int>(this->arguments_.size());

      // 为参数分配内存空间 (Allocate memory space for the arguments)
      // Allocate memory space for the arguments
      c_argv.reset(new const char *[c_argc]);

      // 将参数从 std::string 类型转换为 const char * 类型 (Convert the arguments from std::string
      // type to const char * type) Convert the arguments from std::string type to const char * type
      for (std::size_t i = 0; i < this->arguments_.size(); ++i) {
        c_argv[i] = this->arguments_[i].c_str();
      }
    }

    // 解析参数并将结果赋值给 node_options_->arguments (Parse the arguments and assign the result to
    // node_options_->arguments) Parse the arguments and assign the result to
    // node_options_->arguments
    rcl_ret_t ret =
        rcl_parse_arguments(c_argc, c_argv.get(), this->allocator_, &(node_options_->arguments));

    // 如果解析参数失败，抛出异常 (If parsing the arguments fails, throw an exception)
    // If parsing the arguments fails, throw an exception
    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to parse arguments");
    }

    // 获取未解析的 ROS 参数 (Get unparsed ROS arguments)
    // Get unparsed ROS arguments
    std::vector<std::string> unparsed_ros_arguments = detail::get_unparsed_ros_arguments(
        c_argc, c_argv.get(), &(node_options_->arguments), this->allocator_);

    // 如果有未解析的 ROS 参数，抛出异常 (If there are unparsed ROS arguments, throw an exception)
    // If there are unparsed ROS arguments, throw an exception
    if (!unparsed_ros_arguments.empty()) {
      throw exceptions::UnknownROSArgsError(std::move(unparsed_ros_arguments));
    }
  }

  // 返回 node_options_ 的指针 (Return the pointer of node_options_)
  // Return the pointer of node_options_
  return node_options_.get();
}

/**
 * @brief 获取当前节点选项的上下文 (Get the context of the current node options)
 * @return 返回共享指针类型的上下文对象 (Return the shared pointer type context object)
 */
rclcpp::Context::SharedPtr NodeOptions::context() const { return this->context_; }

/**
 * @brief 设置节点选项的上下文 (Set the context of the node options)
 * @param context 共享指针类型的上下文对象 (Shared pointer type context object)
 * @return 返回设置后的节点选项引用 (Return the reference to the set node options)
 */
NodeOptions &NodeOptions::context(rclcpp::Context::SharedPtr context) {
  this->context_ = context;  // 设置上下文对象 (Set the context object)
  return *this;
}

/**
 * @brief 获取当前节点选项的参数列表 (Get the argument list of the current node options)
 * @return 返回字符串向量类型的参数列表 (Return the string vector type argument list)
 */
const std::vector<std::string> &NodeOptions::arguments() const { return this->arguments_; }

/**
 * @brief 设置节点选项的参数列表 (Set the argument list of the node options)
 * @param arguments 字符串向量类型的参数列表 (String vector type argument list)
 * @return 返回设置后的节点选项引用 (Return the reference to the set node options)
 */
NodeOptions &NodeOptions::arguments(const std::vector<std::string> &arguments) {
  this->node_options_.reset();  // 重置节点选项以便在下次访问时重新创建 (Reset node options to make
                                // it be recreated on next access)
  this->arguments_ = arguments;  // 设置参数列表 (Set the argument list)
  return *this;
}

/**
 * @brief 获取当前节点选项的参数覆盖列表 (Get the parameter override list of the current node
 * options)
 * @return 返回参数向量类型的参数覆盖列表 (Return the parameter vector type parameter override list)
 */
std::vector<rclcpp::Parameter> &NodeOptions::parameter_overrides() {
  return this->parameter_overrides_;
}

/**
 * @brief 获取当前节点选项的参数覆盖列表 (Get the parameter override list of the current node
 * options)
 * @return 返回参数向量类型的参数覆盖列表 (Return the parameter vector type parameter override list)
 */
const std::vector<rclcpp::Parameter> &NodeOptions::parameter_overrides() const {
  return this->parameter_overrides_;
}

/**
 * @brief 设置节点选项的参数覆盖列表 (Set the parameter override list of the node options)
 * @param parameter_overrides 参数向量类型的参数覆盖列表 (Parameter vector type parameter override
 * list)
 * @return 返回设置后的节点选项引用 (Return the reference to the set node options)
 */
NodeOptions &NodeOptions::parameter_overrides(
    const std::vector<rclcpp::Parameter> &parameter_overrides) {
  this->parameter_overrides_ =
      parameter_overrides;  // 设置参数覆盖列表 (Set the parameter override list)
  return *this;
}

/**
 * @brief 获取是否使用全局参数的布尔值 (Get the boolean value of whether to use global arguments)
 * @return 返回布尔值 (Return the boolean value)
 */
bool NodeOptions::use_global_arguments() const { return this->use_global_arguments_; }

/**
 * @brief 设置是否使用全局参数 (Set whether to use global arguments)
 * @param use_global_arguments 布尔值 (Boolean value)
 * @return 返回设置后的节点选项引用 (Return the reference to the set node options)
 */
NodeOptions &NodeOptions::use_global_arguments(bool use_global_arguments) {
  this->node_options_.reset();  // 重置节点选项以便在下次访问时重新创建 (Reset node options to make
                                // it be recreated on next access)
  this->use_global_arguments_ =
      use_global_arguments;  // 设置是否使用全局参数 (Set whether to use global arguments)
  return *this;
}

/**
 * @brief 获取是否启用 rosout 的布尔值 (Get the boolean value of whether to enable rosout)
 * @return 返回布尔值 (Return the boolean value)
 */
bool NodeOptions::enable_rosout() const { return this->enable_rosout_; }

/**
 * @brief 设置是否启用 rosout (Set whether to enable rosout)
 * @param enable_rosout 布尔值 (Boolean value)
 * @return 返回设置后的节点选项引用 (Return the reference to the set node options)
 */
NodeOptions &NodeOptions::enable_rosout(bool enable_rosout) {
  this->node_options_.reset();  // 重置节点选项以便在下次访问时重新创建 (Reset node options to make
                                // it be recreated on next access)
  this->enable_rosout_ = enable_rosout;  // 设置是否启用 rosout (Set whether to enable rosout)
  return *this;
}

/**
 * @brief 获取是否使用进程内通信的布尔值 (Get the boolean value of whether to use intra-process
 * communication)
 * @return 返回布尔值 (Return the boolean value)
 */
bool NodeOptions::use_intra_process_comms() const { return this->use_intra_process_comms_; }

/**
 * @brief 设置是否使用进程内通信 (Set whether to use intra-process communication)
 * @param use_intra_process_comms 布尔值 (Boolean value)
 * @return 返回设置后的节点选项引用 (Return the reference to the set node options)
 */
NodeOptions &NodeOptions::use_intra_process_comms(bool use_intra_process_comms) {
  this->use_intra_process_comms_ = use_intra_process_comms;  // 设置是否使用进程内通信 (Set whether
                                                             // to use intra-process communication)
  return *this;
}

/**
 * @brief 获取是否启用主题统计的布尔值 (Get the boolean value of whether to enable topic statistics)
 * @return 返回布尔值 (Return the boolean value)
 */
bool NodeOptions::enable_topic_statistics() const { return this->enable_topic_statistics_; }

/**
 * @brief 设置是否启用主题统计 (Set whether to enable topic statistics)
 * @param enable_topic_statistics 布尔值 (Boolean value)
 * @return 返回设置后的节点选项引用 (Return the reference to the set node options)
 */
NodeOptions &NodeOptions::enable_topic_statistics(bool enable_topic_statistics) {
  this->enable_topic_statistics_ =
      enable_topic_statistics;  // 设置是否启用主题统计 (Set whether to enable topic statistics)
  return *this;
}

/**
 * @brief 获取是否启动参数服务的布尔值 (Get the boolean value of whether to start parameter
 * services)
 * @return 返回布尔值 (Return the boolean value)
 */
bool NodeOptions::start_parameter_services() const { return this->start_parameter_services_; }

/**
 * @brief 设置是否启动参数服务 (Set whether to start parameter services)
 * @param start_parameter_services 布尔值 (Boolean value)
 * @return 返回设置后的节点选项引用 (Return the reference to the set node options)
 */
NodeOptions &NodeOptions::start_parameter_services(bool start_parameter_services) {
  this->start_parameter_services_ =
      start_parameter_services;  // 设置是否启动参数服务 (Set whether to start parameter services)
  return *this;
}

/**
 * @brief 获取是否启动参数事件发布器的值 (Get the value of whether to start the parameter event
 * publisher)
 *
 * @return bool 返回是否启动参数事件发布器的值 (Return the value of whether to start the parameter
 * event publisher)
 */
bool NodeOptions::start_parameter_event_publisher() const {
  // 返回当前节点选项中是否启动参数事件发布器的值 (Return the value of whether to start the
  // parameter event publisher in the current node options)
  return this->start_parameter_event_publisher_;
}

/**
 * @brief 设置是否启动参数事件发布器的值 (Set the value of whether to start the parameter event
 * publisher)
 *
 * @param[in] start_parameter_event_publisher 新的启动参数事件发布器的值 (New value for starting the
 * parameter event publisher)
 * @return NodeOptions& 当前节点选项的引用 (Reference to the current node options)
 */
NodeOptions &NodeOptions::start_parameter_event_publisher(bool start_parameter_event_publisher) {
  // 设置当前节点选项中是否启动参数事件发布器的值 (Set the value of whether to start the parameter
  // event publisher in the current node options)
  this->start_parameter_event_publisher_ = start_parameter_event_publisher;
  // 返回当前节点选项的引用，以便进行链式调用 (Return a reference to the current node options for
  // chained calls)
  return *this;
}

/**
 * @brief 获取时钟类型 (Get the clock type)
 *
 * @return const rcl_clock_type_t& 返回时钟类型的引用 (Return a reference to the clock type)
 */
const rcl_clock_type_t &NodeOptions::clock_type() const { return this->clock_type_; }

/**
 * @brief 设置时钟类型 (Set the clock type)
 *
 * @param[in] clock_type 新的时钟类型 (New clock type)
 * @return NodeOptions& 当前节点选项的引用 (Reference to the current node options)
 */
NodeOptions &NodeOptions::clock_type(const rcl_clock_type_t &clock_type) {
  // 设置当前节点选项中的时钟类型 (Set the clock type in the current node options)
  this->clock_type_ = clock_type;
  // 返回当前节点选项的引用，以便进行链式调用 (Return a reference to the current node options for
  // chained calls)
  return *this;
}

/**
 * @brief 获取时钟服务质量 (QoS) (Get the clock Quality of Service (QoS))
 *
 * @return const rclcpp::QoS& 返回时钟服务质量 (QoS) 的引用 (Return a reference to the clock Quality
 * of Service (QoS))
 */
const rclcpp::QoS &NodeOptions::clock_qos() const { return this->clock_qos_; }

/**
 * @brief 设置时钟服务质量 (QoS) (Set the clock Quality of Service (QoS))
 *
 * @param[in] clock_qos 新的时钟服务质量 (QoS) (New clock Quality of Service (QoS))
 * @return NodeOptions& 当前节点选项的引用 (Reference to the current node options)
 */
NodeOptions &NodeOptions::clock_qos(const rclcpp::QoS &clock_qos) {
  // 设置当前节点选项中的时钟服务质量 (QoS) (Set the clock Quality of Service (QoS) in the current
  // node options)
  this->clock_qos_ = clock_qos;
  // 返回当前节点选项的引用，以便进行链式调用 (Return a reference to the current node options for
  // chained calls)
  return *this;
}

/**
 * @brief 获取是否使用时钟线程的值 (Get the value of whether to use the clock thread)
 *
 * @return bool 返回是否使用时钟线程的值 (Return the value of whether to use the clock thread)
 */
bool NodeOptions::use_clock_thread() const { return this->use_clock_thread_; }

/**
 * @brief 设置是否使用时钟线程的值 (Set the value of whether to use the clock thread)
 *
 * @param[in] use_clock_thread 新的是否使用时钟线程的值 (New value for whether to use the clock
 * thread)
 * @return NodeOptions& 当前节点选项的引用 (Reference to the current node options)
 */
NodeOptions &NodeOptions::use_clock_thread(bool use_clock_thread) {
  // 设置当前节点选项中是否使用时钟线程的值 (Set the value of whether to use the clock thread in the
  // current node options)
  this->use_clock_thread_ = use_clock_thread;
  // 返回当前节点选项的引用，以便进行链式调用 (Return a reference to the current node options for
  // chained calls)
  return *this;
}

/**
 * @brief 获取参数事件服务质量 (QoS) (Get the parameter event Quality of Service (QoS))
 *
 * @return const rclcpp::QoS& 返回参数事件服务质量 (QoS) 的引用 (Return a reference to the parameter
 * event Quality of Service (QoS))
 */
const rclcpp::QoS &NodeOptions::parameter_event_qos() const { return this->parameter_event_qos_; }

/**
 * @brief 设置参数事件服务质量 (QoS) (Set the parameter event Quality of Service (QoS))
 *
 * @param[in] parameter_event_qos 新的参数事件服务质量 (QoS) (New parameter event Quality of Service
 * (QoS))
 * @return NodeOptions& 当前节点选项的引用 (Reference to the current node options)
 */
NodeOptions &NodeOptions::parameter_event_qos(const rclcpp::QoS &parameter_event_qos) {
  // 设置当前节点选项中的参数事件服务质量 (QoS) (Set the parameter event Quality of Service (QoS) in
  // the current node options)
  this->parameter_event_qos_ = parameter_event_qos;
  // 返回当前节点选项的引用，以便进行链式调用 (Return a reference to the current node options for
  // chained calls)
  return *this;
}

/**
 * @brief 获取 rosout 服务质量 (QoS) (Get the rosout Quality of Service (QoS))
 *
 * @return const rclcpp::QoS& 返回 rosout 服务质量 (QoS) 的引用 (Return a reference to the rosout
 * Quality of Service (QoS))
 */
const rclcpp::QoS &NodeOptions::rosout_qos() const { return this->rosout_qos_; }

/**
 * @brief 设置 rosout 服务质量 (QoS) (Set the rosout Quality of Service (QoS))
 *
 * @param[in] rosout_qos 新的 rosout 服务质量 (QoS) (New rosout Quality of Service (QoS))
 * @return NodeOptions& 当前节点选项的引用 (Reference to the current node options)
 */
NodeOptions &NodeOptions::rosout_qos(const rclcpp::QoS &rosout_qos) {
  // 重置节点选项，使其在下次访问时重新创建 (Reset the node options to make it be recreated on the
  // next access)
  this->node_options_.reset();
  // 设置当前节点选项中的 rosout 服务质量 (QoS) (Set the rosout Quality of Service (QoS) in the
  // current node options)
  this->rosout_qos_ = rosout_qos;
  // 返回当前节点选项的引用，以便进行链式调用 (Return a reference to the current node options for
  // chained calls)
  return *this;
}

/**
 * @brief 获取参数事件发布器选项 (Get the parameter event publisher options)
 *
 * @return const rclcpp::PublisherOptionsBase& 返回参数事件发布器选项的引用 (Return a reference to
 * the parameter event publisher options)
 */
const rclcpp::PublisherOptionsBase &NodeOptions::parameter_event_publisher_options() const {
  return parameter_event_publisher_options_;
}

/**
 * @brief 设置参数事件发布器选项 (Set the parameter event publisher options)
 *
 * @param[in] parameter_event_publisher_options 新的参数事件发布器选项 (New parameter event
 * publisher options)
 * @return NodeOptions& 当前节点选项的引用 (Reference to the current node options)
 */
NodeOptions &NodeOptions::parameter_event_publisher_options(
    const rclcpp::PublisherOptionsBase &parameter_event_publisher_options) {
  // 设置当前节点选项中的参数事件发布器选项 (Set the parameter event publisher options in the
  // current node options)
  parameter_event_publisher_options_ = parameter_event_publisher_options;
  // 返回当前节点选项的引用，以便进行链式调用 (Return a reference to the current node options for
  // chained calls)
  return *this;
}

/**
 * @brief 获取是否允许未声明参数的值 (Get the value of whether to allow undeclared parameters)
 *
 * @return bool 返回是否允许未声明参数的值 (Return the value of whether to allow undeclared
 * parameters)
 */
bool NodeOptions::allow_undeclared_parameters() const { return this->allow_undeclared_parameters_; }

/**
 * @brief 设置是否允许未声明参数的值 (Set the value of whether to allow undeclared parameters)
 *
 * @param[in] allow_undeclared_parameters 新的是否允许未声明参数的值 (New value for whether to allow
 * undeclared parameters)
 * @return NodeOptions& 当前节点选项的引用 (Reference to the current node options)
 */
NodeOptions &NodeOptions::allow_undeclared_parameters(bool allow_undeclared_parameters) {
  // 设置当前节点选项中是否允许未声明参数的值 (Set the value of whether to allow undeclared
  // parameters in the current node options)
  this->allow_undeclared_parameters_ = allow_undeclared_parameters;
  // 返回当前节点选项的引用，以便进行链式调用 (Return a reference to the current node options for
  // chained calls)
  return *this;
}

/**
 * @brief 获取是否自动从覆盖中声明参数的值 (Get the value of whether to automatically declare
 * parameters from overrides)
 *
 * @return bool 返回是否自动从覆盖中声明参数的值 (Return the value of whether to automatically
 * declare parameters from overrides)
 */
bool NodeOptions::automatically_declare_parameters_from_overrides() const {
  return this->automatically_declare_parameters_from_overrides_;
}

/**
 * @brief 设置是否自动从覆盖中声明参数的值 (Set the value of whether to automatically declare
 * parameters from overrides)
 *
 * @param[in] automatically_declare_parameters_from_overrides 新的是否自动从覆盖中声明参数的值 (New
 * value for whether to automatically declare parameters from overrides)
 * @return NodeOptions& 当前节点选项的引用 (Reference to the current node options)
 */
NodeOptions &NodeOptions::automatically_declare_parameters_from_overrides(
    bool automatically_declare_parameters_from_overrides) {
  // 设置当前节点选项中是否自动从覆盖中声明参数的值 (Set the value of whether to automatically
  // declare parameters from overrides in the current node options)
  this->automatically_declare_parameters_from_overrides_ =
      automatically_declare_parameters_from_overrides;
  // 返回当前节点选项的引用，以便进行链式调用
  return *this;
}

/**
 * @brief 获取节点选项的分配器 (Get the allocator of the node options)
 *
 * @return 分配器引用 (const reference to the allocator)
 */
const rcl_allocator_t &NodeOptions::allocator() const {
  // 返回当前节点选项的分配器 (Return the allocator of the current node options)
  return this->allocator_;
}

/**
 * @brief 设置节点选项的分配器 (Set the allocator of the node options)
 *
 * @param allocator 要设置的分配器 (The allocator to set)
 * @return 当前节点选项的引用，以便进行链式调用 (Reference to the current node options, for chaining
 * calls)
 */
NodeOptions &NodeOptions::allocator(rcl_allocator_t allocator) {
  // 重置节点选项，使其在下次访问时重新创建 (Reset node options to make it be recreated on next
  // access)
  this->node_options_.reset();

  // 设置新的分配器 (Set the new allocator)
  this->allocator_ = allocator;

  // 返回当前对象的引用，以便进行链式调用 (Return reference to the current object, for chaining
  // calls)
  return *this;
}

}  // namespace rclcpp

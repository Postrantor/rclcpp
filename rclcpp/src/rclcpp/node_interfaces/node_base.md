#include "rclcpp/node_interfaces/node_base.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "../logging_mutex.hpp"
#include "rcl/arguments.h"
#include "rclcpp/exceptions.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

using rclcpp::exceptions::throw_from_rcl_error;

using rclcpp::node_interfaces::NodeBase;

NodeBase::NodeBase(
    const std::string& node_name,
    const std::string& namespace_,
    rclcpp::Context::SharedPtr context,
    const rcl_node_options_t& rcl_node_options,
    bool use_intra_process_default,
    bool enable_topic_statistics_default,
    rclcpp::CallbackGroup::SharedPtr default_callback_group)
    : context_(context),
      use_intra_process_default_(use_intra_process_default),
      enable_topic_statistics_default_(enable_topic_statistics_default),
      node_handle_(nullptr),
      default_callback_group_(default_callback_group),
      associated_with_executor_(false),
      notify_guard_condition_(context),
      notify_guard_condition_is_valid_(false) {}

NodeBase::~NodeBase() {}

const char* NodeBase::get_name() const {}

rclcpp::Context::SharedPtr NodeBase::get_context() {}

std::shared_ptr<const rcl_node_t> NodeBase::get_shared_rcl_node_handle() const {}

rclcpp::CallbackGroup::SharedPtr NodeBase::create_callback_group(
    rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node) {}

rclcpp::CallbackGroup::SharedPtr NodeBase::get_default_callback_group() {}

bool NodeBase::callback_group_in_node(rclcpp::CallbackGroup::SharedPtr group) {}

void NodeBase::for_each_callback_group(const CallbackGroupFunction& func) {}

std::atomic_bool& NodeBase::get_associated_with_executor_atomic() {}

rclcpp::GuardCondition& NodeBase::get_notify_guard_condition() {}

bool NodeBase::get_use_intra_process_default() const {}

std::string NodeBase::resolve_topic_or_service_name(
    const std::string& name, bool is_service, bool only_expand) const {}

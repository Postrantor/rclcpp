using rclcpp::node_interfaces::NodeParameters;

RCLCPP_LOCAL
void local_perform_automatically_declare_parameters_from_overrides(
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
    std::function<bool(const std::string &)> has_parameter,
    std::function<void(
        const std::string &,
        const rclcpp::ParameterValue &,
        const rcl_interfaces::msg::ParameterDescriptor &,
        bool)> declare_parameter) {}

NodeParameters::NodeParameters(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    const std::vector<rclcpp::Parameter> &parameter_overrides,
    bool start_parameter_services,
    bool start_parameter_event_publisher,
    const rclcpp::QoS &parameter_event_qos,
    const rclcpp::PublisherOptionsBase &parameter_event_publisher_options,
    bool allow_undeclared_parameters,
    bool automatically_declare_parameters_from_overrides)
    : allow_undeclared_(allow_undeclared_parameters),
      events_publisher_(nullptr),
      node_logging_(node_logging),
      node_clock_(node_clock) {}

void NodeParameters::perform_automatically_declare_parameters_from_overrides() {}

NodeParameters::~NodeParameters() {}

RCLCPP_LOCAL
bool __are_doubles_equal(double x, double y, double ulp = 100.0) {}

static std::string format_range_reason(const std::string &name, const char *range_type) {}

RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult __check_parameter_value_in_range(
    const rcl_interfaces::msg::ParameterDescriptor &descriptor,
    const rclcpp::ParameterValue &value) {}

static std::string format_type_reason(
    const std::string &name, const std::string &old_type, const std::string &new_type) {}

RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult __check_parameters(
    std::map<std::string, rclcpp::node_interfaces::ParameterInfo> &parameter_infos,
    const std::vector<rclcpp::Parameter> &parameters,
    bool allow_undeclared) {}

using PreSetParametersCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::PreSetParametersCallbackType;
using PreSetParametersCallbackHandle = rclcpp::node_interfaces::PreSetParametersCallbackHandle;
using PreSetCallbacksHandleContainer =
    rclcpp::node_interfaces::NodeParameters::PreSetCallbacksHandleContainer;

using OnSetParametersCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType;
using OnSetParametersCallbackHandle = rclcpp::node_interfaces::OnSetParametersCallbackHandle;
using OnSetCallbacksHandleContainer =
    rclcpp::node_interfaces::NodeParameters::OnSetCallbacksHandleContainer;

using PostSetParametersCallbackType =
    rclcpp::node_interfaces::NodeParametersInterface::PostSetParametersCallbackType;
using PostSetParametersCallbackHandle = rclcpp::node_interfaces::PostSetParametersCallbackHandle;
using PostSetCallbacksHandleContainer =
    rclcpp::node_interfaces::NodeParameters::PostSetCallbacksHandleContainer;

RCLCPP_LOCAL
void __call_pre_set_parameters_callbacks(
    std::vector<rclcpp::Parameter> &parameters,
    PreSetCallbacksHandleContainer &callback_container) {}

RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult __call_on_set_parameters_callbacks(
    const std::vector<rclcpp::Parameter> &parameters,
    OnSetCallbacksHandleContainer &callback_container) {}

RCLCPP_LOCAL
void __call_post_set_parameters_callbacks(
    const std::vector<rclcpp::Parameter> &parameters,
    PostSetCallbacksHandleContainer &callback_container) {}

RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult __set_parameters_atomically_common(
    const std::vector<rclcpp::Parameter> &parameters,
    std::map<std::string, rclcpp::node_interfaces::ParameterInfo> &parameter_infos,
    OnSetCallbacksHandleContainer &on_set_callback_container,
    PostSetCallbacksHandleContainer &post_set_callback_container,
    bool allow_undeclared = false) {}

RCLCPP_LOCAL
rcl_interfaces::msg::SetParametersResult __declare_parameter_common(
    const std::string &name,
    const rclcpp::ParameterValue &default_value,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
    std::map<std::string, rclcpp::node_interfaces::ParameterInfo> &parameters_out,
    const std::map<std::string, rclcpp::ParameterValue> &overrides,
    OnSetCallbacksHandleContainer &on_set_callback_container,
    PostSetCallbacksHandleContainer &post_set_callback_container,
    rcl_interfaces::msg::ParameterEvent *parameter_event_out,
    bool ignore_override = false) {}

static const rclcpp::ParameterValue &declare_parameter_helper(
    const std::string &name,
    rclcpp::ParameterType type,
    const rclcpp::ParameterValue &default_value,
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor,
    bool ignore_override,
    std::map<std::string, rclcpp::node_interfaces::ParameterInfo> &parameters,
    const std::map<std::string, rclcpp::ParameterValue> &overrides,
    OnSetCallbacksHandleContainer &on_set_callback_container,
    PostSetCallbacksHandleContainer &post_set_callback_container,
    rclcpp::Publisher<rcl_interfaces::msg::ParameterEvent> *events_publisher,
    const std::string &combined_name,
    rclcpp::node_interfaces::NodeClockInterface &node_clock) {}

const rclcpp::ParameterValue &NodeParameters::declare_parameter(
    const std::string &name,
    const rclcpp::ParameterValue &default_value,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
    bool ignore_override) {}

const rclcpp::ParameterValue &NodeParameters::declare_parameter(
    const std::string &name,
    rclcpp::ParameterType type,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor,
    bool ignore_override) {}

void NodeParameters::undeclare_parameter(const std::string &name) {}

bool NodeParameters::has_parameter(const std::string &name) const {}

std::vector<rcl_interfaces::msg::SetParametersResult> NodeParameters::set_parameters(
    const std::vector<rclcpp::Parameter> &parameters) {}

template <typename ParameterVectorType>
auto __find_parameter_by_name(ParameterVectorType &parameters, const std::string &name) {}

rcl_interfaces::msg::SetParametersResult NodeParameters::set_parameters_atomically(
    const std::vector<rclcpp::Parameter> &parameters) {}

std::vector<rclcpp::Parameter> NodeParameters::get_parameters(
    const std::vector<std::string> &names) const {}

rclcpp::Parameter NodeParameters::get_parameter(const std::string &name) const {}

bool NodeParameters::get_parameter(const std::string &name, rclcpp::Parameter &parameter) const {}

bool NodeParameters::get_parameters_by_prefix(
    const std::string &prefix, std::map<std::string, rclcpp::Parameter> &parameters) const {}

std::vector<rcl_interfaces::msg::ParameterDescriptor> NodeParameters::describe_parameters(
    const std::vector<std::string> &names) const {}

std::vector<uint8_t> NodeParameters::get_parameter_types(
    const std::vector<std::string> &names) const {}

rcl_interfaces::msg::ListParametersResult NodeParameters::list_parameters(
    const std::vector<std::string> &prefixes, uint64_t depth) const {}

void NodeParameters::remove_pre_set_parameters_callback(
    const PreSetParametersCallbackHandle *const handle) {}

void NodeParameters::remove_on_set_parameters_callback(
    const OnSetParametersCallbackHandle *const handle) {}

void NodeParameters::remove_post_set_parameters_callback(
    const PostSetParametersCallbackHandle *const handle) {}

PreSetParametersCallbackHandle::SharedPtr NodeParameters::add_pre_set_parameters_callback(
    PreSetParametersCallbackType callback) {}

OnSetParametersCallbackHandle::SharedPtr NodeParameters::add_on_set_parameters_callback(
    OnSetParametersCallbackType callback) {}

PostSetParametersCallbackHandle::SharedPtr NodeParameters::add_post_set_parameters_callback(
    PostSetParametersCallbackType callback) {}

const std::map<std::string, rclcpp::ParameterValue> &NodeParameters::get_parameter_overrides()
    const {}

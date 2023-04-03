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

#include "rclcpp/qos_event.hpp"

#include <string>

namespace rclcpp {
/*!
 * \brief 构造一个不支持的事件类型异常 (Construct an UnsupportedEventTypeException)
 * \param ret rcl_ret_t 类型，表示返回值 (rcl_ret_t type, representing the return value)
 * \param error_state 指向 rcl_error_state_t 结构体的指针 (Pointer to the rcl_error_state_t
 * structure)
 * \param prefix 异常消息前缀 (Exception message prefix)
 */
UnsupportedEventTypeException::UnsupportedEventTypeException(
    rcl_ret_t ret, const rcl_error_state_t* error_state, const std::string& prefix)
    : UnsupportedEventTypeException(exceptions::RCLErrorBase(ret, error_state), prefix) {}

/*!
 * \brief 构造一个不支持的事件类型异常 (Construct an UnsupportedEventTypeException)
 * \param base_exc 引用 exceptions::RCLErrorBase 类型的异常 (Reference to exceptions::RCLErrorBase
 * type exception)
 * \param prefix 异常消息前缀 (Exception message prefix)
 */
UnsupportedEventTypeException::UnsupportedEventTypeException(
    const exceptions::RCLErrorBase& base_exc, const std::string& prefix)
    : exceptions::RCLErrorBase(base_exc),
      std::runtime_error(prefix + (prefix.empty() ? "" : ": ") + base_exc.formatted_message) {}

/*!
 * \brief QOSEventHandlerBase 的析构函数 (Destructor of QOSEventHandlerBase)
 */
QOSEventHandlerBase::~QOSEventHandlerBase() {
  // 由于 rmw 事件监听器持有对此回调的引用，因此在此类销毁时需要清除它。
  // 对于其他 rclcpp 实体（如 pub/subs），由于它们拥有底层 rmw 实体，这种清除是不需要的，
  // 这些实体在 rclcpp 析构函数中被销毁，因此不存在悬空指针的风险。
  if (on_new_event_callback_) {
    clear_on_ready_callback();
  }

  if (rcl_event_fini(&event_handle_) != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
        "rclcpp", "Error in destruction of rcl event handle: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }
}

/*!
 * \brief 获取准备好的事件数量 (Get the number of ready events)
 * \return 准备好的事件数量 (Number of ready events)
 */
size_t QOSEventHandlerBase::get_number_of_ready_events() { return 1; }

/*!
 * \brief 将 Waitable 添加到等待集 (Add the Waitable to a wait set)
 * \param wait_set 指向 rcl_wait_set_t 结构体的指针 (Pointer to the rcl_wait_set_t structure)
 */
void QOSEventHandlerBase::add_to_wait_set(rcl_wait_set_t* wait_set) {
  rcl_ret_t ret = rcl_wait_set_add_event(wait_set, &event_handle_, &wait_set_event_index_);
  if (RCL_RET_OK != ret) {
    exceptions::throw_from_rcl_error(ret, "Couldn't add event to wait set");
  }
}

/*!
 * \brief 检查 Waitable 是否准备好 (Check if the Waitable is ready)
 * \param wait_set 指向 rcl_wait_set_t 结构体的指针 (Pointer to the rcl_wait_set_t structure)
 * \return 如果 Waitable 准备好，则返回 true，否则返回 false (Returns true if the Waitable is ready,
 * otherwise returns false)
 */
bool QOSEventHandlerBase::is_ready(rcl_wait_set_t* wait_set) {
  return wait_set->events[wait_set_event_index_] == &event_handle_;
}

/*!
 * \brief 设置新事件回调函数 (Set the new event callback function)
 * \param callback rcl_event_callback_t 类型的回调函数 (Callback function of type
 * rcl_event_callback_t)
 * \param user_data 用户数据，作为回调函数参数 (User data, as a callback
 * function parameter)
 */
void QOSEventHandlerBase::set_on_new_event_callback(
    rcl_event_callback_t callback, const void* user_data) {
  rcl_ret_t ret = rcl_event_set_callback(&event_handle_, callback, user_data);

  if (RCL_RET_OK != ret) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(ret, "failed to set the on new message callback for QOS Event");
  }
}

}  // namespace rclcpp

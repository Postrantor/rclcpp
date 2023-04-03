// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/exceptions.hpp"

#include <cstdio>
#include <functional>
#include <string>
#include <vector>

using namespace std::string_literals;

namespace rclcpp {
namespace exceptions {

/**
 * @brief 格式化名称验证错误信息 (Format name validation error message)
 *
 * @param[in] name_type 名称类型（如节点名、主题名等）(Name type, such as node name, topic name,
 * etc.)
 * @param[in] name 无效的名称 (Invalid name)
 * @param[in] error_msg 错误信息 (Error message)
 * @param[in] invalid_index 无效字符在名称中的索引位置 (Index position of the invalid character in
 * the name)
 * @return std::string 返回格式化后的错误信息 (Return the formatted error message)
 */
std::string NameValidationError::format_error(
    const char* name_type, const char* name, const char* error_msg, size_t invalid_index) {
  // 初始化空字符串 (Initialize an empty string)
  std::string msg = "";
  // 添加错误信息，包括名称类型和具体错误 (Add error message, including name type and specific
  // error)
  msg += "Invalid "s + name_type + ": " + error_msg + ":\n";
  // 添加无效名称 (Add invalid name)
  msg += "  '"s + name + "'\n";
  // 在无效字符下方添加指示符 '^' (Add an indicator '^' below the invalid character)
  msg += "   "s + std::string(invalid_index, ' ') + "^\n";
  // 返回格式化后的错误信息 (Return the formatted error message)
  return msg;
}

/**
 * @brief 将 RCL 错误转换为异常指针 (Convert RCL error to exception pointer)
 *
 * @param[in] ret RCL 错误代码 (RCL error code)
 * @param[in] prefix 错误信息前缀 (Error message prefix)
 * @param[in] error_state RCL 错误状态，如果为 nullptr，则使用当前的 rcl 错误状态 (RCL error state,
 * if nullptr, use the current rcl error state)
 * @param[in] reset_error 重置错误状态的函数指针，如果不为空，则在处理完错误后调用此函数 (Function
 * pointer to reset the error state, if not null, call this function after processing the error)
 * @return std::exception_ptr 返回异常指针，根据错误代码创建对应类型的异常 (Return exception
 * pointer, create corresponding exception type according to error code)
 */
std::exception_ptr from_rcl_error(
    rcl_ret_t ret,
    const std::string& prefix,
    const rcl_error_state_t* error_state,
    void (*reset_error)()) {
  // 如果错误代码为 RCL_RET_OK，抛出 invalid_argument 异常 (If the error code is RCL_RET_OK, throw
  // an invalid_argument exception)
  if (RCL_RET_OK == ret) {
    throw std::invalid_argument("ret is RCL_RET_OK");
  }
  // 如果 error_state 为空，获取当前的 rcl 错误状态 (If error_state is null, get the current rcl
  // error state)
  if (!error_state) {
    error_state = rcl_get_error_state();
  }
  // 如果 rcl 错误状态仍未设置，抛出 runtime_error 异常 (If the rcl error state is still not set,
  // throw a runtime_error exception)
  if (!error_state) {
    throw std::runtime_error("rcl error state is not set");
  }
  // 如果前缀不为空，添加 ": " 到前缀末尾 (If the prefix is not empty, add ": " to the end of the
  // prefix)
  std::string formatted_prefix = prefix;
  if (!prefix.empty()) {
    formatted_prefix += ": ";
  }
  // 创建基础异常对象 (Create a base exception object)
  RCLErrorBase base_exc(ret, error_state);
  // 如果 reset_error 不为空，调用此函数重置错误状态 (If reset_error is not null, call this function
  // to reset the error state)
  if (reset_error) {
    reset_error();
  }
  // 根据错误代码创建对应类型的异常，并返回异常指针 (Create corresponding exception type according
  // to error code and return exception pointer)
  switch (ret) {
    case RCL_RET_BAD_ALLOC:
      return std::make_exception_ptr(RCLBadAlloc(base_exc));
    case RCL_RET_INVALID_ARGUMENT:
      return std::make_exception_ptr(RCLInvalidArgument(base_exc, formatted_prefix));
    case RCL_RET_INVALID_ROS_ARGS:
      return std::make_exception_ptr(RCLInvalidROSArgsError(base_exc, formatted_prefix));
    default:
      return std::make_exception_ptr(RCLError(base_exc, formatted_prefix));
  }
}

/**
 * @brief 抛出一个来自 RCL 错误的异常 (Throw an exception from an RCL error)
 *
 * @param ret RCL 返回值 (RCL return value)
 * @param prefix 异常消息前缀 (Exception message prefix)
 * @param error_state 指向 RCL 错误状态的指针 (Pointer to the RCL error state)
 * @param reset_error 用于重置错误状态的函数指针 (Function pointer for resetting the error state)
 */
void throw_from_rcl_error(
    rcl_ret_t ret,
    const std::string& prefix,
    const rcl_error_state_t* error_state,
    void (*reset_error)()) {
  // 我们期望这个函数抛出一个标准错误，
  // 或者生成一个错误指针（在 err 中捕获并立即抛出）
  // (We expect this function to either throw a standard error,
  // or to generate an error pointer (which is caught in err, and immediately thrown))
  auto err = from_rcl_error(ret, prefix, error_state, reset_error);
  std::rethrow_exception(err);
}

/**
 * @brief RCLErrorBase 构造函数 (RCLErrorBase constructor)
 *
 * @param ret RCL 返回值 (RCL return value)
 * @param error_state 指向 RCL 错误状态的指针 (Pointer to the RCL error state)
 */
RCLErrorBase::RCLErrorBase(rcl_ret_t ret, const rcl_error_state_t* error_state)
    : ret(ret),
      message(error_state->message),
      file(error_state->file),
      line(error_state->line_number),
      formatted_message(rcl_get_error_string().str) {}

/**
 * @brief RCLError 构造函数 (RCLError constructor)
 *
 * @param ret RCL 返回值 (RCL return value)
 * @param error_state 指向 RCL 错误状态的指针 (Pointer to the RCL error state)
 * @param prefix 异常消息前缀 (Exception message prefix)
 */
RCLError::RCLError(rcl_ret_t ret, const rcl_error_state_t* error_state, const std::string& prefix)
    : RCLError(RCLErrorBase(ret, error_state), prefix) {}

/**
 * @brief RCLError 构造函数 (RCLError constructor)
 *
 * @param base_exc 基本异常 (Base exception)
 * @param prefix 异常消息前缀 (Exception message prefix)
 */
RCLError::RCLError(const RCLErrorBase& base_exc, const std::string& prefix)
    : RCLErrorBase(base_exc), std::runtime_error(prefix + base_exc.formatted_message) {}

/**
 * @brief RCLBadAlloc 构造函数 (RCLBadAlloc constructor)
 *
 * @param ret RCL 返回值 (RCL return value)
 * @param error_state 指向 RCL 错误状态的指针 (Pointer to the RCL error state)
 */
RCLBadAlloc::RCLBadAlloc(rcl_ret_t ret, const rcl_error_state_t* error_state)
    : RCLBadAlloc(RCLErrorBase(ret, error_state)) {}

/**
 * @brief RCLBadAlloc 构造函数 (RCLBadAlloc constructor)
 *
 * @param base_exc 基本异常 (Base exception)
 */
RCLBadAlloc::RCLBadAlloc(const RCLErrorBase& base_exc) : RCLErrorBase(base_exc), std::bad_alloc() {}

/**
 * @brief RCLInvalidArgument 构造函数 (RCLInvalidArgument constructor)
 *
 * @param ret RCL 返回值 (RCL return value)
 * @param error_state 指向 RCL 错误状态的指针 (Pointer to the RCL error state)
 * @param prefix 异常消息前缀 (Exception message prefix)
 */
RCLInvalidArgument::RCLInvalidArgument(
    rcl_ret_t ret, const rcl_error_state_t* error_state, const std::string& prefix)
    : RCLInvalidArgument(RCLErrorBase(ret, error_state), prefix) {}

/**
 * @brief RCLInvalidArgument 构造函数 (RCLInvalidArgument constructor)
 *
 * @param base_exc 基本异常 (Base exception)
 * @param prefix 异常消息前缀 (Exception message prefix)
 */
RCLInvalidArgument::RCLInvalidArgument(const RCLErrorBase& base_exc, const std::string& prefix)
    : RCLErrorBase(base_exc), std::invalid_argument(prefix + base_exc.formatted_message) {}

/**
 * @brief RCLInvalidROSArgsError 构造函数 (RCLInvalidROSArgsError constructor)
 *
 * @param ret RCL 返回值 (RCL return value)
 * @param error_state 指向 RCL 错误状态的指针 (Pointer to the RCL error state)
 * @param prefix 异常消息前缀 (Exception message prefix)
 */
RCLInvalidROSArgsError::RCLInvalidROSArgsError(
    rcl_ret_t ret, const rcl_error_state_t* error_state, const std::string& prefix)
    : RCLInvalidROSArgsError(RCLErrorBase(ret, error_state), prefix) {}

/**
 * @brief RCLInvalidROSArgsError 构造函数 (RCLInvalidROSArgsError constructor)
 *
 * @param base_exc 基本异常 (Base exception)
 * @param prefix 异常消息前缀 (Exception message prefix)
 */
RCLInvalidROSArgsError::RCLInvalidROSArgsError(
    const RCLErrorBase& base_exc, const std::string& prefix)
    : RCLErrorBase(base_exc), std::runtime_error(prefix + base_exc.formatted_message) {}

}  // namespace exceptions
}  // namespace rclcpp

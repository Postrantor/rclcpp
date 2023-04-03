// generated from rclcpp/resource/logging.hpp.em

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

#ifndef RCLCPP__LOGGING_HPP_
#define RCLCPP__LOGGING_HPP_

#include <sstream>
#include <type_traits>

#include "rclcpp/logger.hpp"
#include "rclcpp/utilities.hpp"
#include "rcutils/logging_macros.h"

// 这些用于编译低于最小严重性的日志宏。
// These are used for compiling out logging macros lower than a minimum severity.
#define RCLCPP_LOG_MIN_SEVERITY_DEBUG 0
#define RCLCPP_LOG_MIN_SEVERITY_INFO 1
#define RCLCPP_LOG_MIN_SEVERITY_WARN 2
#define RCLCPP_LOG_MIN_SEVERITY_ERROR 3
#define RCLCPP_LOG_MIN_SEVERITY_FATAL 4
#define RCLCPP_LOG_MIN_SEVERITY_NONE 5

// 定义用于获取第一个参数的宏
// Define a macro to get the first argument
#define RCLCPP_FIRST_ARG(N, ...) N
// 定义用于获取除第一个参数外的所有参数的宏
// Define a macro to get all arguments except the first one
#define RCLCPP_ALL_BUT_FIRST_ARGS(N, ...) __VA_ARGS__

/**
 * \def RCLCPP_LOG_MIN_SEVERITY
 * 在您的构建选项中定义
 * RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL]
 * 以编译出低于该严重性的内容。
 * 使用 RCLCPP_LOG_MIN_SEVERITY_NONE 编译出所有宏。
 *
 * Define RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL]
 * in your build options to compile out anything below that severity.
 * Use RCLCPP_LOG_MIN_SEVERITY_NONE to compile out all macros.
 */
#ifndef RCLCPP_LOG_MIN_SEVERITY
#define RCLCPP_LOG_MIN_SEVERITY RCLCPP_LOG_MIN_SEVERITY_DEBUG
#endif

@{
// 导入有序字典 (Import OrderedDict)
from collections import OrderedDict
// 导入深拷贝 (Import deepcopy)
from copy import deepcopy
// 导入 rcutils.logging 中的功能组合、后缀获取、严重性、节流参数等模块 (Import feature_combinations, get_suffix_from_features, severities, throttle_args, and throttle_params modules from rcutils.logging)
from rcutils.logging import feature_combinations
from rcutils.logging import get_suffix_from_features
from rcutils.logging import severities
from rcutils.logging import throttle_args
from rcutils.logging import throttle_params

// 设置节流参数的条件 (Set the condition for throttle_args)
throttle_args['condition_before'] = 'RCUTILS_LOG_CONDITION_THROTTLE_BEFORE(clock, duration)'
// 删除节流参数中的 'get_time_point_value' 键 (Delete the 'get_time_point_value' key in throttle_params)
del throttle_params['get_time_point_value']
// 添加 'clock' 键到节流参数中，并设置其描述 (Add the 'clock' key to throttle_params and set its description)
throttle_params['clock'] = 'rclcpp::Clock that will be used to get the time point.'
// 将 'clock' 键移到节流参数的开头 (Move the 'clock' key to the beginning of throttle_params)
throttle_params.move_to_end('clock', last=False)

// 创建一个有序字典用于存储 rclcpp 的功能组合 (Create an OrderedDict to store rclcpp feature combinations)
rclcpp_feature_combinations = OrderedDict()
// 遍历功能组合，将其添加到 rclcpp_feature_combinations 字典中 (Iterate through feature_combinations and add them to the rclcpp_feature_combinations dictionary)
for combinations, feature in feature_combinations.items():
    // 跳过包含 'named' 的功能组合 (Skip feature combinations containing 'named')
    if 'named' in combinations:
        continue
    rclcpp_feature_combinations[combinations] = feature

// 为每个可用的功能组合添加一个流变量 (Add a stream variant for each available feature combination)
stream_arg = 'stream_arg'
for combinations, feature in list(rclcpp_feature_combinations.items()):
    // 将 'stream' 添加到组合中 (Add 'stream' to the combinations)
    combinations = ('stream', ) + combinations
    // 深拷贝特征 (Deep copy the feature)
    feature = deepcopy(feature)
    // 在特征参数中添加 'stream_arg' 键，并设置其描述 (Add the 'stream_arg' key to the feature.params and set its description)
    feature.params[stream_arg] = 'The argument << into a stringstream'
    // 将新的组合和特征添加到 rclcpp_feature_combinations 字典中 (Add the new combinations and feature to the rclcpp_feature_combinations dictionary)
    rclcpp_feature_combinations[combinations] = feature

// 定义一个函数，根据特征获取 rclcpp 后缀 (Define a function to get the rclcpp suffix based on features)
def get_rclcpp_suffix_from_features(features):
    // 获取特征的后缀 (Get the suffix from the features)
    suffix = get_suffix_from_features(features)
    // 如果特征中包含 'stream'，在后缀前添加 '_STREAM' (If 'stream' is in the features, add '_STREAM' before the suffix)
    if 'stream' in features:
        suffix = '_STREAM' + suffix
    // 返回后缀 (Return the suffix)
    return suffix
}@
@[for severity in severities]@
/**
 * @@name 日志宏，用于记录严重程度为 @(severity) 的日志。
 * @@name Logging macros for severity @(severity).
 */
///@@{
#if (RCLCPP_LOG_MIN_SEVERITY > RCLCPP_LOG_MIN_SEVERITY_ @(severity))
// 当在编译时禁用严重程度为 @(severity) 的日志时，生成空的日志宏。
// empty logging macros for severity @(severity) when being disabled at compile time
@[ for feature_combination in rclcpp_feature_combinations.keys()]@
@{suffix = get_rclcpp_suffix_from_features(feature_combination)}@
/// 由于预处理器定义了 RCLCPP_LOG_MIN_SEVERITY，因此生成空的日志宏。
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define RCLCPP_ @(severity) @(suffix)(...)
@[ end for]@

#else
@[ for feature_combination in rclcpp_feature_combinations.keys()]@
@{suffix = get_rclcpp_suffix_from_features(feature_combination)}@
// RCLCPP_@(severity)@(suffix) 宏被包围在 do { .. } while (0) 中
// 以实现标准 C 宏习惯用法，使宏在所有上下文中都是安全的；
// 有关更多信息，请参阅 http://c-faq.com/cpp/multistmt.html。
// The RCLCPP_@(severity)@(suffix) macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def RCLCPP_@(severity)@(suffix)
 * 使用严重程度 @(severity) 记录日志消息。
 * Log a message with severity @(severity)@
@[ if rclcpp_feature_combinations[feature_combination].doc_lines]@
 * 以下条件：
 * with the following conditions:
@[ else]@
 * .
@[ end if]@
@[ for doc_line in rclcpp_feature_combinations[feature_combination].doc_lines]@
 * @(doc_line)
@[ end for]@
 * \param logger 要使用的 `rclcpp::Logger`
 * \param logger The `rclcpp::Logger` to use
@[ for param_name, doc_line in rclcpp_feature_combinations[feature_combination].params.items()]@
 * \param @(param_name) @(doc_line)
@[ end for]@
@[ if 'stream' not in feature_combination]@
 * \param ... 格式字符串，后跟格式字符串的可变参数。
 * \param ... The format string, followed by the variable arguments for the format string.
@[ end if]@
 */
@{params = rclcpp_feature_combinations[feature_combination].params.keys()}@
#define RCLCPP_@(severity)@(suffix)(logger@(''.join([', ' + p for p in params]))@
@[ if 'stream' not in feature_combination]@
, ...)
@[ end if]@
) \
  do {
  static_assert(
      ::std::is_same<
          typename std::remove_cv_t<typename std::remove_reference_t<decltype(logger)> >,
          typename ::rclcpp::Logger>::value,
      "First argument to logging macros must be an rclcpp::Logger");
  @[ if 'throttle' in feature_combination ] @ auto get_time_point =
      [&c = clock](rcutils_time_point_value_t* time_point) -> rcutils_ret_t {
    try {
      *time_point = c.now().nanoseconds();
    } catch (...) {
      RCUTILS_SAFE_FWRITE_TO_STDERR(
          "[rclcpp|logging.hpp] RCLCPP_@(severity)@(suffix) could not get current time stamp\n");
      return RCUTILS_RET_ERROR;
    }
    return RCUTILS_RET_OK;
  };
  @[ end if ] @[ if 'stream' in feature_combination ] @std::stringstream rclcpp_stream_ss_;
  rclcpp_stream_ss_ << @(stream_arg);
@[ end if]@
    RCUTILS_LOG_@(severity)@(get_suffix_from_features(feature_combination))_NAMED( \
@{params = ['get_time_point' if p == 'clock' and 'throttle' in feature_combination else p for p in params]}@
@[ if params]@
@(''.join(['      ' + p + ', \\n' for p in params if p != stream_arg]))@
@[ end if]@
      (logger).get_name(), \
@[ if 'stream' not in feature_combination]@
      __VA_ARGS__);
@[ else]@
      "%s", rclcpp_stream_ss_.str().c_str());
@[ end if ] @
}
while (0)

@[ end for]@
#endif
///@@}

@[end for]@

#endif  // RCLCPP__LOGGING_HPP_

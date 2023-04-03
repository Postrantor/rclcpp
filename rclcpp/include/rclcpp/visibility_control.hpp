// Copyright 2015 Open Source Robotics Foundation, Inc.
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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef RCLCPP__VISIBILITY_CONTROL_HPP_
#define RCLCPP__VISIBILITY_CONTROL_HPP_

// 这段逻辑是从 gcc wiki 的示例中借用的（然后添加了命名空间）：
// 以下链接为 gcc wiki 示例页面：
//     https://gcc.gnu.org/wiki/Visibility
//
// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
// The following link is the gcc wiki example page:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
// 如果编译器是 GNUC，则使用 __attribute__ 设置导出和导入
// If the compiler is GNUC, use __attribute__ to set export and import
#ifdef __GNUC__
#define RCLCPP_EXPORT __attribute__((dllexport))
#define RCLCPP_IMPORT __attribute__((dllimport))
#else
// 否则，使用 __declspec 设置导出和导入
// Otherwise, use __declspec to set export and import
#define RCLCPP_EXPORT __declspec(dllexport)
#define RCLCPP_IMPORT __declspec(dllimport)
#endif
// 如果正在构建库，则定义 RCLCPP_PUBLIC 为 RCLCPP_EXPORT
// If building the library, define RCLCPP_PUBLIC as RCLCPP_EXPORT
#ifdef RCLCPP_BUILDING_LIBRARY
#define RCLCPP_PUBLIC RCLCPP_EXPORT
#else
// 否则，定义 RCLCPP_PUBLIC 为 RCLCPP_IMPORT
// Otherwise, define RCLCPP_PUBLIC as RCLCPP_IMPORT
#define RCLCPP_PUBLIC RCLCPP_IMPORT
#endif
// 定义 RCLCPP_PUBLIC_TYPE 为 RCLCPP_PUBLIC
// Define RCLCPP_PUBLIC_TYPE as RCLCPP_PUBLIC
#define RCLCPP_PUBLIC_TYPE RCLCPP_PUBLIC
// 定义 RCLCPP_LOCAL 为空
// Define RCLCPP_LOCAL as empty
#define RCLCPP_LOCAL
#else
// 定义 RCLCPP_EXPORT 和 RCLCPP_IMPORT 的属性
// Define the attributes of RCLCPP_EXPORT and RCLCPP_IMPORT
#define RCLCPP_EXPORT __attribute__((visibility("default")))
#define RCLCPP_IMPORT
// 如果 GNUC 版本大于等于 4
// If the GNUC version is greater than or equal to 4
#if __GNUC__ >= 4
// 定义 RCLCPP_PUBLIC 和 RCLCPP_LOCAL 的可见性属性
// Define the visibility attributes of RCLCPP_PUBLIC and RCLCPP_LOCAL
#define RCLCPP_PUBLIC __attribute__((visibility("default")))
#define RCLCPP_LOCAL __attribute__((visibility("hidden")))
#else
// 否则，定义 RCLCPP_PUBLIC 和 RCLCPP_LOCAL 为空
// Otherwise, define RCLCPP_PUBLIC and RCLCPP_LOCAL as empty
#define RCLCPP_PUBLIC
#define RCLCPP_LOCAL
#endif
// 定义 RCLCPP_PUBLIC_TYPE 为空
// Define RCLCPP_PUBLIC_TYPE as empty
#define RCLCPP_PUBLIC_TYPE
#endif

#endif  // RCLCPP__VISIBILITY_CONTROL_HPP_

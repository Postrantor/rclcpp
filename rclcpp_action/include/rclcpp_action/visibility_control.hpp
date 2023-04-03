// Copyright 2018 Open Source Robotics Foundation, Inc.
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

// This header must be included by all rclcpp headers which declare symbols
// 这个头文件必须被所有声明了 rclcpp 库中符号的头文件包含
// which are defined in the rclcpp library. When not building the rclcpp
// 这些符号在 rclcpp 库中定义。当不构建 rclcpp 库时，
// library, i.e. when using the headers in other package's code, the contents
// 即在其他包的代码中使用这些头文件时，此头文件的内容
// of this header change the visibility of certain symbols which the rclcpp
// 会改变某些符号的可见性，这些符号是 rclcpp 库不能拥有的，
// library cannot have, but the consuming code must have inorder to link.
// 但消费者代码必须拥有以便链接。

#ifndef RCLCPP_ACTION__VISIBILITY_CONTROL_HPP_
#define RCLCPP_ACTION__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define RCLCPP_ACTION_EXPORT __attribute__((dllexport))
#define RCLCPP_ACTION_IMPORT __attribute__((dllimport))
#else
#define RCLCPP_ACTION_EXPORT __declspec(dllexport)
#define RCLCPP_ACTION_IMPORT __declspec(dllimport)
#endif
#ifdef RCLCPP_ACTION_BUILDING_LIBRARY
#define RCLCPP_ACTION_PUBLIC RCLCPP_ACTION_EXPORT
#else
#define RCLCPP_ACTION_PUBLIC RCLCPP_ACTION_IMPORT
#endif
#define RCLCPP_ACTION_PUBLIC_TYPE RCLCPP_ACTION_PUBLIC
#define RCLCPP_ACTION_LOCAL
#else
#define RCLCPP_ACTION_EXPORT __attribute__((visibility("default")))
#define RCLCPP_ACTION_IMPORT
#if __GNUC__ >= 4
#define RCLCPP_ACTION_PUBLIC __attribute__((visibility("default")))
#define RCLCPP_ACTION_LOCAL __attribute__((visibility("hidden")))
#else
#define RCLCPP_ACTION_PUBLIC
#define RCLCPP_ACTION_LOCAL
#endif
#define RCLCPP_ACTION_PUBLIC_TYPE
#endif

#endif  // RCLCPP_ACTION__VISIBILITY_CONTROL_HPP_

// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__FUNCTION_TRAITS_HPP_
#define RCLCPP__FUNCTION_TRAITS_HPP_

#include <functional>
#include <memory>
#include <tuple>

namespace rclcpp {

namespace function_traits {

// NOTE(esteve): 以下代码主要用于支持请求ID的可选服务回调。我们本应该使用两个重载的 create_service
// 方法， 但不幸的是，VS2015 上的 std::function
// 构造函数过于贪婪，因此我们需要一种机制来检查回调函数中每个参数的数量和类型。 参见
// http://blogs.msdn.com/b/vcblog/archive/2015/06/19/c-11-14-17-features-in-vs-2015-rtm.aspx
// NOTE(esteve): The following code is mainly used to support service callbacks that can optionally
// take the request id. We should have used two overloaded create_service methods, but
// unfortunately, the constructor of std::function on VS2015 is too greedy, so we need a mechanism
// to check the arity and type of each argument in a callback function. See
// http://blogs.msdn.com/b/vcblog/archive/2015/06/19/c-11-14-17-features-in-vs-2015-rtm.aspx

// 移除元组中的第一个元素
// Remove the first item in a tuple
template <typename T>
struct tuple_tail;

// 定义一个偏特化模板，用于将元组的头部和尾部分开
// Define a partial specialization template for separating the head and tail of a tuple
template <typename Head, typename... Tail>
struct tuple_tail<std::tuple<Head, Tail...>> {
  // 使用别名定义一个新的元组类型，包含原元组除第一个元素之外的所有元素
  // Define a new tuple type using an alias, containing all elements of the original tuple except
  // the first one
  using type = std::tuple<Tail...>;
};

/**
 * @brief 从 std::function 中提取函数特征的模板元编程工具 (A template metaprogramming tool for
 * extracting function traits from std::function)
 *
 * @tparam FunctionT 要分析的函数类型 (The function type to analyze)
 */
template <typename FunctionT>
struct function_traits {
  /// 使用 tuple_tail 删除第一个参数，并将剩余参数作为 arguments 类型 (Use tuple_tail to remove the
  /// first parameter and use the remaining parameters as the arguments type)
  using arguments = typename tuple_tail<
      typename function_traits<decltype(&FunctionT::operator())>::arguments>::type;

  /// 参数的数量 (Number of parameters)
  static constexpr std::size_t arity = std::tuple_size<arguments>::value;

  /**
   * @brief 获取第 N 个参数的类型 (Get the type of the Nth parameter)
   *
   * @tparam N 参数索引 (Parameter index)
   */
  template <std::size_t N>
  using argument_type = typename std::tuple_element<N, arguments>::type;

  /// 函数的返回类型 (The return type of the function)
  using return_type = typename function_traits<decltype(&FunctionT::operator())>::return_type;
};

/**
 * @brief 提取自由函数的特征 (Extract traits of free functions)
 *
 * @tparam ReturnTypeT 自由函数的返回类型 (Return type of the free function)
 * @tparam Args 自由函数的参数类型 (Parameter types of the free function)
 */
template <typename ReturnTypeT, typename... Args>
struct function_traits<ReturnTypeT(Args...)> {
  /// 参数类型元组 (Tuple of parameter types)
  using arguments = std::tuple<Args...>;

  /// 参数的数量 (Number of parameters)
  static constexpr std::size_t arity = std::tuple_size<arguments>::value;

  /**
   * @brief 获取第 N 个参数的类型 (Get the type of the Nth parameter)
   *
   * @tparam N 参数索引 (Parameter index)
   */
  template <std::size_t N>
  using argument_type = typename std::tuple_element<N, arguments>::type;

  /// 函数的返回类型 (The return type of the function)
  using return_type = ReturnTypeT;
};

/**
 * @brief 提取函数指针的特征 (Extract traits of function pointers)
 *
 * @tparam ReturnTypeT 函数指针的返回类型 (Return type of the function pointer)
 * @tparam Args 函数指针的参数类型 (Parameter types of the function pointer)
 */
template <typename ReturnTypeT, typename... Args>
struct function_traits<ReturnTypeT (*)(Args...)> : function_traits<ReturnTypeT(Args...)> {};

/*
在这段代码中，我们为对象方法和对象常量方法的 std::bind 特化了 function_traits
结构。这些模板特化根据不同的编译器和标准库来定义，以适应各种环境。每个特化都继承自
function_traits<ReturnTypeT(Args...)>，使得我们可以方便地获取函数特征。

In this code snippet, we specialize the function_traits structure for std::bind of object methods
and object const methods. These template specializations are defined depending on different
compilers and standard libraries to adapt to various environments. Each specialization inherits from
function_traits<ReturnTypeT(Args...)>, allowing us to easily obtain function traits.
*/

/**
 * @brief 用于对象方法的 std::bind 特化 (Specialization of std::bind for object methods)
 * @tparam ClassT 类类型 (Class type)
 * @tparam ReturnTypeT 返回值类型 (Return type)
 * @tparam Args 参数类型 (Argument types)
 * @tparam FArgs 绑定时的函数参数类型 (Function argument types when binding)
 */
template <typename ClassT, typename ReturnTypeT, typename... Args, typename... FArgs>
#if defined DOXYGEN_ONLY
struct function_traits<std::bind<ReturnTypeT (ClassT::*)(Args...), FArgs...>>
#elif defined _LIBCPP_VERSION   // libc++ (Clang)
struct function_traits<std::__bind<ReturnTypeT (ClassT::*)(Args...), FArgs...>>
#elif defined _GLIBCXX_RELEASE  // glibc++ (GNU C++ >= 7.1)
struct function_traits<std::_Bind<ReturnTypeT (ClassT::*(FArgs...))(Args...)>>
#elif defined __GLIBCXX__       // glibc++ (GNU C++)
struct function_traits<std::_Bind<std::_Mem_fn<ReturnTypeT (ClassT::*)(Args...)>(FArgs...)>>
#elif defined _MSC_VER          // MS Visual Studio
struct function_traits<std::_Binder<std::_Unforced, ReturnTypeT (ClassT::*)(Args...), FArgs...>>
#else
#error "Unsupported C++ compiler / standard library"
#endif
    : function_traits<ReturnTypeT(Args...)> {
};

/**
 * @brief 用于对象常量方法的 std::bind 特化 (Specialization of std::bind for object const methods)
 * @tparam ClassT 类类型 (Class type)
 * @tparam ReturnTypeT 返回值类型 (Return type)
 * @tparam Args 参数类型 (Argument types)
 * @tparam FArgs 绑定时的函数参数类型 (Function argument types when binding)
 */
template <typename ClassT, typename ReturnTypeT, typename... Args, typename... FArgs>
#if defined DOXYGEN_ONLY
struct function_traits<std::bind<ReturnTypeT (ClassT::*)(Args...) const, FArgs...>>
#elif defined _LIBCPP_VERSION   // libc++ (Clang)
struct function_traits<std::__bind<ReturnTypeT (ClassT::*)(Args...) const, FArgs...>>
#elif defined _GLIBCXX_RELEASE  // glibc++ (GNU C++ >= 7.1)
struct function_traits<std::_Bind<ReturnTypeT (ClassT::*(FArgs...))(Args...) const>>
#elif defined __GLIBCXX__       // glibc++ (GNU C++)
struct function_traits<std::_Bind<std::_Mem_fn<ReturnTypeT (ClassT::*)(Args...) const>(FArgs...)>>
#elif defined _MSC_VER          // MS Visual Studio
struct function_traits<
    std::_Binder<std::_Unforced, ReturnTypeT (ClassT::*)(Args...) const, FArgs...>>
#else
#error "Unsupported C++ compiler / standard library"
#endif
    : function_traits<ReturnTypeT(Args...)> {
};

// std::bind for free functions（为自由函数添加 std::bind）
template <typename ReturnTypeT, typename... Args, typename... FArgs>
#if defined DOXYGEN_ONLY
// 使用 Doxygen 文档生成工具时的特化结构（Specialization for using with Doxygen documentation tool）
struct function_traits<std::bind<ReturnTypeT (&)(Args...), FArgs...>>
#elif defined _LIBCPP_VERSION  // libc++ (Clang)
// 针对 Clang 编译器的特化结构（Specialization for Clang compiler）
struct function_traits<std::__bind<ReturnTypeT (&)(Args...), FArgs...>>
#elif defined __GLIBCXX__      // glibc++ (GNU C++)
// 针对 GNU C++ 编译器的特化结构（Specialization for GNU C++ compiler）
struct function_traits<std::_Bind<ReturnTypeT (*(FArgs...))(Args...)>>
#elif defined _MSC_VER         // MS Visual Studio
// 针对 MS Visual Studio 编译器的特化结构（Specialization for MS Visual Studio compiler）
struct function_traits<std::_Binder<std::_Unforced, ReturnTypeT (&)(Args...), FArgs...>>
#else
#error "Unsupported C++ compiler / standard library"
#endif
    // 从基类中继承成员（Inherit members from base class）
    : function_traits<ReturnTypeT(Args...)> {
};

// Lambdas（Lambda 表达式）
template <typename ClassT, typename ReturnTypeT, typename... Args>
// 为 Lambda 表达式定义特化结构（Specialization for lambda expressions）
struct function_traits<ReturnTypeT (ClassT::*)(Args...) const>
    // 从基类中继承成员（Inherit members from base class）
    : function_traits<ReturnTypeT(ClassT &, Args...)> {};

template <typename FunctionT>
struct function_traits<FunctionT &> : function_traits<FunctionT> {};

template <typename FunctionT>
struct function_traits<FunctionT &&> : function_traits<FunctionT> {};

/* NOTE(esteve):
 * VS2015 does not support expression SFINAE, so we're using this template to evaluate
 * the arity of a function.
 */
// 使用此模板来计算函数的元数，因为 VS2015 不支持表达式 SFINAE（Evaluate the arity of a function
// using this template since VS2015 does not support expression SFINAE）
template <std::size_t Arity, typename FunctorT>
struct arity_comparator
    : std::integral_constant<bool, (Arity == function_traits<FunctorT>::arity)> {};

/**
 * @brief 检查参数列表是否匹配的模板结构体 (Template structure to check if argument lists match)
 *
 * @tparam FunctorT 函数对象类型 (Function object type)
 * @tparam Args 参数类型 (Argument types)
 */
template <typename FunctorT, typename... Args>
struct check_arguments
    : std::is_same<typename function_traits<FunctorT>::arguments, std::tuple<Args...>> {};

/**
 * @brief 检查两个函数对象是否具有相同参数列表的模板结构体 (Template structure to check if two
 * function objects have the same argument list)
 *
 * @tparam FunctorAT 第一个函数对象类型 (First function object type)
 * @tparam FunctorBT 第二个函数对象类型 (Second function object type)
 */
template <typename FunctorAT, typename FunctorBT>
struct same_arguments : std::is_same<
                            typename function_traits<FunctorAT>::arguments,
                            typename function_traits<FunctorBT>::arguments> {};

// 详细命名空间，用于实现一些辅助功能 (Detail namespace for implementing some helper
// functionalities)
namespace detail {

/**
 * @brief 辅助模板结构体，将给定的返回类型和参数类型转换为 std::function 类型 (Helper template
 * structure to convert given return type and argument types to std::function type)
 *
 * @tparam ReturnTypeT 返回类型 (Return type)
 * @tparam Args 参数类型 (Argument types)
 */
template <typename ReturnTypeT, typename... Args>
struct as_std_function_helper;

/**
 * @brief 特化辅助模板结构体，处理 std::tuple 参数类型 (Specialized helper template structure to
 * handle std::tuple argument type)
 *
 * @tparam ReturnTypeT 返回类型 (Return type)
 * @tparam Args 参数类型 (Argument types)
 */
template <typename ReturnTypeT, typename... Args>
struct as_std_function_helper<ReturnTypeT, std::tuple<Args...>> {
  using type = std::function<ReturnTypeT(Args...)>;
};

}  // namespace detail

/**
 * @brief 模板结构体，将给定的函数对象转换为 std::function 类型 (Template structure to convert given
 * function object to std::function type)
 *
 * @tparam FunctorT 函数对象类型 (Function object type)
 * @tparam FunctionTraits 函数特征类型，默认为 function_traits<FunctorT> (Function traits type,
 * defaults to function_traits<FunctorT>)
 */
template <typename FunctorT, typename FunctionTraits = function_traits<FunctorT>>
struct as_std_function {
  using type = typename detail::as_std_function_helper<
      typename FunctionTraits::return_type,
      typename FunctionTraits::arguments>::type;
};

}  // namespace function_traits

}  // namespace rclcpp

#endif  // RCLCPP__FUNCTION_TRAITS_HPP_

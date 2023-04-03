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

#ifndef RCLCPP__EXPERIMENTAL__BUFFERS__BUFFER_IMPLEMENTATION_BASE_HPP_
#define RCLCPP__EXPERIMENTAL__BUFFERS__BUFFER_IMPLEMENTATION_BASE_HPP_

namespace rclcpp {
namespace experimental {
namespace buffers {

/**
 * @brief 缓冲区实现基类模板 (Buffer implementation base class template)
 *        这是一个用于管理缓冲区的基类模板。该模板接受一个类型参数
 * BufferT，表示缓冲区中存储的数据类型。这个类提供了一些基本的缓冲区操作，如入队、出队、清空缓冲区和检查缓冲区是否有数据。所有这些操作都是虚函数，需要在派生类中实现。
 * @tparam BufferT 缓冲区数据类型 (Buffer data type)
 */
template <typename BufferT>
class BufferImplementationBase {
public:
  /**
   * @brief 析构函数 (Destructor)
   */
  virtual ~BufferImplementationBase() {}

  /**
   * @brief 出队操作，从缓冲区中移除并返回一个元素 (Dequeue operation, remove and return an element
   * from the buffer)
   *
   * @return BufferT 返回移除的元素 (Return the removed element)
   */
  virtual BufferT dequeue() = 0;

  /**
   * @brief 入队操作，将一个元素添加到缓冲区中 (Enqueue operation, add an element to the buffer)
   *
   * @param request 要添加到缓冲区的元素 (Element to be added to the buffer)
   */
  virtual void enqueue(BufferT request) = 0;

  /**
   * @brief 清空缓冲区 (Clear the buffer)
   */
  virtual void clear() = 0;

  /**
   * @brief 检查缓冲区是否有数据 (Check if the buffer has data)
   *
   * @return bool 如果缓冲区有数据则返回 true，否则返回 false (Return true if the buffer has data,
   * otherwise return false)
   */
  virtual bool has_data() const = 0;
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__BUFFER_IMPLEMENTATION_BASE_HPP_

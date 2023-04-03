// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__WAIT_SET_POLICIES__DETAIL__WRITE_PREFERRING_READ_WRITE_LOCK_HPP_
#define RCLCPP__WAIT_SET_POLICIES__DETAIL__WRITE_PREFERRING_READ_WRITE_LOCK_HPP_

#include <condition_variable>
#include <functional>
#include <mutex>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp {
namespace wait_set_policies {
namespace detail {

/// 写优先的读写锁。
/// Writer-preferring read-write lock.
/**
 * 这个类是基于维基百科页面中描述的 "write-preferring RW lock" 的实现：
 * This class is based on an implementation of a "write-preferring RW lock" as described in this
 * wikipedia page:
 *
 * https://en.wikipedia.org/wiki/Readers%E2%80%93writer_lock#Using_a_condition_variable_and_a_mutex
 *
 * 为了保留历史，将其复制到此处：
 * Copying here for posterity:
 *
 * \verbatim
 *   对于一个写优先的读写锁，可以使用两个整数计数器和一个布尔标志：
 *   For a write-preferring RW lock one can use two integer counters and one boolean flag:
 *
 *       num_readers_active: 获取锁的读取器数量（整数）
 *       num_readers_active: the number of readers that have acquired the lock (integer)
 *       num_writers_waiting: 等待访问的写入器数量（整数）
 *       num_writers_waiting: the number of writers waiting for access (integer)
 *       writer_active: 是否有写入器获取了锁（布尔值）。
 *       writer_active: whether a writer has acquired the lock (boolean).
 *
 *   初始时，num_readers_active 和 num_writers_waiting 为零，writer_active 为 false。
 *   Initially num_readers_active and num_writers_waiting are zero and writer_active is false.
 *
 *   锁定和释放操作可以实现为：
 *   The lock and release operations can be implemented as
 *
 *   开始读取
 *   Begin Read
 *
 *       锁定 g
 *       Lock g
 *       当 num_writers_waiting > 0 或 writer_active 时：
 *       While num_writers_waiting > 0 or writer_active:
 *           等待 cond，g[a]
 *           wait cond, g[a]
 *       增加 num_readers_active
 *       Increment num_readers_active
 *       解锁 g
 *       Unlock g.
 *
 *   结束读取
 *   End Read
 *
 *       锁定 g
 *       Lock g
 *       减少 num_readers_active
 *       Decrement num_readers_active
 *       如果 num_readers_active = 0：
 *       If num_readers_active = 0:
 *           通知 cond（广播）
 *           Notify cond (broadcast)
 *       解锁 g。
 *       Unlock g.
 *
 *   开始写入
 *   Begin Write
 *
 *       锁定 g
 *       Lock g
 *       增加 num_writers_waiting
 *       Increment num_writers_waiting
 *       当 num_readers_active > 0 或 writer_active 为 true 时：
 *       While num_readers_active > 0 or writer_active is true:
 *           等待 cond，g
 *           wait cond, g
 *       减少 num_writers_waiting
 *       Decrement num_writers_waiting
 *       将 writer_active 设置为 true
 *       Set writer_active to true
 *       解锁 g。
 *       Unlock g.
 *
 *   结束写入
 *   End Write
 *
 *       锁定 g
 *       Lock g
 *       将 writer_active 设置为 false
 *       Set writer_active to false
 *       通知 cond（广播）
 *       Notify cond (broadcast)
 *       解锁 g。
 *       Unlock g.
 * \endverbatim
 *
 * 它会优先考虑任何等待的写调用而不是等待的读调用，这意味着过多的写调用可能会导致读调用饥饿。
 * It will prefer any waiting write calls to any waiting read calls, meaning
 * that excessive write calls can starve read calls.
 *
 * 这个类在两个重要方面与该设计有所不同。
 * This class diverges from that design in two important ways.
 * 首先，它是一个单一的读取器，单一的写入器版本。
 * First, it is a single reader, single writer version.
 * 其次，它允许用户在写入器进入等待状态后运行自定义代码，此功能的目的是允许用户中断可能长时间阻塞的读取活动。
 * Second, it allows for user defined code to be run after a writer enters the
 * waiting state, and the purpose of this feature is to allow the user to
 * interrupt any potentially long blocking read activities.
 *
 * 这两个特性使得新的等待写入器不仅可以确保在任何排队的读取器之前获得锁，而且还可以在需要时安全地中断读取活动，而无需在获得锁之前启动新的读取活动。
 * Together these two features allow new waiting writers to not only ensure
 * they get the lock before any queued readers, but also that it can safely
 * interrupt read activities if needed, without allowing new read activities to
 * start before it gains the lock.
 *
 * 第一个差异可以防止多个读取活动同时发生，但写入器只能可靠地中断其中一个的情况。
 * The first difference prevents the case that a multiple read activities occur
 * at the same time but the writer can only reliably interrupt one of them.
 * 通过阻止多个并发读取活动，可以避免这种情况。
 * By preventing multiple read activities concurrently, this case is avoided.
 * 第二个差异允许用户定义如何中断可能阻塞需要尽快进行的写入活动的读取活动。
 * The second difference allows the user to define how to interrupt read
 * activity that could be blocking the write activities that need to happen
 * as soon as possible.
 *
 * 为了实现这些差异，这个类用 "reader_active" 布尔值替换了 "num_readers_active" 计数器。
 * To implement the differences, this class replaces the "num_readers_active"
 * counter with a "reader_active" boolean.
 * 它还将上面的 "开始读取" 部分更改为以下内容：
 * It also changes the "Begin Read" section from above, like this:
 *
 * \verbatim
 *   开始读取
 *   Begin Read
 *
 *       锁定 g
 *       Lock g
 *       当 num_writers_waiting > 0 或 writer_active 或 reader_active 时： // 已更改
 *       While num_writers_waiting > 0 or writer_active or reader_active:  // changed
 *           等待 cond，g[a]
 *           wait cond, g[a]
 *       将 reader_active 设置为 true // 已更改
 *       Set reader_active to true  // changed
 *       解锁 g。
 *       Unlock g.
 * \endverbatim
 *
 * 并将上面的 "结束读取" 部分更改为以下内容：
 * And changes the "End Read" section from above, like this:
 *
 * \verbatim
 *   结束读取
 *   End Read
 *
 *       锁定 g
 *       Lock g
 *       将 reader_active 设置为 false // 已更改
 *       Set reader_active to false  // changed
 *       通知 cond（广播） // 已更改，现在无条件
 *       Notify cond (broadcast)  // changed, now unconditional
 *       解锁 g。
 *       Unlock g.
 * \endverbatim
 *
 * "开始写入" 部分也进行了如下更新：
 * The "Begin Write" section is also updated as follows:
 *
 * \verbatim
 *   开始写入
 *   Begin Write
 *
 *       锁定 g
 *       Lock g
 *       增加 num_writers_waiting
 *       Increment num_writers_waiting
 *       调用用户定义的 enter_waiting 函数 // 新
 *       Call user defined enter_waiting function  // new
 *       当 reader_active 为 true 或 writer_active 为 true 时： // 已更改
 *       While reader_active is true or writer_active is true:  // changed
 *           等待 cond，g
 *           wait cond, g
 *       减少 num_writers_waiting
 *       Decrement num_writers_waiting
 *       将 writer_active 设置为 true
 *       Set writer_active to true
 *       解锁 g。
 *       Unlock g.
 * \endverbatim
 *
 * 该实现使用一个条件变量、一个锁和几个状态变量。
 * The implementation uses a single condition variable, single lock, and several
 * state variables.
 *
 * 此类的典型用法如下：
 * The typical use of this class is as follows:
 *
 *     类 MyClass
 *     class MyClass
 *     {
 *       WritePreferringReadWriteLock wprw_lock_;
 *     public:
 *       MyClass() {}
 *       void do_some_reading()
 *       {
 *         使用 rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
 *         using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
 *         std::lock_guard<WritePreferringReadWriteLock::ReadMutex>
 * lock(wprw_lock_.get_read_mutex());
 *         // 进行阅读...
 *         // Do reading...
 *       }
 *       void do_some_writing()
 *       {
 *         使用 rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
 *         using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
 *         std::lock_guard<WritePreferringReadWriteLock::WriteMutex>
 * lock(wprw_lock_.get_write_mutex());
 *         // 进行写入...
 *         // Do writing...
 *       }
 *     };
 */
/**
 * @class WritePreferringReadWriteLock
 * @brief 可写优先的读写锁 (Write-preferring read-write lock)
 *
 * 这个类实现了一种可写优先的读写锁，它可以帮助你在多线程环境中同步对共享资源的访问。
 */
class WritePreferringReadWriteLock final {
public:
  /**
   * @brief 构造函数 (Constructor)
   * @param enter_waiting_function 当有线程等待时的回调函数，默认为 nullptr (Callback function when
   * a thread is waiting, default is nullptr)
   */
  RCLCPP_PUBLIC
  explicit WritePreferringReadWriteLock(std::function<void()> enter_waiting_function = nullptr);

  /**
   * @class ReadMutex
   * @brief 用于 WritePreferringReadWriteLock 的读互斥锁 (Read mutex for the
   * WritePreferringReadWriteLock)
   *
   * 实现了 "C++ named requirements: BasicLockable"。
   */
  class RCLCPP_PUBLIC ReadMutex {
  public:
    /// 上锁 (Lock)
    void lock();

    /// 解锁 (Unlock)
    void unlock();

  protected:
    /// 构造函数 (Constructor)
    explicit ReadMutex(WritePreferringReadWriteLock& parent_lock);

    /// 父级锁引用 (Reference to parent lock)
    WritePreferringReadWriteLock& parent_lock_;

    /// 声明 WritePreferringReadWriteLock 为友元类 (Declare WritePreferringReadWriteLock as a friend
    /// class)
    friend class WritePreferringReadWriteLock;
  };

  /**
   * @class WriteMutex
   * @brief 用于 WritePreferringReadWriteLock 的写互斥锁 (Write mutex for the
   * WritePreferringReadWriteLock)
   *
   * 实现了 "C++ named requirements: BasicLockable"。
   */
  class RCLCPP_PUBLIC WriteMutex {
  public:
    /// 上锁 (Lock)
    void lock();

    /// 解锁 (Unlock)
    void unlock();

  protected:
    /// 构造函数 (Constructor)
    explicit WriteMutex(WritePreferringReadWriteLock& parent_lock);

    /// 父级锁引用 (Reference to parent lock)
    WritePreferringReadWriteLock& parent_lock_;

    /// 声明 WritePreferringReadWriteLock 为友元类 (Declare WritePreferringReadWriteLock as a friend
    /// class)
    friend class WritePreferringReadWriteLock;
  };

  /**
   * @brief 获取可用于标准构造的读互斥锁，如 std::lock_guard (Return read mutex which can be used
   * with standard constructs like std::lock_guard)
   * @return 读互斥锁引用 (Read mutex reference)
   */
  RCLCPP_PUBLIC
  ReadMutex& get_read_mutex();

  /**
   * @brief 获取可用于标准构造的写互斥锁，如 std::lock_guard (Return write mutex which can be used
   * with standard constructs like std::lock_guard)
   * @return 写互斥锁引用 (Write mutex reference)
   */
  RCLCPP_PUBLIC
  WriteMutex& get_write_mutex();

protected:
  bool reader_active_ =
      false;  ///< 标记是否有读者活跃 (Flag indicating if there is an active reader)
  std::size_t number_of_writers_waiting_ = 0;  ///< 等待中的写者数量 (Number of waiting writers)
  bool writer_active_ =
      false;  ///< 标记是否有写者活跃 (Flag indicating if there is an active writer)
  std::mutex mutex_;                              ///< 互斥锁 (Mutex)
  std::condition_variable condition_variable_;    ///< 条件变量 (Condition variable)
  ReadMutex read_mutex_;                          ///< 读互斥锁实例 (Read mutex instance)
  WriteMutex write_mutex_;                        ///< 写互斥锁实例 (Write mutex instance)
  std::function<void()> enter_waiting_function_;  ///< 等待回调函数 (Waiting callback function)
};

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DETAIL__WRITE_PREFERRING_READ_WRITE_LOCK_HPP_

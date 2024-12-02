/**
 * @file Timer.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 定时器模块
 * @version 0.1
 * @date 2024-04-18
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "util/Mutex.h"
#include <functional>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace robot {
class Command;
class TimerManager;

/**
 * @brief 定时器
 *
 */
class Timer : public std::enable_shared_from_this<Timer> {
    friend class TimerManager;

  public:
    typedef std::shared_ptr<Timer> Ptr;
    Timer(uint64_t ms, std::shared_ptr<Command> command, bool recurring, TimerManager *manager);
    Timer(uint64_t ms, TimerManager *manager);
    Timer(uint64_t next);
    void setCommand(std::shared_ptr<Command> command);
    bool cancel();

    bool isPause() const { return m_pause_; }
    void setPause(bool pause = true) { m_pause_ = pause; }

  private:
    /**
     * @brief 定时器比较函数
     *
     */
    struct Comparator {
        /**
         * @brief 比较定时器的只能指针大小
         *
         * @param lhs 定时器智能指针
         * @param rhs 定时器智能指针
         * @return true
         * @return false
         */
        bool operator()(const Timer::Ptr &lhs, const Timer::Ptr &rhs) const;
    };

  private:
    ///  是否循环定时器
    bool m_recurring_ = false;
    ///  执行周期
    uint64_t m_ms_ = 0;
    ///  精确的执行事件
    uint64_t m_next_ = 0;
    ///  回调函数
    std::shared_ptr<Command> m_command_;
    ///  定时器管理器
    TimerManager *m_manager_ = nullptr;
    bool m_pause_ = false;
};
class TimerManager {
    friend class Timer;

  public:
    typedef std::shared_ptr<TimerManager> Ptr;
    ///  读写锁类型
    typedef RWMutex RWMutexType;
    /**
     * @brief 构造函数
     *
     */
    TimerManager() {}
    /**
     * @brief 析构函数
     *
     */
    virtual ~TimerManager() {}
    /**
     * @brief 添加定时器
     *
     * @param ms 定时器执行间隔时间
     * @param cb 定时器回调函数
     * @param recurring 是否循环定时器
     * @return Timer::ptr
     */
    Timer::Ptr addTimer(uint64_t ms, std::shared_ptr<Command> command, bool recurring = false);
    void addTimer(Timer::Ptr timer);
    uint64_t getNextTimer();
    /**
     * @brief 获取需要执行的定时器的回调函数列表
     *
     * @param cbs 回调函数容器
     */
    void listExpiredCb(std::vector<std::shared_ptr<Command>> &cbs);
    /**
     * @brief 是否有定时器
     *
     * @return true
     * @return false
     */
    bool hasTimer();

  protected:
    /**
     * @brief 当有新的定时器插入到定时器首部时，执行该函数
     *
     */
    virtual void onTimerInsertedAtFront() = 0;
    /**
     * @brief 将定时器添加到管理器中
     *
     * @param val
     * @param lock
     */
    void addTimer(Timer::Ptr val, RWMutexType::WriteLock &lock);

  private:
    /**
     * @brief 检测服务器时间是否被调后了
     *
     * @param now_ms
     * @return true
     * @return false
     */
    bool detectClockRollover(uint64_t now_ms);
    /**
     * @brief 判断需要运行的定时器
     *
     */
    void handleExpiredTimers(std::vector<Timer::Ptr> &expired, uint64_t now_ms,
                             std::set<robot::Timer::Ptr, robot::Timer::Comparator>::iterator &it);

  protected:
    ///  锁
    RWMutexType m_mutex_;
    ///  定时器集合
    std::set<Timer::Ptr, Timer::Comparator> m_timers_;
    ///  是否触发onTimerInsertedAtFront
    bool m_tickled_ = false;
    ///  上次执行时间
    uint64_t m_previouseTime = 0;
};

} // namespace robot
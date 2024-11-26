/**
 * @file Schedule.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "CommandBase.h"
#include "Mutex.h"
#include "Thread.h"
#include "Timer.h"
#include "Util.h"
#include "command/group/CommandGroupBase.h"
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <sys/epoll.h>
#include <unistd.h>
#include <vector>

namespace RobotGenius {

class Command;
class CommandGroupBase;

class Impl {
  public:
    bool inRunLoop = false;
    // std::map<CommandGroupBase::ptr, bool> command_groups;
    // std::map<Command::ptr, bool> commands;
    std::vector<Command::ptr> commands;
};

/**
 * @brief 协程调度器
 *
 * @details 封装的是N-M的协程调度器
 *          内部有一个线程池，支持协程在线程池里面切换
 *
 */
class Scheduler : public TimerManager {
  public:
    typedef std::shared_ptr<Scheduler> ptr_;
    typedef Mutex MutexType;
    enum Event {
        /// 无事件
        NONE = 0x0,
        /// 读事件(EPOLLIN)
        READ = 0x1,
        /// 写事件(EPOLLOUT)
        WRITE = 0x4,
    };

    /**
     * @brief 构造函数
     *
     * @param threads 线程数量
     * @param use_caller
     * 是否使用当前调用线程,即主程序所在的线程是否纳入到协程调度器中
     * @param name 协程调度器名称
     */
    explicit Scheduler(size_t threads = 1, bool use_caller = false, const std::string name = "");
    /**
     * @brief 析构函数
     *
     */
    virtual ~Scheduler();
    Scheduler(const Scheduler &) = delete;
    Scheduler &operator=(const Scheduler &) = delete;

    /**
     * Returns the Scheduler instance.
     *
     * @return the instance
     */
    static Scheduler &GetInstance(size_t threads = 1, bool use_caller = true, const std::string name = "");

    /**
     * @brief 获取协程调度器名称
     *
     * @return const std::string&
     */
    const std::string &getName() const { return m_name_; }
    /**
     * @brief 获取当前调度器
     *
     * @return Scheduler*
     */
    static Scheduler *GetThis();
    /**
     * @brief 启动协程调度器，创建线程
     *
     */
    void start();
    /**
     * @brief
     * 停止协程调度器，只有调度器中所有线程都结束了，stop才会结束，因为代码中有join
     *
     */
    void stop();
    void tickle();
    bool schedule(Command::ptr command_ptr);
    bool schedule(std::vector<Command::ptr> commands);

    /**
     * @brief 协程调度函数
     *
     */
    void run();

    /**
     * @brief 协程无任务可调度时执行idle
     *
     */
    virtual void idle();
    /**
     * @brief 设置当前的协程调度器
     *
     */
    void setThis();
    bool hasIdleThreads() { return m_idleThreadCount_ > 0; }
    void onTimerInsertedAtFront() override;
    bool stopping(uint64_t &timeout);

  private:
    ///  mutex
    MutexType m_mutex_;
    ///  线程池
    std::vector<Thread::ptr_> m_threads_;

    ///  协程调度器名称
    std::string m_name_;

  protected:
    ///  协程下的线程id数组
    std::vector<int> m_threadIds_;
    ///  线程数量
    size_t m_threadCount_ = 0;
    ///  工作中线程数量
    std::atomic<size_t> m_activeThreadCount_ = {0};
    ///  空闲中线程数量
    std::atomic<size_t> m_idleThreadCount_ = {0};
    ///  是否正在停止
    bool m_stopping_ = true;
    ///  是否自动停止
    bool m_autoStop_ = false;
    ///  主线程id（use_caller）
    int m_rootThread_ = 0;

  public:
    std::shared_ptr<Impl> m_impl;

  private:
    ///  epoll文件句柄
    int m_epfd_ = 0;
    ///  pipe文件句柄
    int m_tickleFds_[2];
    ///  当前等待执行的事件数量
    std::atomic<size_t> m_pendingEventCount_ = {0};
    // uint64_t m_start_time_ = 0;
    // uint64_t m_next_time_ = 0;
};

void schedulerManagerStart(size_t threads = 1, bool use_caller = true, const std::string name = "");
void schedulerManagerStop();

} //  namespace RobotGenius

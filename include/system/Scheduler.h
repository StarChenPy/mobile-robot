#pragma once
#include "Timer.h"
#include "command/group/ICommandGroup.h"
#include "Mutex.h"
#include "Thread.h"
#include "util/Util.h"
#include <atomic>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <sys/epoll.h>
#include <unistd.h>
#include <vector>

namespace robot {

/**
 * @brief 内部实现类
 * @details 包含协程运行状态和命令队列
 */
    class Impl {
    public:
        bool inRunLoop = false; ///< 当前是否处于调度循环中
        std::vector<ICommand::ptr> commands; ///< 待执行的命令队列
    };

/**
 * @brief 协程调度器
 * @details 实现N-M的协程调度，包含线程池和协程任务管理功能。
 */
    class Scheduler : public TimerManager {
    public:
        using ptr = std::shared_ptr<Scheduler>;
        using MutexType = Mutex;

        /**
         * @brief 事件类型
         */
        enum Event {
            NONE = 0x0, ///< 无事件
            READ = 0x1, ///< 读事件 (EPOLLIN)
            WRITE = 0x4 ///< 写事件 (EPOLLOUT)
        };

        /**
         * @brief 构造函数
         * @param threads 线程数量
         * @param use_caller 是否将调用线程纳入调度器
         * @param name 调度器名称
         */
        explicit Scheduler(size_t threads = 1, bool use_caller = false, std::string  name = "");

        /**
         * @brief 析构函数
         */
        ~Scheduler() override;

        Scheduler(const Scheduler &) = delete;
        Scheduler &operator=(const Scheduler &) = delete;

        /**
         * @brief 获取调度器实例
         * @param threads 线程数量
         * @param use_caller 是否将调用线程纳入调度器
         * @param name 调度器名称
         * @return 调度器实例
         */
        static Scheduler &getInstance(size_t threads = 1, bool use_caller = true, const std::string &name = "");

        /**
         * @brief 获取调度器名称
         * @return 调度器名称
         */
        const std::string &getName() const { return m_name_; }

        /**
         * @brief 获取当前线程的调度器实例
         * @return 当前线程的调度器
         */
        static Scheduler *GetThis();

        /**
         * @brief 启动调度器，创建线程池
         */
        void start();

        /**
         * @brief 停止调度器，等待所有线程结束
         */
        void stop();

        /**
         * @brief 唤醒调度器
         */
        void tickle();

        /**
         * @brief 添加单个任务到调度器
         * @param command_ptr 任务指针
         * @return 是否成功添加任务
         */
        bool schedule(const ICommand::ptr &command_ptr);

        /**
         * @brief 添加多个任务到调度器
         * @param commands 任务列表
         * @return 是否成功添加所有任务
         */
        bool schedule(const std::vector<ICommand::ptr> &commands);

        /**
         * @brief 调度任务
         */
        void run();

        /**
         * @brief 空闲时执行的函数
         */
        virtual void idle();

        /**
         * @brief 设置当前线程的调度器
         */
        void setThis();

        /**
         * @brief 检查是否有空闲线程
         * @return 是否有空闲线程
         */
        bool hasIdleThreads() const { return m_idleThreadCount_ > 0; }

        /**
         * @brief 前置定时器插入的处理逻辑
         */
        void onTimerInsertedAtFront() override;

        /**
         * @brief 判断是否可以停止
         * @param timeout 返回的下一次超时时间
         * @return 是否可以停止
         */
        bool stopping(uint64_t &timeout);

    private:
        MutexType m_mutex_; ///< 互斥锁
        std::vector<Thread::ptr> threads_; ///< 线程池
        std::string m_name_; ///< 调度器名称

    protected:
        std::vector<int> threadIds_; ///< 线程ID列表
        size_t threadCount_ = 0; ///< 线程数量
        std::atomic<size_t> m_activeThreadCount_ = {0}; ///< 活跃线程数量
        std::atomic<size_t> m_idleThreadCount_ = {0}; ///< 空闲线程数量
        bool m_stopping_ = true; ///< 是否停止标志
        bool autoStop_ = false; ///< 是否自动停止
        int m_rootThread_ = 0; ///< 主线程ID (用于use_caller)

    public:
        std::shared_ptr<Impl> m_impl; ///< 内部实现对象

    private:
        int m_epfd_ = 0; ///< epoll 文件描述符
        int m_tickleFds_[2]{}; ///< 用于唤醒的管道描述符
        std::atomic<size_t> m_pendingEventCount_ = {0}; ///< 等待处理的事件数量
    };

} // namespace robot

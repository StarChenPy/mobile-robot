#include "system/Scheduler.h"
#include <iostream>
#include <map>
#include <memory>
#include <utility>

namespace robot {

    // 当前线程协程调度器，每个线程拥有一个独立的调度器
    static thread_local Scheduler *t_scheduler = nullptr;

    /**
     * @brief 构造函数
     * @details 如果 use_caller 为 true，表示在主程序所在线程创建协程调度器
     */
    Scheduler::Scheduler(size_t threads, bool use_caller, std::string name)
            : m_name_(std::move(name)) {
        m_impl = std::make_shared<Impl>();
        m_epfd_ = epoll_create(5000);

        if (m_epfd_ > 0) {
            int rt = pipe(m_tickleFds_);
            epoll_event event{};
            memset(&event, 0, sizeof(epoll_event));
            event.events = EPOLLIN | EPOLLET;
            event.data.fd = m_tickleFds_[0];

            rt = fcntl(m_tickleFds_[0], F_SETFL, O_NONBLOCK);
            if (rt) {
                LOG(ERROR) << "设置管道为非阻塞模式失败";
            }
            rt = epoll_ctl(m_epfd_, EPOLL_CTL_ADD, m_tickleFds_[0], &event);
            if (rt) {
                LOG(ERROR) << "将管道添加到 epoll 中失败";
            }
        } else {
            LOG(ERROR) << "创建 epoll 实例失败";
        }

        if (use_caller) {
            --threads;
            t_scheduler = this;
            robot::Thread::SetName(m_name_);
            threadIds_.push_back(m_rootThread_);
        } else {
            m_rootThread_ = -1;
        }

        threadCount_ = threads;
    }

    Scheduler::~Scheduler() = default;

    Scheduler &Scheduler::getInstance(size_t threads, bool use_caller, const std::string &name) {
        static Scheduler scheduler(threads, use_caller, name);
        return scheduler;
    }

    Scheduler *Scheduler::GetThis() { return t_scheduler; }

    void Scheduler::start() {
        MutexType::Lock lock(m_mutex_);
        if (!m_stopping_) {
            return;
        }
        m_stopping_ = false;

        threads_.resize(threadCount_);
        for (size_t i = 0; i < threadCount_; ++i) {
            threads_[i] = std::make_shared<Thread>([this] { run(); }, m_name_ + "_" + std::to_string(i));
            threadIds_.push_back(threads_[i]->getId());
        }
        lock.unlock();
        LOG(INFO) << "调度器已启动";
    }

    void Scheduler::stop() {
        autoStop_ = true;
        run();

        if (threadCount_ == 0) {
            LOG(INFO) << "调度器: " << this << " " << this->m_name_ << " 已停止";
            return;
        }

        for (size_t i = 0; i < threadCount_; ++i) {
            tickle();
        }

        std::vector<Thread::ptr> thrs;
        {
            MutexType::Lock lock(m_mutex_);
            thrs.swap(threads_);
        }

        for (auto &i : thrs) {
            if (i) {
                i->join();
            }
        }

        LOG(INFO) << "调度器已停止";
    }

    void Scheduler::setThis() { t_scheduler = this; }

    bool Scheduler::schedule(const ICommand::ptr &command_ptr) {
        MutexType::Lock lock(m_mutex_);
        if (m_impl->inRunLoop) {
            auto it = std::find(m_impl->commands.begin(), m_impl->commands.end(), command_ptr);
            if (it != m_impl->commands.end()) {
                return false;
            }
            m_impl->commands.push_back(command_ptr);
            tickle();
            return true;
        }
        return false;
    }

    bool Scheduler::schedule(const std::vector<ICommand::ptr> &commands) {
        bool result = true;
        for (const auto &command : commands) {
            result = result && schedule(command);
        }
        return result;
    }

    void handleCommand(const ICommand::ptr &cmd) {
        switch (cmd->state_) {
            case ICommand::State::WAIT:
                cmd->schedule();
                cmd->state_ = ICommand::State::INIT;
                break;
            case ICommand::State::INIT:
                cmd->initialize();
                if (cmd->isFinished()) {
                    cmd->state_ = ICommand::State::FINISHED;
                    if (cmd->parent_) {
                        cmd->parent_->state_ = ICommand::State::RUNNING;
                        cmd->parent_->schedule();
                    }
                }
                cmd->schedule();
                cmd->state_ = ICommand::State::RUNNING;
                break;
            case ICommand::State::RUNNING:
                if (cmd->isFinished()) {
                    cmd->state_ = ICommand::State::FINISHED;
                } else {
                    if (cmd->state_ != ICommand::State::PAUSED) {
                        cmd->execute();
                    }
                }
                if (cmd->state_ != ICommand::State::PAUSED) {
                    cmd->schedule();
                }
                break;
            case ICommand::State::CANCELED:
                cmd->cancel();
                break;
            case ICommand::State::FINISHED:
                cmd->end();
                if (cmd->parent_) {
                    if (cmd->parent_->state_ != ICommand::State::STOP) {
                        cmd->parent_->state_ = ICommand::State::RUNNING;
                        cmd->parent_->schedule();
                    }
                    cmd->parent_.reset();
                }
                cmd->state_ = ICommand::State::STOP;
                break;
            default:
                LOG(ERROR) << "未处理的命令状态: " << static_cast<int>(cmd->state_);
                break;
        }
    }

    void Scheduler::run() {
        m_impl->inRunLoop = true;
        ICommand::ptr cmd;

        while (true) {
            cmd.reset();
            bool tickle_me = false;

            {
                MutexType::Lock lock(m_mutex_);
                auto command = m_impl->commands.begin();
                if (command != m_impl->commands.end() && *command) {
                    cmd = *command;
                    m_impl->commands.erase(command);
                    tickle_me = true;
                }

                if (m_stopping_ && !cmd) {
                    break;
                }
            }

            if (tickle_me) {
                ++m_activeThreadCount_;
                tickle();
            }

            if (cmd) {
                handleCommand(cmd);
                --m_activeThreadCount_;
            }

            {
                MutexType::Lock lock(m_mutex_);
                if (autoStop_ && !cmd && m_impl->commands.empty() && !hasTimer() && m_activeThreadCount_ <= 0) {
                    break;
                }
            }

            ++m_idleThreadCount_;
            idle();
            --m_idleThreadCount_;
        }
    }

    void Scheduler::tickle() {
        int rt = write(m_tickleFds_[1], "T", 1);
        if (rt != 1) {
            LOG(ERROR) << "tickle 写入失败，返回值: " << rt;
        }
    }

    void Scheduler::idle() {
        epoll_event events[64] = {};
        constexpr uint64_t MAX_TIMEOUT = 3000;
        uint64_t next_timeout = 0;

        stopping(next_timeout);
        next_timeout = next_timeout != ~0ull ? std::min(next_timeout, MAX_TIMEOUT) : MAX_TIMEOUT;

        int rt = epoll_wait(m_epfd_, events, 64, static_cast<int>(next_timeout));
        if (rt < 0 && errno == EINTR) {
            LOG(WARNING) << "epoll_wait 被中断";
        }

        std::vector<ICommand::ptr> cbs;
        listExpiredCb(cbs);
        for (auto &cb : cbs) {
            if (cb->state_ == ICommand::State::PAUSED && !cb->getTimer()->isPause()) {
                cb->state_ = ICommand::State::RUNNING;
            }
        }

        if (!cbs.empty()) {
            schedule(cbs);
            for (size_t i = 0; i < cbs.size(); ++i) {
                tickle();
            }
        }

        for (int i = 0; i < rt; ++i) {
            if (events[i].data.fd == m_tickleFds_[0]) {
                uint8_t dummy[256];
                while (read(m_tickleFds_[0], dummy, sizeof(dummy)) > 0) {}
            }
        }
    }

    bool Scheduler::stopping(uint64_t &timeout) {
        timeout = getNextTimer();
        return timeout == ~0ull;
    }

    void Scheduler::onTimerInsertedAtFront() { tickle(); }
} // namespace robot
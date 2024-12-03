#include "util/Scheduler.h"
#include <iostream>
#include <map>

namespace robot {
    void printBorder(int width) { std::cout << "+" << std::setfill('=') << std::setw(width - 1) << "+" << std::endl; }

    void printCenteredText(const std::string &text, int width) {
        int padding = (width - text.size()) / 2;
        std::cout << "|" << std::setfill(' ') << std::setw(padding) << " " << text
                  << std::setw(width - padding - text.size() - 1) << " "
                  << "|" << std::endl;
    }
    void printText() {
        const int width = 50;
        const std::string welcome = "Welcome to";
        const std::string product = "Robotgenius";

        printBorder(width);
        printCenteredText(welcome, width);
        printCenteredText(product, width);
        printBorder(width);

        std::cout << std::endl;

        std::cout << "Robotgenius is designed to help you with all your robotic needs." << std::endl;
        std::cout << "Experience the future of automation and artificial intelligence." << std::endl;
    }

///  当前携程调度器，一个线程只有一个调度器
    static thread_local Scheduler *t_scheduler = nullptr;

/**
 * @brief 构造函数
 *
 * @details 如果user_caller为true，说明再主程序所在的线程创建协程调度器
 */
    Scheduler::Scheduler(size_t threads, bool use_caller, const std::string name) : m_name_(name) {
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
                std::cout << "fcntl failed" << std::endl;
            }
            rt = epoll_ctl(m_epfd_, EPOLL_CTL_ADD, m_tickleFds_[0], &event);
            if (rt) {
                std::cout << "epoll_ctl failed" << std::endl;
            }
        } else {
            std::cout << "epoll_create failed" << std::endl;
        }

        if (use_caller) {
            --threads;
            t_scheduler = this;

            robot::Thread::SetName(m_name_);
            m_threadIds_.push_back(m_rootThread_);
        } else {
            m_rootThread_ = -1;
        }
        m_threadCount_ = threads;
    }
    Scheduler::~Scheduler() = default;

    Scheduler &Scheduler::getInstance(size_t threads, bool use_caller, const std::string& name) {
        static Scheduler scheduler(threads, use_caller, name);
        return scheduler;
    }
    Scheduler *Scheduler::GetThis() { return t_scheduler; }

    void Scheduler::start() {
        printText();
        MutexType::Lock lock(m_mutex_);
        ///  刚启动时，m_stopping默认为真，所以条件结果需要置反
        if (!m_stopping_) {
            return;
        }
        m_stopping_ = false;

        m_threads_.resize(m_threadCount_);
        ///  创建线程
        for (size_t i = 0; i < m_threadCount_; ++i) {
            m_threads_[i].reset(new Thread(std::bind(&Scheduler::run, this), m_name_ + "_" + std::to_string(i)));
            m_threadIds_.push_back(m_threads_[i]->getId());
        }
        lock.unlock();
    }
    void Scheduler::stop() {
        m_autoStop_ = true;
        run();
        if (m_threadCount_ == 0) {
            std::cout << "Schedule: " << this << "  " << this->m_name_ << " stopped" << std::endl;
        }
        ///  如果m_rootThread != -1,说明是use_caller的线程
        ///  协程调度器停止线程时，如果时再user_caller上创建线程，
        ///  那么只有主线程可以停止其他线程或者主线程自己结束，非主线程不能停止任意线程程序
        ///
        ///  如果不是再use_caller上创建的线程，那么任意线程都可以非自己的线程执行停止

        for (size_t i = 0; i < m_threadCount_; ++i) {
            tickle();
        }
        std::vector<Thread::ptr_> thrs;
        {
            MutexType::Lock lock(m_mutex_);
            thrs.swap(m_threads_);
        }

        for (auto &i : thrs) {
            if (i) {
                i->join();
            }
        }
    }

    void Scheduler::setThis() { t_scheduler = this; }

    bool Scheduler::schedule(ICommand::ptr command_ptr) {
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

    bool Scheduler::schedule(const std::vector<ICommand::ptr>& commands) {
        bool result = true;
        for (const auto& command : commands) {
            result = result && schedule(command);
        }
        return result;
    }

    void Scheduler::run() {
        m_impl->inRunLoop = true;
        bool tickle_me;
        ICommand::ptr cmd;
        // bool  need_to_active = false;
        ///  Scheduler构造函数已经初始化在use_caller上的线程主协程
        ///  但是非use_caller上的线程主协程无初始化，所以需要在run中进行判断并且初始化线程主协程
        while (true) {
            cmd.reset();
            tickle_me = false;
            {
                MutexType::Lock lock(m_mutex_);
                auto command = m_impl->commands.begin();
                if (command != m_impl->commands.end() && *command) {
                    cmd = *command;
                    m_impl->commands.erase(command);
                    tickle_me = true;
                }
                if (m_stopping_) {
                    if (cmd) {
                        if (cmd->state_ == ICommand::State::RUNNING) {
                            cmd->state_ = ICommand::State::CANCELED;
                        }
                    }
                }
            }
            if (tickle_me) {
                ++m_activeThreadCount_;
                tickle();
            }
            if (cmd.get()) {
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
                            if (cmd->state_ != ICommand::State::PAUSED)
                                cmd->execute();
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
                        break;
                }
                --m_activeThreadCount_;
            }
            {
                MutexType::Lock lock(m_mutex_);
                if (m_autoStop_ && !cmd && m_impl->commands.empty() && !hasTimer() && m_activeThreadCount_ <= 0) {
                    break;
                }
            }
            if (m_stopping_) {
                m_autoStop_ = true;
                if (cmd) {
                    if (cmd->state_ == ICommand::State::RUNNING || cmd->state_ == ICommand::State::INIT) {
                        cmd->cancel();
                    }
                    if (!tickle_me) {
                        --m_activeThreadCount_;
                    }
                    m_impl->commands.clear();
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
            std::cout << "tickle failed rt=" << rt << std::endl;
        }
    }
    void Scheduler::idle() {
        auto *events = new epoll_event[64]();
        static const uint64_t MAX_TIMEOUT = 3000;
        static uint64_t next_timeout;
        next_timeout = 0;
        stopping(next_timeout);
        int rt;
        if (next_timeout != ~0ull) {
            next_timeout = std::min(next_timeout, MAX_TIMEOUT);
        } else {
            next_timeout = MAX_TIMEOUT;
        }
        rt = epoll_wait(m_epfd_, events, 64, static_cast<int>(next_timeout));
        if (rt < 0 && errno == EINTR) {
            std::cout << "create epoll wait failed" << std::endl;
        }
        std::vector<ICommand::ptr> cbs;
        listExpiredCb(cbs);
        cbs.erase(
                std::remove_if(cbs.begin(), cbs.end(),
                               [](const ICommand::ptr &cb) { return cb->getWorkCommandState() != ICommand::State::PAUSED; }),
                cbs.end());
        for (const auto& cb : cbs) {
            if (cb->state_ == ICommand::State::PAUSED && !cb->getTimer()->isPause()) {
                cb->state_ = ICommand::State::RUNNING;
            }
        }
        if (!cbs.empty()) {
            Scheduler::getInstance().schedule(cbs);
            for (int i = 0; i < cbs.size(); i++) {
                tickle();
            }
            cbs.clear();
        }
        for (int i = 0; i < rt; ++i) {
            epoll_event &event = events[i];
            if (event.data.fd == m_tickleFds_[0]) {
                uint8_t dummy[256];
                while (read(m_tickleFds_[0], dummy, sizeof(dummy)) > 0)
                    ;
                continue;
            }
        }
    }

    bool Scheduler::stopping(uint64_t &timeout) {
        timeout = getNextTimer();
        return timeout == ~0ull;
    }

    void Scheduler::onTimerInsertedAtFront() { tickle(); }
} //  namespace robot

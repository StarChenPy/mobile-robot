/**
 * @file scheduler.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 调度器模块
 * @version 0.1
 * @date 2022-09-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <iostream>
#include <map>
#include "command/Scheduler.h"
#include "command/CommandBase.h"


// #include "hook.h"

namespace RobotGenius {


void printBorder(int width) {
    std::cout << "+" << std::setfill('=') << std::setw(width - 1) << "+" << std::endl;
}

void printCenteredText(const std::string& text, int width) {
    int padding = (width - text.size()) / 2;
    std::cout << "|" << std::setfill(' ') << std::setw(padding) << " "
              << text << std::setw(width - padding - text.size() - 1) << " " << "|" << std::endl;
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
static thread_local Scheduler* t_scheduler = nullptr;


/**
 * @brief 构造函数
 * 
 * @details 如果user_caller为true，说明再主程序所在的线程创建协程调度器
 */
Scheduler::Scheduler(size_t threads, bool use_caller, const std::string name)
  :m_name_(name) {
//   HGSYS_ASSERT(threads > 0);
  m_impl = std::make_shared<Impl>();
  m_epfd_ = epoll_create(5000);
  if (m_epfd_  > 0) {
    int rt = pipe(m_tickleFds_);
    epoll_event event;
    memset(&event, 0, sizeof(epoll_event));
    event.events = EPOLLIN | EPOLLET;
    event.data.fd = m_tickleFds_[0];

    rt = fcntl(m_tickleFds_[0], F_SETFL, O_NONBLOCK);
    if (rt) {
      std::cout << "fcntl failed" << std::endl;
    }
    rt = epoll_ctl(m_epfd_, EPOLL_CTL_ADD, m_tickleFds_[0], &event);
    if(rt) {
      std::cout << "epoll_ctl failed" << std::endl;
    }
  } else {
    std::cout << "epoll_create failed" << std::endl;
  }
  

  if (use_caller) {
    --threads;
    // HGSYS_ASSERT(GetThis() == nullptr);
    t_scheduler = this;

    RobotGenius::Thread::SetName(m_name_);
    // HGSYS_LOG_INFO(g_logger) << "create root_thread:" << m_rootThread_;
    m_threadIds_.push_back(m_rootThread_);
  } else {
    m_rootThread_ = -1;
  }
  m_threadCount_ = threads;
  // m_idleThreadCount_ = threads;
}
Scheduler::~Scheduler() {
//   HGSYS_ASSERT(m_stopping_);
  // if (GetThis() == this) {
  //   t_scheduler = nullptr;
  // }
}

Scheduler& Scheduler::GetInstance(size_t threads, bool use_caller, const std::string name) {
  static Scheduler scheduler(threads, use_caller, name);
  return scheduler;
}
Scheduler* Scheduler::GetThis() {
  return t_scheduler;
}

void Scheduler::start() {
  printText();
  MutexType::Lock lock(m_mutex_);
  ///  刚启动时，m_stopping默认为真，所以条件结果需要置反
  if (!m_stopping_) {
    return;
  }
  m_stopping_ = false;
//   HGSYS_ASSERT(m_threads_.empty());

  m_threads_.resize(m_threadCount_);
  ///  创建线程
  for (size_t i = 0; i< m_threadCount_; ++i) {
    m_threads_[i].reset(new Thread(std::bind(&Scheduler::run, this)
      , m_name_ + "_" + std::to_string(i)));
      m_threadIds_.push_back(m_threads_[i]->getId());
  }
  lock.unlock();
  // if (m_rootFiber_) {
  //   // m_rootFiber_->swapIn();
  //   m_rootFiber_->call();
  //   HGSYS_LOG_INFO(g_logger) << "call out" << m_rootFiber_->getState();
  // }
}
void Scheduler::stop() {
  m_autoStop_ = true;
  run();
  if (m_threadCount_ == 0) {
    // HGSYS_LOG_INFO(g_logger) << "Schedule: " << this << "  " << this->m_name_ << " stopped";
    std::cout << "Schedule: " << this << "  " << this->m_name_ << " stopped" << std::endl;
    // m_stopping_ = true;
    // HGSYS_LOG_INFO(g_logger) << "finish stopping";
  }
  // bool exit_on_this_fiber = false;
  ///  如果m_rootThread != -1,说明是use_caller的线程
  ///  协程调度器停止线程时，如果时再user_caller上创建线程，
  ///  那么只有主线程可以停止其他线程或者主线程自己结束，非主线程不能停止任意线程程序
  ///
  ///  如果不是再use_caller上创建的线程，那么任意线程都可以非自己的线程执行停止

  // m_stopping_ = true;
  // run();
  // std::cout << "stop stop" << std::endl;
  for (size_t i = 0; i < m_threadCount_; ++i) {
    tickle();
  }
  std::vector<Thread::ptr_> thrs;
  {
    MutexType::Lock lock(m_mutex_);
    thrs.swap(m_threads_);
  }
  
  for (auto& i : thrs) {
    if (i)
    {i->join();}
  }

}

void Scheduler::setThis() {
  t_scheduler = this;
}


bool Scheduler::schedule(Command::Ptr command_ptr) {
  MutexType::Lock lock(m_mutex_);
  if (m_impl->inRunLoop) {
    auto it = std::find(m_impl->commands.begin(), m_impl->commands.end(), command_ptr);
    if (it != m_impl->commands.end()) {
      // std::cout <<  "A command that is running "
      //                          "cannot be independently scheduled" << std::endl;
      return false;
    }
    m_impl->commands.push_back(command_ptr);
    tickle();
    return true;
  }
  return false;
}

bool Scheduler::schedule(std::vector<Command::Ptr> commands) {
  bool result = true;
  for (auto command : commands) {
    result = result && schedule(command);
  }
  return result;
}

void Scheduler::run() {
  // set_hook_enable(true);
  // setThis();
  m_impl->inRunLoop = true;
  bool tickle_me = false;
  Command::Ptr cmd;
  // bool  need_to_active = false;
  ///  Scheduler构造函数已经初始化在use_caller上的线程主协程
  ///  但是非use_caller上的线程主协程无初始化，所以需要在run中进行判断并且初始化线程主协程
  // HGSYS_LOG_INFO(g_logger) << "thread_id=" << RobotGenius::getThreadId();
  while (true) {
    cmd.reset();
    tickle_me = false;
    {
      MutexType::Lock lock(m_mutex_);
      // m_stopping_ = m_autoStop_ && m_impl->commands.empty() && m_impl->command_groups.empty();
      auto command = m_impl->commands.begin();
      if (command != m_impl->commands.end() && *command) {
        cmd = *command;
        m_impl->commands.erase(command);
        tickle_me = true;
      }
      // if (need_to_active) {
      //   ++m_activeThreadCount_;
      //   tickle_me = true;
      // }
      if (m_stopping_ ) {
        if (cmd) {
          if (cmd->m_state == Command::State::RUNNING ) {
            cmd->m_state =Command::State::CANCELED;
          }
        }
      }
    }
    if (tickle_me) {
      ++m_activeThreadCount_;
      tickle();
      // tickle_me = false;
    }
    if (cmd.get()) {
      switch (cmd->m_state) {
        case Command::State::WAIT:
          cmd->schedule();
          cmd->m_state = Command::State::INIT;
          break;
        case Command::State::INIT:
          cmd->initialize();
          if (cmd->isFinisheddec()) {
            cmd->m_state = Command::State::FINISHED;
            if (cmd->m_parent) {
              cmd->m_parent->m_state = Command::State::RUNNING;
              cmd->m_parent->schedule();
            }
          }
          cmd->schedule();
          cmd->m_state = Command::State::RUNNING;
          break;
        case Command::State::RUNNING:
          if (cmd->isFinisheddec()) {
            cmd->m_state = Command::State::FINISHED;
          } else {
            if (cmd->m_state != Command::State::HOLDON) cmd->execute();
          }
          if (cmd->m_state != Command::State::HOLDON) {
            cmd->schedule();
          }
          break;
        case Command::State::CANCELED:
          cmd->cancel();
          break;
        case Command::State::FINISHED:
          // if (cmd->m_state != Command::State::STOP)  {
          cmd->end();
          // }
          
          if (cmd->m_parent) {
            if (cmd->m_parent->m_state != Command::State::STOP) {
              cmd->m_parent->m_state = Command::State::RUNNING;
              cmd->m_parent->schedule();
            }
            cmd->m_parent.reset();
          }
          cmd->m_state = Command::State::STOP;
          break;
        default:
          break;
      }
      --m_activeThreadCount_;
    }
    // if (need_to_active) {
    //   ++m_activeThreadCount_;
    //   tickle_me = true;
    // }
    {
      MutexType::Lock lock(m_mutex_);
      if (m_autoStop_ && !cmd && m_impl->commands.empty() && !hasTimer() && m_activeThreadCount_ <= 0) {
        // std::cout << "m_autoStop_:" << m_autoStop_ << std::endl;
        // std::cout << "m_impl->commands.empty()" << m_impl->commands.empty() << std::endl;
        // std::cout << "hasTimer" << hasTimer() << std::endl;
        break;
      }
    }
    if (m_stopping_) {
      m_autoStop_ = true;
      if (cmd) {
        if (cmd->m_state == Command::State::RUNNING || cmd->m_state == Command::State::INIT) {
          cmd->cancel();
        }
        if (!tickle_me) {
          tickle_me = true;
          --m_activeThreadCount_;
        }
        m_impl->commands.clear();
      }
    }
    
    // if (cmd) {
    //   cmd->setScheduleStatus(false);
    // }
    ++m_idleThreadCount_;
    idle();
    --m_idleThreadCount_;
  }
}

void Scheduler::tickle() {
  // if (!hasIdleThreads()) {
  //   std::cout << "no idle threads" << std::endl;
  //   return;
  // }
  int rt = write(m_tickleFds_[1], "T", 1);
  // std::cout << "tickle" << std::endl;
  if (rt != 1) {
    std::cout << "tickle failed rt=" << rt << std::endl;
  }
}
void Scheduler::idle() {
  epoll_event* events = new epoll_event[64]();
  static const uint64_t MAX_TIMEOUT = 3000;
  static uint64_t next_timeout;
  next_timeout = 0;
  stopping(next_timeout);
  // if (stopping(next_timeout)) {
    // std::cout << "idle stopping" << std::endl;
    // return;
  // }
  // uint64_t next_timeout = 0;
  // if (getCurrentMs() >  MAX_TIMEOUT) {
  //   return;
  // }
  // HGSYS_LOG_INFO(g_logger) << "next_timeout=" << next_timeout;
  int rt = 0;
  // std::cout << RobotGenius::getThreadId() << " idle" << std::endl;
  // do {
    // std::cout << "tickle:" << rt << std::endl;
    // tickle();
    if (next_timeout != ~0ull) {
      next_timeout = std::min(next_timeout, MAX_TIMEOUT);
    } else {
      next_timeout = MAX_TIMEOUT;
    }
    rt = epoll_wait(m_epfd_, events, 64, static_cast<int>(next_timeout));
    if (rt < 0 && errno == EINTR) {
      std::cout << "create epoll wait failed" << std::endl;
    } 
  // } while (true);
  std::vector<Command::Ptr> cbs;
  listExpiredCb(cbs);
  cbs.erase(std::remove_if(cbs.begin(), cbs.end(),
        [](const Command::Ptr& cb) {
            return cb->getWorkCommandState() != Command::State::HOLDON;
        }), cbs.end());
  for (auto cb : cbs) {
    // if (cb->getWorkCommandState() == Command::State::RUNNING) {
    //   cb->m_state = Command::State::HOLDON;
    // } else {
      if (cb->m_state == Command::State::HOLDON && !cb->getTimer()->isPause()) {
        cb->m_state = Command::State::RUNNING;
      }
      
    // }
  }
  if (!cbs.empty()) {
    Scheduler::GetInstance().schedule(cbs);
    for (int i = 0; i < cbs.size(); i++) {
      tickle();
    }
    cbs.clear();
  }
  for(int i = 0; i < rt; ++i) {
    epoll_event& event = events[i];
    if(event.data.fd == m_tickleFds_[0]) {
        uint8_t dummy[256];
        while(read(m_tickleFds_[0], dummy, sizeof(dummy)) > 0);
        continue;
    }
  }
}

bool Scheduler::stopping(uint64_t& timeout) {
  timeout = getNextTimer();
  // HGSYS_LOG_INFO(g_logger) << "timeout=" << timeout;
  return timeout == ~0ull;
}

void Scheduler::onTimerInsertedAtFront() {
  tickle();
}

void schedulerManagerStart(size_t threads, bool use_caller, const std::string name) {
  Scheduler::GetInstance(threads, use_caller, name).start();
} 
void schedulerManagerStop() {
  Scheduler::GetInstance().stop();
}
}  //  namespace RobotGenius


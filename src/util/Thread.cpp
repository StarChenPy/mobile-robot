#include "util/Thread.h"
#include "util/Util.h"
#include <iostream>
#include <string>

namespace robot {

    static thread_local Thread *t_thread = nullptr;
    static thread_local std::string t_thread_name = "UNKNOW";

    Thread *Thread::GetThis() { return t_thread; }
    const std::string &Thread::GetName() { return t_thread_name; }
    void Thread::SetName(const std::string &name) {
        if (name.empty()) {
            return;
        }
        if (t_thread) {
            t_thread->m_name_ = name;
        }
        t_thread_name = name;
    }
    Thread::Thread(std::function<void()> cb, const std::string &name) : m_cb_(cb), m_name_(name) {
        if (name.empty()) {
            m_name_ = "UNKNOW";
        }
        int rt = pthread_create(&m_thread_, nullptr, &Thread::run, this);
        if (rt) {
            std::cout << "pthread_create thread fail, rt= " << rt << "name=" << name << std::endl;
            throw std::logic_error("pthread_create error");
        }
        /**
         * @brief 等待线程已启动
         *
         * @details
         * 在run函数中有信号量notify，正确且等待启动一个线程后才会退出线程初始化
         *          开始下一个线程的初始化
         *
         */
        m_semaphore_.wait();
    }
    Thread::~Thread() {
        if (m_thread_) {
            pthread_detach(m_thread_);
        }
    }

    void Thread::join() {
        if (m_thread_) {
            int rt = pthread_join(m_thread_, nullptr);
            if (rt) {
                std::cout << "pthread_join thread fail, rt=" << rt << "name=" << m_name_;
                throw std::logic_error("pthread_join error");
            }
            m_thread_ = 0;
        }
    }
    void *Thread::run(void *arg) {
        auto *thread = reinterpret_cast<Thread *>(arg);
        t_thread = thread;
        t_thread_name = thread->m_name_;
        thread->m_id_ = robot::getThreadId();
        pthread_setname_np(pthread_self(), thread->m_name_.substr(0, 15).c_str());

        std::function<void()> cb;
        cb.swap(thread->m_cb_);

        thread->m_semaphore_.notify();
        cb();

        return nullptr;
    }
} //  namespace robot

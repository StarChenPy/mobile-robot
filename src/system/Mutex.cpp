#include "system/Mutex.h"
#include <stdexcept>

namespace robot {
    Semaphore::Semaphore(uint32_t count) {
        if (sem_init(&this->m_semaphore_, 0, count)) {
            throw std::logic_error("sem_init error");
        }
    }
    Semaphore::~Semaphore() { sem_destroy(&this->m_semaphore_); }

    void Semaphore::wait() {
        if (sem_wait(&this->m_semaphore_)) {
            throw std::logic_error("sem_wait error");
        }
    }
    void Semaphore::notify() {
        if (sem_post(&this->m_semaphore_)) {
            throw std::logic_error("sem_post error");
        }
    }
} // namespace robot
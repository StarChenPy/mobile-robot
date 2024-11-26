/**
 * @file Mutex.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Mutex.h"
#include <stdexcept>

namespace RobotGenius {
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

} // namespace RobotGenius
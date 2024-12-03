#include "util/Timer.h"
#include "command/ICommand.h"

namespace robot {

Timer::Timer(uint64_t next) : m_next_(next) {}

bool Timer::Comparator::operator()(const Timer::ptr &lhs, const Timer::ptr &rhs) const {
    if (!lhs && !rhs) {
        return false;
    }
    if (!lhs) {
        return true;
    }
    if (!rhs) {
        return false;
    }
    if (lhs->m_next_ < rhs->m_next_) {
        return true;
    }
    if (rhs->m_next_ < lhs->m_next_) {
        return false;
    }
    return lhs.get() < rhs.get();
}

Timer::Timer(uint64_t ms, ICommand::ptr command, bool recurring, TimerManager *manager)
    : m_ms_(ms), m_command_(command), m_recurring_(recurring), m_manager_(manager) {
    m_next_ = getCurrentMs() + m_ms_;
}

Timer::Timer(uint64_t ms, TimerManager *manager) {
    m_ms_ = ms;
    m_recurring_ = true;
    m_manager_ = manager;
    m_next_ = getCurrentMs() + m_ms_;
}
void Timer::setCommand(ICommand::ptr command) { m_command_ = command; }
bool Timer::cancel() {
    TimerManager::RWMutexType::WriteLock lock(m_manager_->m_mutex_);
    if (m_command_) {
        auto it = m_manager_->m_timers_.find(shared_from_this());
        if (it != m_manager_->m_timers_.end()) {
            m_manager_->m_timers_.erase(it);
        }
        return true;
    }
    return false;
}

Timer::ptr TimerManager::addTimer(uint64_t ms, ICommand::ptr command, bool recurring) {
    Timer::ptr timer(new Timer(ms, command, recurring, this));
    RWMutexType::WriteLock lock(m_mutex_);
    addTimer(timer, lock);
    return timer;
}

void TimerManager::addTimer(Timer::ptr val) {
    RWMutexType::WriteLock lock(m_mutex_);
    addTimer(val, lock);
}

uint64_t TimerManager::getNextTimer() {
    RWMutexType::ReadLock lock(m_mutex_);
    m_tickled_ = false;
    ///   如果没有定时器，那么说明程序可以直接等待长时间
    if (m_timers_.empty()) {
        return ~0ull;
    }
    const Timer::ptr &next = *m_timers_.begin();
    uint64_t now_ms = getCurrentMs();
    // HGSYS_LOG_INFO(g_logger) << "m_next_=" << next->m_next_
    //                         << " nows_ms=" << now_ms;
    if (now_ms >= next->m_next_) {
        return 0;
    } else {
        return next->m_next_ - now_ms;
    }
}

void TimerManager::addTimer(Timer::ptr val, RWMutexType::WriteLock &lock) {
    ///  set进行inster后返回一个pair，第一个为位置，第二个为是否成功
    auto it = m_timers_.insert(val).first;
    bool at_front = (it == m_timers_.begin()) && !m_tickled_;
    m_tickled_ = at_front ? true : false;
    lock.unlock();
    if (at_front) {
        onTimerInsertedAtFront();
    }
}

bool TimerManager::detectClockRollover(uint64_t now_ms) {
    bool rollover = false;
    if (now_ms < m_previouseTime && now_ms < (m_previouseTime - 60 * 60 * 1000)) {
        rollover = true;
    }
    m_previouseTime = now_ms;
    return rollover;
}

bool TimerManager::hasTimer() {
    RWMutexType::ReadLock lock(m_mutex_);
    return !m_timers_.empty();
}

void TimerManager::handleExpiredTimers(
        std::vector<Timer::ptr> &expired, uint64_t now_ms,
        std::set<robot::Timer::ptr, robot::Timer::Comparator>::iterator &it) {
    expired.clear();
    RWMutexType::ReadLock lock(m_mutex_);
    if (m_timers_.empty()) {
        return;
    }

    bool rollover = detectClockRollover(now_ms);
    if (!rollover && ((*m_timers_.begin())->m_next_ > now_ms)) {
        return;
    }
    Timer::ptr now_timer(new Timer(now_ms));

    it = rollover ? m_timers_.end() : m_timers_.lower_bound(now_timer);
    ///  超时的可能有多个计时器，it是为了取出最后一个超时的定时器，结合begin就能知道是哪些计时器超时了
    while (it != m_timers_.end() && (*it)->m_next_ == now_ms) {
        ++it;
    }
    expired.insert(expired.begin(), m_timers_.begin(), it);
    // lock.unlock();
}
void TimerManager::listExpiredCb(std::vector<ICommand::ptr> &cbs) {
    uint64_t now_ms = getCurrentMs();

    std::vector<Timer::ptr> expired;
    // {
    //     RWMutexType::ReadLock lock(m_mutex_);
    //     if (m_timers_.empty()) {
    //       return;
    //     }
    // }
    RWMutexType::WriteLock lock(m_mutex_);
    if (m_timers_.empty()) {
        return;
    }
    bool rollover = detectClockRollover(now_ms);
    if (!rollover && ((*m_timers_.begin())->m_next_ > now_ms)) {
        return;
    }
    Timer::ptr now_timer(new Timer(now_ms));

    auto it = rollover ? m_timers_.end() : m_timers_.lower_bound(now_timer);
    ///  超时的可能有多个计时器，it是为了取出最后一个超时的定时器，结合begin就能知道是哪些计时器超时了
    while (it != m_timers_.end() && (*it)->m_next_ == now_ms) {
        ++it;
    }
    expired.insert(expired.begin(), m_timers_.begin(), it);

    // {
    // RWMutexType::WriteLock lock(m_mutex_);
    m_timers_.erase(m_timers_.begin(), it);
    cbs.reserve(expired.size());
    for (auto &timer : expired) {
        // std::cout << "timer->m_command_->getWorkCommandState()="
        //           << timer->m_command_->getWorkCommandState() << std::endl;
        // if (timer->m_command_->getWorkCommandState() ==
        // ICommand::State::RUNNING)
        // {
        //   std::cout << "timer is running" << std::endl;
        //   continue;
        // }
        cbs.push_back(timer->m_command_);
        ///  非循环的定时器是先运行再进入等待延时的
        ///  所以如果是非循环那么即使超时也不用再运行了
        if (timer->m_recurring_) {
            timer->m_next_ = now_ms + timer->m_ms_;
            m_timers_.insert(timer);
        } else {
            timer->m_command_ = nullptr;
        }
    }
    // lock.unlock();
    // }
}

} // namespace robot

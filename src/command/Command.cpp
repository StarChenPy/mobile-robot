/**
 * @file Command.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-12
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "command/Command.h"
#include "command/TimerCommand.h"
#include "util/Scheduler.h"
#include <memory>

namespace robot {
Command::~Command() = default;

bool Command::schedule() {
    // 如果没有工作命令，返回false
    // 子类有可能没有给m_work_command_赋值
    isScheduled_ = Scheduler::getInstance().schedule(getPtr());
    Scheduler::getInstance().tickle();
    return isScheduled_;
}
Command::ptr Command::getPtr() {
    try {
        std::shared_ptr<Command> selfPtr = shared_from_this();
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        return std::shared_ptr<Command>(this);
    }
}
Command::ptr Command::withTimer(uint64_t ms) {
    return std::make_shared<TimerCommand>(ms, getPtr());
}

bool Command::isFinishedDec() { return stopFlag_ || isFinished() || globalStop_; }
void Command::cancel() {
    stopFlag_ = true;
    state_ = Command::State::FINISHED;
    if (!hasTimer_)
        schedule();
    else
        Scheduler::getInstance().tickle();
}
} // namespace robot
/**
 * @file TimerCommand.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 定时命令
 * @version 0.1
 * @date 2024-04-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "command/TimerCommand.h"
#include "command/Scheduler.h"

namespace robot {

TimerCommand::TimerCommand(uint64_t ms, Command::ptr command) : m_ms_(ms) {
    workCommand_ = command;
    workCommand_->state_ = Command::State::PAUSED;
}
Command::ptr TimerCommand::reset() {
    workCommand_ = workCommand_->reset();
    return std::make_shared<TimerCommand>(m_ms_, workCommand_);
}
void TimerCommand::initialize() {
    workCommand_->initialize();
    workCommand_->hasTimer_ = true;
    workCommand_->state_ = Command::State::PAUSED;
    timer_ = std::make_shared<Timer>(m_ms_, &Scheduler::GetInstance());
    timer_->setCommand(getPtr());
    Scheduler::GetInstance().addTimer(timer_);
}
void TimerCommand::execute() {
    workCommand_->state_ = Command::State::RUNNING;
    workCommand_->execute();
    workCommand_->state_ = Command::State::PAUSED;
    state_ = Command::State::PAUSED;
}
void TimerCommand::end() {
    if (!workCommand_->isFinished()) {
        workCommand_->cancel();
        if (timer_.get()) {
            timer_->cancel();
        }
    }
    workCommand_->end();
    if (timer_.get()) {
        timer_->cancel();
    }
}

bool TimerCommand::isFinished() {
    // std::cout << workCommand_->isFinished() << std::endl;
    return workCommand_->isFinished();
}

} // namespace robot
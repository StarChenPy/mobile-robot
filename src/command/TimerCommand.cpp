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
#include "util/Scheduler.h"

namespace robot {

TimerCommand::TimerCommand(uint64_t ms, ICommand::ptr command) : m_ms_(ms) {
    workCommand_ = command;
    workCommand_->state_ = ICommand::State::PAUSED;
}

void TimerCommand::initialize() {
    workCommand_->initialize();
    workCommand_->hasTimer_ = true;
    workCommand_->state_ = ICommand::State::PAUSED;
    timer_ = std::make_shared<Timer>(m_ms_, &Scheduler::getInstance());
    timer_->setCommand(getPtr());
    Scheduler::getInstance().addTimer(timer_);
}

void TimerCommand::execute() {
    workCommand_->state_ = ICommand::State::RUNNING;
    workCommand_->execute();
    workCommand_->state_ = ICommand::State::PAUSED;
    state_ = ICommand::State::PAUSED;
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
    return workCommand_->isFinished();
}

} // namespace robot
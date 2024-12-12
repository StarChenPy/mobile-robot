#include <utility>

#include "command/TimerCommand.h"
#include "system/Scheduler.h"

namespace robot {

TimerCommand::TimerCommand(uint64_t ms, ICommand::ptr command) : cycle(ms) {
    workCommand_ = std::move(command);
    workCommand_->state_ = ICommand::State::PAUSED;
}

void TimerCommand::initialize() {
    workCommand_->initialize();
    workCommand_->hasTimer_ = true;
    workCommand_->state_ = ICommand::State::PAUSED;
    timer_ = std::make_shared<Timer>(cycle, &Scheduler::getInstance());
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
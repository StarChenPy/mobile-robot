#include "command/group/ParallelDeadlineGroup.h"

namespace robot {

    ParallelDeadlineGroup::ParallelDeadlineGroup() {
        commands_.clear();
    }
    void ParallelDeadlineGroup::initialize() {
        if (isFinished()) {
            return;
        }
        if (isScheduleDeadlineCommand_)
            deadlineCommand_->schedule();
        for (const auto& command : commands_) {
            if (!command->isFinished()) {
                command->schedule();
            }
        }
    }

    void ParallelDeadlineGroup::execute() {
        state_ = ICommand::State::PAUSED;
    }

    void ParallelDeadlineGroup::end() {
        for (const auto& command : commands_) {
            if (!command->isFinished()) {
                command->cancel();
            }
            if (command->state_ != ICommand::State::STOP) {
                command->schedule();
            }
        }
        if (!deadlineCommand_->isFinished()) {
            deadlineCommand_->cancel();
        }
        if (deadlineCommand_->state_ != ICommand::State::STOP) {
            if (isScheduleDeadlineCommand_)
                deadlineCommand_->schedule();
        }
        commands_.clear();
        if (nextCommand_.get()) {
            nextCommand_->schedule();
        }
    }

    bool ParallelDeadlineGroup::isFinished() {
        if (!deadlineCommand_) {
            return true;
        }
        if (deadlineCommand_->isFinished()) {
            for (const auto& command : commands_) {
                command->cancel();
            }
            return true;
        }
        return false;
    }

    void ParallelDeadlineGroup::disableScheduleDeadlineCommand() { isScheduleDeadlineCommand_ = false; }
    void ParallelDeadlineGroup::enableScheduleDeadlineCommand() { isScheduleDeadlineCommand_ = true; }
    void ParallelDeadlineGroup::setDeadlineCommand(ICommand::ptr command, bool schedule) {
        deadlineCommand_ = command;
        deadlineCommand_->parent_ = getPtr();
        isScheduleDeadlineCommand_ = schedule;
    }

    ParallelDeadlineGroup::ptr ParallelDeadlineGroup::create() {
        return std::make_shared<ParallelDeadlineGroup>();
    }
} // namespace robot
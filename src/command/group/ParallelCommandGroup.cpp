#include "command/group/ParallelCommandGroup.h"

namespace robot {
    void ParallelCommandGroup::initialize() {
        for (const auto& command : commands_) {
            command->schedule();
        }
        state_ = ICommand::State::PAUSED;
    }

    void ParallelCommandGroup::execute() {
        state_ = ICommand::State::PAUSED;
    }

    bool ParallelCommandGroup::isFinished() {
        for (const auto& command : commands_) {
            if (!command->isFinished()) {
                return false;
            }
        }
        return true;
    }

    void ParallelCommandGroup::end() {
        for (const auto& command : commands_) {
            if (!command->isFinished()) {
                command->cancel();
            }
            if (command->state_ != ICommand::State::STOP) {
                command->schedule();
            }
        }
        commands_.clear();
        if (nextCommand_.get()) {
            nextCommand_->schedule();
        }
    }

    ICommandGroup::ptr ParallelCommandGroup::create() {
        return std::make_shared<ParallelCommandGroup>();
    }
} //  namespace robot
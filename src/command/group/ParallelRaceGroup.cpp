#include "command/group/ParallelRaceGroup.h"

namespace robot {
    void ParallelRaceGroup::initialize() {
        for (const auto& command : commands_) {
            command->schedule();
        }
        state_ = ICommand::State::PAUSED;
    }

    void ParallelRaceGroup::execute() {
        state_ = ICommand::State::PAUSED;
    }

    void ParallelRaceGroup::end() {
        for (const auto& command : commands_) {
            if (!command->isFinished()) {
                command->cancel();
            }
        }
        commands_.clear();
        if (nextCommand_.get()) {
            nextCommand_->schedule();
        }
    }

    bool ParallelRaceGroup::isFinished() {
        stopCommandIndex_ = 0;
        for (const auto& command : commands_) {
            if (command->isFinished()) {
                stopCommandIndex_++;
                return true;
            }
        }
        return commands_.empty();
    }

    ICommandGroup::ptr ParallelRaceGroup::create() {
        return std::make_shared<ParallelRaceGroup>();
    }
} // namespace robot
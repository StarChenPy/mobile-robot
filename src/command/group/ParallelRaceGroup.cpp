/**
 * @file ParallelRaceGroup.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "command/group/ParallelRaceGroup.h"

namespace robot {

ParallelRaceGroup::ParallelRaceGroup() {
    commands_.clear();
    isGroup_ = true;
}
void ParallelRaceGroup::initialize() {
    for (const auto& command : commands_) {
        command->schedule();
    }
    state_ = Command::State::PAUSED;
}
void ParallelRaceGroup::execute() {
    state_ = Command::State::PAUSED;
}

void ParallelRaceGroup::end() {
    for (const auto& command : commands_) {
        if (!command->isFinishedDec()) {
            command->cancel();
        }
    }
    commands_.clear();
    if (nextCommand_.get()) {
        nextCommand_->schedule();
    }
}
bool ParallelRaceGroup::isFinished() {
    stop_command_index = 0;
    for (const auto& command : commands_) {
        if (command->isFinishedDec()) {
            stop_command_index++;
            return true;
        }
    }
    return commands_.empty();
}
Command::ptr ParallelRaceGroup::reset() {
    ParallelRaceGroup::ptr RG = createParallelRaceGroup();
    for (auto command : commands_) {
        command = command->reset();
        RG->addCommand(command);
    }
    return RG;
}
ParallelRaceGroup::ptr createParallelRaceGroup() {
    ParallelRaceGroup::ptr parallel = std::make_shared<ParallelRaceGroup>();
    return parallel;
}
} // namespace robot
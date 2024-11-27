/**
 * @file ParallelCommandGroup.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "command/group/ParallelCommandGroup.h"

namespace robot {

ParallelCommandGroup::ParallelCommandGroup() {
    commands_.clear();
    isGroup_ = true;
}

void ParallelCommandGroup::initialize() {
    // std::cout << "ParallelCommandGroup initialize" << std::endl;
    for (auto command : commands_) {
        command->schedule();
    }
    state_ = Command::State::PAUSED;
}
void ParallelCommandGroup::execute() {
    // std::cout << "ParallelCommandGroup execute" << std::endl;
    state_ = Command::State::PAUSED;
}

bool ParallelCommandGroup::isFinished() {
    for (auto command : commands_) {
        if (!command->isFinished()) {
            return false;
        }
    }
    return true;
}
void ParallelCommandGroup::end() {
    for (auto command : commands_) {
        if (!command->isFinished()) {
            command->cancel();
        }
        if (command->state_ != Command::State::STOP) {
            command->schedule();
        }
    }
    commands_.clear();
    if (m_next_command_.get()) {
        m_next_command_->schedule();
    }
}
Command::ptr ParallelCommandGroup::reset() {
    ParallelCommandGroup::Ptr PG = createParallelCommandGroup();
    for (auto command : commands_) {
        command = command->reset();
        PG->addCommand(command);
    }
    return PG;
}
ParallelCommandGroup::Ptr createParallelCommandGroup() {
    ParallelCommandGroup::Ptr parallel_command_group = std::make_shared<ParallelCommandGroup>();
    return parallel_command_group;
}
} //  namespace robot
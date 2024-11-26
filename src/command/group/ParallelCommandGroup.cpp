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

namespace RobotGenius {

ParallelCommandGroup::ParallelCommandGroup() {
    m_commands_.clear();
    m_is_group_ = true;
}

void ParallelCommandGroup::initialize() {
    // std::cout << "ParallelCommandGroup initialize" << std::endl;
    for (auto command : m_commands_) {
        command->schedule();
    }
    m_state = Command::State::HOLDON;
}
void ParallelCommandGroup::execute() {
    // std::cout << "ParallelCommandGroup execute" << std::endl;
    m_state = Command::State::HOLDON;
}

bool ParallelCommandGroup::isFinished() {
    for (auto command : m_commands_) {
        if (!command->isFinished()) {
            return false;
        }
    }
    return true;
}
void ParallelCommandGroup::end() {
    for (auto command : m_commands_) {
        if (!command->isFinished()) {
            command->cancel();
        }
        if (command->m_state != Command::State::STOP) {
            command->schedule();
        }
    }
    m_commands_.clear();
    if (m_next_command_.get()) {
        m_next_command_->schedule();
    }
}
Command::ptr ParallelCommandGroup::reset() {
    ParallelCommandGroup::Ptr PG = createParallelCommandGroup();
    for (auto command : m_commands_) {
        command = command->reset();
        PG->AddCommands(command);
    }
    return PG;
}
ParallelCommandGroup::Ptr createParallelCommandGroup() {
    ParallelCommandGroup::Ptr parallel_command_group = std::make_shared<ParallelCommandGroup>();
    return parallel_command_group;
}
} //  namespace RobotGenius
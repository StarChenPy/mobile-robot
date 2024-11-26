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

namespace RobotGenius {

ParallelRaceGroup::ParallelRaceGroup() {
    m_commands_.clear();
    m_is_group_ = true;
}
void ParallelRaceGroup::initialize() {
    // std::cout << "ParallelRaceGroup initialize" << std::endl;
    for (auto command : m_commands_) {
        command->schedule();
    }
    m_state = Command::State::HOLDON;
}
void ParallelRaceGroup::execute() {
    // std::cout << "ParallelRaceGroup execute" << std::endl;
    m_state = Command::State::HOLDON;
}

void ParallelRaceGroup::end() {
    for (auto command : m_commands_) {
        if (!command->isFinisheddec()) {
            command->cancel();
        }
        // if (command->m_state != Command::State::STOP) {
        // command->schedule();
        // }
    }
    m_commands_.clear();
    if (m_next_command_.get()) {
        m_next_command_->schedule();
    }
}
bool ParallelRaceGroup::isFinished() {
    stop_command_index = 0;
    for (auto command : m_commands_) {
        if (command->isFinisheddec()) {
            stop_command_index++;
            return true;
        }
    }
    return m_commands_.empty();
}
Command::ptr ParallelRaceGroup::reset() {
    ParallelRaceGroup::ptr RG = createParallelRaceGroup();
    for (auto command : m_commands_) {
        command = command->reset();
        RG->AddCommands(command);
    }
    return RG;
}
ParallelRaceGroup::ptr createParallelRaceGroup() {
    ParallelRaceGroup::ptr parallel = std::make_shared<ParallelRaceGroup>();
    return parallel;
}
} // namespace RobotGenius
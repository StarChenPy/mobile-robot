/**
 * @file SequentialCommandGroup.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "command/group/SequentialCommandGroup.h"

namespace RobotGenius {

void SequentialCommandGroup::initialize() {
    // std::cout << "SequentialCommandGroup initialize" << std::endl;
    m_is_finished_ = false;
    if (m_commands_.empty()) {
        m_is_finished_ = true;
        return;
    }
    m_current_command_index_ = 0;
    // std::cout << m_current_command_index_ << std::endl;
    m_current_command_ = m_commands_[m_current_command_index_];
    m_current_command_->schedule();
    // m_current_command_->initialize();
    // m_current_command_->m_state = Command::State::RUNNING;
}

void SequentialCommandGroup::execute() {
    // std::cout << "SequentialCommandGroup execute" << std::endl;
    if (!m_current_command_)
        return;
    if (m_current_command_->isFinished()) {

        if (m_current_command_index_ + 1 < m_commands_.size()) {
            // std::cout << "m_current_command_index_" << m_commands_.size() <<
            // " " << m_current_command_index_ << " " << m_current_command_ <<
            // std::endl;
            m_current_command_index_++;
            m_current_command_ = m_commands_[m_current_command_index_];
            // std::cout << "m_current_command_index_" <<
            // m_current_command_index_ << " " << m_current_command_ <<
            // std::endl; m_current_command_->m_state = Command::State::INIT;
            if (m_current_command_ && m_current_command_->m_state != Command::State::STOP) {
                m_current_command_->schedule();
            }
        } else {
            m_is_finished_ = true;
        }
    }
    if (m_state != Command::State::STOP && !m_is_finished_) {
        m_state = Command::State::HOLDON;
    }
}

void SequentialCommandGroup::end() {
    // std::cout << "SequentialCommandGroup end" << std::endl;
    // for (int index = m_current_command_index_; index < m_commands_.size();
    // index++) {
    if (m_commands_[m_current_command_index_]->m_state != Command::State::STOP &&
        m_commands_[m_current_command_index_]->m_state != Command::State::FINISHED) {
        m_commands_[m_current_command_index_]->cancel();
        // m_commands_[index]->schedule();
    }
    // }
    m_commands_.clear();
    if (m_next_command_.get()) {
        m_next_command_->schedule();
    }
    // std::cout <<  "SequentialCommandGroup:end" << std::endl;
}
bool SequentialCommandGroup::isFinished() { return m_is_finished_; }
Command::ptr SequentialCommandGroup::reset() {
    SequentialCommandGroup::Ptr Seq = createSequentialCommandGroup();
    for (auto command : m_commands_) {
        command = command->reset();
        Seq->AddCommands(command);
    }
    return Seq;
}
std::shared_ptr<SequentialCommandGroup> createSequentialCommandGroup() {
    SequentialCommandGroup::Ptr sequential_command_group = std::make_shared<SequentialCommandGroup>();
    return sequential_command_group;
}
} //  namespace RobotGenius
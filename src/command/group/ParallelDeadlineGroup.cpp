/**
 * @file ParallelDeadlineGroup.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "command/group/ParallelDeadlineGroup.h"

namespace robot {

ParallelDeadlineGroup::ParallelDeadlineGroup() {
    commands_.clear();
    isGroup_ = true;
}
void ParallelDeadlineGroup::initialize() {
    if (isFinished()) {
        return;
    }
    if (isScheduleDeadlineCommand_)
        m_deadline_command_->schedule();
    for (auto command : commands_) {
        if (!command->isFinished()) {
            command->schedule();
        }
    }
}

void ParallelDeadlineGroup::execute() {
    // std::cout << "ParallelDeadlineGroup execute" << std::endl;
    state_ = Command::State::PAUSED;
}

void ParallelDeadlineGroup::end() {
    // std::cout <<  "ParallelDeadlineGroup:end" << std::endl;
    for (auto command : commands_) {
        if (!command->isFinishedDec()) {
            command->cancel();
        }
        if (command->state_ != Command::State::STOP) {
            command->schedule();
        }
        // std::cout <<  "ParallelDeadlineGroup:end" << std::endl;
    }
    if (!m_deadline_command_->isFinishedDec()) {
        m_deadline_command_->cancel();
    }
    if (m_deadline_command_->state_ != Command::State::STOP) {
        if (isScheduleDeadlineCommand_)
            m_deadline_command_->schedule();
    }
    commands_.clear();
    if (nextCommand_.get()) {
        nextCommand_->schedule();
    }
}
bool ParallelDeadlineGroup::isFinished() {
    if (!m_deadline_command_) {
        return true;
    }
    if (m_deadline_command_->isFinishedDec()) {
        for (auto command : commands_) {
            command->cancel();
        }
        return true;
    }
    return false;
}

void ParallelDeadlineGroup::disableScheduleDeadlineCommand() { isScheduleDeadlineCommand_ = false; }
void ParallelDeadlineGroup::enableScheduleDeadlineCommand() { isScheduleDeadlineCommand_ = true; }
void ParallelDeadlineGroup::setDeadlineCommand(Command::ptr command, bool schedule) {
    m_deadline_command_ = command;
    m_deadline_command_->parent_ = getPtr();
    isScheduleDeadlineCommand_ = schedule;
}

Command::ptr ParallelDeadlineGroup::reset() {
    ParallelDeadlineGroup::ptr DG = createParallelDeadlineGroup();
    for (auto command : commands_) {
        command = command->reset();
        DG->addCommand(command);
    }
    if (isScheduleDeadlineCommand_)
        m_deadline_command_ = m_deadline_command_->reset();
    DG->setDeadlineCommand(m_deadline_command_);
    return DG;
}

ParallelDeadlineGroup::ptr createParallelDeadlineGroup() {
    ParallelDeadlineGroup::ptr parallel_deadline_group = std::make_shared<ParallelDeadlineGroup>();
    return parallel_deadline_group;
}

} // namespace robot
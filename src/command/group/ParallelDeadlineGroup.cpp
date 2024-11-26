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

namespace RobotGenius {

ParallelDeadlineGroup::ParallelDeadlineGroup() {
  m_commands_.clear();
  m_is_group_ = true;
}
void ParallelDeadlineGroup::initialize() {
  if (isFinished()) {
    return;
  }
  if (is_schedule_deadline_command_) m_deadline_command_->schedule();
  for (auto command : m_commands_) {
    if (!command->isFinished()) {
      command->schedule();
    }
  }
}

void ParallelDeadlineGroup::execute() {
  // std::cout << "ParallelDeadlineGroup execute" << std::endl;
  m_state = Command::State::HOLDON;
}


void ParallelDeadlineGroup::end() {
  // std::cout <<  "ParallelDeadlineGroup:end" << std::endl; 
  for (auto command : m_commands_) {
    if(!command->isFinisheddec()) {
      command->cancel();
    }
    if (command->m_state != Command::State::STOP) {
      command->schedule();
    }
      // std::cout <<  "ParallelDeadlineGroup:end" << std::endl; 
  }
  if (!m_deadline_command_->isFinisheddec()) {
    m_deadline_command_->cancel();
  }
  if (m_deadline_command_->m_state != Command::State::STOP) {
    if (is_schedule_deadline_command_) m_deadline_command_->schedule();
  }
  m_commands_.clear();
  if (m_next_command_.get()) {
    m_next_command_->schedule();
  }
}
bool ParallelDeadlineGroup::isFinished() {
  if (!m_deadline_command_) {
    return true;
  }
  if (m_deadline_command_->isFinisheddec()) {
    for (auto command : m_commands_) {
      command->cancel();
    }
    return true;
  }
  return false;
}

void ParallelDeadlineGroup::disableShceduleDeadlineCommand() {
  is_schedule_deadline_command_ = false;
}
void ParallelDeadlineGroup::enableShceduleDeadlineCommand() {
  is_schedule_deadline_command_ = true;
}
void ParallelDeadlineGroup::setDeadlineCommand(Command::Ptr command,bool schedule) {
  m_deadline_command_ = command;
  m_deadline_command_->m_parent = getPtr();
  is_schedule_deadline_command_ = schedule;
}

Command::Ptr ParallelDeadlineGroup::reset() {
  ParallelDeadlineGroup::Ptr DG = createParallelDeadlineGroup();
  for (auto command : m_commands_) {
    command = command->reset();
    DG->AddCommands(command);
  }
  if (is_schedule_deadline_command_) m_deadline_command_ = m_deadline_command_->reset();
  DG->setDeadlineCommand(m_deadline_command_);
  return DG;

}


ParallelDeadlineGroup::Ptr createParallelDeadlineGroup() {
  ParallelDeadlineGroup::Ptr parallel_deadline_group = std::make_shared<ParallelDeadlineGroup>();
  return parallel_deadline_group;
}



}  // namespace RobotGenius
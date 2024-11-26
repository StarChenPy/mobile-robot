/**
 * @file TimerCommand.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 定时命令
 * @version 0.1
 * @date 2024-04-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "command/TimerCommand.h"
#include "command/Scheduler.h"

namespace RobotGenius {

TimerCommand::TimerCommand(uint64_t ms, Command::Ptr command) : m_ms_(ms){
  m_work_command_ = command;
  m_work_command_->m_state = Command::State::HOLDON;
  
}
Command::Ptr TimerCommand::reset() {
  m_work_command_ = m_work_command_->reset();
  return std::make_shared<TimerCommand>(m_ms_, m_work_command_);
}
void TimerCommand::initialize() {
  m_work_command_->initialize();
  m_work_command_->has_timer_ = true;
  m_work_command_->m_state = Command::State::HOLDON;
  m_timer_ = std::make_shared<Timer>(m_ms_, &Scheduler::GetInstance());
  m_timer_->setCommand(getPtr());
  Scheduler::GetInstance().addTimer(m_timer_);
}
void TimerCommand::execute() {
  m_work_command_->m_state = Command::State::RUNNING;
  m_work_command_->execute();
  m_work_command_->m_state = Command::State::HOLDON;
  m_state = Command::State::HOLDON;
}
void TimerCommand::end() {
  // m_work_command_->m_state = Command::State::FINISHED;
  if (!m_work_command_->isFinished()) {
    m_work_command_->cancel();
    if (m_timer_.get()) {
      m_timer_->cancel();
    }
  }
  m_work_command_->end();
  if (m_timer_.get()) {
    m_timer_->cancel();
  }
}

bool TimerCommand::isFinished() {
  // std::cout << m_work_command_->isFinished() << std::endl;
  return m_work_command_->isFinished();
}

}  // namespace RobotGenius
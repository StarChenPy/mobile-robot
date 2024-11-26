/**
 * @file Command.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 
 * @version 0.1
 * @date 2024-04-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "command/Command.h"
#include <iostream>
#include <string>
#include "command/Scheduler.h"
#include "command/TimerCommand.h"
#include "command/group/ParallelDeadlineGroup.h"
namespace RobotGenius {
Command::~Command() {}
Command::Command(const Command& rhs) {
  m_isscheduled_ = false;
}

Command& Command::operator=(const Command& rhs) {
  m_isscheduled_ = false;
  return *this;
}

bool Command::schedule() {
  // 如果没有工作命令，返回false
  // 子类有可能没有给m_work_command_赋值
  // m_state = State::INIT;
  // std::cout << "C++:" << getPtr() << std::endl;
  // getPtr();
  // if (m_isscheduled_) {
  //   return false;
  // }
  // setScheduleStatus(true);
  m_isscheduled_ = Scheduler::GetInstance().schedule(getPtr());
  // std::cout << "aaaa" << std::endl;
  Scheduler::GetInstance().tickle();
  return m_isscheduled_;
}
Command::Ptr Command::getPtr() {
  try {
    std::shared_ptr<Command> selfPtr = shared_from_this();
    // std::cout << "C++:" << selfPtr.get()<< std::endl;
    return selfPtr;
  } catch(const std::bad_weak_ptr&) {
    // std::cout << "C++:" << this<< std::endl;
    return std::shared_ptr<Command>(this);
  }
}
Command::Ptr Command::withTimer(uint64_t ms) {
//  if (m_is_group_) {
//    std::cout << "Group command cannot have timer" << std::endl;
//    return nullptr;
//  }
  return TimerCommand::Ptr(new TimerCommand(ms, getPtr()));
}

int Command::getThreadId() {
  return getThreadId();
}

bool Command::isFinisheddec() {
  return m_stop_flag_ || isFinished() || gloabl_stop_;
}
void Command::cancel() {
  m_stop_flag_ = true;
  m_state = Command::State::FINISHED;
  if (!has_timer_) schedule();
  else Scheduler::GetInstance().tickle();
}
}  // namespace RobotGenius
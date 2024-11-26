/**
 * @file TimerCommand.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 定时命令
 * @version 0.1
 * @date 2024-04-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once
#include <memory>
#include <string>
#include "CommandBase.h"
#include "Timer.h"

namespace RobotGenius {

class TimerCommand : public CommandBase {

 public:
  typedef std::shared_ptr<TimerCommand> Ptr;
  TimerCommand(uint64_t ms, Command::Ptr command);
//   void setCommand(Command::ptr command);
  virtual ~TimerCommand() {}
  
 public:
  void initialize() override;
  void execute() override;
  void end() override;
  bool isFinished() override;
  Command::Ptr reset() override;
 private:
    uint64_t m_ms_;
};



}  // namespace RobotGenius
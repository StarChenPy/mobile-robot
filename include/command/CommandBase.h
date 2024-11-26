/**
 * @file CommandBase.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 
 * @version 0.1
 * @date 2024-04-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once
#include <memory>
#include "Command.h"
#include "Util.h"
#include "command/group/CommandGroupBase.h"
#include <iostream>
namespace RobotGenius {
class CommandBase : public Command {
 public:
  typedef std::shared_ptr<CommandBase> Ptr;
  CommandBase() {
    m_is_group_ = false;
  }
  virtual ~CommandBase();
 public:
  /**
   * @brief 将命令送进调度器队列
   * 
   * @return true 
   * @return false 
   */
  // bool schedule() override final;
  virtual void initialize() {}
  virtual void execute() {}
  virtual void end() {}
  virtual void cancel() {}
  virtual bool isFinished();
 protected:
  bool m_isscheduled_ = false;
  
};


}  // namespace RobotGenius
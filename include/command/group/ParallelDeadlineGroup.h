/**
 * @file ParallelDeadlineGroup.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 指定的命令结束则结束整个命令组，所有命令为并行执行
 * @version 0.1
 * @date 2024-04-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once
#include <memory>
#include <vector>
#include "CommandGroupBase.h"
#include "command/Scheduler.h"

namespace RobotGenius {

class ParallelDeadlineGroup : public CommandGroupBase {
 public:
  typedef std::shared_ptr<ParallelDeadlineGroup> Ptr;
  ParallelDeadlineGroup();
  template <class... Types>
  explicit ParallelDeadlineGroup(Types... commands) {
    (m_commands_.push_back(commands), ...);
    m_deadline_command_ = *m_commands_.begin();
    m_commands_.erase(m_commands_.begin());
  }
  virtual ~ParallelDeadlineGroup() {}

 public:
  void initialize() override;
  void execute() override;
  void end() override;
  bool isFinished() override;
  void disableShceduleDeadlineCommand();
  void enableShceduleDeadlineCommand();
  void setDeadlineCommand(Command::Ptr command,bool schedule = true);
  Command::Ptr reset() override;

 protected:
  Command::Ptr m_deadline_command_;
  bool is_schedule_deadline_command_ = true;
};

ParallelDeadlineGroup::Ptr createParallelDeadlineGroup();

}  // namespace RobotGenius
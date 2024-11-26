/**
 * @file ParallelCommandGroup.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 并行执行任务
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

class ParallelCommandGroup : public CommandGroupBase {
 public:
  typedef std::shared_ptr<ParallelCommandGroup> Ptr;
  ParallelCommandGroup();
  template <class... Types>
  explicit ParallelCommandGroup(Types... commands) {
    (m_commands_.push_back(commands), ...);
  }
  virtual ~ParallelCommandGroup() {}

 public:
  void initialize() override ;
  void execute() override ;
  void end() override  ;
  bool isFinished() override ;
  Command::Ptr reset() override;

};

ParallelCommandGroup::Ptr createParallelCommandGroup();


}  // namespace RobotGenius
/**
 * @file ConditionalCommand.h
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
#include <functional>
#include "CommandBase.h"

namespace RobotGenius {
class ConditionalComamand : public CommandBase {

 public:
  typedef std::shared_ptr<ConditionalComamand> Ptr;
  ConditionalComamand(std::function<bool()> condition, CommandBase::Ptr on_true_command, CommandBase::Ptr on_false_command);
  virtual ~ConditionalComamand() {}
 public:
  void initialize() override;
  void execute() override;
  void end() override;
  bool isFinished() override;
 protected:
  CommandBase::Ptr m_on_true_command_;
  CommandBase::Ptr m_on_false_command_;
  std::function<bool()> m_condition_;
};
}  // namespace RobotGenius
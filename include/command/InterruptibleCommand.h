/**
 * @file InterruptibleCommand.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-30
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "CommandBase.h"
#include "Timer.h"
#include <memory>
#include <string>

namespace RobotGenius {

class InterruptibleCommand : public CommandBase {

  public:
    typedef std::shared_ptr<InterruptibleCommand> Ptr;
    InterruptibleCommand(Command::ptr primary_command, Command::ptr interrupt_condition, Command::ptr fallback_command);
    //   void setCommand(Command::ptr command);
    virtual ~InterruptibleCommand() {}

  public:
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    Command::ptr m_primary_command_;
    Command::ptr m_interrupt_condition_;
    Command::ptr m_fallback_command_;
    enum Status { BEFORE = 0, InterruptB = 1, Interrupt = 2, InterruptA = 3, AFTER = 4, END = 5 };
    Status m_status = Status::BEFORE;
};

} // namespace RobotGenius
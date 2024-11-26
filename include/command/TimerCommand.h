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
#include "CommandBase.h"
#include "Timer.h"
#include <memory>
#include <string>

namespace robot {

class TimerCommand : public CommandBase {

  public:
    typedef std::shared_ptr<TimerCommand> ptr;
    TimerCommand(uint64_t ms, Command::ptr command);
    //   void setCommand(Command::ptr command);
    virtual ~TimerCommand() {}

  public:
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;
    Command::ptr reset() override;

  private:
    uint64_t m_ms_;
};

} // namespace robot
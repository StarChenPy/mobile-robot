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
#include "Command.h"
#include "Util.h"
#include "command/group/CommandGroupBase.h"
#include <iostream>
#include <memory>
namespace robot {
class CommandBase : public Command {
  public:
    typedef std::shared_ptr<CommandBase> Ptr;
    CommandBase() { isGroup_ = false; }
    ~CommandBase() override;

  public:
    /**
     * @brief 将命令送进调度器队列
     *
     * @return true
     * @return false
     */
    void initialize() override {}
    void execute() override {}
    void end() override {}
    virtual void cancel() {}
    bool isFinished() override;

  protected:
    bool isScheduled = false;
};

} // namespace robot
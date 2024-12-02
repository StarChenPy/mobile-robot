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
#include "CommandBase.h"
#include <functional>
#include <memory>

namespace robot {
class ConditionalCommand : public CommandBase {

  public:
    typedef std::shared_ptr<ConditionalCommand> ptr;
    ConditionalCommand(std::function<bool()> condition, CommandBase::ptr on_true_command,
                       CommandBase::ptr on_false_command);
    virtual ~ConditionalCommand() {}

  public:
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  protected:
    CommandBase::ptr m_on_true_command_;
    CommandBase::ptr m_on_false_command_;
    std::function<bool()> m_condition_;
};
} // namespace robot
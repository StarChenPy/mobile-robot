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
#include "ICommand.h"
#include <functional>
#include <memory>

namespace robot {
class ConditionalCommand : public ICommand {
  public:
    typedef std::shared_ptr<ConditionalCommand> ptr;

    ConditionalCommand(std::function<bool()> condition, ICommand::ptr onTrueCommand, ICommand::ptr onFalseCommand);
    ~ConditionalCommand() override = default;

  public:
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

private:
    ICommand::ptr onTrueCommand_;
    ICommand::ptr onFalseCommand_;
    std::function<bool()> condition_;
};
} // namespace robot
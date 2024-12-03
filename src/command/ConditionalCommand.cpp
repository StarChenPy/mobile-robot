/**
 * @file ConditionalCommand.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <utility>

#include "command/ConditionalCommand.h"
namespace robot {

    ConditionalCommand::ConditionalCommand(std::function<bool()> condition, ICommand::ptr onTrueCommand,
                                           ICommand::ptr onFalseCommand) {
        condition_ = std::move(condition);
        onTrueCommand_ = std::move(onTrueCommand);
        onFalseCommand_ = std::move(onFalseCommand);
    }

    void ConditionalCommand::initialize() {
        workCommand_ = condition_() ? onTrueCommand_ : onFalseCommand_;
        workCommand_->initialize();
    }

    void ConditionalCommand::execute() {
        workCommand_->execute();
    }

    void ConditionalCommand::end() {
        workCommand_->end();
    }

    bool ConditionalCommand::isFinished() {
        return workCommand_->isFinished();
    }

} // namespace robot
#include "command/ConditionalCommand.h"
#include <utility>

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

    ICommand::ptr ConditionalCommand::create(const std::function<bool()>& condition, const ICommand::ptr& onTrueCommand,
                                             const ICommand::ptr& onFalseCommand) {
        return std::make_shared<ConditionalCommand>(condition, onTrueCommand, onFalseCommand);
    }

} // namespace robot
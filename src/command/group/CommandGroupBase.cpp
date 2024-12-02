#include "command/group/CommandGroupBase.h"

namespace robot {
CommandGroupBase::CommandGroupBase() {
    commands_.clear();
    isGroup_ = true;
}

void CommandGroupBase::addCommand(const Command::ptr& command) {
    command->isFinished();
    auto it = std::find(commands_.begin(), commands_.end(), command);
    if (it == commands_.end()) {
        command->parent_ = getPtr();
        commands_.push_back(command);
    }
}

void CommandGroupBase::addCommand(const std::vector<Command::ptr>& commands) {
    for (auto & command : commands) {
        addCommand(command);
    }
}
}
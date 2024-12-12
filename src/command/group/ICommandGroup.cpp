#include "command/group/ICommandGroup.h"
#include <glog/logging.h>

namespace robot {
    ICommandGroup::ICommandGroup() {
        commands_.clear();
    }

    void ICommandGroup::addCommand(const ICommand::ptr& command) {
        auto it = std::find(commands_.begin(), commands_.end(), command);
        if (it == commands_.end()) {
            command->parent_ = getPtr();
            commands_.push_back(command);
        }
    }

    void ICommandGroup::addCommand(const std::vector<ICommand::ptr>& commands) {
        for (auto & command : commands) {
            addCommand(command);
        }
    }
}
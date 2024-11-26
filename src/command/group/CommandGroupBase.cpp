#include "command/group/CommandGroupBase.h"

namespace RobotGenius {
CommandGroupBase::CommandGroupBase() {
    m_commands_.clear();
    m_is_group_ = true;
}

void CommandGroupBase::AddCommands(Command::ptr command) {
    command->isFinished();
    auto it = std::find(m_commands_.begin(), m_commands_.end(), command);
    if (it == m_commands_.end()) {
        command->m_parent = getPtr();
        m_commands_.push_back(command);
    }
}

void CommandGroupBase::AddCommands(std::vector<Command::ptr> commands) {
    for (auto iter = commands.begin(); iter != commands.end(); iter++) {
        AddCommands(*iter);
    }
}
} // namespace RobotGenius
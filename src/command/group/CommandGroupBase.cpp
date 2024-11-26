#include "command/group/CommandGroupBase.h"

namespace RobotGenius {
CommandGroupBase::CommandGroupBase() {
  m_commands_.clear();
  m_is_group_ = true;
}

void CommandGroupBase::AddCommands(Command::Ptr command) {
    // std::cout << command.get() << std::endl;
    command->isFinished();  
    auto it = std::find(m_commands_.begin(), m_commands_.end(), command);
    // std::cout << "AddCommands Check" << std::endl;
    if (it == m_commands_.end()) {
      command->m_parent = getPtr();
      m_commands_.push_back(command);
      // std::cout << "AddCommands push" << std::endl;
    }
  }

  void CommandGroupBase::AddCommands(
      std::vector<Command::Ptr> commands) {
      for (auto iter = commands.begin(); iter != commands.end(); iter++) {
        AddCommands(*iter);
      }
    }


}
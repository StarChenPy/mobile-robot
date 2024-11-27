/**
 * @file CommandGroupBase.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 命令组的基类
 * @version 0.1
 * @date 2024-04-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "command/Command.h"
#include <iostream>
#include <memory>
#include <vector>
namespace robot {

class CommandGroupBase : public Command {
  public:
    typedef std::shared_ptr<CommandGroupBase> ptr;

  public:
    CommandGroupBase();
    template <typename... Rest> void AddCommands(Command::ptr first, Rest... rest) {
        addCommand(first);   // 添加第一个命令
        addCommand(rest...); // 递归调用展开剩余参数
    }
    void addCommands() {}
    void addCommands(const std::vector<Command::ptr>& commands);
    void addCommand(const Command::ptr& command);
    void addEndCommand(Command::ptr command) { m_next_command_ = command; }
  protected:
    std::vector<Command::ptr> commands_;
    Command::ptr m_next_command_;
};
} // namespace robot
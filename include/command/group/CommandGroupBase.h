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
    template <typename... Rest> void addCommand(Command::ptr first, Rest... rest) {
        addCommand(first);   // 添加第一个命令
        addCommand(rest...); // 递归调用展开剩余参数
    }
    void addCommand() {}
    void addCommand(const std::vector<Command::ptr>& commands);
    void addCommand(const Command::ptr& command);
    void addEndCommand(Command::ptr command) { nextCommand_ = command; }
  protected:
    std::vector<Command::ptr> commands_;
    Command::ptr nextCommand_;
};
} // namespace robot
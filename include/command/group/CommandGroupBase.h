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
#include <memory>
#include <vector>
#include "command/Command.h"
#include <iostream>
namespace RobotGenius {

 
class CommandGroupBase : public Command {
 public:
  typedef std::shared_ptr<CommandGroupBase> Ptr;
 public:
  CommandGroupBase();
  template<typename... Rest>
  void AddCommands(Command::Ptr first, Rest... rest) {
      AddCommands(first); // 添加第一个命令
      AddCommands(rest...); // 递归调用展开剩余参数
  }
  void AddCommands() {}
  void AddCommands(std::vector<Command::Ptr> commands);
  void AddCommands(Command::Ptr command);
  void AddEndCommand(Command::Ptr command) {
    m_next_command_ = command;
  }

  /**
   * @brief 将命令送进调度器队列
   * 
   * @return true 
   * @return false 
   */
  // bool schedule() override final;
 protected:
  std::vector<Command::Ptr> m_commands_;
  Command::Ptr m_next_command_;
};
}  // namespace RobotGenius
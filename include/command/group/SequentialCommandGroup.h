/**
 * @file SequentialCommandGroup.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 顺序结构，命令组按照顺序运行命令
 * @version 0.1
 * @date 2024-04-14
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "CommandGroupBase.h"
#include <memory>
#include <vector>

namespace robot {

class SequentialCommandGroup : public CommandGroupBase {
  public:
    typedef std::shared_ptr<SequentialCommandGroup> Ptr;
    SequentialCommandGroup() {}
    template <class... Types> explicit SequentialCommandGroup(Types... commands) {
        (commands.push_back(commands), ...);
    }
    virtual ~SequentialCommandGroup() {}

  public:
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;
    Command::ptr reset() override;

  protected:
    Command::ptr m_current_command_;
    uint64_t m_current_command_index_ = 0;
    bool m_is_finished_ = false;
};
std::shared_ptr<SequentialCommandGroup> createSequentialCommandGroup();

} // namespace robot

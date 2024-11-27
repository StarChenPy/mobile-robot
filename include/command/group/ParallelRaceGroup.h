/**
 * @file ParallelRaceGroup.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 任意的一条命令结束，整个命令组都结束
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "CommandGroupBase.h"
#include <memory>
#include <vector>

namespace robot {
class ParallelRaceGroup : public CommandGroupBase {
  public:
    typedef std::shared_ptr<ParallelRaceGroup> ptr;
    template <class... Types> explicit ParallelRaceGroup(Types... commands) { (commands.push_back(commands), ...); }
    ParallelRaceGroup();
    ~ParallelRaceGroup() = default;
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;
    Command::ptr reset() override;

  public:
    uint64_t stop_command_index = 0;
};

ParallelRaceGroup::ptr createParallelRaceGroup();

} // namespace robot

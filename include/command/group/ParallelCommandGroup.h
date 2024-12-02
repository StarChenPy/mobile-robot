/**
 * @file ParallelCommandGroup.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 并行执行任务
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "CommandGroupBase.h"
#include "util/Scheduler.h"
#include <memory>
#include <vector>

namespace robot {

class ParallelCommandGroup : public CommandGroupBase {
  public:
    typedef std::shared_ptr<ParallelCommandGroup> Ptr;

    ParallelCommandGroup();
    template <class... Types> explicit ParallelCommandGroup(Types... commands) {
        (commands.push_back(commands), ...);
    }
    ~ParallelCommandGroup() {};

  public:
    void initialize();
    void execute();
    void end();
    bool isFinished();
    Command::ptr reset();
};

ParallelCommandGroup::Ptr createParallelCommandGroup();

} // namespace robot
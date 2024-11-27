/**
 * @file ParallelDeadlineGroup.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 指定的命令结束则结束整个命令组，所有命令为并行执行
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "CommandGroupBase.h"
#include "command/Scheduler.h"
#include <memory>
#include <vector>

namespace robot {

class ParallelDeadlineGroup : public CommandGroupBase {
  public:
    typedef std::shared_ptr<ParallelDeadlineGroup> ptr;
    ParallelDeadlineGroup();
    template <class... Types> explicit ParallelDeadlineGroup(Types... commands) {
        (commands_.push_back(commands), ...);
        m_deadline_command_ = *commands_.begin();
        commands_.erase(commands_.begin());
    }
    virtual ~ParallelDeadlineGroup() {}

  public:
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;
    void disableScheduleDeadlineCommand();
    void enableScheduleDeadlineCommand();
    void setDeadlineCommand(Command::ptr command, bool schedule = true);
    Command::ptr reset() override;

  protected:
    Command::ptr m_deadline_command_;
    bool isScheduleDeadlineCommand_ = true;
};

ParallelDeadlineGroup::ptr createParallelDeadlineGroup();

} // namespace robot
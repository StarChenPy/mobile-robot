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
#include "ICommandGroup.h"
#include "system/Scheduler.h"
#include <memory>
#include <vector>

namespace robot {

class ParallelDeadlineGroup : public ICommandGroup {
  public:
    typedef std::shared_ptr<ParallelDeadlineGroup> ptr;
    /**
     * 并行执行命令组
     * 在指定命令结束后结束
     */
    ParallelDeadlineGroup();
    ~ParallelDeadlineGroup() override = default;

  public:
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

    void disableScheduleDeadlineCommand();
    void enableScheduleDeadlineCommand();
    void setDeadlineCommand(ICommand::ptr command, bool schedule = true);

    static ParallelDeadlineGroup::ptr create();
  protected:
    ICommand::ptr deadlineCommand_;
    bool isScheduleDeadlineCommand_ = true;
};

} // namespace robot
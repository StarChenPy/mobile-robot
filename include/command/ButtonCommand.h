/**
 * @file ScramCommand.h
 * @author Longping.Huang
 * @brief
 * @version 0.1
 * @date 2024-09-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "system/Robot.h"
#include "util/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

using namespace std;
using namespace robot;
using namespace VMX;

class EStopCommand : public CommandBase {
public:
    typedef std::shared_ptr<EStopCommand> ptr;
    EStopCommand() {}
    ~EStopCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

private:
    bool is_finished = false;
};
Command::ptr createEStopCommand();

class StartCommand : public CommandBase {
public:
    typedef std::shared_ptr<StartCommand> Ptr;
    StartCommand() {}
    ~StartCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

private:
    bool is_finished = false;
    int64_t m_last_time;
};
Command::ptr createStartCommand();
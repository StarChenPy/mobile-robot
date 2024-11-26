/**
 * @file LidarAlongWallCommand.h
 * @author Zijian.Yan (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "RobotCfg.h"
#include "RobotGenius.h"
#include "params.h"
#include "system/Robot.h"

// #include "command/LidarReadCommand.h"
using namespace std;
using namespace robot;

class AlongRightWallCommand : public CommandBase {
  public:
    typedef std::shared_ptr<AlongRightWallCommand> Ptr;
    AlongRightWallCommand(double v, double d) : speed(v), calib_d(d) {}
    ~AlongRightWallCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;
    double calib_d = 30;
    double speed = 5;
};

class AlongLeftWallCommand : public CommandBase {
  public:
    typedef std::shared_ptr<AlongLeftWallCommand> Ptr;
    AlongLeftWallCommand(double v, double d) : speed(v), calib_d(d) {}
    ~AlongLeftWallCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;
    double calib_d = 30;
    double speed = 5;
};

class isReachXCommand : public CommandBase {
  public:
    typedef std::shared_ptr<isReachXCommand> Ptr;
    isReachXCommand(Pose pose) : EndPose(pose) {}
    ~isReachXCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    uint8_t m_sleep_time;
    int64_t m_last_time;

    Pose EndPose;
    double d_min = 4;
};

Command::ptr createLidarAlongRightWallCommand(double speed, double d);
Command::ptr LidarReadAlongRightWallCommandDG(double speed, double d);

Command::ptr createLidarAlongLeftWallCommand(double speed, double d);
Command::ptr LidarReadAlongLeftWallCommandDG(double speed, double d);

Command::ptr MoveAlongRightWallRG(Pose pose, double d_wall);
Command::ptr MoveAlongLeftWallRG(Pose pose, double d_wall);
Command::ptr MoveAlongRightWallRG(Pose pose, double d_wall, double speed);
Command::ptr MoveAlongLeftWallRG(Pose pose, double d_wall, double speed);
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
#include "RobotGenius.h"
#include "RobotCfg.h"
#include "params.h"
#include "system/Robot.h"

// #include "command/LidarReadCommand.h"
using namespace std;
using namespace RobotGenius;

class AlongRightWallCommand : public CommandBase {
 public:
  typedef std::shared_ptr<AlongRightWallCommand> Ptr;
  AlongRightWallCommand(double v, double d) : speed(v), calib_d(d){}
  ~AlongRightWallCommand() {}

  void initialize() override ;
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
  AlongLeftWallCommand(double v, double d) : speed(v), calib_d(d){}
  ~AlongLeftWallCommand() {}

  void initialize() override ;
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
  isReachXCommand(Pose pose) : EndPose(pose){}
  ~isReachXCommand() {}

  void initialize() override ;
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


Command::Ptr createLidarAlongRightWallCommand(double speed, double d);
Command::Ptr LidarReadAlongRightWallCommandDG(double speed, double d);

Command::Ptr createLidarAlongLeftWallCommand(double speed, double d);
Command::Ptr LidarReadAlongLeftWallCommandDG(double speed, double d);

Command::Ptr MoveAlongRightWallRG(Pose pose, double d_wall);
Command::Ptr MoveAlongLeftWallRG(Pose pose, double d_wall);
Command::Ptr MoveAlongRightWallRG(Pose pose, double d_wall, double speed);
Command::Ptr MoveAlongLeftWallRG(Pose pose, double d_wall, double speed);
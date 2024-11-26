/**
 * @file LidarCalibCommand.h
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

class LidarCalibCommand : public CommandBase {
 public:
  typedef std::shared_ptr<LidarCalibCommand> Ptr;
  LidarCalibCommand(double d) : calib_d(d){}
  ~LidarCalibCommand() {}

  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
  
 private:
  bool is_finished = false;
  int64_t m_last_time;
  double calib_d = 30;

};

// Command::ptr createLidarCalibCommand(double d){
//   return std::make_shared<LidarCalibCommand>(d)->withTimer(100);
// }

Command::Ptr createLidarCalibCommand(double d);
Command::Ptr LidarReadCalibDG(double d);
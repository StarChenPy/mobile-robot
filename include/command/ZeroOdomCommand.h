/**
 * @file ZeroOdomCommand.h
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
using namespace std;
using namespace RobotGenius;

class ZeroOdomCommand : public CommandBase {
 public:
  typedef std::shared_ptr<ZeroOdomCommand> Ptr;
  ZeroOdomCommand() {}
  ~ZeroOdomCommand() {}

  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
  
 private:
  bool is_finished = false;
  int64_t m_last_time;

};
Command::Ptr createZeroOdomCommand();

class SetOdomCommand : public CommandBase {
 public:
  typedef std::shared_ptr<SetOdomCommand> Ptr;
  SetOdomCommand(double x, double y, double theta) : set_x(x), set_y(y), set_theta(theta){}
  ~SetOdomCommand() {}

  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
  
 private:
  bool is_finished = false;
  int64_t m_last_time;

  double set_x = 0;
  double set_y = 0;
  double set_theta = 0;

};
Command::Ptr createSetOdomCommand(double x, double y, double theta);
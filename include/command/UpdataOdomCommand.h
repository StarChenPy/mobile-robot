/**
 * @file UpdataOdomCommand.h
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

class UpdataOdomCommand : public CommandBase {
 public:
  typedef std::shared_ptr<UpdataOdomCommand> Ptr;
  UpdataOdomCommand() {}
  ~UpdataOdomCommand() {}

  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
  
 private:
  bool is_finished = false;
  int64_t m_last_time;

};

Command::Ptr createUpdataOdomCommand();
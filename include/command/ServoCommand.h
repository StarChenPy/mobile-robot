/**
 * @file ServoCommand.h
 * @author Zijian.Yan (jiapeng.lin@high-genius.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-12
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


class ClampServoCommand : public CommandBase {
 public:
  typedef std::shared_ptr<ClampServoCommand> Ptr;
  ClampServoCommand(double val) : ClampServo_val(val) {}
  ClampServoCommand(double val, double cnt_limit) : ClampServo_val(val),counter_limit(cnt_limit) {}
  ~ClampServoCommand() {}

  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
  
 private:
  bool is_finished = false;
  int64_t m_last_time;

  int counter = 0;
  int counter_limit = 10;
  double ClampServo_val = CLAMP_SERVO_MIN;
  
};

// 指令封装  len:打开宽度，单位cm
Command::Ptr createClampServoCommand(double len);
Command::Ptr createClampServoCommand(double len, double cnt_limit);


class TelescopicServoCommand : public CommandBase {
 public:
  typedef std::shared_ptr<TelescopicServoCommand> Ptr;
  TelescopicServoCommand(double val) : TelescopicServo_val(val) {}
  TelescopicServoCommand(double val, double cnt_limit) : TelescopicServo_val(val),counter_limit(cnt_limit) {}
  ~TelescopicServoCommand() {}

  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
  
 private:
  bool is_finished = false;
  int64_t m_last_time;

  int counter = 0;
  int counter_limit = 10;
  double TelescopicServo_val = TELESCOPIC_SERVO_MIN;

};

// 指令封装  dis:伸缩距离，单位cm
Command::Ptr createcTelescopicServoCommand(double dis);
Command::Ptr createcTelescopicServoCommand(double dis, double cnt_limit);



class RaiseServoCommand : public CommandBase {
 public:
  typedef std::shared_ptr<RaiseServoCommand> Ptr;
  RaiseServoCommand(double val) : RaiseServo_val(val) {}
  RaiseServoCommand(double val, double cnt_limit) : RaiseServo_val(val),counter_limit(cnt_limit)  {}
  ~RaiseServoCommand() {}

  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
  
 private:
  bool is_finished = false;
  int64_t m_last_time;

  int counter = 0;
  int counter_limit = 10;
  double RaiseServo_val = RAISE_SERVO_MIN;
  
};

// 指令封装  angle:抬起角度，单位：度
Command::Ptr createRaiseServoCommand(double angle);
Command::Ptr createRaiseServoCommand(double angle, double cnt_limit);


class RotatingServoCommand : public CommandBase {
 public:
  typedef std::shared_ptr<RotatingServoCommand> Ptr;
  RotatingServoCommand(double val) : RotatingServo_val(val) {}
  RotatingServoCommand(double val, double cnt_limit) : RotatingServo_val(val),counter_limit(cnt_limit)  {}
  ~RotatingServoCommand() {}

  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
  
 private:
  bool is_finished = false;
  int64_t m_last_time;

  int counter = 0;
  int counter_limit = 10;
  double RotatingServo_val = RAISE_SERVO_MIN;
  
};

// 指令封装  angle:角度，单位：度
Command::Ptr createRotatingServoCommand(double angle);
Command::Ptr createRotatingServoCommand(double angle, double cnt_limit);
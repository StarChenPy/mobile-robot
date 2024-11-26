/**
 * @file MotorPIDCommand.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-05
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


class LeftMotorPIDCommand : public CommandBase {
 public:
  typedef std::shared_ptr<LeftMotorPIDCommand> Ptr;
  LeftMotorPIDCommand();
  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
 private:
  bool is_finished = false;
  int64_t m_last_time;

};

class RightMotorPIDCommand : public CommandBase {
 public:
  typedef std::shared_ptr<RightMotorPIDCommand> Ptr;
  RightMotorPIDCommand();
  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
 private:
  bool is_finished = false;
  int64_t m_last_time;

};

class TurnMotorPIDCommand : public CommandBase {
 public:
  typedef std::shared_ptr<TurnMotorPIDCommand> Ptr;
  TurnMotorPIDCommand();
  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
 private:
  bool is_finished = false;
  int64_t m_last_time;

};

class LiftMotorPIDCommand : public CommandBase {
 public:
  typedef std::shared_ptr<LiftMotorPIDCommand> Ptr;
  LiftMotorPIDCommand();
  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
 private:
  bool is_finished = false;
  int64_t m_last_time;

};

Command::Ptr createLeftMotorPIDCommand();
Command::Ptr createRightMotorPIDCommand();
Command::Ptr createTurnMotorPIDCommand();
Command::Ptr createLiftMotorPIDCommand();


class LeftMotorCommand : public CommandBase {
 public:
  typedef std::shared_ptr<LeftMotorCommand> Ptr;
  LeftMotorCommand();
  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
 private:
  bool is_finished = false;
  int64_t m_last_time;

};

class RightMotorCommand : public CommandBase {
 public:
  typedef std::shared_ptr<RightMotorCommand> Ptr;
  RightMotorCommand();
  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
 private:
  bool is_finished = false;
  int64_t m_last_time;

};

class SetRightMotorSpeedCommand : public CommandBase {
 public:
  typedef std::shared_ptr<SetRightMotorSpeedCommand> Ptr;
  SetRightMotorSpeedCommand(double speed = 10, uint32_t counter = 5) : m_speed(speed), m_counter(counter){

  }
  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
 private:
  bool is_finished = false;
  int64_t m_last_time;
  double m_speed;
  uint32_t m_counter = 0;
  uint32_t m_current_counter = 0;

};
class SetLeftMotorSpeedCommand : public CommandBase {
 public:
  typedef std::shared_ptr<SetLeftMotorSpeedCommand> Ptr;
  SetLeftMotorSpeedCommand(double speed = 10, uint32_t counter = 5) : m_speed(speed), m_counter(counter){

  }
  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
 private:
  bool is_finished = false;
  int64_t m_last_time;
  double m_speed;
  uint32_t m_counter = 0;
  uint32_t m_current_counter = 0;
};


class TurnMotorCommand : public CommandBase {
 public:
  typedef std::shared_ptr<TurnMotorCommand> Ptr;
  TurnMotorCommand();
  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
 private:
  bool is_finished = false;
  int64_t m_last_time;

};

class LiftMotorCommand : public CommandBase {
 public:
  typedef std::shared_ptr<LiftMotorCommand> Ptr;
  LiftMotorCommand();
  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;
 private:
  bool is_finished = false;
  int64_t m_last_time;

};


class LiftMotorDistancePIDCommand : public CommandBase {
 public:
  typedef std::shared_ptr<LiftMotorDistancePIDCommand> Ptr;
  LiftMotorDistancePIDCommand(int32_t setpoint) : m_setpoint(setpoint){}
  LiftMotorDistancePIDCommand(int32_t setpoint, double e_enc, uint8_t cnt) : m_setpoint(setpoint), E_ENC(e_enc), CNT(cnt){}
  void initialize() override  ;
  void execute() override ;
  void end() override ;
  bool isFinished() override;

 private:
  bool is_finished = false;
  int32_t m_setpoint = 0;
  int32_t m_conter = 0;
  double E_ENC = LIFTMOTORDISTANCEERROR;
  uint8_t CNT = LIFTMOTORDISTANCECounter;
};
Command::Ptr LiftDistancePIDCommand(double h);
Command::Ptr LiftDistancePIDCommand(double h, double e_enc = LIFTMOTORDISTANCEERROR, uint8_t cnt = LIFTMOTORDISTANCECounter);

class ResetLiftMotorDistancePIDCommand : public CommandBase {
 public:
  typedef std::shared_ptr<ResetLiftMotorDistancePIDCommand> Ptr;
  ResetLiftMotorDistancePIDCommand(int32_t setpoint = 10) : m_setpoint(setpoint){

  }
  void initialize() override  ;
  void execute() override ;
  void end() override ;
  bool isFinished() override;

 private:
  bool is_finished = false;
  int32_t m_setpoint = 0;
  int32_t m_conter = 10;
  int32_t m_current_counter = 0;
};
Command::Ptr ResetLiftMotorDistance(int32_t speed);



class ResetLiftMotorDistancePIDCAssistance : public CommandBase {
 public:
  typedef std::shared_ptr<ResetLiftMotorDistancePIDCAssistance> Ptr;
  ResetLiftMotorDistancePIDCAssistance(double speed = 5, int32_t counter = 1) : m_speed(10),m_conter(counter){

  }
  void initialize() override  ;
  void execute() override ;
  void end() override ;
  bool isFinished() override;

 private:
  bool is_finished = false;
  int32_t m_conter = 0;
  double m_speed = 0;
  int32_t m_current_counter = 0;
};
Command::Ptr LiftMotorDistancePIDCommandAssistance(int32_t setpoint);





class TurnMotorDistancePIDCommand : public CommandBase {
 public:
  typedef std::shared_ptr<TurnMotorDistancePIDCommand> Ptr;
  TurnMotorDistancePIDCommand(int32_t setpoint) : m_setpoint(setpoint){}
  TurnMotorDistancePIDCommand(int32_t setpoint, double e_enc, uint8_t cnt) : m_setpoint(setpoint), E_ENC(e_enc), CNT(cnt){}
  void initialize() override  ;
  void execute() override ;
  void end() override ;
  bool isFinished() override;

 private:
  bool is_finished = false;
  int32_t m_setpoint = 0;
  int32_t m_conter = 0;
  double E_ENC = TURNMOTORDISTANCEERROR;
  uint8_t CNT = TURNMOTORDISTANCECounter;
};
Command::Ptr TurnAnglePIDCommand(double angle);
Command::Ptr TurnAnglePIDCommand(double angle, double e_enc = TURNMOTORDISTANCEERROR, uint8_t cnt = TURNMOTORDISTANCECounter);

Command::Ptr TurnMotorDistancePIDCommandAssistance(int32_t setpoint);

class ResetTurnMotorDistancePIDCommand : public CommandBase {
 public:
  typedef std::shared_ptr<ResetTurnMotorDistancePIDCommand> Ptr;
  ResetTurnMotorDistancePIDCommand(int32_t setpoint = 10) : m_setpoint(setpoint){

  }
  void initialize() override  ;
  void execute() override ;
  void end() override ;
  bool isFinished() override;

 private:
  bool is_finished = false;
  int32_t m_setpoint = 0;
  int32_t m_conter = 10;
  int32_t m_current_counter = 0;
};
Command::Ptr ResetTurnMotorAngle(int32_t speed);


Command::Ptr ResetTurnMotorDistance(int32_t setpoint = 5);


class ResetTurnMotorDistancePIDommandAssistance : public CommandBase {
 public:
  typedef std::shared_ptr<ResetTurnMotorDistancePIDommandAssistance> Ptr;
  ResetTurnMotorDistancePIDommandAssistance(double speed = 0, int32_t counter = 10) : m_speed(speed),m_conter(counter){

  }
  void initialize() override  ;
  void execute() override ;
  void end() override ;
  bool isFinished() override;

 private:
  bool is_finished = false;
  int32_t m_conter = 0;
  double m_speed = 0;
  int32_t m_current_counter = 0;
};



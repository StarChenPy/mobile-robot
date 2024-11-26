#include "command/MotorPIDCommand.h"
#include "RobotCfg.h"
#include "params.h"
#include "command/ENCComand.h"


LeftMotorPIDCommand::LeftMotorPIDCommand(){

}
void LeftMotorPIDCommand::initialize() {
  is_finished = false;
}
void LeftMotorPIDCommand::execute() {
  uint8_t command_status;
  LABVIEW::TestIOStatusShareAddress->read(command_status);
  if(command_status != COMMEND_WAIT){
    Robot::GetInstance().setLeftMotorProcess(LeftENC->read());
    Robot::GetInstance().controlLeftMotor();
    Robot::GetInstance().setLeftMotorLastENCCounter(LeftENC->get());
  }
}
void LeftMotorPIDCommand::end() {
  // LeftMotor->setSpeedAndDir(0, false, true);
  Robot::GetInstance().setLeftMotorSpeed(0);
  std::cout << "LeftMotorPIDCommand end" << std::endl;
}
bool LeftMotorPIDCommand::isFinished() {
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}

RightMotorPIDCommand::RightMotorPIDCommand(){

}

void RightMotorPIDCommand::initialize() {
  is_finished = false;
}

void RightMotorPIDCommand::execute() {
  uint8_t command_status;
  LABVIEW::TestIOStatusShareAddress->read(command_status);
  if(command_status != COMMEND_WAIT){
    Robot::GetInstance().setRightMotorProcess(RightENC->read());
    Robot::GetInstance().controlRightMotor();
    Robot::GetInstance().setRightMotorLastENCCounter(RightENC->get());
  }
}

void RightMotorPIDCommand::end() {
  // RightMotor->setSpeedAndDir(0, false, true);
  Robot::GetInstance().setRightMotorSpeed(0);
  std::cout << "RightMotorPIDCommand end" << std::endl;
}

bool RightMotorPIDCommand::isFinished() {
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}


TurnMotorPIDCommand::TurnMotorPIDCommand(){

}

void TurnMotorPIDCommand::initialize() {
  is_finished = false;
}

void TurnMotorPIDCommand::execute() {
  uint8_t command_status;
  LABVIEW::TestIOStatusShareAddress->read(command_status);
  if(command_status != COMMEND_WAIT){
    Robot::GetInstance().setTurnMotorProcess(TurnENC->read());
    Robot::GetInstance().controlTurnMotor();
    Robot::GetInstance().setTurnMotorLastENCCounter(TurnENC->get());
  }
}

void TurnMotorPIDCommand::end() {
  // TurnMotor->setSpeedAndDir(0, false, true);
  Robot::GetInstance().setTurnMotorSpeed(0);
  std::cout << "TurnMotorPIDCommand end" << std::endl;
  return;
}

bool TurnMotorPIDCommand::isFinished() {
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}



LiftMotorPIDCommand::LiftMotorPIDCommand(){

}

void LiftMotorPIDCommand::initialize() {
  is_finished = false;
}

void LiftMotorPIDCommand::execute() {
  uint8_t command_status;
  LABVIEW::TestIOStatusShareAddress->read(command_status);
  if(command_status != COMMEND_WAIT){
    Robot::GetInstance().setLiftMotorProcess(LiftENC->read());
    Robot::GetInstance().controlLiftMotor();
    Robot::GetInstance().setLiftMotorLastENCCounter(LiftENC->get());
  }
}

void LiftMotorPIDCommand::end() {
  // LiftMotor->setSpeedAndDir(0, false, true);
  Robot::GetInstance().setLiftMotorSpeed(0);
  std::cout << "LiftMotorPIDCommand end" << std::endl;
  return;
}

bool LiftMotorPIDCommand::isFinished() {
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}

Command::Ptr createLeftMotorPIDCommand(){
  return std::make_shared<LeftMotorPIDCommand>()->withTimer(20);
}
Command::Ptr createRightMotorPIDCommand(){
  return std::make_shared<RightMotorPIDCommand>()->withTimer(20);
}
Command::Ptr createTurnMotorPIDCommand(){
  return std::make_shared<TurnMotorPIDCommand>()->withTimer(20);
}
Command::Ptr createLiftMotorPIDCommand(){
  return std::make_shared<LiftMotorPIDCommand>()->withTimer(20);
}



LeftMotorCommand::LeftMotorCommand(){

}

void LeftMotorCommand::initialize() {
  is_finished = false;
}

void LeftMotorCommand::execute() {
    Robot::GetInstance().setLeftMotorSpeedWithoutPID();
}

void LeftMotorCommand::end() {
  return;
}

bool LeftMotorCommand::isFinished() {
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}


RightMotorCommand::RightMotorCommand(){

}

void RightMotorCommand::initialize() {
  is_finished = false;
}

void RightMotorCommand::execute() {
    Robot::GetInstance().setRightMotorSpeedWithoutPID();
}

void RightMotorCommand::end() {
  return;
}

bool RightMotorCommand::isFinished() {
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}

TurnMotorCommand::TurnMotorCommand() {

}

void TurnMotorCommand::initialize() {
  is_finished = false;
}

void TurnMotorCommand::execute() {
    Robot::GetInstance().setTurnMotorSpeedWithoutPID();
}


void TurnMotorCommand::end() {
  return;
}

bool TurnMotorCommand::isFinished() {
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}

LiftMotorCommand::LiftMotorCommand() {

}

void LiftMotorCommand::initialize() {
  is_finished = false;
}

void LiftMotorCommand::execute() {
    Robot::GetInstance().setLiftMotorSpeedWithoutPID();
}

void LiftMotorCommand::end() {
  // std::cout << "LiftMotorCommand end" << std::endl;
}

bool LiftMotorCommand::isFinished() {
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}




void LiftMotorDistancePIDCommand::initialize()   {
  uint8_t updata_status = COMMEND_WAIT;
  LABVIEW::LiftDistanceStatusShareAddress->write(updata_status);
  // Robot::GetInstance().resetLiftMotorPID();
  // Robot::GetInstance().reset_lift_enc();
}
void LiftMotorDistancePIDCommand::execute()  {
  Robot::GetInstance().LiftMotorDistancePID(m_setpoint);
  std::cout << "Lift ENC: " << LiftENC->get() << " Lift set_point_: " << m_setpoint << std::endl;
}
void LiftMotorDistancePIDCommand::end()  {
  std::cout << "LiftMotorDistancePIDCommand end!" <<std::endl;
  Robot::GetInstance().setLiftMotorSpeed(0);

  uint8_t updata_status = COMMEND_END;
  LABVIEW::LiftDistanceStatusShareAddress->write(updata_status);
}
bool LiftMotorDistancePIDCommand::isFinished()  {
  uint8_t command_status;
  LABVIEW::LiftDistanceStatusShareAddress->read(command_status);
  if(command_status == COMMEND_CANCEL){
    is_finished = true;
  }
  if (abs(m_setpoint - LiftENC->get()) < E_ENC) {
    m_conter ++;
  } else {
    m_conter = 0;
  }
  return Robot::GetInstance().getStopSignal() || m_conter > CNT || is_finished;
}

//标定完指令 参数：升降高度 单位：cm
Command::Ptr LiftDistancePIDCommand(double h) {
  if(h > 0){
    h = 0;
    std::cout << "升降电机输入为正数" << std::endl;
  }
  int32_t setpoint = static_cast<int32_t>(h * (1000 / 10.7 / 2));
  LiftMotorDistancePIDCommand::Ptr Lift_Motor_Distance_PID_Command = std::make_shared<LiftMotorDistancePIDCommand>(setpoint);
  return Lift_Motor_Distance_PID_Command->withTimer(100);
}
Command::Ptr LiftDistancePIDCommand(double h, double e_enc, uint8_t cnt) {
  if(h > 0){
    h = 0;
    std::cout << "升降电机输入为正数" << std::endl;
  }
  int32_t setpoint = static_cast<int32_t>(h * (1000 / 10.7 / 2));
  LiftMotorDistancePIDCommand::Ptr Lift_Motor_Distance_PID_Command = std::make_shared<LiftMotorDistancePIDCommand>(setpoint, e_enc, cnt);
  return Lift_Motor_Distance_PID_Command->withTimer(100);
}



void ResetLiftMotorDistancePIDCommand::initialize()   {
  uint8_t updata_status = COMMEND_WAIT;
  LABVIEW::ResetLiftStatusShareAddress->write(updata_status);
  // Robot::GetInstance().resetLiftMotorPID();
  // Robot::GetInstance().reset_lift_enc();
}
void ResetLiftMotorDistancePIDCommand::execute()  {
  Robot::GetInstance().setLiftMotorSpeed(m_setpoint);
  std::cout <<  "LiftLimit->read():" << LiftLimit->read() <<std::endl;
  static bool finished = false;
  if (!finished) {
    // std::cout << "TurningLimit->read() " << TurningLimit->read() << std::endl;
    finished = !LiftLimit->read();
  }
  
  if (finished) {
    m_current_counter ++;
    m_setpoint = 0;
  } else {
    m_current_counter = 0;
  }
  is_finished = m_current_counter > m_conter;
  if (is_finished) {
    finished = false;
  }
  // std::cout << "Lift ENC:" << LiftENC->get() << std::endl;
}
void ResetLiftMotorDistancePIDCommand::end()  {
    Robot::GetInstance().reset_lift_enc();
  Robot::GetInstance().resetLiftMotorPID();
  Robot::GetInstance().setLiftMotorSpeed(0);
  // LiftMotor->setSpeedAndDir(0, true, true);
  std::cout << "ResetLiftMotorDistancePIDCommand end!" <<std::endl;
  uint8_t updata_status = COMMEND_END;
  LABVIEW::ResetLiftStatusShareAddress->write(updata_status);
}
bool ResetLiftMotorDistancePIDCommand::isFinished()  {
  uint8_t command_status;
  LABVIEW::ResetLiftStatusShareAddress->read(command_status);
  if(command_status == COMMEND_CANCEL){
    is_finished = true;
  }
  cout << "LiftLimit: " << LiftLimit->read() << endl;
  if (!LiftLimit->read()) {
    // LiftMotor->setSpeedAndDir(0, true, true);
    Robot::GetInstance().setLiftMotorSpeed(0);
  }
  return Robot::GetInstance().getStopSignal() || is_finished;
}

Command::Ptr ResetLiftMotorDistance(int32_t speed) {
  ResetLiftMotorDistancePIDCommand::Ptr reset_lift_command = std::make_shared<ResetLiftMotorDistancePIDCommand>(speed);
  return reset_lift_command->withTimer(100);
}



void ResetLiftMotorDistancePIDCAssistance::initialize() {
  m_current_counter = 0;
}
void ResetLiftMotorDistancePIDCAssistance::execute() {
  m_current_counter ++;
  Robot::GetInstance().setLiftMotorSpeedWithoutPID(m_speed);
}
void ResetLiftMotorDistancePIDCAssistance::end() {
    Robot::GetInstance().reset_lift_enc();
  Robot::GetInstance().resetLiftMotorPID();
  Robot::GetInstance().setLiftMotorSpeedWithoutPID(0);
}
bool ResetLiftMotorDistancePIDCAssistance::isFinished() {
  return Robot::GetInstance().getStopSignal() || m_current_counter > m_conter;
}

Command::Ptr LiftMotorDistancePIDCommandAssistance(int32_t setpoint) {
  LiftMotorDistancePIDCommand::Ptr Lift_Motor_Distance_PID_Command = std::make_shared<LiftMotorDistancePIDCommand>(setpoint);
  return Lift_Motor_Distance_PID_Command->withTimer(100);
}





void TurnMotorDistancePIDCommand::initialize() {
  uint8_t updata_status = COMMEND_WAIT;
  LABVIEW::TurnAngleStatusShareAddress->write(updata_status);
  // Robot::GetInstance().setTurnMotorDistance(set_point_);
  // Robot::GetInstance().reset_turn_enc();
}
void TurnMotorDistancePIDCommand::execute() {
  std::cout << "Turn ENC: " << TurnENC->get() << " Turn set_point_: " << m_setpoint << std::endl;
  Robot::GetInstance().setTurnMotorDistance(m_setpoint);
}
void TurnMotorDistancePIDCommand::end() {
  Robot::GetInstance().setTurnMotorSpeed(0);
  std::cout << "TurnMotorDistancePIDCommand end!" <<std::endl;

  uint8_t updata_status = COMMEND_END;
  LABVIEW::TurnAngleStatusShareAddress->write(updata_status);
}
bool TurnMotorDistancePIDCommand::isFinished() {
  uint8_t command_status;
  LABVIEW::TurnAngleStatusShareAddress->read(command_status);
  if(command_status == COMMEND_CANCEL){
    is_finished = true;
  }
  if (abs(m_setpoint - TurnENC->get()) < E_ENC) {
    m_conter ++;
  } else {
    m_conter = 0;
  }
  return Robot::GetInstance().getStopSignal() || m_conter > CNT || is_finished;
}
//标定完指令 参数：旋转角度 单位：度
Command::Ptr TurnAnglePIDCommand(double angle) {
  if(angle > 270){
    angle = 0;
    std::cout << "旋转电机输入大于180°" << std::endl;
  }else if(angle < -270){
    angle = 0;
    std::cout << "旋转电机输入小于-180°" << std::endl;
  }
  int32_t setpoint = static_cast<int32_t>(angle * (1670.0 / 90.0) - 2);
  TurnMotorDistancePIDCommand::Ptr Turn_Motor_Distance_PID_Command = std::make_shared<TurnMotorDistancePIDCommand>(setpoint);
  return Turn_Motor_Distance_PID_Command->withTimer(100);
}
Command::Ptr TurnAnglePIDCommand(double angle, double e_enc, uint8_t cnt) {
  if(angle > 270){
    angle = 0;
    std::cout << "旋转电机输入大于180°" << std::endl;
  }else if(angle < -270){
    angle = 0;
    std::cout << "旋转电机输入小于-180°" << std::endl;
  }
  int32_t setpoint = static_cast<int32_t>(angle * (1670.0 / 90.0) - 2);
  TurnMotorDistancePIDCommand::Ptr Turn_Motor_Distance_PID_Command = std::make_shared<TurnMotorDistancePIDCommand>(setpoint, e_enc, cnt);
  return Turn_Motor_Distance_PID_Command->withTimer(100);
}


Command::Ptr TurnMotorDistancePIDCommandAssistance(int32_t setpoint) {
  TurnMotorDistancePIDCommand::Ptr Turn_Motor_Distance_PID_Command = std::make_shared<TurnMotorDistancePIDCommand>(setpoint);
  return Turn_Motor_Distance_PID_Command->withTimer(100);
}



void ResetTurnMotorDistancePIDCommand::initialize() {
  uint8_t updata_status = COMMEND_WAIT;
  LABVIEW::ResetTurnAngleStatusShareAddress->write(updata_status);
  // Robot::GetInstance().resetTurnMotorPID();
  // Robot::GetInstance().reset_turn_enc();
}
void ResetTurnMotorDistancePIDCommand::execute() {
  Robot::GetInstance().setTurnMotorSpeed(m_setpoint);
  static bool finished = false;
  if (!finished) {
    std::cout << "TurningLimit->read() " << TurningLimit->read() << std::endl;
    finished = TurningLimit->read() > 1.0;
  }
  
  if (finished) {
    m_current_counter ++;
    m_setpoint = 0;
  } else {
    m_current_counter = 0;
  }
  is_finished = m_current_counter > m_conter;
  if (is_finished) {
    finished = false;
  }
}
void ResetTurnMotorDistancePIDCommand::end() {
  Robot::GetInstance().setTurnMotorSpeed(0);
    Robot::GetInstance().reset_turn_enc();
  Robot::GetInstance().resetTurnMotorPID();
  std::cout << "ResetTurnMotorDistancePIDCommand end!" <<std::endl;
  
  uint8_t updata_status = COMMEND_END;
  LABVIEW::ResetTurnAngleStatusShareAddress->write(updata_status);
}
bool ResetTurnMotorDistancePIDCommand::isFinished() {
  uint8_t command_status;
  LABVIEW::ResetTurnAngleStatusShareAddress->read(command_status);
  if(command_status == COMMEND_CANCEL){
    is_finished = true;
  }
  return Robot::GetInstance().getStopSignal() || is_finished;
}
Command::Ptr ResetTurnMotorAngle(int32_t speed){
  return std::make_shared<ResetTurnMotorDistancePIDCommand>(speed)->withTimer(100);
}



void ResetTurnMotorDistancePIDommandAssistance::initialize() {
  m_current_counter = 0;
}
void ResetTurnMotorDistancePIDommandAssistance::execute() {
  m_current_counter ++;
  Robot::GetInstance().setTurnMotorSpeedWithoutPID(m_speed);
}

void ResetTurnMotorDistancePIDommandAssistance::end() {
    Robot::GetInstance().reset_turn_enc();
  Robot::GetInstance().resetTurnMotorPID();
  Robot::GetInstance().setTurnMotorSpeedWithoutPID(0);
}

bool ResetTurnMotorDistancePIDommandAssistance::isFinished() {
  return Robot::GetInstance().getStopSignal() || m_current_counter > m_conter;
}
Command::Ptr ResetTurnMotorDistance(int32_t setpoint) {
  ResetTurnMotorDistancePIDCommand::Ptr reset_turn_command = std::make_shared<ResetTurnMotorDistancePIDCommand>(setpoint);
  // readTurnENCCommand::ptr read_turn_enc_command = std::make_shared<readTurnENCCommand>();
  // TurnMotorPIDCommand::ptr turn_motor_pid_command = std::make_shared<TurnMotorPIDCommand>();
  // ParallelDeadlineGroup::ptr DG = std::make_shared<ParallelDeadlineGroup>();
  // DG->AddCommands(read_turn_enc_command->withTimer(20),turn_motor_pid_command->withTimer(20));
  // DG->setDeadlineCommand(reset_turn_command->withTimer(20));
  return reset_turn_command->withTimer(20);
}



void SetRightMotorSpeedCommand::initialize() {
  is_finished = false;
  m_current_counter = 0;
}

void SetRightMotorSpeedCommand::execute() {
  Robot::GetInstance().setRightMotorSpeed(m_speed);
  is_finished = m_current_counter > m_counter;
  m_current_counter ++;
}
void SetRightMotorSpeedCommand::end() {
  // Robot::GetInstance().setRightMotorSpeed(0);
}

bool SetRightMotorSpeedCommand::isFinished() {
  return Robot::GetInstance().getStopSignal() || is_finished;
}


void SetLeftMotorSpeedCommand::initialize() {
  is_finished = false;
  m_current_counter = 0;
}

void SetLeftMotorSpeedCommand::execute() {
  Robot::GetInstance().setLeftMotorSpeed(m_speed);
  is_finished = m_counter < m_current_counter;
  m_current_counter ++;
}

void SetLeftMotorSpeedCommand::end() {
}

bool SetLeftMotorSpeedCommand::isFinished() {
  return Robot::GetInstance().getStopSignal() || is_finished;
}



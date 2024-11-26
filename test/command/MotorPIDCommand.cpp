#include "command/MotorPIDCommand.h"
int main() {
  
  Scheduler::GetInstance(1, false).start();
  sleep(1); 
  LeftMotorPIDCommand::Ptr command0 = std::make_shared<LeftMotorPIDCommand>();
  RightMotorPIDCommand::Ptr command1 = std::make_shared<RightMotorPIDCommand>();
  // TurnMotorPIDCommand::Ptr command2 = std::make_shared<TurnMotorPIDCommand>();
  // LiftMotorPIDCommand::Ptr command3 = std::make_shared<LiftMotorPIDCommand>();
  
  command0->withTimer(20)->schedule();
  command1->withTimer(20)->schedule();
  // command2->withTimer(20)->schedule();
  // command3->withTimer(20)->schedule();
  Robot::GetInstance().setLeftMotorSpeed(10);
  Robot::GetInstance().setRightMotorSpeed(10);
  // Robot::GetInstance().setTurnMotorSpeed(-10);
  // Robot::GetInstance().setLiftMotorSpeed(SEEDLIFTPWM);
  // sleep(1); 
  sleep(3);
  Scheduler::GetInstance().stop();
  return 0;
}
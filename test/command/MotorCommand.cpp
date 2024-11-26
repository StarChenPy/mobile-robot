#include "command/MotorPIDCommand.h"


int main() {

  Scheduler::GetInstance(1, false).start();
  sleep(1); 
  LeftMotorCommand::Ptr command0 = std::make_shared<LeftMotorCommand>();
  RightMotorCommand::Ptr command1 = std::make_shared<RightMotorCommand>();
  // TurnMotorCommand::Ptr command2 = std::make_shared<TurnMotorCommand>();
  // LiftMotorCommand::Ptr command3 = std::make_shared<LiftMotorCommand>();
  Robot::GetInstance().setLeftMotorSpeedWithoutPID(10);
  Robot::GetInstance().setRightMotorSpeedWithoutPID(0);
  // Robot::GetInstance().setTurnMotorSpeedWithoutPID(0);
  // Robot::GetInstance().setLiftMotorSpeedWithoutPID(SEEDLIFTPWM);
  command0->schedule();
  command1->schedule();
  // command2->schedule();
  // command3->schedule();
  sleep(3);
  Scheduler::GetInstance().stop();

  return 0;
}
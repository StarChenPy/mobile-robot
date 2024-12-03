/**
 * @file testIRcal.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "command/sensor/CalCommand.h"
#include "command/MotorPIDCommand.h"
#include "command/sensor/ENCComand.h"


// int main() {
//   Scheduler::GetInstance(2, false).start();
//   // IRCalCommand::Ptr ir_cal = std::make_shared<IRCalCommand>(20, 0.5, 0.5);
//   LeftMotorPIDCommand::Ptr command0 = std::make_shared<LeftMotorPIDCommand>();
//   RightMotorPIDCommand::Ptr command1 = std::make_shared<RightMotorPIDCommand>();
//   // readLeftENCCommand::Ptr command21 = std::make_shared<readLeftENCCommand>();
//   // readRightENCCommand::Ptr command31 = std::make_shared<readRightENCCommand>();
//   // ir_cal->withTimer(20)->schedule();
//   command0->withTimer(20)->schedule();
//   command1->withTimer(20)->schedule();
//   // command21->withTimer(20)->schedule();
//   // command31->withTimer(20)->schedule();
//   IRCalCommandAssistance(21, 0.5, 0.8)->schedule();
//   sleep(3);
//   Scheduler::GetInstance().stop();
//   return 0;
// }



int main() {
  LeftMotorPIDCommand::Ptr left = std::make_shared<LeftMotorPIDCommand>();
  RightMotorPIDCommand::Ptr right = std::make_shared<RightMotorPIDCommand>();
  Scheduler::GetInstance(2, false).start();

  SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
  S->AddCommands(IRCalCommandAssistance(21, 0.5, 0.8));
  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  DG->AddCommands(
    left->withTimer(20),
    right->withTimer(20)
  );
  DG->setDeadlineCommand(S);
  DG->schedule();
  
  sleep(1);
  Scheduler::GetInstance().stop();
  return 0;
}
 
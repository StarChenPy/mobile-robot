/**
 * @file testMotorDistanceCommand.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "command/MotorPIDCommand.h"
#include "command/ENCComand.h"
#include "robotgenius/RobotGenius.h"


int main() {
  Scheduler::GetInstance(2, false).start();

  TurnMotorPIDCommand::Ptr turn_ = std::make_shared<TurnMotorPIDCommand>();
  // turn_->withTimer(20)->schedule();

  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    ResetTurnMotorAngle(-5),
    TurnAnglePIDCommand(90)
    // std::make_shared<ResetTurnMotorDistancePIDCommand>(-5)->withTimer(100),
    // std::make_shared<TurnMotorDistancePIDCommand>(1000)->withTimer(100)
  );

  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();

  DG->AddCommands(
    turn_->withTimer(20)
  );
  DG->setDeadlineCommand(sequential);
  DG->schedule();

  sleep(1);
  Scheduler::GetInstance().stop();
}


// int main() {
//   Scheduler::GetInstance(4, false).start();

//   TurnMotorPIDCommand::Ptr turn_ = std::make_shared<TurnMotorPIDCommand>();
//   turn_->withTimer(20)->schedule();

//   ResetTurnMotorDistance(-5)->schedule();
// //   ResetTurnMotorDistancePIDCommand::Ptr reset_turn_command = std::make_shared<ResetTurnMotorDistancePIDCommand>(10);
// // readTurnENCCommand::Ptr command22 = std::make_shared<readTurnENCCommand>();
// //   // readLiftENCCommand::Ptr command32 = std::make_shared<readLiftENCCommand>();
// //   TurnMotorPIDCommand::Ptr command2 = std::make_shared<TurnMotorPIDCommand>();
// //   ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
// //   command22->withTimer(20)->schedule();
// //   command2->withTimer(20)->schedule();
// //   reset_turn_command->withTimer(100)->schedule();
// //   // Robot::GetInstance().setTurnMotorSpeed(-10);
// //   // DG->AddCommands(command22->withTimer(20),command2->withTimer(20));
// //   // DG->setDeadlineCommand(reset_turn_command->withTimer(100));
// //   // DG->schedule();
//   sleep(3);
//   Scheduler::GetInstance().stop();
//   return 0;
// }
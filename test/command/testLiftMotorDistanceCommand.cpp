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
  Scheduler::GetInstance(4, false).start();
  LiftMotorPIDCommand::Ptr lift = std::make_shared<LiftMotorPIDCommand>();

  SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  
  S->AddCommands(
    ResetLiftMotorDistance(10),
    LiftDistancePIDCommand(-20)
    // LiftMotorDistancePIDCommandAssistance(-1000)
    // std::make_shared<ResetLiftMotorDistancePIDCommand>()->withTimer(100),
    // std::make_shared<LiftMotorDistancePIDCommand>(-800)->withTimer(100)
  );

  DG->AddCommands(
    lift->withTimer(20)
  );
  DG->setDeadlineCommand(S);
  DG->schedule();
  sleep(3);
  Scheduler::GetInstance().stop();
}


// int main() {
//   Scheduler::GetInstance(2, false).start();
//   LiftMotorPIDCommand::Ptr lift = std::make_shared<LiftMotorPIDCommand>();
//   lift->withTimer(20)->schedule();

//   SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
  
//   S->AddCommands(
//     ResetLiftMotorDistance()
//     // LiftMotorDistancePIDCommandAssistance(-400),
//     // LiftMotorDistancePIDCommandAssistance(-800),
//     // std::make_shared<ResetLiftMotorDistancePIDCommand>(),
//     // std::make_shared<LiftMotorDistancePIDCommand>(-1000)
//   );

//   S->schedule();
//   sleep(3);
//   Scheduler::GetInstance().stop();
// }
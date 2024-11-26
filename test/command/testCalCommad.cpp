/**
 * @file testLabview.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius,com)
 * @brief 测试共享内存
 * @version 0.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream>
#include <string>
#include "share.h"

#include "command/MotorPIDCommand.h"
#include "command/ButtonCommand.h"
#include "command/CalCommand.h"
#include "command/ZeroOdomCommand.h"
#include "command/UpdataOdomCommand.h"


int main() {
  Scheduler::GetInstance(5, false).start();

  ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
  RG->AddCommands(
    // SingleIRCalCommandAssistance(25, 0.5, 0.5, 3),
    // IRCalCommandAssistance(20, 0.5, 0.5, 5, 0),
    USCalCommandAssistance(20, 0.5, 0.5, 5, 3,0),
    createEStopCommand()
  );

  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  DG->AddCommands(
    createLeftMotorPIDCommand(),
    createRightMotorPIDCommand()
  );
  DG->setDeadlineCommand(RG);
  DG->schedule();


  sleep(1);
  Scheduler::GetInstance().stop();

  return 0;
}




// int main() {
//   Scheduler::GetInstance(2, false).start();
//   createUpdataOdomCommand()->schedule();
//   for(int i = 0; i < 30; i++){
//     createSetOdomCommand(i*10, i+10, i*10+150)->schedule();

//     sleep(1);
//   }
//   Scheduler::GetInstance().stop();

//   return 0;
// }
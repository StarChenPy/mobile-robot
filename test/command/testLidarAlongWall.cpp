#include "command/LidarAlongWallCommand.h"
#include "command/MotorPIDCommand.h"
#include "command/ButtonCommand.h"


int main() {
  Scheduler::GetInstance(4, false).start();

  SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
  S->AddCommands(
    // createStartCommand(),
    // LidarReadAlongRightWallCommandDG(10, 40)
    LidarReadAlongLeftWallCommandDG(5, 35)
  );
  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  DG->AddCommands(
    createLeftMotorPIDCommand(),
    createRightMotorPIDCommand()
  );
  DG->setDeadlineCommand(S);
  DG->schedule();
  
  sleep(3);
  Scheduler::GetInstance().stop();
  return 0;
}
 
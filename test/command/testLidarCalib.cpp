#include "command/LidarReadCommand.h"
#include "command/LidarCalibCommand.h"
#include "command/MotorPIDCommand.h"
#include "command/UpdataOdomCommand.h"
#include "command/RotateCommand.h"
#include "command/ZeroOdomCommand.h"


int main() {
  // LidarReadCommand::Ptr lidar_command = std::make_shared<LidarReadCommand>();
  // LidarCalibCommand::Ptr calib_command = std::make_shared<LidarCalibCommand>(30);
  LeftMotorPIDCommand::Ptr left = std::make_shared<LeftMotorPIDCommand>();
  RightMotorPIDCommand::Ptr right = std::make_shared<RightMotorPIDCommand>();
  Scheduler::GetInstance(4, false).start();
  sleep(1);

  SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
  // S->AddCommands(calib_command->withTimer(100));
  S->AddCommands(
    LidarReadCalibDG(25)
    // createRotateCommand(-90),
    // LidarReadCalibDG(30),
    // createRotateCommand(180)
  );
  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  DG->AddCommands(
    // lidar_command->withTimer(100),
    left->withTimer(20),
    right->withTimer(20),
    createUpdataOdomCommand()
  );
  DG->setDeadlineCommand(S);
  DG->schedule();
  
  sleep(3);
  Scheduler::GetInstance().stop();
  return 0;
}
 
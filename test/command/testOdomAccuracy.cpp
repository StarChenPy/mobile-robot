#include "command/LidarReadCommand.h"
#include "command/LidarCalibCommand.h"
#include "command/MotorPIDCommand.h"
#include "command/TrackingPointCommand.h"
#include "command/UpdataOdomCommand.h"
#include "command/RotateCommand.h"
#include "command/ZeroOdomCommand.h"


int main() {
  LidarReadCommand::Ptr lidar_command = std::make_shared<LidarReadCommand>();
  LidarCalibCommand::Ptr calib_command = std::make_shared<LidarCalibCommand>(30);
  LeftMotorPIDCommand::Ptr left = std::make_shared<LeftMotorPIDCommand>();
  RightMotorPIDCommand::Ptr right = std::make_shared<RightMotorPIDCommand>();
  UpdataOdomCommand::Ptr pose_ = std::make_shared<UpdataOdomCommand>();
  Scheduler::GetInstance(4, false).start();
  sleep(1);
  // lidar_command->withTimer(100)->schedule();

  SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  S->AddCommands(
    std::make_shared<LidarCalibCommand>(22)->withTimer(100),
    std::make_shared<RotateCommand>(-90)->withTimer(100),
    std::make_shared<LidarCalibCommand>(30)->withTimer(100),
    std::make_shared<ZeroOdomCommand>(),
    std::make_shared<TrackingPointCommand>(Pose(60, 0, 90))->withTimer(100),
    std::make_shared<TrackingPointCommand>(Pose(60, 100, 90))->withTimer(100)
  );
  DG->AddCommands(
    left->withTimer(20),
    right->withTimer(20), 
    std::make_shared<UpdataOdomCommand>()->withTimer(20), 
    lidar_command->withTimer(100)
  );
  DG->setDeadlineCommand(S);
  DG->schedule();
  
  sleep(3);
  Scheduler::GetInstance().stop();
  return 0;
}
 

// S->AddCommands(std::make_shared<LidarCalibCommand>(22)->withTimer(100),
//                  std::make_shared<RotateCommand>(-90)->withTimer(100),
//                  std::make_shared<LidarCalibCommand>(30)->withTimer(100),
//                  std::make_shared<ZeroOdomCommand>(),
//                  std::make_shared<TrackingPointCommand>(Pose(100, 0, 0))->withTimer(100)
//                 );
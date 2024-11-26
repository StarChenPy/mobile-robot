#include "command/TrackingPointCommand.h"
#include "command/MotorPIDCommand.h"
#include "command/UpdataOdomCommand.h"
#include "command/RotateCommand.h"
#include "command/ZeroOdomCommand.h"
#include "command/LidarReadCommand.h"
#include "command/LidarCalibCommand.h"

vector<Pose> points = {Pose(50,0,90), Pose(50,50,0)};
vector<Pose> points_2 = {Pose(50,50,0), Pose(100,0,0)};


int main() {
  Scheduler::GetInstance(4, false).start();
  sleep(1); 
  LeftMotorPIDCommand::Ptr left = std::make_shared<LeftMotorPIDCommand>();
  RightMotorPIDCommand::Ptr right = std::make_shared<RightMotorPIDCommand>();
  LidarReadCommand::Ptr lidar_command = std::make_shared<LidarReadCommand>();
  LidarCalibCommand::Ptr calib_command = std::make_shared<LidarCalibCommand>(30);

  UpdataOdomCommand::Ptr pose_ = std::make_shared<UpdataOdomCommand>();
  TrackingPointCommand::Ptr ctrl_1 = std::make_shared<TrackingPointCommand>(Pose(150,0,-90));
  TrackingPointCommand::Ptr ctrl_2 = std::make_shared<TrackingPointCommand>(Pose(150,-50,0));
  RotateCommand::Ptr Rotate = std::make_shared<RotateCommand>(90);


  SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
  S->AddCommands( 
    // std::make_shared<LidarCalibCommand>(30)->withTimer(100),
    // std::make_shared<ZeroOdomCommand>(),
    // std::make_shared<TrackingPointCommand>(Pose(100,0,90), 30)->withTimer(100)
    // std::make_shared<TrackingXYCommand>(Pose(50,50,180))->withTimer(100),
    // std::make_shared<TrackingPointCommand>(Pose(-100,0,0),30)->withTimer(100),
    // std::make_shared<TrackingXYCommand>(Pose(50,50,180),10)->withTimer(100),
    createTrackingXYCommand(Pose(-40,0,0), 30),
    createTrackingXYCommand(Pose(-40,0,-180), 30),
    createTrackingXYCommand(Pose(-80,0,-180), 30)
    // createRotateCommand(90),
    // createTrackingPointCommand(Pose(0,0,0), 30)
    // createTrackingPointCommand(Pose(-50,-50,0), 10),
    // createTrackingVectorCommand(points, 10),
    // createTrackingVectorXYCommand(points_2)
  );
  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  DG->AddCommands(
    left->withTimer(20),
    right->withTimer(20), 
    // lidar_command->withTimer(100),
    pose_->withTimer(20)
  );
  DG->setDeadlineCommand(S);
  DG->schedule();

  sleep(3);
  Scheduler::GetInstance().stop();
  std::cout << "-------------end-------------" << std::endl;

  return 0;
}
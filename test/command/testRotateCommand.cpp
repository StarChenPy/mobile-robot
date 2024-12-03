#include "command/TrackingPointCommand.h"
#include "command/MotorPIDCommand.h"
#include "command/UpdataOdomCommand.h"
#include "command/RotateCommand.h"
#include "command/sensor/ZeroOdomCommand.h"


int main() {
  Scheduler::GetInstance(4, false).start();
  sleep(1); 
  LeftMotorPIDCommand::Ptr left = std::make_shared<LeftMotorPIDCommand>();
  RightMotorPIDCommand::Ptr right = std::make_shared<RightMotorPIDCommand>();
  UpdataOdomCommand::Ptr pose_ = std::make_shared<UpdataOdomCommand>();
  RotateCommand::Ptr Rotate = std::make_shared<RotateCommand>(90);

  // pose_->withTimer(20)->schedule();
  Robot::GetInstance().chassis_ctrl->set_v_pidlimit(30);

  SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
  // S->AddCommands(std::make_shared<RotateCommand>(90)->withTimer(100));
  S->AddCommands(
    // std::make_shared<RotateCommand>(90)->withTimer(100),
    // std::make_shared<RotateCommand>(90)->withTimer(100),
    // std::make_shared<RotateCommand>(-90)->withTimer(100),
    // std::make_shared<RotateCommand>(180)->withTimer(100),
    std::make_shared<RotateCommand>(90)->withTimer(100)
  );
  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  DG->AddCommands(left->withTimer(20),right->withTimer(20), pose_->withTimer(20));
  DG->setDeadlineCommand(S);
  DG->schedule();

  sleep(3);
  Scheduler::GetInstance().stop();
  std::cout << "-------------end-------------" << std::endl;

  return 0;
}
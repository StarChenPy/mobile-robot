#include "command/RoboticArmFun.h"
#include "command/VisionCtrlCommand.h"
#include "command/MotorPIDCommand.h"
#include "command/LidarReadCommand.h"
#include "command/LidarCalibCommand.h"
#include "command/TrackingPointCommand.h"
#include "command/UpdataOdomCommand.h"
#include "command/RotateCommand.h"
#include "command/ZeroOdomCommand.h"

//车辆校准
Command::Ptr CarCalibAction(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    createRotateCommand(90),
    createLidarCalibCommand(22),
    createZeroOdomCommand(),
    createTrackingPointCommand(Pose(10,0,0),10),
    createRotateCommand(-90),
    createLidarCalibCommand(22),
    createZeroOdomCommand()
  );
  return sequential;
}

int main() {
  Scheduler::GetInstance(5, false).start();
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    ResetLiftAndTurn(-5),
    CarMoveStatus(),
    createcTelescopicServoCommand(TELESCOPIC_DIS_MAX),
    createTrackingPointCommand(Pose(100,0,0),10),
    createZeroOdomCommand()
    // CarCalibAction(),
    // PickBasketAction(),
    // // // createZeroOdomCommand(),
    // createTrackingPointCommand(Pose(-72,0,0),20),
    // PutBasketAction()
  );

  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
// DG->m_parent= Command::getPtr();
// Command::State ::HOLDON;
  DG->AddCommands(
    createTurnMotorPIDCommand(),
    createLiftMotorPIDCommand(),
    createLeftMotorPIDCommand(),
    createRightMotorPIDCommand(),
    createUpdataOdomCommand(),
    createLidarReadCommand()
  );
  DG->setDeadlineCommand(sequential);
  DG->schedule();


  sleep(1);
  Scheduler::GetInstance().stop();
}


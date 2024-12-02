#include "command/RoboticArmFun.h"
#include "command/VisionCtrlCommand.h"
#include "command/MotorPIDCommand.h"

int main() {
  Scheduler::GetInstance(2, false).start();

  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    // ResetLiftAndTurn(-5),
    // VisionStatus(),
    createVisionIdentifyCommand(0)     //识别前方是否有指定水果
    // createVisionCtrlCommand(0),
    // PickFruitStatus(3),
    // PutFruit2BasketAction()
  );

  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();

  DG->AddCommands(
    createTurnMotorPIDCommand(),
    createLiftMotorPIDCommand(),
    createLeftMotorPIDCommand(),
    createRightMotorPIDCommand()
  );
  DG->setDeadlineCommand(sequential);
  DG->schedule();

  sleep(1);
  Scheduler::GetInstance().stop();
}


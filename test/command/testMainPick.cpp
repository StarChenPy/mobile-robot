#include "command/MotorPIDCommand.h"
#include "command/TurnCtrlCommand.h"
#include "command/LiftHeightCommand.h"
#include "command/ServoCommand.h"
#include "command/RaiseServoCommand.h"
#include "command/TelescopicServoCommand.h"


// 测试控制
int main() {
  TurnMotorPIDCommand::Ptr Turn_motor = std::make_shared<TurnMotorPIDCommand>();
  TurnAngleCommand::Ptr turn_angle = std::make_shared<TurnAngleCommand>();
  LiftMotorPIDCommand::Ptr Lift_motor = std::make_shared<LiftMotorPIDCommand>();
  LiftHeightCommand::Ptr height = std::make_shared<LiftHeightCommand>();
  Scheduler::GetInstance(2, false).start();
  sleep(1);
  SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  S->AddCommands(
    std::make_shared<ClampServoCommand>(CLAMP_SERVO_MAX)->withTimer(100),
    std::make_shared<TurnCtrlCommand>(90)->withTimer(100),
    std::make_shared<LiftCtrlCommand>(-10)->withTimer(100),
    std::make_shared<TelescopicServoCommand>(TELESCOPIC_SERVO_MAX)->withTimer(100),
    std::make_shared<RaiseServoCommand>(RAISE_SERVO_MIN)->withTimer(100),
    std::make_shared<ClampServoCommand>(CLAMP_SERVO_MIN)->withTimer(100),
    std::make_shared<LiftCtrlCommand>(0)->withTimer(100),
    std::make_shared<TurnCtrlCommand>(0)->withTimer(100),
    std::make_shared<RaiseServoCommand>(RAISE_SERVO_MAX)->withTimer(100)
  );
  DG->AddCommands(
    Turn_motor->withTimer(20),
    turn_angle->withTimer(20),
    Lift_motor->withTimer(20),
    height->withTimer(20)
  );
  DG->setDeadlineCommand(S);
  DG->schedule();

  sleep(3);
  Scheduler::GetInstance().stop();
  return 0;
}
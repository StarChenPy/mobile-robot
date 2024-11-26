#include "command/ServoCommand.h"

int main() {
  Scheduler::GetInstance(1, false).start();
  sleep(1);
  SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
  S->AddCommands(
    // std::make_shared<TelescopicServoCommand>(TELESCOPIC_SERVO_MIN)->withTimer(100),
    // std::make_shared<ClampServoCommand>(CLAMP_SERVO_MAX)->withTimer(100),
    // std::make_shared<RaiseServoCommand>((RAISE_SERVO_MIN+RAISE_SERVO_MAX)/2)->withTimer(100),

    // std::make_shared<TelescopicServoCommand>(0.23)->withTimer(100)
    // std::make_shared<ClampServoCommand>(CLAMP_SERVO_MIN)->withTimer(100),
    // std::make_shared<RaiseServoCommand>(RAISE_SERVO_MIN)->withTimer(100)

    createRotatingServoCommand(45,5),
    createRotatingServoCommand(-45,5),
    createRotatingServoCommand(90,5),
    createRotatingServoCommand(-90,5),

    createClampServoCommand(5),         //CLAMP_LEN_MIN = 2.0    CLAMP_LEN_MAX = 10.0
    createRaiseServoCommand(90),       //RAISE_ANGLE_MIN = 0.0   RAISE_ANGLE_MAX = 90.0
    createcTelescopicServoCommand(5)    //TELESCOPIC_DIS_MIN = 0.0   TELESCOPIC_DIS_MAX = 9.0
  );
  S->schedule();
  sleep(3);
  Scheduler::GetInstance().stop();
  return 0;
}


#include "command/UpdataOdomCommand.h"


int main() {
  UpdataOdomCommand::Ptr command0 = std::make_shared<UpdataOdomCommand>();
  Scheduler::GetInstance(2, false).start();
  sleep(1);
  command0->withTimer(ODOM_PERIOD)->schedule();
  for (int i = 0; i < 10; i++) {
    LeftMotor->setEnable();
    LeftMotor->setSpeedAndDir(10, true, false);
    // std::cout << "LeftENC read:" << LeftENC->read() << std::endl;
    // std::cout << "LeftENC get " << LeftENC->get() << std::endl;
    RightMotor->setEnable();
    RightMotor->setSpeedAndDir(10, false, true);
    // std::cout << "RightENC read:" << RightENC->read() << std::endl;
    // std::cout << "RightENC get " << RightENC->get() << std::endl;
    sleep(1);
  }
  sleep(3);
  Scheduler::GetInstance().stop();
  return 0;
}
 
#include "RobotCfg.h"

int main() {
  for (int i = 0; i < 10; i++) {
    LeftMotor->setEnable();
    LeftMotor->setSpeedAndDir(i * 10, true, false);
    std::cout << "LeftENC read:" << LeftENC->read() << std::endl;
    std::cout << "LeftENC get " << LeftENC->get() << std::endl;
    RightMotor->setEnable();
    RightMotor->setSpeedAndDir(i * 10, true, false);
    std::cout << "RightENC read:" << RightENC->read() << std::endl;
    std::cout << "RightENC get " << RightENC->get() << std::endl;
    // TurnMotor->setEnable();
    // TurnMotor->setSpeedAndDir(i * 10, true, false);
    // std::cout << "TurnENC read:" << TurnENC->read() << std::endl;
    // std::cout << "TurnENC get " << TurnENC->get() << std::endl;
    // LiftMotor->setEnable();
    // LiftMotor->setSpeedAndDir(i * 10, true, false);
    // std::cout << "LiftENC read:" << LiftENC->read() << std::endl;
    // std::cout << "LiftENC get " << LiftENC->get() << std::endl;
    sleep(1);
  }
  return 0;
}

#include <robotgenius/RobotGenius.h>

using namespace VMX;
using namespace Titan;

// 升降测试
int main() {
  ENC::ptr encLift = std::make_shared<ENC>(0);
  Motor::ptr motorLift = std::make_shared<Motor>(0);  

  for (int i = 0; i < 15; i++){
    motorLift->setEnable();
    // motorLift->setSpeedAndDir(40, true, false);
    motorLift->setSpeedAndDir(0, false, false);
    std::cout << "readLeft:" << encLift->read() << std::endl;
    std::cout << "getLeft:" << encLift->get() << std::endl;
    sleep(1);
  } 
  return 0;
}

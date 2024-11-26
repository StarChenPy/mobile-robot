#include <robotgenius/RobotGenius.h>

using namespace VMX;
using namespace Titan;

int main() {
  ENC::ptr encLeft = std::make_shared<ENC>(1);
  Motor::ptr motorLeft = std::make_shared<Motor>(1);  
  ENC::ptr encRight = std::make_shared<ENC>(2);
  Motor::ptr motorRight = std::make_shared<Motor>(2);  

  for (int i = 0; i < 10; i++) {
    motorLeft->setEnable();
    motorLeft->setSpeedAndDir(i*5, true, false);
    std::cout << "readLeft:" << encLeft->read() << std::endl;
    std::cout << "getLeft:" << encLeft->get() << std::endl;
    // motorRight->setEnable();
    // motorRight->setSpeedAndDir(i*5, true, false);
    // std::cout << "readRight:" << encRight->read() << std::endl;
    // std::cout << "getRight: " << encRight->get() << std::endl;
    sleep(1);
  }
  return 0;
}

#include <robotgenius/RobotGenius.h>

using namespace VMX;
using namespace Titan;


// 转盘测试
int main() {
  ENC::ptr encTurntable = std::make_shared<ENC>(3);
  Motor::ptr motorTurntable = std::make_shared<Motor>(3);  

  for (int i = 0; i < 5; i++) {
    motorTurntable->setEnable();
    motorTurntable->setSpeedAndDir(20, true, false);  //正值向右转；
    std::cout << "readTurntable:" << encTurntable->read() << std::endl;
    std::cout << "getTurntable:" << encTurntable->get() << std::endl;
    sleep(1);
  }
  motorTurntable->setSpeedAndDir(0, true, false); 
  return 0;
}
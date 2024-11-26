#include <robotgenius/RobotGenius.h>

// 测试三个按钮led灯光
using namespace VMX;
int main() {
  PWM::ptr startled = std::make_shared<PWM>(19);
  PWM::ptr resetled = std::make_shared<PWM>(20);
  PWM::ptr stopled = std::make_shared<PWM>(21);

  resetled->setDutyCycle(0);
  stopled->setDutyCycle(0);
  startled->setDutyCycle(1);
  sleep(3);
  startled->setDutyCycle(0);
  resetled->setDutyCycle(1);
  sleep(3);
  resetled->setDutyCycle(0);
  stopled->setDutyCycle(1);
  sleep(3);
  stopled->setDutyCycle(0);
  return 0;
}

#include <robotgenius/RobotGenius.h>

using namespace VMX;

static float Sinit = 0.095;
static float Smin = 0.115;
static float Smax = 0.25;


// 伸缩臂舵机测试
int main() {
  PWM::ptr Fb = std::make_shared<PWM>(16,100);
  Fb->setDutyCycle(Sinit);
  sleep(2);
  // Fb->setDutyCycle(Smin);
  // sleep(2);
  Fb->setDutyCycle(0.12);
  sleep(2);
  // Fb->setDutyCycle(Sinit);

  // Fb->setDutyCycle(0.12);
  sleep(200);

  return 0;
}


// std::make_shared<TelescopicServoCommand>(0.12)->withTimer(1000)   //0.1 - 0.22， 0.12
// std::make_shared<ClampServoCommand>(0.25)->withTimer(1000)        //0.1-0.15-0.25
// std::make_shared<RaiseServoCommand>(0.25)->withTimer(1000)        // 0.1 - 0.25

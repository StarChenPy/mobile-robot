#include <robotgenius/RobotGenius.h>

using namespace VMX;

static float Sinit = 0.2;
static float Smin = 0.05;
static float Smax = 0.255;


// 夹手舵机测试
int main() {
  PWM::ptr Ch = std::make_shared<PWM>(14,100);
  Ch->setDutyCycle(Smin);
  sleep(2);
  // Ch->setDutyCycle(Smax);
  // sleep(2);
  Ch->setDutyCycle(0.255);

  // Ch->setDutyCycle(0.055);
  sleep(200);

  return 0; 
}



// std::make_shared<TelescopicServoCommand>(0.12)->withTimer(1000)   //0.1 - 0.22， 0.12
// std::make_shared<ClampServoCommand>(0.25)->withTimer(1000)        //0.1-0.15-0.25
// std::make_shared<RaiseServoCommand>(0.25)->withTimer(1000)        // 0.05 - 0.255
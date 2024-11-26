#include <robotgenius/RobotGenius.h>

using namespace VMX;

static float Sinit = 0.2;
static float Smin = 0.05;
static float Smax = 0.235;


// // 抬手舵机测试
// int main() {
//   PWM::ptr Rh = std::make_shared<PWM>(15,100);
//   Rh->setDutyCycle(Sinit);
//   sleep(2);
//   Rh->setDutyCycle(0.05);
//   // sleep(2);
//   // Rh->setDutyCycle(Smax);
//   // sleep(2);
//   // Rh->setDutyCycle(Sinit);

//   sleep(200);

//   return 0;
// }

// 旋转舵机测试
int main() {
  PWM::ptr RS = std::make_shared<PWM>(17,100);
  RS->setDutyCycle(Sinit);
  sleep(2);
  RS->setDutyCycle(0.05);
  // sleep(2);
  // Rh->setDutyCycle(Smax);
  // sleep(2);
  // Rh->setDutyCycle(Sinit);

  sleep(200);

  return 0;
}


// std::make_shared<TelescopicServoCommand>(0.12)->withTimer(1000)   //0.1 - 0.22， 0.12
// std::make_shared<ClampServoCommand>(0.25)->withTimer(1000)            //0.1-0.15-0.25
// std::make_shared<RaiseServoCommand>(0.25)->withTimer(1000)        // 0.1 - 0.25

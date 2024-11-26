#include <robotgenius/RobotGenius.h>
#include<cmath>

using namespace VMX;

double IRDistance(double input){
    return 101.012 - 166.289*input + 141.969*std::pow(input,2) - 66.134*std::pow(input,3) + 15.868*std::pow(input,4) - 1.532*std::pow(input,5);
}

// 101.012 - 166.289*x + 141.969*x**2 - 66.134*x**3 + 15.868*x**4 - 1.532*x**5

// 读取红外
int main() {
  AI::ptr LeftIR = std::make_shared<AI>(22);
  AI::ptr RightIR = std::make_shared<AI>(23);
  for (int i = 0; i < 20; i++) {
    static double LeftIR_s  = 0.0;
    static double RightIR_s  = 0.0;
    LeftIR_s = IRDistance(LeftIR->read());
    RightIR_s = IRDistance(RightIR->read());
    std::cout << "LeftIR:" << LeftIR_s << std::endl;
    std::cout << "RightIR:" << RightIR_s << std::endl;
    sleep(1);
  }
  return 0;
}

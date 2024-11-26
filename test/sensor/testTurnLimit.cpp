#include <robotgenius/RobotGenius.h>


// 读取转盘限位
using namespace VMX;
int main() {
  AI::ptr Turnlimit = std::make_shared<AI>(25);
  for (int i = 0; i < 15; i++) {
    static bool Turn_s = false;
    Turnlimit->read();
    Turn_s = Turnlimit->get()>1.0;
    std::cout << "Turnlimit:" << Turn_s << std::endl;
    sleep(1);
  }
  return 0;
}

#include <robotgenius/RobotGenius.h>


// 读取升降限位
using namespace VMX;
int main() {
  DI::ptr up_limit = std::make_shared<DI>(9);
  AI::ptr downlimit = std::make_shared<AI>(24);
  for (int i = 0; i < 20; i++) {
    static bool value_up = 0.0;
    static float value_down = 0.0;
    value_up = !up_limit->read();
    value_down = downlimit->read();
    bool down_s= value_down>1.0;
    std::cout << "Up_limit:" << value_up << std::endl;
    std::cout << "Down_limit:" << down_s << std::endl;
    sleep(1);
  }
  return 0;
}

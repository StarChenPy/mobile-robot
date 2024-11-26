#include <robotgenius/RobotGenius.h>


// 读取急停信号
using namespace VMX;
int main() {
  DI::ptr  Estoplimit = std::make_shared<DI>(11);
  for (int i = 0; i < 15; i++) {
    static bool Estop = 0.0;
    Estop = Estoplimit->read();
    std::cout << "Estoplimit:" << Estop << std::endl;
    sleep(1);
  }
  return 0;
}

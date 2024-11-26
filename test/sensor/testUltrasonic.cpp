#include <robotgenius/sensor/ultrasound.h>
#include <unistd.h>
using namespace VMX;
using namespace Sensor;


// 读取超声波
int main() {
  Ultrasound::Ptr Leftus = std::make_shared<Ultrasound>(12, 8);
  Ultrasound::Ptr Rightus = std::make_shared<Ultrasound>(13,10);
  for (int i = 0; i <15; i++) {
    Leftus->trig();
    Rightus->trig();
    std::cout << "Leftus:" << Leftus->echo() * 0.017 << std::endl;
    std::cout << "Rightus: " << Rightus->echo() * 0.017 << std::endl;
    sleep(1);
  }
  return 0;
}

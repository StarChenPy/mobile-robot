#include <robotgenius/hal/Titan.h>
#include <iostream>
#include <unistd.h>

using namespace Titan;

// 读取三个按钮信号
int limit_s[4] = {0,2,3};
int main() {
  TitanQuandLimit::ptr Button = std::make_shared<TitanQuandLimit>();
  Button->setEnable();

  for (int i=0; i < 20; i++){
    Button->read();
    std::cout << "Start: " << Button->get(limit_s[1]) << std::endl;
    std::cout << "Reset: "  << Button->get(limit_s[0]) << std::endl;
    std::cout << "Stop: "  << Button->get(limit_s[2]) << std::endl;
    sleep(1);
  }
  return 0;
}


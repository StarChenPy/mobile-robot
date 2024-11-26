#include "command/Vision/VisionCommand.h"


int main() {
  Scheduler::GetInstance(4, false).start();
  IdentifyFruitCommand::Ptr identifyFruitCommand = std::make_shared<IdentifyFruitCommand>();
  
  identifyFruitCommand->withTimer(200)->schedule();

  Scheduler::GetInstance().stop();
  std::cout << Vision::GetInstance().getResult().label << std::endl;
  return 0;
  
}


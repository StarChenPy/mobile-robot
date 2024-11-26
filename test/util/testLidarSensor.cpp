#include "command/LidarReadCommand.h"


int main() {
  LidarReadCommand::Ptr command0 = std::make_shared<LidarReadCommand>();
  Scheduler::GetInstance(2, false).start();
  sleep(1);
  command0->withTimer(100)->schedule();
  while(1){
    std::vector<LidarData> data;
    Robot::GetInstance().lidar_read->getLidarData(data);
    std::cout << "Lidar data" << std::endl;
    for(int i = 0; i < data.size(); i++){
      std::cout << "angle = " << data[i].Angle * (180/M_PI) << " r = " << data[i].Range * 100 << " Intensity = " << data[i].Intensity << std::endl;
    }
    sleep(1);
  }
  sleep(3);
  Scheduler::GetInstance().stop();
  return 0;
}
 
#include "command/LidarReadCommand.h"


void LidarReadCommand::initialize() {
  is_finished = false;
}
void LidarReadCommand::execute() {
  Robot::GetInstance().lidar_read->Task();
  
  // std::cout << "UpdataOdomCommand execute dt = " << static_cast<double>(dt) << std::endl;
  // is_finished = true;
}
void LidarReadCommand::end() {
  std::cout << "LidarReadCommand end!" <<std::endl;
}
bool LidarReadCommand::isFinished() {
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}


Command::Ptr createLidarReadCommand(){
  return std::make_shared<LidarReadCommand>()->withTimer(100);
}
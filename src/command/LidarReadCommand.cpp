#include "command/LidarReadCommand.h"

void LidarReadCommand::initialize() { isFinished_ = false; }
void LidarReadCommand::execute() {
    Robot::GetInstance().lidar_read->Task();

    // std::cout << "UpdataOdomCommand execute dt = " << static_cast<double>(dt)
    // << std::endl; isFinished_ = true;
}
void LidarReadCommand::end() { std::cout << "LidarReadCommand end!" << std::endl; }
bool LidarReadCommand::isFinished() {
    if (Robot::GetInstance().getStopSignal()) {
        stopAll();
    }
    return isFinished_ || Robot::GetInstance().getStopSignal();
}

Command::ptr createLidarReadCommand() { return std::make_shared<LidarReadCommand>()->withTimer(100); }
#include "command/lidar/LidarReadCommand.h"

void LidarReadCommand::initialize() { isFinished_ = false; }
void LidarReadCommand::execute() {
    Robot::getInstance().lidar_read->Task();

    // std::cout << "UpdateOdomCommand execute dt = " << static_cast<double>(dt)
    // << std::endl; isFinished_ = true;
}
void LidarReadCommand::end() { std::cout << "LidarReadCommand end!" << std::endl; }
bool LidarReadCommand::isFinished() {
    if (Robot::getInstance().getStopSignal()) {
        stopAll();
    }
    return isFinished_ || Robot::getInstance().getStopSignal();
}

Command::ptr createLidarReadCommand() { return std::make_shared<LidarReadCommand>()->withTimer(100); }
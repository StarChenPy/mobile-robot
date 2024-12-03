#include "command/lidar/LidarReadCommand.h"

void LidarReadCommand::execute() {
    Robot::getInstance().lidar_read->Task();
}

void LidarReadCommand::end() { std::cout << "LidarReadCommand end!" << std::endl; }

ICommand::ptr LidarReadCommand::create() {
    return std::make_shared<LidarReadCommand>()->withTimer(100);
}
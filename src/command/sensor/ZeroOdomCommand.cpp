#include "command/sensor/ZeroOdomCommand.h"

void ZeroOdomCommand::initialize() {
    isFinished_ = false;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
}
void ZeroOdomCommand::execute() {
    Robot::getInstance().odom->zeroPose();
    // 打印
    Robot::getInstance().odom->print();
    // sleep(3);
    isFinished_ = true;
}
void ZeroOdomCommand::end() {
    std::cout << "ZeroOdomCommand end!" << std::endl;
}
ICommand::ptr createZeroOdomCommand() { return std::make_shared<ZeroOdomCommand>()->withTimer(100); }

void SetOdomCommand::initialize() {
    isFinished_ = false;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
}
void SetOdomCommand::execute() {
    Robot::getInstance().odom->setPose(set_x, set_y, set_theta);
    // 打印
    Robot::getInstance().odom->print();
    // sleep(3);
    isFinished_ = true;
}
void SetOdomCommand::end() {
    std::cout << "SetOdomCommand end!" << std::endl;
}
ICommand::ptr createSetOdomCommand(double x, double y, double theta) {
    return std::make_shared<SetOdomCommand>(x, y, theta)->withTimer(100);
}

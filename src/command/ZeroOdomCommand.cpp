#include "command/ZeroOdomCommand.h"

void ZeroOdomCommand::initialize() {
    is_finished = false;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
}
void ZeroOdomCommand::execute() {
    int time = robot::getCurrentMs();
    m_last_time = time;

    Robot::getInstance().odom->zeroPose();

    // 打印
    Robot::getInstance().odom->print();

    // sleep(3);
    is_finished = true;
}
void ZeroOdomCommand::end() {
    std::cout << "ZeroOdomCommand end!" << std::endl;
}
bool ZeroOdomCommand::isFinished() {
    if (Robot::getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getInstance().getStopSignal();
}
Command::ptr createZeroOdomCommand() { return std::make_shared<ZeroOdomCommand>()->withTimer(100); }

void SetOdomCommand::initialize() {
    is_finished = false;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
}
void SetOdomCommand::execute() {
    int time = robot::getCurrentMs();
    m_last_time = time;

    Robot::getInstance().odom->setPose(set_x, set_y, set_theta);
    // 打印
    Robot::getInstance().odom->print();

    // sleep(3);
    is_finished = true;
}
void SetOdomCommand::end() {
    std::cout << "SetOdomCommand end!" << std::endl;
}
bool SetOdomCommand::isFinished() {
    if (Robot::getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getStopSignal();
}
Command::ptr createSetOdomCommand(double x, double y, double theta) {
    return std::make_shared<SetOdomCommand>(x, y, theta)->withTimer(100);
}

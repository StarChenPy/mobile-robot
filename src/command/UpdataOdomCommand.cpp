#include "command/UpdateOdomCommand.h"

void UpdateOdomCommand::initialize() {
    is_finished = false;
    Robot::getInstance().odom->zeroPose();
}
void UpdateOdomCommand::execute() {
    int time = robot::getCurrentMs();
    int dt = time - m_last_time;
    m_last_time = time;
    // 获取相应数据
    double gyro = VMX::getYaw();
    double L_enc = leftEnc->read();
    double R_enc = rightEnc->read();
    // 运算
    Robot::getInstance().odom->CarClassisOdometer(L_enc, R_enc, gyro, static_cast<double>(dt));
    // 打印
    // Robot::getInstance().odom->print();
}
void UpdateOdomCommand::end() {
    std::cout << "!!!!!!!!!!!!!!!!!UpdateOdomCommand end!!!!!!!!!!!!!!!!!!!!" << std::endl;
}
bool UpdateOdomCommand::isFinished() {
    if (Robot::getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getStopSignal();
}

Command::ptr createUpdataOdomCommand() { return std::make_shared<UpdateOdomCommand>()->withTimer(20); }
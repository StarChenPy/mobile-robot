#include "command/sensor/UpdateOdomCommand.h"

void UpdateOdomCommand::initialize() {
    isFinished_ = false;
    Robot::getInstance().odom->zeroPose();
}
void UpdateOdomCommand::execute() {
    int time = getCurrentMs();
    int dt = time - lastTime_;
    lastTime_ = time;
    // 获取相应数据
    double gyro = VMX::getYaw();
    double L_enc = leftEnc->read();
    double R_enc = rightEnc->read();
    // 运算
    Robot::getInstance().odom->CarClassisOdometer(L_enc, R_enc, gyro, static_cast<double>(dt));
}

void UpdateOdomCommand::end() {
    std::cout << "!!!!!!!!!!!!!!!!!UpdateOdomCommand end!!!!!!!!!!!!!!!!!!!!" << std::endl;
}

ICommand::ptr UpdateOdomCommand::create() {
    return std::make_shared<UpdateOdomCommand>()->withTimer(20);
}

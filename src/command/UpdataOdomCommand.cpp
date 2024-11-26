#include "command/UpdataOdomCommand.h"

void UpdataOdomCommand::initialize() {
    is_finished = false;
    Robot::GetInstance().odom->zeroPose();
}
void UpdataOdomCommand::execute() {
    int time = robot::getCurrentMs();
    int dt = time - m_last_time;
    m_last_time = time;
    // 获取相应数据
    double gyro = VMX::getYaw();
    double L_enc = LeftENC->read();
    double R_enc = RightENC->read();
    // 运算
    Robot::GetInstance().odom->CarClassisOdometer(L_enc, R_enc, gyro, static_cast<double>(dt));
    // 打印
    // Robot::instance().odom->print();
    Pose cur_pose = Robot::GetInstance().odom->getPose();
    LABVIEW::Pose labview_pose(cur_pose.x_, cur_pose.y_, cur_pose.theta_);
    LABVIEW::UpdateOdomShareAddress->write(labview_pose);

    // std::cout << "UpdataOdomCommand execute dt = " << static_cast<double>(dt)
    // << std::endl; is_finished = true;
}
void UpdataOdomCommand::end() {
    std::cout << "!!!!!!!!!!!!!!!!!UpdataOdomCommand end!!!!!!!!!!!!!!!!!!!!" << std::endl;
}
bool UpdataOdomCommand::isFinished() {
    if (Robot::GetInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::GetInstance().getStopSignal();
}

Command::ptr createUpdataOdomCommand() { return std::make_shared<UpdataOdomCommand>()->withTimer(20); }
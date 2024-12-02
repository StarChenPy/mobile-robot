#include "command/RotateCommand.h"

void RotateCommand::initialize() {
    is_finished = false;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
    Pose init = Robot::getInstance().odom->getPose();
    Init_Phi = init.theta_;
}
void RotateCommand::execute() {
    Pose cur = Robot::getInstance().odom->getPose();
    is_finished = Robot::getInstance().chassis_ctrl->RotateTask(Init_Phi + target_angle, cur.theta_);
    double R_setpoint = Robot::getInstance().chassis_ctrl->get_R_setpoint();
    double L_setpoint = Robot::getInstance().chassis_ctrl->get_L_setpoint();
    Robot::getInstance().setRightMotorSpeed(R_setpoint);
    Robot::getInstance().setLeftMotorSpeed(L_setpoint);

    int time = robot::getCurrentMs();
    m_last_time = time;
}
void RotateCommand::end() {
    std::cout << "RotateCommand end!" << std::endl;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
}
bool RotateCommand::isFinished() {
    if (Robot::getInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getInstance().getStopSignal();
}

Command::ptr createRotateCommand(double angle) { return std::make_shared<RotateCommand>(angle)->withTimer(100); }

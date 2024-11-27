#include "command/RotateCommand.h"

void RotateCommand::initialize() {
    uint8_t updata_status = COMMEND_WAIT;
    LABVIEW::RotateStatusShareAddress->write(updata_status);

    is_finished = false;
    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);
    Pose init = Robot::GetInstance().odom->getPose();
    Init_Phi = init.theta_;
}
void RotateCommand::execute() {
    Pose cur = Robot::GetInstance().odom->getPose();
    is_finished = Robot::GetInstance().chassis_ctrl->RotateTask(Init_Phi + target_angle, cur.theta_);
    double R_setpoint = Robot::GetInstance().chassis_ctrl->get_R_setpoint();
    double L_setpoint = Robot::GetInstance().chassis_ctrl->get_L_setpoint();
    Robot::GetInstance().setRightMotorSpeed(R_setpoint);
    Robot::GetInstance().setLeftMotorSpeed(L_setpoint);

    int time = robot::getCurrentMs();
    int dt = time - m_last_time;
    m_last_time = time;
    // std::cout << "RotateCommand execute dt = " << dt << std::endl;
    // isFinished_ = true;
}
void RotateCommand::end() {
    std::cout << "RotateCommand end!" << std::endl;
    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);

    uint8_t updata_status = COMMEND_END;
    LABVIEW::RotateStatusShareAddress->write(updata_status);
}
bool RotateCommand::isFinished() {
    uint8_t command_status;
    LABVIEW::RotateStatusShareAddress->read(command_status);
    if (command_status == COMMEND_CANCEL) {
        is_finished = true;
    }
    if (Robot::GetInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::GetInstance().getStopSignal();
}

Command::ptr createRotateCommand(double angle) { return std::make_shared<RotateCommand>(angle)->withTimer(100); }

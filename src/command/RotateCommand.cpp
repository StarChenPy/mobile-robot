#include "command/RotateCommand.h"

namespace robot {
    void RotateCommand::initialize() {
        isFinished_ = false;
        Robot::getInstance().setRightMotorSpeed(0);
        Robot::getInstance().setLeftMotorSpeed(0);
        Pose init = Robot::getInstance().odom->getPose();
        initPhi_ = init.theta_;
    }
    void RotateCommand::execute() {
        Pose cur = Robot::getInstance().odom->getPose();
        isFinished_ = Robot::getInstance().chassis_ctrl->RotateTask(initPhi_ + targetAngle_, cur.theta_);
        double R_setpoint = Robot::getInstance().chassis_ctrl->get_R_setpoint();
        double L_setpoint = Robot::getInstance().chassis_ctrl->get_L_setpoint();
        Robot::getInstance().setRightMotorSpeed(R_setpoint);
        Robot::getInstance().setLeftMotorSpeed(L_setpoint);

        int time = getCurrentMs();
        lastTime_ = time;
    }
    void RotateCommand::end() {
        std::cout << "RotateCommand end!" << std::endl;
        Robot::getInstance().setRightMotorSpeed(0);
        Robot::getInstance().setLeftMotorSpeed(0);
    }

    ICommand::ptr RotateCommand::create(double angle) {
        return std::make_shared<RotateCommand>(angle)->withTimer(100);
    }
} // namespace robot

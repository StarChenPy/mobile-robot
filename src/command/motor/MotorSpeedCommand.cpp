//
// Created by 34253 on 2024/11/28.
//

#include "command/motor/MotorSpeedCommand.h"
#include "system/Robot.h"

namespace robot {
    MotorSpeedCommand::MotorSpeedCommand(double speed, uint32_t counter) {
        speed_ = speed;
        counter_ = counter;
        counter_ = 0;
        currentCounter_ = 0;
    }
    void MotorSpeedCommand::initialize() {
        isFinished_ = false;
        currentCounter_ = 0;
    }
    bool MotorSpeedCommand::isFinished() {
        return Robot::getStopSignal() || isFinished_;
    }

    Command::ptr MotorSpeedCommand::create() {
        return std::make_shared<MotorSpeedCommand>()->withTimer(20);
    }

    LeftMotorSpeedCommand::LeftMotorSpeedCommand(double speed, uint32_t counter) : MotorSpeedCommand(speed, counter) {}
    void LeftMotorSpeedCommand::execute() {
        Robot::getInstance().setLeftMotorSpeed(speed_);
        isFinished_ = currentCounter_ > counter_;
        currentCounter_++;
    }
    void LeftMotorSpeedCommand::end() {
        Robot::getInstance().setLeftMotorSpeed(0);
    }

    RightMotorSpeedCommand::RightMotorSpeedCommand(double speed, uint32_t counter) : MotorSpeedCommand(speed, counter) {}
    void RightMotorSpeedCommand::execute() {
        Robot::getInstance().setRightMotorSpeed(speed_);
        isFinished_ = currentCounter_ > counter_;
        currentCounter_++;
    }
    void RightMotorSpeedCommand::end() {
         Robot::getInstance().setRightMotorSpeed(0);
    }
} // namespace robot
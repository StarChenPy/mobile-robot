#include "command/motor/MotorSpeedCommand.h"
#include "system/Robot.h"

namespace robot {
    void LeftMotorSpeedCommand::execute() {
        Robot::getInstance().setLeftMotorSpeed(speed_);
        isFinished_ = currentCounter_ > counter_;
        currentCounter_++;
    }
    void LeftMotorSpeedCommand::end() {
        Robot::getInstance().setLeftMotorSpeed(0);
    }

    ICommand::ptr LeftMotorSpeedCommand::create(double speed, uint32_t counter) {
        return std::make_shared<LeftMotorSpeedCommand>(speed, counter)->withTimer(20);
    }

    void RightMotorSpeedCommand::execute() {
        Robot::getInstance().setRightMotorSpeed(speed_);
        isFinished_ = currentCounter_ > counter_;
        currentCounter_++;
    }
    void RightMotorSpeedCommand::end() {
         Robot::getInstance().setRightMotorSpeed(0);
    }

    ICommand::ptr RightMotorSpeedCommand::create(double speed, uint32_t counter) {
        return std::make_shared<RightMotorSpeedCommand>(speed, counter)->withTimer(20);
    }
} // namespace robot
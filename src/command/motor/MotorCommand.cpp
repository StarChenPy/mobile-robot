#include "command/motor/MotorCommand.h"
#include "system/Robot.h"

namespace robot {
    MotorCommand::MotorCommand() {
        isFinished_ = false;
    }
    void MotorCommand::initialize() {
        isFinished_ = false;
    }
    bool MotorCommand::isFinished() {
        if (Robot::getStopSignal()) {
            stopAll();
        }
        return isFinished_ || Robot::getStopSignal();
    }
    Command::ptr MotorCommand::create() {
        return std::make_shared<MotorCommand>()->withTimer(20);
    }

    LeftMotorCommand::LeftMotorCommand() = default;
    void LeftMotorCommand::execute() {
        Robot::getInstance().setLeftMotorSpeedWithoutPID();
    }

    RightMotorCommand::RightMotorCommand() = default;
    void RightMotorCommand::execute() {
        Robot::getInstance().setRightMotorSpeedWithoutPID();
    }

    TurnMotorCommand::TurnMotorCommand() = default;
    void TurnMotorCommand::execute() {
        Robot::getInstance().setTurnMotorSpeedWithoutPID();
    }

    LiftMotorCommand::LiftMotorCommand() = default;
    void LiftMotorCommand::execute() {
        Robot::getInstance().setLiftMotorSpeedWithoutPID();
    }
} // namespace robot
#include "command/motor/MotorCommand.h"
#include "system/Robot.h"

namespace robot {
    LeftMotorCommand::LeftMotorCommand() = default;
    void LeftMotorCommand::execute() {
        Robot::getInstance().setLeftMotorSpeedWithoutPID();
    }

    ICommand::ptr LeftMotorCommand::create() {
        return std::make_shared<LeftMotorCommand>()->withTimer(20);
    }

RightMotorCommand::RightMotorCommand() = default;
    void RightMotorCommand::execute() {
        Robot::getInstance().setRightMotorSpeedWithoutPID();
    }

    ICommand::ptr RightMotorCommand::create() {
        return std::make_shared<RightMotorCommand>()->withTimer(20);
    }

    TurnMotorCommand::TurnMotorCommand() = default;
    void TurnMotorCommand::execute() {
        Robot::getInstance().setTurnMotorSpeedWithoutPID();
    }

    ICommand::ptr TurnMotorCommand::create() {
        return std::make_shared<TurnMotorCommand>()->withTimer(20);
    }

    LiftMotorCommand::LiftMotorCommand() = default;
    void LiftMotorCommand::execute() {
        Robot::getInstance().setLiftMotorSpeedWithoutPID();
    }

    ICommand::ptr LiftMotorCommand::create() {
        return std::make_shared<LiftMotorCommand>()->withTimer(20);
    }
} // namespace robot
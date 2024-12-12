#include "command/motor/MotorPIDCommand.h"
#include "system/RobotCfg.h"

void LeftMotorPIDCommand::execute() {
    Robot::getInstance().setLeftMotorProcess(leftEnc->read());
    Robot::getInstance().controlLeftMotor();
    Robot::getInstance().setLeftMotorLastENCCounter(leftEnc->get());
}
void LeftMotorPIDCommand::end() {
    ICommand::end();
    Robot::getInstance().setLeftMotorSpeed(0);
}
ICommand::ptr LeftMotorPIDCommand::create() {
    return std::make_shared<LeftMotorPIDCommand>()->withTimer(20);
}

void RightMotorPIDCommand::execute() {
    Robot::getInstance().setRightMotorProcess(rightEnc->read());
    Robot::getInstance().controlRightMotor();
    Robot::getInstance().setRightMotorLastENCCounter(rightEnc->get());
}
void RightMotorPIDCommand::end() {
    ICommand::end();
    Robot::getInstance().setRightMotorSpeed(0);
}
ICommand::ptr RightMotorPIDCommand::create() {
    return std::make_shared<RightMotorPIDCommand>()->withTimer(20);
}

void TurnMotorPIDCommand::execute() {
    Robot::getInstance().setTurnMotorProcess(turnEnc->read());
    Robot::getInstance().controlTurnMotor();
    Robot::getInstance().setTurnMotorLastENCCounter(turnEnc->get());
}
void TurnMotorPIDCommand::end() {
    ICommand::end();
    Robot::getInstance().setTurnMotorSpeed(0);
}
ICommand::ptr TurnMotorPIDCommand::create() {
    return std::make_shared<TurnMotorPIDCommand>()->withTimer(20);
}

void LiftMotorPIDCommand::execute() {
    Robot::getInstance().setLiftMotorProcess(liftEnc->read());
    Robot::getInstance().controlLiftMotor();
    Robot::getInstance().setLiftMotorLastENCCounter(liftEnc->get());
}
void LiftMotorPIDCommand::end() {
    ICommand::end();
    Robot::getInstance().setLiftMotorSpeed(0);
}
ICommand::ptr LiftMotorPIDCommand::create() {
    return std::make_shared<LiftMotorPIDCommand>()->withTimer(20);
}

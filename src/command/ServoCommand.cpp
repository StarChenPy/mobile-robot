#include "command/ServoCommand.h"

//**********************************夹手*****************************************

ClampServoCommand::ClampServoCommand(double len) {
    clampLen_ = CLAMP_LEN_A * len + CLAMP_LEN_B;
}

void ClampServoCommand::execute() {
    clampServo->setDutyCycle(clampLen_);
    isFinished_ = true;
}

ICommand::ptr ClampServoCommand::create(double len) {
    return std::make_shared<ClampServoCommand>(len)->withTimer(100);
}

//**********************************伸缩*****************************************

TelescopicServoCommand::TelescopicServoCommand(double len) {
    telescopicLen_ = TELESCOPIC_DIS_A * len + TELESCOPIC_DIS_B;
}

void TelescopicServoCommand::execute() {
    telescopicServo->setDutyCycle(telescopicLen_);
    isFinished_ = true;
}

ICommand::ptr TelescopicServoCommand::create(double dis) {
    return std::make_shared<TelescopicServoCommand>(dis)->withTimer(100);
}

//**********************************摆手*****************************************

RaiseServoCommand::RaiseServoCommand(double angle) {
    raiseAngle_ = RAISE_ANGLE_A * angle + RAISE_ANGLE_B;
}

void RaiseServoCommand::execute() {
    raiseServo->setDutyCycle(raiseAngle_);
    isFinished_ = true;
}

ICommand::ptr RaiseServoCommand::create(double angle) {
    return std::make_shared<RaiseServoCommand>(angle)->withTimer(100);
}

//**********************************旋转*****************************************

RotatingServoCommand::RotatingServoCommand(double angle) {
    rotatingAngle_ = ROTATING_ANGLE_A * angle + ROTATING_ANGLE_B;
}

void RotatingServoCommand::execute() {
    rotatingServo->setDutyCycle(rotatingAngle_);
    isFinished_ = true;
}

ICommand::ptr RotatingServoCommand::create(double angle) {
    return std::make_shared<RotatingServoCommand>(angle)->withTimer(100);
}
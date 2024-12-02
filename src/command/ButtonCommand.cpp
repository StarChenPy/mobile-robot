#include "command/ButtonCommand.h"

void EStopCommand::initialize() { is_finished = false; }
void EStopCommand::execute() {

    bool stopButton = stopLimit->read();
    if (!stopButton) {
        leftMotor->setSpeedAndDir(0, false, true);
        rightMotor->setSpeedAndDir(0, false, true);
        liftMotor->setSpeedAndDir(0, false, true);
        turnMotor->setSpeedAndDir(0, false, true);
        std::cout << "EStop titanButton is pushed!" << std::endl;
        is_finished = true;
    }
}
void EStopCommand::end() {
    stopAll();
    leftMotor->setSpeedAndDir(0, false, true);
    rightMotor->setSpeedAndDir(0, false, true);
    liftMotor->setSpeedAndDir(0, false, true);
    turnMotor->setSpeedAndDir(0, false, true);
    std::cout << "EStopCommand done!" << std::endl;
}
bool EStopCommand::isFinished() { return is_finished; }
Command::ptr createEStopCommand() { return std::make_shared<EStopCommand>()->withTimer(100); }

//开始按钮
void StartCommand::initialize() {
    is_finished = false;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
    titanButton->setEnable();
}
void StartCommand::execute() {
    uint64_t time = robot::getCurrentMs();
    m_last_time = time;

    titanButton->read();
    std::cout << "Start: " << titanButton->get(2) << std::endl;
    if (titanButton->get(2)) {
        is_finished = false;
    } else {
        is_finished = true;
    }
}
void StartCommand::end() {
    sleep(5);
    std::cout << "Command Start!!!" << std::endl;
}
bool StartCommand::isFinished() {
    if (Robot::getInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getInstance().getStopSignal();
}
Command::ptr createStartCommand() { return std::make_shared<StartCommand>()->withTimer(100); }

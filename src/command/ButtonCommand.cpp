#include "command/ButtonCommand.h"

void EStopCommand::initialize() { is_finished = false; }
void EStopCommand::execute() {

    bool stopButton = StopLimit->read();
    if (!stopButton) {
        LeftMotor->setSpeedAndDir(0, false, true);
        RightMotor->setSpeedAndDir(0, false, true);
        LiftMotor->setSpeedAndDir(0, false, true);
        TurnMotor->setSpeedAndDir(0, false, true);
        std::cout << "EStop Button is pushed!" << std::endl;
        is_finished = true;
    }
}
void EStopCommand::end() {
    stopAll();
    LeftMotor->setSpeedAndDir(0, false, true);
    RightMotor->setSpeedAndDir(0, false, true);
    LiftMotor->setSpeedAndDir(0, false, true);
    TurnMotor->setSpeedAndDir(0, false, true);
    std::cout << "EStopCommand done!" << std::endl;
}
bool EStopCommand::isFinished() { return is_finished; }
Command::ptr createEStopCommand() { return std::make_shared<EStopCommand>()->withTimer(100); }

//开始按钮
void StartCommand::initialize() {
    is_finished = false;
    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);
    Button->setEnable();
}
void StartCommand::execute() {
    uint64_t time = RobotGenius::getCurrentMs();
    m_last_time = time;

    Button->read();
    std::cout << "Start: " << Button->get(2) << std::endl;
    if (Button->get(2)) {
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
    if (Robot::GetInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::GetInstance().getStopSignal();
}
Command::ptr createStartCommand() { return std::make_shared<StartCommand>()->withTimer(100); }

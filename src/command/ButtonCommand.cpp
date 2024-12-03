#include "command/ButtonCommand.h"

namespace robot {
    void EStopCommand::initialize() {}
    void EStopCommand::execute() {
        if (Robot::getStopSignal()) {
            leftMotor->setSpeedAndDir(0, false, true);
            rightMotor->setSpeedAndDir(0, false, true);
            liftMotor->setSpeedAndDir(0, false, true);
            turnMotor->setSpeedAndDir(0, false, true);
            std::cout << "EStop titanButton is pushed!" << std::endl;
            isFinished_ = true;
        }
    }
    void EStopCommand::end() {
        leftMotor->setSpeedAndDir(0, false, true);
        rightMotor->setSpeedAndDir(0, false, true);
        liftMotor->setSpeedAndDir(0, false, true);
        turnMotor->setSpeedAndDir(0, false, true);
        std::cout << "EStopCommand done!" << std::endl;
    }
    ICommand::ptr EStopCommand::create() {
        return std::make_shared<EStopCommand>()->withTimer(100);
    }

//开始按钮
    void StartCommand::initialize() {
        isFinished_ = false;
        Robot::getInstance().setRightMotorSpeed(0);
        Robot::getInstance().setLeftMotorSpeed(0);
        titanButton->setEnable();
    }
    void StartCommand::execute() {
        titanButton->read();
        std::cout << "Start: " << titanButton->get(2) << std::endl;
        if (titanButton->get(2)) {
            isFinished_ = false;
        } else {
            isFinished_ = true;
        }
    }
    void StartCommand::end() {
        sleep(5);
        std::cout << "ICommand Start!!!" << std::endl;
    }
    ICommand::ptr StartCommand::create() {
        return std::make_shared<StartCommand>()->withTimer(100);
    }
}


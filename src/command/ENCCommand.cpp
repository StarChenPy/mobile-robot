/**
 * @file ENCCommand.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "command/ENCComand.h"
#include "system/Robot.h"

readLeftENCCommand::readLeftENCCommand() { is_finished = false; }
void readLeftENCCommand::initialize() { is_finished = false; }
void readLeftENCCommand::execute() { LeftENC->read(); }
void readLeftENCCommand::end() { is_finished = true; }
bool readLeftENCCommand::isFinished() { return is_finished || Robot::GetInstance().getStopSignal(); }

readRightENCCommand::readRightENCCommand() { is_finished = false; }
void readRightENCCommand::initialize() { is_finished = false; }
void readRightENCCommand::execute() { RightENC->read(); }
void readRightENCCommand::end() { is_finished = true; }
bool readRightENCCommand::isFinished() { return is_finished || Robot::GetInstance().getStopSignal(); }

readTurnENCCommand::readTurnENCCommand() { is_finished = false; }
void readTurnENCCommand::initialize() { is_finished = false; }
void readTurnENCCommand::execute() {
    TurnENC->read();
    // std::cout << "TurnENC: " << TurnENC->get() << std::endl;
}
void readTurnENCCommand::end() { is_finished = true; }
bool readTurnENCCommand::isFinished() { return is_finished || Robot::GetInstance().getStopSignal(); }

readLiftENCCommand::readLiftENCCommand() { is_finished = false; }
void readLiftENCCommand::initialize() { is_finished = false; }
void readLiftENCCommand::execute() { LiftENC->read(); }
void readLiftENCCommand::end() { is_finished = true; }
bool readLiftENCCommand::isFinished() { return is_finished || Robot::GetInstance().getStopSignal(); }

readOdomENCCommand::readOdomENCCommand() { is_finished = false; }
void readOdomENCCommand::initialize() { is_finished = false; }
void readOdomENCCommand::execute() {
    LeftENC->read();
    RightENC->read();
}
void readOdomENCCommand::end() { is_finished = true; }
bool readOdomENCCommand::isFinished() { return is_finished || Robot::GetInstance().getStopSignal(); }

readAllENCCommand::readAllENCCommand() { is_finished = false; }
void readAllENCCommand::initialize() { is_finished = false; }
void readAllENCCommand::execute() {
    LeftENC->read();
    RightENC->read();
    TurnENC->read();
    LiftENC->read();
}
void readAllENCCommand::end() { is_finished = true; }
bool readAllENCCommand::isFinished() { return is_finished || Robot::GetInstance().getStopSignal(); }

resetLeftENCCommand::resetLeftENCCommand() { is_finished = false; }
void resetLeftENCCommand::initialize() { is_finished = false; }
void resetLeftENCCommand::execute() {
    LeftENC->reset();
    is_finished = true;
}
void resetLeftENCCommand::end() { is_finished = true; }
bool resetLeftENCCommand::isFinished() { return is_finished || Robot::GetInstance().getStopSignal(); }

resetRightENCCommand::resetRightENCCommand() { is_finished = false; }
void resetRightENCCommand::initialize() { is_finished = false; }
void resetRightENCCommand::execute() {
    RightENC->reset();
    is_finished = true;
}
void resetRightENCCommand::end() { is_finished = true; }
bool resetRightENCCommand::isFinished() { return is_finished || Robot::GetInstance().getStopSignal(); }

resetTurnENCCommand::resetTurnENCCommand() { is_finished = false; }
void resetTurnENCCommand::initialize() { is_finished = false; }
void resetTurnENCCommand::execute() {
    TurnENC->reset();
    is_finished = true;
}
void resetTurnENCCommand::end() { is_finished = true; }
bool resetTurnENCCommand::isFinished() { return is_finished || Robot::GetInstance().getStopSignal(); }

resetLiftENCCommand::resetLiftENCCommand() { is_finished = false; }
void resetLiftENCCommand::initialize() { is_finished = false; }
void resetLiftENCCommand::execute() {
    LiftENC->reset();
    is_finished = true;
}
void resetLiftENCCommand::end() { is_finished = true; }
bool resetLiftENCCommand::isFinished() { return is_finished || Robot::GetInstance().getStopSignal(); }

resetOdomENCCommand::resetOdomENCCommand() { is_finished = false; }
void resetOdomENCCommand::initialize() { is_finished = false; }
void resetOdomENCCommand::execute() {
    LeftENC->reset();
    RightENC->reset();
    is_finished = true;
}
void resetOdomENCCommand::end() { is_finished = true; }
bool resetOdomENCCommand::isFinished() { return is_finished || Robot::GetInstance().getStopSignal(); }

resetAllENCCommand::resetAllENCCommand() { is_finished = false; }
void resetAllENCCommand::initialize() { is_finished = false; }
void resetAllENCCommand::execute() {
    LeftENC->reset();
    RightENC->reset();
    TurnENC->reset();
    LiftENC->reset();
    is_finished = true;
}
void resetAllENCCommand::end() { is_finished = true; }
bool resetAllENCCommand::isFinished() { return is_finished || Robot::GetInstance().getStopSignal(); }

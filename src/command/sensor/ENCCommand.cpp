#include "command/sensor/ENCComand.h"

void readLeftENCCommand::execute() {
    leftEnc->read();
    isFinished_ = true;
}

void readRightENCCommand::execute() {
    rightEnc->read();
    isFinished_ = true;
}

void readTurnENCCommand::execute() {
    turnEnc->read();
    isFinished_ = true;
}

void readLiftENCCommand::execute() {
    liftEnc->read();
    isFinished_ = true;
}

void readOdomENCCommand::execute() {
    leftEnc->read();
    rightEnc->read();
    isFinished_ = true;
}

void readAllENCCommand::execute() {
    leftEnc->read();
    rightEnc->read();
    turnEnc->read();
    liftEnc->read();
    isFinished_ = true;
}

void resetLeftENCCommand::execute() {
    leftEnc->reset();
    isFinished_ = true;
}

void resetRightENCCommand::execute() {
    rightEnc->reset();
    isFinished_ = true;
}

void resetTurnENCCommand::execute() {
    turnEnc->reset();
    isFinished_ = true;
}

void resetLiftENCCommand::execute() {
    liftEnc->reset();
    isFinished_ = true;
}

void resetOdomENCCommand::execute() {
    leftEnc->reset();
    rightEnc->reset();
    isFinished_ = true;
}

void resetAllENCCommand::execute() {
    leftEnc->reset();
    rightEnc->reset();
    turnEnc->reset();
    liftEnc->reset();
    isFinished_ = true;
}

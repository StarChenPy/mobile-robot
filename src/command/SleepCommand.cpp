#include "command/SleepCommand.h"

namespace robot {
SleepCommand::SleepCommand(int sleepTime) {
    this->sleepTime = sleepTime;
}

void SleepCommand::initialize() {
    isScheduled = false;
}
void SleepCommand::execute() {
    sleep(sleepTime);
    isScheduled = true;
}
bool SleepCommand::isFinished() {
    return isScheduled;
}

Command::ptr SleepCommand::create(int sleepTime) {
    return std::make_shared<SleepCommand>(sleepTime)->withTimer(20);
}
}

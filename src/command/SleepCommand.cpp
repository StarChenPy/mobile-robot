#include "command/SleepCommand.h"

void robot::SleepCommand::execute() {
    if (count++ > 0) {
        isFinished_ = true;
    }
}

robot::ICommand::ptr robot::SleepCommand::create(int time) {
    return std::make_shared<SleepCommand>()->withTimer(time);
}

#include "command/MotorPIDCommand.h"
#include "command/SleepCommand.h"

int main() {
    Scheduler::GetInstance(2, false).start();

    SequentialCommandGroup::Ptr sequential = createSequentialCommandGroup();
    sequential->addCommand(SleepCommand::create(12));

    Scheduler::GetInstance(2, false).stop();
}
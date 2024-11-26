#include "command/MotorPIDCommand.h"

int main() {
    Scheduler::GetInstance(2, false).start();

    SequentialCommandGroup::Ptr sequential = createSequentialCommandGroup();


    Scheduler::GetInstance(2, false).stop();
}
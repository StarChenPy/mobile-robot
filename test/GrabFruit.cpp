#include "command/group/ParallelRaceGroup.h"
#include "command/sensor/UpdateOdomCommand.h"
#include "command/motor/MotorPIDCommand.h"
#include "util/RoboticArmFun.h"
#include "util/Log.h"

int main(int argc, char *argv[]) {
    Log::init(argv[0]);
    Scheduler::getInstance(2, false).start();

    ICommandGroup::ptr init = ParallelCommandGroup::create();

    init->addCommand(
            UpdateOdomCommand::create(),
            LeftMotorPIDCommand::create(),
            RightMotorPIDCommand::create(),
            LiftMotorPIDCommand::create(),
            TurnMotorPIDCommand::create());

    ICommandGroup::ptr task = SequentialCommandGroup::create();
    task->addCommand(resetLiftAndTurn());

    ICommandGroup::ptr root = ParallelRaceGroup::create();
    root->addCommand(init, task);

    root->schedule();

    Scheduler::getInstance().stop();
    Log::shutdown();
}
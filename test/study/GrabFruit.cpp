#include "command/group/ParallelRaceGroup.h"
#include "command/sensor/UpdateOdomCommand.h"
#include "command/motor/MotorDistancePIDCommand.h"
#include "command/motor/MotorPIDCommand.h"
#include "util/Log.h"

int main(int argc, char *argv[]) {
    Log::init(argv[0]);
    Scheduler::getInstance(2, false).start();
    LOG(INFO) << "调度器已启动";

    robot::ICommandGroup::ptr sequential = make_shared<ParallelRaceGroup>();
    sequential->addCommand(UpdateOdomCommand::create(),
                           LiftMotorPIDCommand::create(),
                           ResetLiftMotorCommand::create(10));
    LOG(INFO) << "命令组已启动";
    sequential->schedule();

    Scheduler::getInstance(2, false).stop();
    LOG(INFO) << "调度器已结束";
    Log::shutdown();
}
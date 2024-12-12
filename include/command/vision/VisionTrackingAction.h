#include "VisionCommand.h"
#include "command/lidar/LidarAlongWallCommand.h"
#include "command/lidar/LidarCalibCommand.h"
#include "command/lidar/LidarReadCommand.h"
#include "util/RoboticArmFun.h"
#include "command/RotateCommand.h"
#include "command/ServoCommand.h"
#include "command/TrackingPointCommand.h"
#include "command/sensor/UpdateOdomCommand.h"
#include "VisionCtrlCommand.h"
#include "command/sensor/ZeroOdomCommand.h"
#include "command/motor/MotorPIDCommand.h"
#include "system/RobotCfg.h"

namespace robot {
    //视觉第一三列校准
    ICommand::ptr VisionCalibAction();
    //视觉第二列校准
    ICommand::ptr VisionCalibAction_2();
    ICommand::ptr VisionPick_S(int fruit);
    ICommand::ptr IdentifyTracking_R(int column, int fruit);
    ICommand::ptr TrackingIdentifyPick_S(Pose pose, int fruit, int column);
    ICommand::ptr TrackingIdentifyPick_DG(Pose pose, int fruit, int column);


    class VisionTrackingTackCommand : public ICommand {
    public:
        VisionTrackingTackCommand(Pose pose, int fruit, int col);

        void execute() override;
        void end() override;

    private:
        ICommand::ptr action_;
        Pose endPose_;
        int pickFruit_;
        int column_;
    };

    class isReachPointCommand : public ICommand {
    public:
        explicit isReachPointCommand(Pose pose);

        void execute() override;
        void end() override;

    private:
        Pose endPose_;
        double dMin;
    };
} // namespace robot

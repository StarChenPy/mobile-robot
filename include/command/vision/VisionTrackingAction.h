#include "VisionCommand.h"
#include "command/lidar/LidarAlongWallCommand.h"
#include "command/lidar/LidarCalibCommand.h"
#include "command/lidar/LidarReadCommand.h"
#include "command/RoboticArmFun.h"
#include "command/RotateCommand.h"
#include "command/ServoCommand.h"
#include "command/TrackingPointCommand.h"
#include "command/UpdateOdomCommand.h"
#include "VisionCtrlCommand.h"
#include "command/ZeroOdomCommand.h"
#include "command/motor/MotorPIDCommand.h"
#include "util/RobotCfg.h"

namespace robot {
    //视觉第一三列校准
    Command::ptr VisionCalibAction();

    //视觉第二列校准
    Command::ptr VisionCalibAction_2();

    Command::ptr VisionPick_S(int fruit);

    Command::ptr IdentifyTracking_R(int column, int fruit);

    Command::ptr TrackingIdentifyPick_S(Pose pose, int fruit, int column);

    Command::ptr TrackingIdentifyPick_DG(Pose pose, int fruit, int column);


    class VisionTrackingTackCommand : public CommandBase {
    public:
        typedef std::shared_ptr<VisionTrackingTackCommand> ptr;

        VisionTrackingTackCommand(Pose pose, int fruit, int col);

        void execute() override;

        void end() override;

    private:
        Command::ptr action_;
        Pose endPose_;
        int pickFruit_;
        int column_;
    };

    class isReachPointCommand : public CommandBase {
    public:
        typedef std::shared_ptr<isReachPointCommand> ptr;

        explicit isReachPointCommand(Pose pose);

        void execute() override;
        void end() override;

    private:
        Pose endPose_;
        double dMin;
    };
} // namespace robot

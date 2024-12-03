#pragma once
#include "system/Robot.h"
#include "util/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

namespace robot {
    class AlongWallCommandBase : public ICommand {
    public:
        AlongWallCommandBase(double distanceFromWall, double speed) : distanceFromWall_(distanceFromWall), speed_(speed) {}
        ~AlongWallCommandBase() override = default;

        void initialize() override;
        void execute() override;
        void end() override;

    protected:
        virtual bool executeWallTask(double speed, const std::vector<LidarData>& lidar_data, double distanceFromWall) = 0;

    private:
        double distanceFromWall_;
        double speed_;
    };

    class AlongRightWallCommand : public AlongWallCommandBase {
    public:
        typedef std::shared_ptr<AlongRightWallCommand> ptr;
        /**
         * 以一定速度沿着右墙行走
         * @param distanceFromWall 离墙距离
         * @param speed 行走速度
         */
        AlongRightWallCommand(double distanceFromWall, double speed);
        ~AlongRightWallCommand() override = default;

        bool executeWallTask(double speed, const std::vector<LidarData> &lidar_data, double distanceFromWall) override;

        static ICommand::ptr create(double distanceFromWall, double speed = 5);
    private:
        double distanceFromWall_ = 0;
        double speed_ = 0;
    };

    class AlongLeftWallCommand : public AlongWallCommandBase {
    public:
        typedef std::shared_ptr<AlongLeftWallCommand> ptr;
        /**
         * 以一定速度沿着左墙行走
         * @param distanceFromWall 离墙距离
         * @param speed 行走速度
         */
        explicit AlongLeftWallCommand(double distanceFromWall, double speed);
        ~AlongLeftWallCommand() override = default;

        bool executeWallTask(double speed, const std::vector<LidarData> &lidar_data, double distanceFromWall) override;

        static ICommand::ptr create(double distanceFromWall, double speed = 5);
    private:
        double distanceFromWall_ = 0;
        double speed_ = 0;
    };

    class IsReachXCommand : public ICommand {
    public:
        typedef std::shared_ptr<IsReachXCommand> ptr;

        explicit IsReachXCommand(Pose pose) : endPose_(pose) {}
        ~IsReachXCommand() override = default;

        void initialize() override;
        void execute() override;
        void end() override;

        static ICommand::ptr create(Pose pose);
    private:
        Pose endPose_;
        double errorRange_ = 4;
    };

    template <typename AlongWallCommandType>
    ICommand::ptr moveAlongWall(Pose pose, double d_wall, double speed);
} // namespace robot

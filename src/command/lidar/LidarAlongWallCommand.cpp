#include "command/lidar/LidarAlongWallCommand.h"
#include "command/lidar/LidarReadCommand.h"

namespace robot {
    void AlongWallCommandBase::initialize() {
        Robot::getInstance().setRightMotorSpeed(0);
        Robot::getInstance().setLeftMotorSpeed(0);
        isFinished_ = false;
    }

    void AlongWallCommandBase::execute() {
        std::vector<LidarData> lidar_data;
        Robot::getInstance().lidar_read->getLidarData(lidar_data);
        if (!lidar_data.empty()) {
            isFinished_ = executeWallTask(speed_, lidar_data, distanceFromWall_);
            double R_setpoint = Robot::getInstance().lidar_calib->get_R_setpoint();
            double L_setpoint = Robot::getInstance().lidar_calib->get_L_setpoint();

            Robot::getInstance().setRightMotorSpeed(isnan(R_setpoint) ? 0 : R_setpoint);
            Robot::getInstance().setLeftMotorSpeed(isnan(L_setpoint) ? 0 : L_setpoint);
        }
    }

    void AlongWallCommandBase::end() {
        std::cout << "AlongWallCommand end!" << std::endl;
        Robot::getInstance().setRightMotorSpeed(0);
        Robot::getInstance().setLeftMotorSpeed(0);
    }

    //右墙
    AlongRightWallCommand::AlongRightWallCommand(double distanceFromWall, double speed) : AlongWallCommandBase(
            distanceFromWall, speed) {}
    bool AlongRightWallCommand::executeWallTask(double speed, const vector<LidarData> &lidar_data, double distanceFromWall) {
        return Robot::getInstance().lidar_calib->LidarAlongRightWallTask(speed_, lidar_data, distanceFromWall_);
    }
    ICommand::ptr AlongRightWallCommand::create(double distanceFromWall, double speed) {
        return std::make_shared<AlongRightWallCommand>(distanceFromWall, speed);
    }

    //左墙
    AlongLeftWallCommand::AlongLeftWallCommand(double distanceFromWall, double speed) : AlongWallCommandBase(
            distanceFromWall, speed) {}
    bool AlongLeftWallCommand::executeWallTask(double speed, const vector<LidarData> &lidar_data, double distanceFromWall) {
        return Robot::getInstance().lidar_calib->LidarAlongLeftWallTask(speed_, lidar_data, distanceFromWall_);
    }
    ICommand::ptr AlongLeftWallCommand::create(double distanceFromWall, double speed) {
        return std::make_shared<AlongLeftWallCommand>(distanceFromWall, speed);
    }

    void IsReachXCommand::initialize() {
        isFinished_ = false;
    }
    void IsReachXCommand::execute() {
        Pose cur = Robot::getInstance().odom->getPose();
        double dx = std::fabs(cur.x_ - endPose_.x_);
        if (dx < errorRange_) {
            isFinished_ = true;
        } else {
            isFinished_ = false;
        }
    }
    void IsReachXCommand::end() {
        Robot::getInstance().setRightMotorSpeed(0);
        Robot::getInstance().setLeftMotorSpeed(0);
        std::cout << "isReachPointCommand end!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }

    ICommand::ptr IsReachXCommand::create(Pose pose) {
        return std::make_shared<IsReachXCommand>(pose)->withTimer(100);
    }

    //沿墙移动
    template <typename AlongWallCommandType>
    ICommand::ptr moveAlongWall(Pose pose, double d_wall, double speed) {
        auto RG = ParallelRaceGroup::create();

        auto DG = std::make_shared<ParallelDeadlineGroup>();
        DG->addCommand(LidarReadCommand::create());
        DG->setDeadlineCommand(AlongWallCommandType::create(d_wall, speed));

        RG->addCommand(DG, IsReachXCommand::create(pose));
        return RG;
    }
} // namespace robot

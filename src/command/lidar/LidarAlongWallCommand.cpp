#include "command/lidar/LidarAlongWallCommand.h"
#include "command/lidar/LidarReadCommand.h"

//右墙
void AlongRightWallCommand::initialize() {
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
    // sleep(1);
    is_finished = false;
}
void AlongRightWallCommand::execute() {
    std::vector<LidarData> lidar_data;
    Robot::getInstance().lidar_read->getLidarData(lidar_data);
    if (!lidar_data.empty()) {
        is_finished = Robot::getInstance().lidar_calib->LidarAlongRightWallTask(speed, lidar_data, calib_d);
        double R_setpoint = Robot::getInstance().lidar_calib->get_R_setpoint();
        double L_setpoint = Robot::getInstance().lidar_calib->get_L_setpoint();

        if (isnan(R_setpoint))
            R_setpoint = 0;
        if (isnan(L_setpoint))
            L_setpoint = 0;
        Robot::getInstance().setRightMotorSpeed(R_setpoint);
        Robot::getInstance().setLeftMotorSpeed(L_setpoint);
    }

    int time = robot::getCurrentMs();
    m_last_time = time;
}
void AlongRightWallCommand::end() {
    std::cout << "AlongRightWallCommand end!" << std::endl;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
}
bool AlongRightWallCommand::isFinished() {
    if (Robot::getInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getInstance().getStopSignal();
}

//左墙
void AlongLeftWallCommand::initialize() {
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
    // sleep(1);
    is_finished = false;
}
void AlongLeftWallCommand::execute() {
    std::vector<LidarData> lidar_data;
    Robot::getInstance().lidar_read->getLidarData(lidar_data);
    if (!lidar_data.empty()) {
        is_finished = Robot::getInstance().lidar_calib->LidarAlongLeftWallTask(speed, lidar_data, calib_d);
        // Robot::getInstance().lidar_calib->LidarAlongLeftWallTask(speed,
        // lidar_data, calib_d);
        double R_setpoint = Robot::getInstance().lidar_calib->get_R_setpoint();
        double L_setpoint = Robot::getInstance().lidar_calib->get_L_setpoint();

        if (isnan(R_setpoint))
            R_setpoint = 0;
        if (isnan(L_setpoint))
            L_setpoint = 0;
        Robot::getInstance().setRightMotorSpeed(R_setpoint);
        Robot::getInstance().setLeftMotorSpeed(L_setpoint);
    }

    int time = robot::getCurrentMs();
    m_last_time = time;
}
void AlongLeftWallCommand::end() {
    std::cout << "AlongLeftWallCommand end!" << std::endl;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
}
bool AlongLeftWallCommand::isFinished() {
    if (Robot::getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getStopSignal();
}

//
void isReachXCommand::initialize() { is_finished = false; }
void isReachXCommand::execute() {
    Pose cur = Robot::getInstance().odom->getPose();
    double dx = std::fabs(cur.x_ - EndPose.x_);
    if (dx < d_min) {
        is_finished = true;
    } else {
        is_finished = false;
    }
}
void isReachXCommand::end() {
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
    std::cout << "isReachPointCommand end!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
}
bool isReachXCommand::isFinished() { return is_finished; }

Command::ptr createLidarAlongLeftWallCommand(double speed, double d) {
    return std::make_shared<AlongLeftWallCommand>(speed, d)->withTimer(100);
}

Command::ptr LidarReadAlongLeftWallCommandDG(double speed, double d) {
    ParallelDeadlineGroup::ptr DG = std::make_shared<ParallelDeadlineGroup>();
    DG->addCommand(std::make_shared<LidarReadCommand>()->withTimer(100));
    DG->setDeadlineCommand(std::make_shared<AlongLeftWallCommand>(speed, d)->withTimer(100));
    return DG;
}

Command::ptr createLidarAlongRightWallCommand(double speed, double d) {
    return std::make_shared<AlongRightWallCommand>(speed, d)->withTimer(100);
}

Command::ptr LidarReadAlongRightWallCommandDG(double speed, double d) {
    ParallelDeadlineGroup::ptr DG = std::make_shared<ParallelDeadlineGroup>();
    DG->addCommand(std::make_shared<LidarReadCommand>()->withTimer(100));
    DG->setDeadlineCommand(std::make_shared<AlongRightWallCommand>(speed, d)->withTimer(100));
    return DG;
}

//沿墙移动
Command::ptr MoveAlongRightWallRG(Pose pose, double d_wall) {
    ParallelRaceGroup::ptr RG = std::make_shared<ParallelRaceGroup>();
    auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
    RG->addCommand(LidarReadAlongRightWallCommandDG(20, d_wall), isReachPoint);
    return RG;
}
Command::ptr MoveAlongLeftWallRG(Pose pose, double d_wall) {
    ParallelRaceGroup::ptr RG = std::make_shared<ParallelRaceGroup>();
    auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
    RG->addCommand(LidarReadAlongLeftWallCommandDG(20, d_wall), isReachPoint);
    return RG;
}
Command::ptr MoveAlongRightWallRG(Pose pose, double d_wall, double speed) {
    ParallelRaceGroup::ptr RG = std::make_shared<ParallelRaceGroup>();
    auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
    RG->addCommand(LidarReadAlongRightWallCommandDG(speed, d_wall), isReachPoint);
    return RG;
}
Command::ptr MoveAlongLeftWallRG(Pose pose, double d_wall, double speed) {
    ParallelRaceGroup::ptr RG = std::make_shared<ParallelRaceGroup>();
    auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
    RG->addCommand(LidarReadAlongLeftWallCommandDG(speed, d_wall), isReachPoint);
    return RG;
}

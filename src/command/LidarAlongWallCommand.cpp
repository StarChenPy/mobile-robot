#include "command/LidarAlongWallCommand.h"
#include "command/LidarReadCommand.h"

//右墙
void AlongRightWallCommand::initialize() {
    uint8_t updata_status = COMMEND_WAIT;
    LABVIEW::AlongRightWallStatusShareAddress->write(updata_status);

    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);
    // sleep(1);
    is_finished = false;
}
void AlongRightWallCommand::execute() {
    std::vector<LidarData> lidar_data;
    Robot::GetInstance().lidar_read->getLidarData(lidar_data);
    if (!lidar_data.empty()) {
        is_finished = Robot::GetInstance().lidar_calib->LidarAlongRightWallTask(speed, lidar_data, calib_d);
        // Robot::instance().lidar_calib->LidarAlongRightWallTask(speed,
        // lidar_data, calib_d);
        double R_setpoint = Robot::GetInstance().lidar_calib->get_R_setpoint();
        double L_setpoint = Robot::GetInstance().lidar_calib->get_L_setpoint();

        if (isnan(R_setpoint))
            R_setpoint = 0;
        if (isnan(L_setpoint))
            L_setpoint = 0;
        Robot::GetInstance().setRightMotorSpeed(R_setpoint);
        Robot::GetInstance().setLeftMotorSpeed(L_setpoint);
    }

    int time = robot::getCurrentMs();
    int dt = time - m_last_time;
    m_last_time = time;
    // std::cout << "AlongRightWallCommand execute dt = " << dt << std::endl;
    // is_finished = true;
}
void AlongRightWallCommand::end() {
    std::cout << "AlongRightWallCommand end!" << std::endl;
    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);

    uint8_t updata_status = COMMEND_END;
    LABVIEW::AlongRightWallStatusShareAddress->write(updata_status);
}
bool AlongRightWallCommand::isFinished() {
    uint8_t command_status;
    LABVIEW::AlongRightWallStatusShareAddress->read(command_status);
    if (command_status == COMMEND_CANCEL) {
        is_finished = true;
    }
    if (Robot::GetInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::GetInstance().getStopSignal();
}

//左墙
void AlongLeftWallCommand::initialize() {
    uint8_t updata_status = COMMEND_WAIT;
    LABVIEW::AlongLeftWallStatusShareAddress->write(updata_status);

    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);
    // sleep(1);
    is_finished = false;
}
void AlongLeftWallCommand::execute() {
    std::vector<LidarData> lidar_data;
    Robot::GetInstance().lidar_read->getLidarData(lidar_data);
    if (!lidar_data.empty()) {
        is_finished = Robot::GetInstance().lidar_calib->LidarAlongLeftWallTask(speed, lidar_data, calib_d);
        // Robot::instance().lidar_calib->LidarAlongLeftWallTask(speed,
        // lidar_data, calib_d);
        double R_setpoint = Robot::GetInstance().lidar_calib->get_R_setpoint();
        double L_setpoint = Robot::GetInstance().lidar_calib->get_L_setpoint();

        if (isnan(R_setpoint))
            R_setpoint = 0;
        if (isnan(L_setpoint))
            L_setpoint = 0;
        Robot::GetInstance().setRightMotorSpeed(R_setpoint);
        Robot::GetInstance().setLeftMotorSpeed(L_setpoint);
    }

    int time = robot::getCurrentMs();
    int dt = time - m_last_time;
    m_last_time = time;
    // std::cout << "AlongLeftWallCommand execute dt = " << dt << std::endl;
    // is_finished = true;
}
void AlongLeftWallCommand::end() {
    std::cout << "AlongLeftWallCommand end!" << std::endl;
    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);

    uint8_t updata_status = COMMEND_END;
    LABVIEW::AlongLeftWallStatusShareAddress->write(updata_status);
}
bool AlongLeftWallCommand::isFinished() {
    uint8_t command_status;
    LABVIEW::AlongLeftWallStatusShareAddress->read(command_status);
    if (command_status == COMMEND_CANCEL) {
        is_finished = true;
    }
    if (Robot::GetInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::GetInstance().getStopSignal();
}

//
void isReachXCommand::initialize() { is_finished = false; }
void isReachXCommand::execute() {
    Pose cur = Robot::GetInstance().odom->getPose();
    double dx = std::fabs(cur.x_ - EndPose.x_);
    if (dx < d_min) {
        is_finished = true;
    } else {
        is_finished = false;
    }
}
void isReachXCommand::end() {
    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);
    std::cout << "isReachPointCommand end!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
}
bool isReachXCommand::isFinished() { return is_finished; }

Command::ptr createLidarAlongLeftWallCommand(double speed, double d) {
    return std::make_shared<AlongLeftWallCommand>(speed, d)->withTimer(100);
}

Command::ptr LidarReadAlongLeftWallCommandDG(double speed, double d) {
    ParallelDeadlineGroup::ptr DG = std::make_shared<ParallelDeadlineGroup>();
    DG->addCommands(std::make_shared<LidarReadCommand>()->withTimer(100));
    DG->setDeadlineCommand(std::make_shared<AlongLeftWallCommand>(speed, d)->withTimer(100));
    return DG;
}

Command::ptr createLidarAlongRightWallCommand(double speed, double d) {
    return std::make_shared<AlongRightWallCommand>(speed, d)->withTimer(100);
}

Command::ptr LidarReadAlongRightWallCommandDG(double speed, double d) {
    ParallelDeadlineGroup::ptr DG = std::make_shared<ParallelDeadlineGroup>();
    DG->addCommands(std::make_shared<LidarReadCommand>()->withTimer(100));
    DG->setDeadlineCommand(std::make_shared<AlongRightWallCommand>(speed, d)->withTimer(100));
    return DG;
}

//沿墙移动
Command::ptr MoveAlongRightWallRG(Pose pose, double d_wall) {
    ParallelRaceGroup::ptr RG = std::make_shared<ParallelRaceGroup>();
    auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
    RG->AddCommands(LidarReadAlongRightWallCommandDG(20, d_wall), isReachPoint);
    return RG;
}
Command::ptr MoveAlongLeftWallRG(Pose pose, double d_wall) {
    ParallelRaceGroup::ptr RG = std::make_shared<ParallelRaceGroup>();
    auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
    RG->AddCommands(LidarReadAlongLeftWallCommandDG(20, d_wall), isReachPoint);
    return RG;
}
Command::ptr MoveAlongRightWallRG(Pose pose, double d_wall, double speed) {
    ParallelRaceGroup::ptr RG = std::make_shared<ParallelRaceGroup>();
    auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
    RG->AddCommands(LidarReadAlongRightWallCommandDG(speed, d_wall), isReachPoint);
    return RG;
}
Command::ptr MoveAlongLeftWallRG(Pose pose, double d_wall, double speed) {
    ParallelRaceGroup::ptr RG = std::make_shared<ParallelRaceGroup>();
    auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
    RG->AddCommands(LidarReadAlongLeftWallCommandDG(speed, d_wall), isReachPoint);
    return RG;
}

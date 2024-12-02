#include "command/lidar/LidarCalibCommand.h"
#include "command/lidar/LidarReadCommand.h"

void LidarCalibCommand::initialize() {
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
    // sleep(1);
    isFinished_ = false;
}
void LidarCalibCommand::execute() {
    std::vector<LidarData> lidar_data;
    Robot::getInstance().lidar_read->getLidarData(lidar_data);
    isFinished_ = Robot::getInstance().lidar_calib->LidarCalibTask(lidar_data, calib_d);
    double R_setpoint = Robot::getInstance().lidar_calib->get_R_setpoint();
    double L_setpoint = Robot::getInstance().lidar_calib->get_L_setpoint();
    std::cout << "R_setpoint = " << R_setpoint << " L_setpoint = " << L_setpoint << std::endl;

    if (isnan(R_setpoint))
        R_setpoint = 0;
    if (isnan(L_setpoint))
        L_setpoint = 0;
    Robot::getInstance().setRightMotorSpeed(R_setpoint);
    Robot::getInstance().setLeftMotorSpeed(L_setpoint);

    int time = robot::getCurrentMs();
    lastTime_ = time;
}
void LidarCalibCommand::end() {
    std::cout << "LidarCalibCommand end!" << std::endl;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
}
bool LidarCalibCommand::isFinished() {
    if (Robot::getStopSignal()) {
        stopAll();
    }
    return isFinished_ || Robot::getStopSignal();
}

Command::ptr createLidarCalibCommand(double d) { return std::make_shared<LidarCalibCommand>(d)->withTimer(100); }

Command::ptr LidarReadCalibDG(double d) {
    ParallelDeadlineGroup::ptr DG = std::make_shared<ParallelDeadlineGroup>();
    DG->addCommand(std::make_shared<LidarReadCommand>()->withTimer(100));
    DG->setDeadlineCommand(std::make_shared<LidarCalibCommand>(d)->withTimer(100));
    return DG;
}
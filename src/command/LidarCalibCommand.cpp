#include "command/LidarCalibCommand.h"
#include "command/LidarReadCommand.h"

void LidarCalibCommand::initialize() {
    uint8_t updata_status = COMMEND_WAIT;
    LABVIEW::LidarCalibStatusShareAddress->write(updata_status);

    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);
    // sleep(1);
    isFinished_ = false;
}
void LidarCalibCommand::execute() {
    std::vector<LidarData> lidar_data;
    Robot::GetInstance().lidar_read->getLidarData(lidar_data);
    isFinished_ = Robot::GetInstance().lidar_calib->LidarCalibTask(lidar_data, calib_d);
    double R_setpoint = Robot::GetInstance().lidar_calib->get_R_setpoint();
    double L_setpoint = Robot::GetInstance().lidar_calib->get_L_setpoint();
    std::cout << "R_setpoint = " << R_setpoint << " L_setpoint = " << L_setpoint << std::endl;

    if (isnan(R_setpoint))
        R_setpoint = 0;
    if (isnan(L_setpoint))
        L_setpoint = 0;
    Robot::GetInstance().setRightMotorSpeed(R_setpoint);
    Robot::GetInstance().setLeftMotorSpeed(L_setpoint);

    int time = robot::getCurrentMs();
    int dt = time - lastTime_;
    lastTime_ = time;
    // std::cout << "LidarCalibCommand execute dt = " << dt << std::endl;
    // isFinished_ = true;
}
void LidarCalibCommand::end() {
    std::cout << "LidarCalibCommand end!" << std::endl;
    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);

    uint8_t updata_status = COMMEND_END;
    LABVIEW::LidarCalibStatusShareAddress->write(updata_status);
}
bool LidarCalibCommand::isFinished() {
    uint8_t command_status;
    LABVIEW::LidarCalibStatusShareAddress->read(command_status);
    if (command_status == COMMEND_CANCEL) {
        isFinished_ = true;
    }
    if (Robot::GetInstance().getStopSignal()) {
        stopAll();
    }
    return isFinished_ || Robot::GetInstance().getStopSignal();
}

Command::ptr createLidarCalibCommand(double d) { return std::make_shared<LidarCalibCommand>(d)->withTimer(100); }

Command::ptr LidarReadCalibDG(double d) {
    ParallelDeadlineGroup::ptr DG = std::make_shared<ParallelDeadlineGroup>();
    DG->addCommand(std::make_shared<LidarReadCommand>()->withTimer(100));
    DG->setDeadlineCommand(std::make_shared<LidarCalibCommand>(d)->withTimer(100));
    return DG;
}
#include "command/LidarCalibCommand.h"

#include "command/LidarReadCommand.h"

void LidarCalibCommand::initialize() {
  uint8_t updata_status = COMMEND_WAIT;
  LABVIEW::LidarCalibStatusShareAddress->write(updata_status);

  Robot::GetInstance().setRightMotorSpeed(0);
  Robot::GetInstance().setLeftMotorSpeed(0);
  // sleep(1);
  is_finished = false;
}
void LidarCalibCommand::execute() {
  std::vector<LidarData> lidar_data;
  Robot::GetInstance().lidar_read->getLidarData(lidar_data);
  is_finished = Robot::GetInstance().lidar_calib->LidarCalibTask(lidar_data, calib_d);
  double R_setpoint = Robot::GetInstance().lidar_calib->get_R_setpoint();
  double L_setpoint = Robot::GetInstance().lidar_calib->get_L_setpoint();
  std::cout << "R_setpoint = " << R_setpoint << " L_setpoint = " << L_setpoint << std::endl;

  if(isnan(R_setpoint)) R_setpoint = 0;
  if(isnan(L_setpoint)) L_setpoint = 0;
  Robot::GetInstance().setRightMotorSpeed(R_setpoint);
  Robot::GetInstance().setLeftMotorSpeed(L_setpoint);

  int time = RobotGenius::getCurrentMs();
  int dt = time - m_last_time;
  m_last_time = time;
  // std::cout << "LidarCalibCommand execute dt = " << dt << std::endl;
  // is_finished = true;
}
void LidarCalibCommand::end() {
  std::cout << "LidarCalibCommand end!" <<std::endl;
  Robot::GetInstance().setRightMotorSpeed(0);
  Robot::GetInstance().setLeftMotorSpeed(0);
  
  uint8_t updata_status = COMMEND_END;
  LABVIEW::LidarCalibStatusShareAddress->write(updata_status);
}
bool LidarCalibCommand::isFinished() {
  uint8_t command_status;
  LABVIEW::LidarCalibStatusShareAddress->read(command_status);
  if(command_status == COMMEND_CANCEL){
    is_finished = true;
  }
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}

Command::Ptr createLidarCalibCommand(double d){
  return std::make_shared<LidarCalibCommand>(d)->withTimer(100);
}

Command::Ptr LidarReadCalibDG(double d){
  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  DG->AddCommands(std::make_shared<LidarReadCommand>()->withTimer(100));
  DG->setDeadlineCommand(std::make_shared<LidarCalibCommand>(d)->withTimer(100));
  return DG;
}
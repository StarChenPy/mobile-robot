#include "command/ServoCommand.h"

//**********************************夹手*****************************************
void ClampServoCommand::initialize() {
  uint8_t updata_status = COMMEND_WAIT;
  LABVIEW::ClampServoStatusShareAddress->write(updata_status);

  is_finished = false;
}
void ClampServoCommand::execute() {
  int time = RobotGenius::getCurrentMs();
  int dt = time - m_last_time;
  m_last_time = time;
  
  ClampServo->setDutyCycle(ClampServo_val);
  counter++;
  if(counter > counter_limit ) {
    is_finished = true;
    counter = 0;
  }
  
  // std::cout << "ClampServoCommand execute dt = " << static_cast<double>(dt) << std::endl;
  // is_finished = true;
}
void ClampServoCommand::end() {
  std::cout << "ClampServoCommand end" <<std::endl;
  uint8_t updata_status = COMMEND_END;
  LABVIEW::ClampServoStatusShareAddress->write(updata_status);
}
bool ClampServoCommand::isFinished() {
  uint8_t command_status;
  LABVIEW::ClampServoStatusShareAddress->read(command_status);
  if(command_status == COMMEND_CANCEL){
    is_finished = true;
  }
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}

// 指令封装  len:打开宽度，单位cm
Command::Ptr createClampServoCommand(double len) {
  // if(len < CLAMP_LEN_MIN){
  //   len = CLAMP_LEN_MIN;
  //   std::cout << "开合舵机输入过小" << std::endl;
  // }else if(len > CLAMP_LEN_MAX){
  //   // len = CLAMP_LEN_MAX;
  //   std::cout << "开合舵机输入大于" << CLAMP_LEN_MAX << "cm" << std::endl;
  // }
  double target = CLAMP_LEN_A * len + CLAMP_LEN_B;
  // std::cout << "Clamp target = " << target << std::endl;
  return std::make_shared<ClampServoCommand>(target)->withTimer(100);
}
Command::Ptr createClampServoCommand(double len, double cnt_limit) {
  // if(len < CLAMP_LEN_MIN){
  //   len = CLAMP_LEN_MIN;
  //   std::cout << "开合舵机输入过小" << std::endl;
  // }else if(len > CLAMP_LEN_MAX){
  //   // len = CLAMP_LEN_MAX;
  //   std::cout << "开合舵机输入大于" << CLAMP_LEN_MAX << "cm" << std::endl;
  // }
  double target = CLAMP_LEN_A * len + CLAMP_LEN_B;
  // std::cout << "Clamp target = " << target << std::endl;
  return std::make_shared<ClampServoCommand>(target, cnt_limit)->withTimer(100);
}



//**********************************伸缩*****************************************
void TelescopicServoCommand::initialize() {
  uint8_t updata_status = COMMEND_WAIT;
  LABVIEW::TelescopicServoStatusShareAddress->write(updata_status);
  is_finished = false;
}
void TelescopicServoCommand::execute() {
  int time = RobotGenius::getCurrentMs();
  int dt = time - m_last_time;
  m_last_time = time;
  
  TelescopicServo->setDutyCycle(TelescopicServo_val);
  counter++;
  if(counter > counter_limit ) {
    is_finished = true;
    counter = 0;
  }
  
  // std::cout << "TelescopicServoCommand execute dt = " << static_cast<double>(dt) << std::endl;
  // is_finished = true;
}
void TelescopicServoCommand::end() {
  std::cout << "TelescopicServoCommand end" <<std::endl;
  uint8_t updata_status = COMMEND_END;
  LABVIEW::TelescopicServoStatusShareAddress->write(updata_status);
}
bool TelescopicServoCommand::isFinished() {
  uint8_t command_status;
  LABVIEW::TelescopicServoStatusShareAddress->read(command_status);
  if(command_status == COMMEND_CANCEL){
    is_finished = true;
  }
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}

// 指令封装  dis:伸缩距离，单位cm
Command::Ptr createcTelescopicServoCommand(double dis) {
  // if(dis < TELESCOPIC_DIS_MIN - 0.4){
  //   dis = TELESCOPIC_DIS_MIN - 0.4;
  //   std::cout << "伸缩舵机输入过小" << std::endl;
  // }else if(dis > TELESCOPIC_DIS_MAX){
  //   dis = TELESCOPIC_DIS_MAX;
  //   std::cout << "伸缩舵机输入过大" << std::endl;
  // }
  double target = TELESCOPIC_DIS_A * dis + TELESCOPIC_DIS_B;
  // std::cout << "Telescopic target = " << target << std::endl;
  return std::make_shared<TelescopicServoCommand>(target)->withTimer(100);
}
Command::Ptr createcTelescopicServoCommand(double dis, double cnt_limit) {
  // if(dis < TELESCOPIC_DIS_MIN){
  //   dis = TELESCOPIC_DIS_MIN;
  //   std::cout << "伸缩舵机输入过小" << std::endl;
  // }else if(dis > TELESCOPIC_DIS_MAX){
  //   dis = TELESCOPIC_DIS_MAX;
  //   std::cout << "伸缩舵机输入过大" << std::endl;
  // }
  double target = TELESCOPIC_DIS_A * dis + TELESCOPIC_DIS_B;
  // std::cout << "Telescopic target = " << target << std::endl;
  return std::make_shared<TelescopicServoCommand>(target, cnt_limit)->withTimer(100);
}



//**********************************摆手*****************************************
void RaiseServoCommand::initialize() {
  uint8_t updata_status = COMMEND_WAIT;
  LABVIEW::RaiseServoStatusShareAddress->write(updata_status);
  is_finished = false;
}
void RaiseServoCommand::execute() {
  int time = RobotGenius::getCurrentMs();
  int dt = time - m_last_time;
  m_last_time = time;
  
  RaiseServo->setDutyCycle(RaiseServo_val);
  counter++;
  if(counter > counter_limit ) {
    is_finished = true;
    counter = 0;
  }
  
  // std::cout << "RaiseServoCommand execute dt = " << static_cast<double>(dt) << std::endl;
  // is_finished = true;
}
void RaiseServoCommand::end() {
  std::cout << "RaiseServoCommand end" <<std::endl;
  uint8_t updata_status = COMMEND_END;
  LABVIEW::RaiseServoStatusShareAddress->write(updata_status);
}
bool RaiseServoCommand::isFinished() {
  uint8_t command_status;
  LABVIEW::RaiseServoStatusShareAddress->read(command_status);
  if(command_status == COMMEND_CANCEL){
    is_finished = true;
  }
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}

// 指令封装  angle:抬起角度，单位：度
Command::Ptr createRaiseServoCommand(double angle) {
  // if(angle < RAISE_ANGLE_MIN){
  //   angle = RAISE_ANGLE_MIN;
  //   std::cout << "抬手舵机输入过小" << std::endl;
  // }else if(angle > RAISE_ANGLE_MAX + 40){
  //   angle = RAISE_ANGLE_MAX + 40;
  //   std::cout << "抬手舵机输入过大" << std::endl;
  // }
  double target = RAISE_ANGLE_A * angle + RAISE_ANGLE_B;
  std::cout << "Raise target = " << target << std::endl;
  return std::make_shared<RaiseServoCommand>(target)->withTimer(100);
}
Command::Ptr createRaiseServoCommand(double angle, double cnt_limit) {
  // if(angle < RAISE_ANGLE_MIN){
  //   angle = RAISE_ANGLE_MIN;
  //   std::cout << "抬手舵机输入过小" << std::endl;
  // }else if(angle > RAISE_ANGLE_MAX + 40){
  //   angle = RAISE_ANGLE_MAX + 40;
  //   std::cout << "抬手舵机输入过大" << std::endl;
  // }
  double target = RAISE_ANGLE_A * angle + RAISE_ANGLE_B;
  // std::cout << "Raise target = " << target << std::endl;
  return std::make_shared<RaiseServoCommand>(target, cnt_limit)->withTimer(100);
}



//**********************************旋转*****************************************
void RotatingServoCommand::initialize() {
  uint8_t updata_status = COMMEND_WAIT;
  LABVIEW::RotatingServoStatusShareAddress->write(updata_status);
  is_finished = false;
}
void RotatingServoCommand::execute() {
  int time = RobotGenius::getCurrentMs();
  int dt = time - m_last_time;
  m_last_time = time;
  
  RotatingServo->setDutyCycle(RotatingServo_val);
  counter++;
  if(counter > counter_limit ) {
    is_finished = true;
    counter = 0;
  }
  
  // std::cout << "RotatingServoCommand execute dt = " << static_cast<double>(dt) << std::endl;
  // is_finished = true;
}
void RotatingServoCommand::end() {
  std::cout << "RotatingServoCommand end" <<std::endl;
  uint8_t updata_status = COMMEND_END;
  LABVIEW::RotatingServoStatusShareAddress->write(updata_status);
}
bool RotatingServoCommand::isFinished() {
  uint8_t command_status;
  LABVIEW::RotatingServoStatusShareAddress->read(command_status);
  if(command_status == COMMEND_CANCEL){
    is_finished = true;
  }
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}

// 指令封装  angle:角度，单位：度
Command::Ptr createRotatingServoCommand(double angle) {
  // if(angle < ROTATING_ANGLE_MIN){
  //   angle = ROTATING_ANGLE_MIN;
  //   std::cout << "旋转舵机输入过小" << std::endl;
  // }else if(angle > ROTATING_ANGLE_MAX){
  //   angle = ROTATING_ANGLE_MAX;
  //   std::cout << "旋转舵机输入过大" << std::endl;
  // }
  double target = ROTATING_ANGLE_A * angle + ROTATING_ANGLE_B;
  std::cout << "Rotating target = " << target << std::endl;
  return std::make_shared<RotatingServoCommand>(target)->withTimer(100);
  // return std::make_shared<RotatingServoCommand>(angle)->withTimer(100);
}
Command::Ptr createRotatingServoCommand(double angle, double cnt_limit) {
  // if(angle < ROTATING_ANGLE_MIN){
  //   angle = ROTATING_ANGLE_MIN;
  //   std::cout << "旋转舵机输入过小" << std::endl;
  // }else if(angle > ROTATING_ANGLE_MAX){
  //   angle = ROTATING_ANGLE_MAX;
  //   std::cout << "旋转舵机输入过大" << std::endl;
  // }
  double target = ROTATING_ANGLE_A * angle + ROTATING_ANGLE_B;
  // std::cout << "Rotating target = " << target << std::endl;
  return std::make_shared<RotatingServoCommand>(target, cnt_limit)->withTimer(100);
}
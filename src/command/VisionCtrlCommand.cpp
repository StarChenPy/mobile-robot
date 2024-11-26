#include "command/VisionCtrlCommand.h"


void VisionCtrlCommand::initialize() {
  is_finished = false;
  Robot::GetInstance().setRightMotorSpeed(0);
  Robot::GetInstance().setLeftMotorSpeed(0);
  Vision::GetInstance().clearBoxs();
  Vision::GetInstance().clearResult();
}
void VisionCtrlCommand::execute() {
  Vision::GetInstance().runMNN(true, "/home/pi/Pick/result.jpg");      //注意路径*********
  std::vector<BoxInfo> boxs = Vision::GetInstance().getBoxs();
  Vision::GetInstance().print();

  double R_setpoint = 0;
  double L_setpoint = 0;
  if(!boxs.empty()){
    int cap_cx = Vision::GetInstance().get_cap_cx();
    for(int i = 0; i < boxs.size(); i++){
      if(boxs[i].label == fruit_label){
        int fruit_cx = (boxs[i].x1 + boxs[i].x2) / 2;
        is_finished = Robot::GetInstance().chassis_ctrl->VisionCtrlTask(fruit_cx, cap_cx);
        // Robot::GetInstance().chassis_ctrl->VisionCtrlTask(fruit_cx, cap_cx);
        R_setpoint = Robot::GetInstance().chassis_ctrl->get_R_setpoint();
        L_setpoint = Robot::GetInstance().chassis_ctrl->get_L_setpoint();
        std::cout << "fruit class: " << Class_names[boxs[i].label] << std::endl;
        break;
      }
    }
  }else{
    std::cout << "水果丢失" << std::endl;
  }
  
  Robot::GetInstance().setRightMotorSpeed(R_setpoint);
  Robot::GetInstance().setLeftMotorSpeed(L_setpoint);

  int time = RobotGenius::getCurrentMs();
  int dt = time - m_last_time;
  m_last_time = time;
  // std::cout << "VisionCtrlCommand execute dt = " << dt << std::endl;
  // is_finished = true;
}
void VisionCtrlCommand::end() {
  std::cout << "VisionCtrlCommand end!" <<std::endl;
  Robot::GetInstance().setRightMotorSpeed(0);
  Robot::GetInstance().setLeftMotorSpeed(0);
}
bool VisionCtrlCommand::isFinished() {
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}

Command::Ptr createVisionCtrlCommand(int label){
  return std::make_shared<VisionCtrlCommand>(label)->withTimer(200);
}




void VisionIdentifyCommand::initialize() {
  is_finished = false;
  Vision::GetInstance().clearBoxs();
  Vision::GetInstance().clearResult();
}
void VisionIdentifyCommand::execute() {
  Vision::GetInstance().runMNN(true, "/home/pi/Pick/result.jpg");      //注意路径*********
  std::vector<BoxInfo> boxs = Vision::GetInstance().getBoxs();
  if(!boxs.empty()){
    Vision::GetInstance().getFruitXH(boxs);
  }
  Vision::GetInstance().print();

  if(!boxs.empty()){
    for(int i = 0; i < boxs.size(); i++){
      if(boxs[i].label == fruit_label){
        // int cx = (boxs[i].x1 + boxs[i].x2) / 2;
        // int cy = (boxs[i].y1 + boxs[i].y2) / 2;
        // cv::Point2f XH = Vision::GetInstance().getXH(cx, cy);
        // std::cout << "X: " << XH.x << " H: " << XH.y << std::endl;
        is_finished = true;
        break;
      }
    }
  }else{
    std::cout << "!!!No fruit: " << Class_names[fruit_label] << std::endl;
  }

  int time = RobotGenius::getCurrentMs();
  int dt = time - m_last_time;
  m_last_time = time;
  // std::cout << "VisionCtrlCommand execute dt = " << dt << std::endl;
  // is_finished = true;
}
void VisionIdentifyCommand::end() {
  std::cout << "VisionIdentifyCommand end!" <<std::endl;
}
bool VisionIdentifyCommand::isFinished() {
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}

Command::Ptr createVisionIdentifyCommand(int label){
  return std::make_shared<VisionIdentifyCommand>(label)->withTimer(200);
}


void VisionMoveCommand::initialize() {
  is_finished = false;
  InitPose = Robot::GetInstance().odom->getPose();
  Robot::GetInstance().setRightMotorSpeed(0);
  Robot::GetInstance().setLeftMotorSpeed(0);

  std::vector<BoxInfo> boxs = Vision::GetInstance().getBoxs();
  std::vector<cv::Point2f> FruitPoints = Vision::GetInstance().getFruitXH(boxs);
  Vision::GetInstance().print();

  double dx = 0;
  for(int i = 0; i < boxs.size(); i++){
    if(boxs[i].label == fruit_label){
      dx = FruitPoints[i].x;
      break;
    }
  }
  std::cout << "Fruit dx = " << dx << std::endl;
  target = {InitPose.x_ + dx, InitPose.y_, InitPose.theta_};
}
void VisionMoveCommand::execute() {
  Pose cur = Robot::GetInstance().odom->getPose();
  is_finished = Robot::GetInstance().chassis_ctrl->TrackingPointTask(target, cur);
  double R_setpoint = Robot::GetInstance().chassis_ctrl->get_R_setpoint();
  double L_setpoint = Robot::GetInstance().chassis_ctrl->get_L_setpoint();
  
  Robot::GetInstance().setRightMotorSpeed(R_setpoint);
  Robot::GetInstance().setLeftMotorSpeed(L_setpoint);

  int time = RobotGenius::getCurrentMs();
  int dt = time - m_last_time;
  m_last_time = time;
  // std::cout << "VisionCtrlCommand execute dt = " << dt << std::endl;
  // is_finished = true;
}
void VisionMoveCommand::end() {
  std::cout << "VisionMoveCommand end!" <<std::endl;
  Robot::GetInstance().setRightMotorSpeed(0);
  Robot::GetInstance().setLeftMotorSpeed(0);
}
bool VisionMoveCommand::isFinished() {
  if(Robot::GetInstance().getStopSignal()) {
    stopAll();
  }
  return is_finished || Robot::GetInstance().getStopSignal();
}

Command::Ptr createVisionMoveCommand(int label){
  return std::make_shared<VisionMoveCommand>(label)->withTimer(100);
}






void VisionHeightCtrlCommand::initialize() {
  is_finished = false;
  Robot::GetInstance().setLiftMotorSpeed(0);
  std::vector<BoxInfo> boxs = Vision::GetInstance().getBoxs();
  std::vector<cv::Point2f> FruitPoints = Vision::GetInstance().getFruitXH(boxs);
  Vision::GetInstance().print();

  double dy = 0;
  for(int i = 0; i < boxs.size(); i++){
    if(boxs[i].label == fruit_label){
      dy = FruitPoints[i].y;
      break;
    }
  }
  std::cout << "Fruit dy = " << dy << std::endl;
  
  double high = 10;
  double low = 4;
  if(dy > high){
    Height_target = -45;
  }else if(dy < high && dy > low){
    Height_target = -52;
  }else if(dy < low){
    Height_target = -58;
  }
  m_setpoint = static_cast<int32_t>(Height_target * (1000 / 10.7 / 2));
}
void VisionHeightCtrlCommand::execute() {
  Robot::GetInstance().LiftMotorDistancePID(m_setpoint);
  std::cout << "Lift ENC: " << LiftENC->get() << " Lift set_point_: " << m_setpoint << std::endl;

  int time = RobotGenius::getCurrentMs();
  int dt = time - m_last_time;
  m_last_time = time;
  // std::cout << "VisionHeightCtrlCommand execute dt = " << dt << std::endl;
  // is_finished = true;
}
void VisionHeightCtrlCommand::end() {
  std::cout << "VisionHeightCtrlCommand end!" <<std::endl;
  Robot::GetInstance().setLiftMotorSpeed(0);
}
bool VisionHeightCtrlCommand::isFinished() {
  if (abs(m_setpoint - LiftENC->get()) < LIFTMOTORDISTANCEERROR) {
    m_conter ++;
  } else {
    m_conter = 0;
  }
  return Robot::GetInstance().getStopSignal() || m_conter > LIFTMOTORDISTANCECounter;
}

Command::Ptr createVisionHeightCtrlCommand(int label){
  return std::make_shared<VisionHeightCtrlCommand>(label)->withTimer(100);
}
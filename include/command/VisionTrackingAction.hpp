#include "RobotCfg.h"
#include "command/TrackingPointCommand.h"
#include "command/Vision/VisionCommand.h"
#include "command/MotorPIDCommand.h"
#include "command/UpdataOdomCommand.h"
#include "command/RotateCommand.h"
#include "command/ZeroOdomCommand.h"
#include "command/VisionCtrlCommand.h"
#include "command/RoboticArmFun.hpp"
#include "command/LidarReadCommand.h"
#include "command/LidarCalibCommand.h"
#include "command/LidarAlongWallCommand.h"
#include "command/ServoCommand.h"

//视觉第一三列校准
Command::Ptr VisionCalibAction(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    createRotateCommand(90),
    createLidarCalibCommand(24),
    createZeroOdomCommand(),
    createTrackingPointCommand(Pose(13,0,0),10),
    createRotateCommand(-90),
    createLidarCalibCommand(22),
    createZeroOdomCommand()
  );
  return sequential;
}

//视觉第二列校准
Command::Ptr VisionCalibAction_2(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    createRotateCommand(-90),
    // LidarReadCalibDG(23),
    createLidarCalibCommand(23),
    // createZeroOdomCommand(),
    // createTrackingPointCommand(Pose(10,0,0),10),
    createRotateCommand(90),
    createLidarCalibCommand(22),
    // LidarReadCalibDG(22),
    createZeroOdomCommand()
  );
  return sequential;
}

Command::Ptr VisionPick_S(int fruit){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    createVisionIdentifyCommand(fruit),
    createVisionMoveCommand(fruit),
    // createVisionCtrlCommand(fruit),   //移动机器人到水果正前方位置
    createRaiseServoCommand(90, 5),
    createVisionHeightCtrlCommand(fruit),      //
    PickFruitSG()               //捉水果动作
  );
  return sequential;
}

Command::Ptr IdentifyTracking_R(int column, int fruit){
  Command::Ptr AlongWall;
  double d_wall = 0;
  if(column == 1){
    d_wall = 42;
    AlongWall = createLidarAlongRightWallCommand(5, d_wall);
  }else if(column == 2){
    d_wall = 23;
    AlongWall = createLidarAlongLeftWallCommand(5, d_wall);
  }else if(column == 3){
    d_wall = 42;
    AlongWall = createLidarAlongRightWallCommand(5, d_wall);
  }
  
  ParallelRaceGroup::Ptr r = std::make_shared<ParallelRaceGroup>();
  r->AddCommands(
    createVisionIdentifyCommand(fruit),     //识别前方是否有指定水果
    AlongWall
    // createTrackingPointCommand(pose, 10)    //坐标移动
  );
  return r;
}

Command::Ptr TrackingIdentifyPick_S(Pose pose, int fruit, int column){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    VisionStatus(),
    IdentifyTracking_R(column, fruit),
    VisionPick_S(fruit)
  );
  return sequential;
}

Command::Ptr TrackingIdentifyPick_DG(Pose pose, int fruit, int column){
  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
  auto identifyAction = createVisionIdentifyCommand(fruit);
  auto trackingAction = createTrackingPointCommand(pose, 2.5);

  auto servoAction = VisionStatus();
  auto pickAction = VisionPick_S(fruit);

  RG->AddCommands(identifyAction, trackingAction);
  sequential->AddCommands(
    servoAction,
    RG,
    pickAction
  );
  DG->AddCommands(sequential);
  DG->setDeadlineCommand(trackingAction);
  DG->disableShceduleDeadlineCommand();
  return DG;
}


class VisionTrackingTackCommand : public CommandBase {
  public:
  typedef std::shared_ptr<VisionTrackingTackCommand> Ptr;
  VisionTrackingTackCommand(Pose pose, int fruit, int col) : EndPose(pose), PickFruit(fruit), column(col){}
  void initialize() override {
  }
  void execute()override {
    std::cout << "isFinisheddec:" << isFinisheddec() << std::endl;
    if ((m_state != Command::State::FINISHED || m_state != Command::State::STOP)&& !isFinisheddec() ) {
    // if (!isFinisheddec() ) {
      m_state = Command::State::HOLDON;
      // auto Action = TrackingIdentifyPick_DG(EndPose, PickFruit, column);
      Action = TrackingIdentifyPick_S(EndPose, PickFruit, column);
      // Action = createTrackingPointCommand(EndPose, 10);
      Action->m_parent = getPtr();
      Action->schedule();
    }
    
  }
  void end() override{
    Action->cancel();
    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);
    std::cout << "VisionTrackingTackCommand end!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  }
  bool isFinished() override {
    return is_finished;
  }

 private:
  bool is_finished = false;
  uint8_t m_sleep_time;
  int64_t m_last_time;

  Command::Ptr Action;
  Pose EndPose;
  int PickFruit;
  int column;
  double d_min = 4;
};



class isReachPointCommand : public CommandBase {
  public:
  typedef std::shared_ptr<isReachPointCommand> Ptr;
  isReachPointCommand(Pose pose) : EndPose(pose){}
  void initialize() override {
  }
  void execute()override {
    Pose cur = Robot::GetInstance().odom->getPose();
    double d = std::sqrt((cur.x_ - EndPose.x_) * (cur.x_ - EndPose.x_) + (cur.y_ - EndPose.y_) * (cur.y_ - EndPose.y_));
    if(d < d_min){
      is_finished = true;
    }else{
      is_finished = false;
    }
  }
  void end() override{
    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);
    std::cout << "isReachPointCommand end!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  }
  bool isFinished() override {
    return is_finished;
  }

 private:
  bool is_finished = false;
  uint8_t m_sleep_time;
  int64_t m_last_time;

  Pose EndPose;
  double d_min = 4;
};




// class isReachXCommand : public CommandBase {
//  public:
//   typedef std::shared_ptr<isReachXCommand> Ptr;
//   isReachXCommand(Pose pose) : EndPose(pose){}
//   void initialize() override {
//   }
//   void execute()override {
//     Pose cur = Robot::GetInstance().odom->getPose();
//     double dx = std::fabs(cur.x_ - EndPose.x_);
//     if(dx < d_min){
//       is_finished = true;
//     }else{
//       is_finished = false;
//     }
//   }
//   void end() override{
//     Robot::GetInstance().setRightMotorSpeed(0);
//     Robot::GetInstance().setLeftMotorSpeed(0);
//     std::cout << "isReachPointCommand end!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
//   }
//   bool isFinished() override {
//     return is_finished;
//   }

//  private:
//   bool is_finished = false;
//   uint8_t m_sleep_time;
//   int64_t m_last_time;

//   Pose EndPose;
//   double d_min = 4;
// };

// Command::Ptr VisionTrackingAction(Pose pose, int fruit, int column){
//   ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
//   auto VisionTrackingTack = std::make_shared<VisionTrackingTackCommand>(pose, fruit, column);
//   auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
//   RG->AddCommands(VisionTrackingTack, isReachPoint);
//   return RG;
// }





// Command::Ptr AlongWallAction(int column){
//   Command::Ptr AlongWall;
//   double d_wall = 0;
//   if(column == 1){
//     d_wall = 40;
//     AlongWall = LidarReadAlongRightWallCommandDG(5, d_wall);
//   }else if(column == 2){
//     d_wall = 25;
//     AlongWall = LidarReadAlongLeftWallCommandDG(5, d_wall);
//   }else if(column == 3){
//     d_wall = 25;
//     AlongWall = LidarReadAlongLeftWallCommandDG(5, d_wall);
//   }

//   return AlongWall;
// }

// Command::Ptr MovePickAction(Pose pose, int fruit, int column){
//   ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
//   auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
//   RG->AddCommands(
//     createVisionIdentifyCommand(fruit),     //识别前方是否有指定水果
//     AlongWallAction(column), 
//     isReachPoint
//   );
//   return RG;
// }


// Command::Ptr AlongColumnWallMoveRG(Pose pose,int column){
//   ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
//   auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
//   RG->AddCommands(
//     AlongWallAction(column), 
//     isReachPoint
//   );
//   return RG;
// }

// Command::Ptr MovePickFruitIC(int fruit, Pose pose, int column){
//   InterruptibleCommand::Ptr IC = std::make_shared<InterruptibleCommand>(
//     AlongColumnWallMoveRG(pose, column),
//     createVisionIdentifyCommand(fruit),
//     VisionPick_S(fruit)
//   );
//   return IC;
// }

// //沿墙移动
// Command::Ptr MoveAlongRightWallRG(Pose pose, double d_wall){
//   ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
//   auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
//   RG->AddCommands(
//     LidarReadAlongRightWallCommandDG(20, d_wall),
//     isReachPoint
//   );
//   return RG;
// }
// Command::Ptr MoveAlongLeftWallRG(Pose pose, double d_wall){
//   ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
//   auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
//   RG->AddCommands(
//     LidarReadAlongLeftWallCommandDG(20, d_wall),
//     isReachPoint
//   );
//   return RG;
// }
// Command::Ptr MoveAlongRightWallRG(Pose pose, double d_wall, double speed){
//   ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
//   auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
//   RG->AddCommands(
//     LidarReadAlongRightWallCommandDG(speed, d_wall),
//     isReachPoint
//   );
//   return RG;
// }
// Command::Ptr MoveAlongLeftWallRG(Pose pose, double d_wall, double speed){
//   ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
//   auto isReachPoint = std::make_shared<isReachXCommand>(pose)->withTimer(100);
//   RG->AddCommands(
//     LidarReadAlongLeftWallCommandDG(speed, d_wall),
//     isReachPoint
//   );
//   return RG;
// }

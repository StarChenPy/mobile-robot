/**
 * @file RoboticArmFun.hpp
 * @author Zijian.Yan (jiapeng.lin@high-genius.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once
#include "RobotGenius.h"
#include "RobotCfg.h"
#include "params.h"
#include "system/Robot.h"

#include "command/MotorPIDCommand.h"
#include "command/ENCComand.h"
#include "command/ServoCommand.h"


using namespace std;
using namespace RobotGenius;

//伸缩舵机逐步运行动作
Command::Ptr TelescopicCtrlAction(double start, double end){
  double d = 0.5;
  double cnt_limit = 1;
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  if(end > start){
    for(double dis = start; dis <= end; dis += d){
      sequential->AddCommands(createcTelescopicServoCommand(dis, cnt_limit));
    }
  }else{
    for(double dis = start; dis >= end; dis -= d){
      sequential->AddCommands(createcTelescopicServoCommand(dis, cnt_limit));
    }
  }
  return sequential;
}

//夹手舵机逐步运行动作
Command::Ptr ClampCtrlAction(double start, double end){
  double d_l = 1;
  double cnt_limit = 1;
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  if(end > start){
    for(double len = start; len <= end; len += d_l){
      sequential->AddCommands(createClampServoCommand(len, cnt_limit));
    }
  }else{
    for(double len = start; len >= end; len -= d_l){
      sequential->AddCommands(createClampServoCommand(len, cnt_limit));
    }
  }
  return sequential;
}

//抬手舵机逐步运行动作
Command::Ptr RaiseCtrlAction(double start, double end){
  double d_angle = 5;
  double cnt_limit = 1;
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  if(end > start){
    for(double angle = start; angle <= end; angle += d_angle){
      sequential->AddCommands(createRaiseServoCommand(angle, cnt_limit));
    }
  }else{
    for(double angle = start; angle >= end; angle -= d_angle){
      sequential->AddCommands(createRaiseServoCommand(angle, cnt_limit));
    }
  }
  return sequential;
}

//升降旋转置零动作
Command::Ptr ResetLiftAndTurn(double turn_speed){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    ResetLiftMotorDistance(10),         //重置升降
    LiftDistancePIDCommand(-1),       //下降1cm
    ResetTurnMotorAngle(turn_speed)   //旋转复位
  );
  return sequential;
}

//抓手并行
Command::Ptr GripperServoPG(double telescopic, double raise, double clamp){
  ParallelCommandGroup::Ptr PG = std::make_shared<ParallelCommandGroup>();
  PG->AddCommands(
    createRaiseServoCommand(raise),               //RAISE_ANGLE_MIN = 0.0   RAISE_ANGLE_MAX = 90.0
    createcTelescopicServoCommand(telescopic),    //TELESCOPIC_DIS_MIN = 0.0   TELESCOPIC_DIS_MAX = 9.0
    createClampServoCommand(clamp)                //CLAMP_LEN_MIN = 2.0    CLAMP_LEN_MAX = 24.5
  );
  return PG;
}

//升降旋转并行
Command::Ptr LiftAndTurnPG(double h, double angle){
  ParallelCommandGroup::Ptr PG = std::make_shared<ParallelCommandGroup>();
  PG->AddCommands(
    LiftDistancePIDCommand(h),
    TurnAnglePIDCommand(angle)
  );
  return PG;
}

//重置机械臂
Command::Ptr ResetRoboticArm(){
  SequentialCommandGroup::Ptr LiftS = std::make_shared<SequentialCommandGroup>();
  LiftS->AddCommands(
    ResetLiftMotorDistance(15),         //重置升降
    LiftDistancePIDCommand(-1)       //下降1cm
  );
  ParallelCommandGroup::Ptr PG = std::make_shared<ParallelCommandGroup>();
  PG->AddCommands(
    LiftS,
    ResetTurnMotorAngle(-10),   //旋转复位
    GripperServoPG(3, 0, 22)
  );
  return PG;
}


//车辆移动机械臂状态
Command::Ptr CarMoveStatus(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    LiftDistancePIDCommand(-1),       //升高最高
    createClampServoCommand(CLAMP_LEN_MAX),   //抓手，打开到最大
    createcTelescopicServoCommand(5),         //伸缩，伸到5cm位置
    createRaiseServoCommand(0),               //摆手，垂直：0°
    TurnAnglePIDCommand(178),                 //旋转180°
    LiftDistancePIDCommand(-25)               //下降25cm
  );
  return sequential;
}

//车辆移动机械臂状态并行
Command::Ptr CarMoveStatusPG(){
  ParallelCommandGroup::Ptr PG = std::make_shared<ParallelCommandGroup>();
  PG->AddCommands(
    GripperServoPG(5, 0, CLAMP_LEN_MAX),
    TurnAnglePIDCommand(178),                 //旋转180°
    LiftDistancePIDCommand(-20)               //下降20cm
  );
  return PG;
}



//初始状态
Command::Ptr InitStatus(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    LiftDistancePIDCommand(-20),
    TurnAnglePIDCommand(180),
    createRaiseServoCommand(90),       //RAISE_ANGLE_MIN = 0.0   RAISE_ANGLE_MAX = 90.0
    createcTelescopicServoCommand(0),    //TELESCOPIC_DIS_MIN = 0.0   TELESCOPIC_DIS_MAX = 9.0
    createClampServoCommand(10)         //CLAMP_LEN_MIN = 2.0    CLAMP_LEN_MAX = 19.5
  );
  return sequential;
}

// 限高
Command::Ptr HeightLimitStatus(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    LiftDistancePIDCommand(-1),
    createcTelescopicServoCommand(5),    //TELESCOPIC_DIS_MIN = 0.0   TELESCOPIC_DIS_MAX = 9.0
    createRaiseServoCommand(90),       //RAISE_ANGLE_MIN = 0.0   RAISE_ANGLE_MAX = 90.0
    createClampServoCommand(10),         //CLAMP_LEN_MIN = 2.0    CLAMP_LEN_MAX = 19.5
    TurnAnglePIDCommand(0),
    LiftDistancePIDCommand(-60)
  );
  return sequential;
}

//看水果抓手状态
Command::Ptr VisionServoStatus(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    createRaiseServoCommand(45),       //RAISE_ANGLE_MIN = 0.0   RAISE_ANGLE_MAX = 90.0
    createcTelescopicServoCommand(-3.6),    //TELESCOPIC_DIS_MIN = 0.0   TELESCOPIC_DIS_MAX = 9.0
    createClampServoCommand(CLAMP_LEN_MAX)         //CLAMP_LEN_MIN = 2.0    CLAMP_LEN_MAX = 24.5
  );
  return sequential;
}


//看水果机械臂状态
Command::Ptr VisionStatus(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    LiftDistancePIDCommand(-1),
    TurnAnglePIDCommand(90),
    VisionServoStatus(),    //三个抓手舵机
    LiftDistancePIDCommand(-40)
  );
  return sequential;
}

Command::Ptr VisionStatusPG(){
  ParallelCommandGroup::Ptr PG = std::make_shared<ParallelCommandGroup>();
  PG->AddCommands(
    GripperServoPG(-3.8, 50, CLAMP_LEN_MAX),
    TurnAnglePIDCommand(90),                 //旋转180°
    LiftDistancePIDCommand(-20)               //下降20cm
  );
  SequentialCommandGroup::Ptr SG = std::make_shared<SequentialCommandGroup>();
  SG->AddCommands(
    PG,
    LiftDistancePIDCommand(-38)
  );
  return SG;
}


//抓水果动作
Command::Ptr PickFruitStatus(int layer){
  double h = -35;
  double raise = 90;
  double telescopic = TELESCOPIC_DIS_MIN;
  double clamp = 7;
  switch (layer){
    case 1:
      h = -25;
      break;
    case 2:
      h = -35;
      break;
    case 3:
      h = -46;
      break;
    default:
      h = 0;
      std::cout << "layer over" << std::endl;
      break;
  }

  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();

  sequential->AddCommands(
    LiftDistancePIDCommand(h),
    createRaiseServoCommand(raise),       //RAISE_ANGLE_MIN = 0.0   RAISE_ANGLE_MAX = 90.0
    createcTelescopicServoCommand(telescopic),    //TELESCOPIC_DIS_MIN = 0.0   TELESCOPIC_DIS_MAX = 9.0
    createClampServoCommand(clamp),         //CLAMP_LEN_MIN = 2.0    CLAMP_LEN_MAX = 19.5
    // createcTelescopicServoCommand(0)
    LiftDistancePIDCommand(-20)
  );
  return sequential;
}

//抓水果动作
Command::Ptr PickFruitSG(){
  double raise = 90;
  double telescopic = -2;
  double clamp = 7;

  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    // createVisionHeightCtrlCommand(0),
    // createRaiseServoCommand(raise),
    createcTelescopicServoCommand(telescopic, 5),
    createClampServoCommand(clamp),
    // createcTelescopicServoCommand(0)
    LiftDistancePIDCommand(-20)
  );
  return sequential;
}



//抓篮子
Command::Ptr PickBasketAction(){
  double pick_h = -2.0;
  double down_h = pick_h - 21;
  double raise = 115;
  double pick_raise = 100;
  double pick_telescopic = 9;
  double down_telescopic = 6;
  double clamp = 18;
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    LiftDistancePIDCommand(pick_h),
    createRaiseServoCommand(pick_raise),
    createcTelescopicServoCommand(TELESCOPIC_DIS_MIN),
    
    createClampServoCommand(CLAMP_LEN_MAX),
    TurnAnglePIDCommand(-90),
    TelescopicCtrlAction(TELESCOPIC_DIS_MIN, pick_telescopic),
    // createRaiseServoCommand(90),
    // createcTelescopicServoCommand(pick_telescopic), 
    createClampServoCommand(clamp),
    createRaiseServoCommand(raise),
    TurnAnglePIDCommand(0),
    createRaiseServoCommand(0),
    TurnAnglePIDCommand(180),

    createRaiseServoCommand(14),
    LiftDistancePIDCommand(down_h),
    createClampServoCommand(CLAMP_LEN_MAX)
  );
  return sequential;
}

//放篮子
Command::Ptr PutBasketAction(){
  double h = -24;
  double raise = 14;
  double telescopic = 9;
  double clamp = 18.2;
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    TurnAnglePIDCommand(180),
    createClampServoCommand(CLAMP_LEN_MAX),
    TelescopicCtrlAction(0, telescopic),
    createRaiseServoCommand(raise), 
    LiftDistancePIDCommand(h),
    createClampServoCommand(clamp),
    LiftDistancePIDCommand(-1),
    TurnAnglePIDCommand(0),
    createRaiseServoCommand(115),
    TurnAnglePIDCommand(-90),
    createRaiseServoCommand(100),
    LiftDistancePIDCommand(-1),
    createClampServoCommand(CLAMP_LEN_MAX),
    createRaiseServoCommand(115,20),
    TelescopicCtrlAction(telescopic,TELESCOPIC_DIS_MIN),
    TurnAnglePIDCommand(0)
    // createcTelescopicServoCommand(0),
    // TurnAnglePIDCommand(0),
    
  );
  return sequential;
}

//放篮子
Command::Ptr PutFruit2BasketAction(){
  double h = -15;
  double raise = 14;
  double telescopic = 9;
  // double clamp = 18.2;
  double turn_angle = 170;
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    // TelescopicCtrlAction(0, telescopic),
    createcTelescopicServoCommand(telescopic),
    createRaiseServoCommand(raise),
    TurnAnglePIDCommand(turn_angle),
    LiftDistancePIDCommand(h),
    createClampServoCommand(CLAMP_LEN_MAX)
  );
  return sequential;
}



//放水果进篮子
Command::Ptr PutFruit2BasketAction_1(int DiJiGe , int ZuoYou){
  double turn_angle;
  double raise;
  double telescopic;

  switch (DiJiGe){
    case 1:
      switch (ZuoYou){
        case 0:
        turn_angle = 190;
        break;
        
        case 1:
        turn_angle = 160;
        break;
      }
      raise = 0;
      telescopic = 0;
      break;

      case 2:
      switch (ZuoYou){
        case 0:
        turn_angle = 188;
        break;

        case 1:
        turn_angle = 170;
        break;
      }
      raise = 10;
      telescopic = 9;
      break;

      case 3:
      switch (ZuoYou){
        case 0:
        turn_angle = 185;
        break;

        case 1:
        turn_angle = 170;
        break;
      }
      raise = 38;
      telescopic = 8;
      break;
  }
  double h = -22;
  // double raise = 14;
  // double telescopic = 9;
  // double clamp = 18.2;
  // double turn_angle = 170;
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    // TelescopicCtrlAction(0, telescopic),
    createcTelescopicServoCommand(telescopic),
    createRaiseServoCommand(raise),
    TurnAnglePIDCommand(turn_angle),
    LiftDistancePIDCommand(h),
    createClampServoCommand(CLAMP_LEN_MAX)
  );
  return sequential;
}


//放水果进篮子并行
Command::Ptr PutFruit2BasketPG(int DiJiGe , int ZuoYou){
  double turn_angle;
  double raise;
  double telescopic;

  switch (DiJiGe){
    case 1:
      switch (ZuoYou){
        case 0:
        turn_angle = 190;
        break;
        
        case 1:
        turn_angle = 160;
        break;
      }
      raise = 0;
      telescopic = 0;
      break;

      case 2:
      switch (ZuoYou){
        case 0:
        turn_angle = 188;
        break;

        case 1:
        turn_angle = 170;
        break;
      }
      raise = 10;
      telescopic = 9;
      break;

      case 3:
      switch (ZuoYou){
        case 0:
        turn_angle = 185;
        break;

        case 1:
        turn_angle = 170;
        break;
      }
      raise = 38;
      telescopic = 8;
      break;
  }
  double h = -19;
  ParallelCommandGroup::Ptr ArmPG = std::make_shared<ParallelCommandGroup>();
  ArmPG->AddCommands(
    // TelescopicCtrlAction(0, telescopic),
    createcTelescopicServoCommand(telescopic),
    createRaiseServoCommand(raise),
    TurnAnglePIDCommand(turn_angle),
    LiftDistancePIDCommand(h)
  );
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    ArmPG,
    createClampServoCommand(CLAMP_LEN_MAX)
  );
  return sequential;
}
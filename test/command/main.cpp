#include "command/RoboticArmFun.h"
#include "command/VisionTrackingAction.hpp"
#include "command/VisionCtrlCommand.h"
#include "command/MotorPIDCommand.h"
#include "command/LidarReadCommand.h"
#include "command/LidarCalibCommand.h"
#include "command/TrackingPointCommand.h"
#include "command/UpdataOdomCommand.h"
#include "command/RotateCommand.h"
#include "command/sensor/ZeroOdomCommand.h"
#include "command/ButtonCommand.h"


//起始点矫正复位
Command::Ptr JiaoZheng(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    LidarReadCalibDG(20),
    createRotateCommand(-90),
    LidarReadCalibDG(36),
    createZeroOdomCommand()
  );
  return sequential;
}

Command::Ptr ZhuaQuGuoLanKaiShi(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    TurnAnglePIDCommand(90),
    createClampServoCommand(24.2),
    createRaiseServoCommand(90),
    createcTelescopicServoCommand(9),
    createRaiseServoCommand(95),
    createClampServoCommand(18)
  );
  return sequential;
}

//抓果篮
Command::Ptr ZhuaQuGuoLan(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(

    //抓第一个果篮
    LiftDistancePIDCommand(-1),
    createcTelescopicServoCommand(-2),

    ZhuaQuGuoLanKaiShi(),

    createRaiseServoCommand(110),
    TurnAnglePIDCommand(180),
    createRaiseServoCommand(0),
    createcTelescopicServoCommand(0.5),
    LiftDistancePIDCommand(-34),
    createClampServoCommand(24.2),

    //复位
    LiftDistancePIDCommand(-1),
    createRaiseServoCommand(90),
    createcTelescopicServoCommand(-2),

    // //抓第二个果篮
    ZhuaQuGuoLanKaiShi(),

    createRaiseServoCommand(110),
    TurnAnglePIDCommand(180),
    createRaiseServoCommand(30),
    createcTelescopicServoCommand(2),
    LiftDistancePIDCommand(-28.5),
    createClampServoCommand(24.2),

    //复位
    LiftDistancePIDCommand(-1),
    createRaiseServoCommand(90),
    createcTelescopicServoCommand(-2),

    //抓第三个果篮
    ZhuaQuGuoLanKaiShi(),

    createRaiseServoCommand(110),
    TurnAnglePIDCommand(180),
    createRaiseServoCommand(53),
    createcTelescopicServoCommand(3.5),
    LiftDistancePIDCommand(-28),
    createClampServoCommand(24.2)

    // //复位
    // LiftDistancePIDCommand(-1),
    // createRaiseServoCommand(90),
    // createcTelescopicServoCommand(-2)


  );
  return sequential;
}

Command::Ptr FangQuGuoLanFuWei(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    LiftDistancePIDCommand(-1),       //
    createcTelescopicServoCommand(9),
    createRaiseServoCommand(120),
    TurnAnglePIDCommand(90),
    createRaiseServoCommand(95),
    createClampServoCommand(24.2, 5),
    createClampServoCommand(20, 5),
    createClampServoCommand(24.2, 5),
    // createRaiseServoCommand(90),
    createcTelescopicServoCommand(0),
    createRaiseServoCommand(30)
  );
  return sequential;
}

//放果篮
Command::Ptr FangQuGuoLan(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    //第三个果篮取出
    LiftDistancePIDCommand(-20),
    TurnAnglePIDCommand(180),
    createClampServoCommand(24.2),
    createcTelescopicServoCommand(6),
    createRaiseServoCommand(45),
    LiftDistancePIDCommand(-33),
    createClampServoCommand(18),

    FangQuGuoLanFuWei(),

    //第二个果篮取出
    TurnAnglePIDCommand(180),
    createClampServoCommand(24.2),
    createcTelescopicServoCommand(2),
    createRaiseServoCommand(30),
    LiftDistancePIDCommand(-31.5),
    createClampServoCommand(18),

      // 放果篮
    FangQuGuoLanFuWei(),


    //第一个果篮取出
    TurnAnglePIDCommand(180),
    createClampServoCommand(24.2),
    createcTelescopicServoCommand(1),
    createRaiseServoCommand(0),
    LiftDistancePIDCommand(-36),
    createClampServoCommand(18),

    // 放果篮
    FangQuGuoLanFuWei()

  );
  return sequential;
}

//通过限高
Command::Ptr TongGuoXianGao(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    LiftDistancePIDCommand(-1),       //下降1cm
    createRaiseServoCommand(90),
    createcTelescopicServoCommand(-2),
    TurnAnglePIDCommand(0),
    LiftDistancePIDCommand(-60),
    createTrackingPointCommand(Pose(-195,0,0),20),//185
    LiftDistancePIDCommand(-30),
    createRaiseServoCommand(0),
    LiftDistancePIDCommand(-1),
    // createTrackingXYCommand(Pose(-170,-25,90),20),
    TurnAnglePIDCommand(180),
    createClampServoCommand(24.2),
    createcTelescopicServoCommand(9),
    LiftDistancePIDCommand(-25),
    createRotateCommand(90),
    LidarReadCalibDG(20),
    createZeroOdomCommand(),
    createTrackingPointCommand(Pose(-320,0,0),20)
  );
  return sequential;
}

Command::Ptr FangLanZiLujing(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
  // LidarReadCalibDG(20),
  //   createZeroOdomCommand(),

    TurnAnglePIDCommand(178),
    createClampServoCommand(24.2),
    createcTelescopicServoCommand(9),
    createRaiseServoCommand(0),
    LiftDistancePIDCommand(-25),

    createTrackingPointCommand(Pose(-320,0,180),20),

    LidarReadCalibDG(20),
    createZeroOdomCommand(),
    createTrackingPointCommand(Pose(5,0,0),20),
    createRotateCommand(-90),
    createZeroOdomCommand(),

    LiftDistancePIDCommand(-1),       //下降1cm
    createRaiseServoCommand(0),
    createcTelescopicServoCommand(0),
    TurnAnglePIDCommand(0),
    LiftDistancePIDCommand(-35),
    createRaiseServoCommand(90),
    createcTelescopicServoCommand(-2),
    LiftDistancePIDCommand(-60),
    createTrackingPointCommand(Pose(150,0,0),20),
    CarMoveStatus(),
    LidarReadCalibDG(30),
    createZeroOdomCommand(),
    createTrackingPointCommand(Pose(-90,0,0),20)
  );
  return sequential;
}
/////////////////////////////

//车辆取放篮子校准
Command::Ptr CarCalibAction(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    createRotateCommand(90),        //旋转90°
    LidarReadCalibDG(22),    //雷达校准22cm
    createZeroOdomCommand(),        //里程计重置
    createTrackingPointCommand(Pose(10,0,0),20),    //坐标移动
    createRotateCommand(-90),       //旋转-90°
    LidarReadCalibDG(22),    //雷达校准22cm
    createZeroOdomCommand()         //里程计重置
  );
  return sequential;
}

//车辆起步校准
Command::Ptr CarMoveCalibAction(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    createRotateCommand(90),
    LidarReadCalibDG(25),
    // createZeroOdomCommand(),
    // createTrackingPointCommand(Pose(10,0,0),10),
    createRotateCommand(-90),
    LidarReadCalibDG(22),
    createZeroOdomCommand()
  );
  return sequential;
}

//进草地校准
Command::Ptr CarGrassCalibAction(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    // CarMoveStatus(),
    createRotateCommand(180),
    LidarReadCalibDG(20),
    createRotateCommand(90),
    createZeroOdomCommand(),
    createTrackingPointCommand(Pose(60,0,0),20),
    LidarReadCalibDG(20),
    createZeroOdomCommand(),
    createTrackingPointCommand(Pose(11,0,0),20),
    createRotateCommand(90),
    createZeroOdomCommand()
    
    // createRotateCommand(-90),
    // LidarReadCalibDG(22),
    // createRotateCommand(180),
    // createZeroOdomCommand()
  );
  return sequential;
}

//车辆回去校准
Command::Ptr CarGoHomeCalibAction(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    createRotateCommand(90),
    LidarReadCalibDG(25),
    createZeroOdomCommand(),
    createTrackingPointCommand(Pose(-67,0,-90),20),
    // createRotateCommand(-90),
    LidarReadCalibDG(20),
    createZeroOdomCommand()
  );
  return sequential;
}

//通道尽头校准
Command::Ptr ExtremityCalibAction(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    // createRotateCommand(90),
    LidarReadCalibDG(24),
    // createZeroOdomCommand(),
    // createTrackingPointCommand(Pose(14,0,0),10),
    // createRotateCommand(90),
    // LidarReadCalibDG(20),
    // createRotateCommand(90),
    createZeroOdomCommand()
  );
  return sequential;
}


//第一列到第二列
Command::Ptr First2SecondMove(){
  vector<Pose> points = {
    Pose(-90,0,-90),
    Pose(-90,-60,-180),
    Pose(-240,-75,-180),
  };
  return createTrackingVectorCommand(points, 20);
}

//第二列到第三列
Command::Ptr Second2ThirdMove(){
  vector<Pose> points = {
    Pose(-240,0,-90),
    Pose(-240,-65,-180),
  };
  return createTrackingVectorCommand(points, 20);
}

//结束任务在终点到第二列
Command::Ptr Third2SecondMove(){
  vector<Pose> points = {
    Pose(-180,0,0),
    Pose(-230,5,-90),
  };
  return createTrackingVectorCommand(points, 20);
}

//第三列到第二列校准
Command::Ptr Third2SecondMoveCalib(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    LidarReadCalibDG(24),
    createZeroOdomCommand(),
    createTrackingPointCommand(Pose(-55,0,-90), 20),
    LidarReadCalibDG(24),
    // LidarReadCalibDG(24),
    // createZeroOdomCommand(),
    // createTrackingPointCommand(Pose(14,0,0),10),
    // createRotateCommand(90),
    // LidarReadCalibDG(22),
    createZeroOdomCommand()
  );
  return sequential;
}

//结束任务在第二列到第一列
Command::Ptr Second2FirstMove(){
  vector<Pose> points = {
    Pose(-160,0,90),
    Pose(-160,-65,0),
    Pose(0,-65,0),
  };
  return createTrackingVectorCommand(points, 20);
}

//结束任务在终点离开草地区域
Command::Ptr LeaveGrassMove(){
  vector<Pose> points = {
    Pose(-240,0,90),
    Pose(-240,65,0),
    Pose(-80,65,90),
    Pose(-80,130,180),
    Pose(-240,130,180),
  };
  return createTrackingVectorCommand(points, 20);
}

//结束任务在终点离开草地区域
Command::Ptr LeaveGrassAction(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    Third2SecondMove(),
    Third2SecondMoveCalib(),
    Second2FirstMove()
  );
  return sequential;
}



int main() {
  double move_speed = 30;
  double hold_high = -20;
  Scheduler::GetInstance(5, false).start();
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    //起步初始化
    // createStartCommand(),
    ResetRoboticArm(),
    CarMoveStatusPG(),

    // JiaoZheng(),
    // ZhuaQuGuoLan(),
    // // TongGuoXianGao(),
    // CarMoveStatusPG(),

    // createZeroOdomCommand(),
    // createTrackingPointCommand(Pose(-185,0,90), move_speed),

    // LidarReadCalibDG(20),
    // createZeroOdomCommand(),
    // createTrackingPointCommand(Pose(-320,0,0), move_speed),


    //进草地前
    CarGrassCalibAction(),
    //抓水果行动
    VisionStatusPG(),
    MovePickAction(Pose(235,0,0), APPLE_LABEL, 1),      //苹果
    // // MoveAlongRightWallRG(Pose(100,0,0), 40),
    VisionPick_S(APPLE_LABEL),
    PutFruit2BasketPG(1,0),          //将水果放入篮子
    CarMoveStatusPG(),
    //第二个水果
    VisionStatusPG(),
    MovePickAction(Pose(235,0,0), APPLE_LABEL, 1),      //苹果
    VisionPick_S(APPLE_LABEL),
    PutFruit2BasketPG(1,1),          //将水果放入篮子
    CarMoveStatusPG(),


    //移动到第二列
    createTrackingPointCommand(Pose(150,-10,90), move_speed),
    LidarReadCalibDG(85),
    createRotateCommand(90),
    createZeroOdomCommand(),
    createTrackingPointCommand(Pose(140,0,0), move_speed),
    LidarReadCalibDG(20),
    createRotateCommand(180),
    createZeroOdomCommand(),

    //抓水果行动
    VisionStatusPG(),
    MoveAlongRightWallRG(Pose(50,0,0), 40, 5),
    MovePickAction(Pose(235,0,0), APPLE_LABEL, 2),      //苹果
    VisionPick_S(APPLE_LABEL),
    PutFruit2BasketPG(2,0),          //将水果放入篮子
    CarMoveStatusPG(),
    //第四个
    VisionStatusPG(),
    MovePickAction(Pose(235,0,0), APPLE_LABEL, 2),      //苹果
    VisionPick_S(APPLE_LABEL),
    PutFruit2BasketPG(2,1),          //将水果放入篮子
    CarMoveStatusPG(),

    //移动回第一列
    createTrackingPointCommand(Pose(150,0,90), move_speed),
    LidarReadCalibDG(20),
    createRotateCommand(90),
    createZeroOdomCommand(),
    MoveAlongRightWallRG(Pose(140,0,0),35),
    LidarReadCalibDG(20),


    // VisionStatus(),
    // // MovePickAction(Pose(235,0,0), 0, 1),      //桃子
    // MoveAlongRightWallRG(Pose(170,0,0), 40),
    // VisionPick_S(0),
    // PutFruit2BasketAction_1(1,1),          //将水果放入篮子

    // VisionStatus(),
    // // MovePickAction(Pose(235,0,0), 0, 1),
    // MoveAlongRightWallRG(Pose(235,0,0), 40),

    // // VisionTrackingAction(Pose(200,0,0), APPLE_LABEL, 1)


    // // 第一列尽头校准移动到第二列
    // CarMoveStatus(),
    // ExtremityCalibAction(),

    // First2SecondMove(),       //坐标移动：从第一列移动到第二列
    // LidarReadCalibDG(20),
    // createRotateCommand(180),
    // createZeroOdomCommand(),
    

    // // 第二列开始抓水果
    // VisionStatus(),
    // MoveAlongRightWallRG(Pose(50,0,0),40),
    // // MovePickAction(Pose(235,0,0), 0, 2),    //桃子
    // MoveAlongLeftWallRG(Pose(80,0,0), 25),
    // VisionPick_S(0, 1),
    // PutFruit2BasketAction_1(2,0),          //将水果放入篮子

    // VisionStatus(),
    // // MovePickAction(Pose(235,0,0), 0, 2),    //石榴
    // MoveAlongLeftWallRG(Pose(200,0,0), 25),
    // VisionPick_S(0, 1),
    // PutFruit2BasketAction_1(2,1),          //将水果放入篮子

    // VisionStatus(),
    // // MovePickAction(Pose(235,0,0), 0, 3),
    // MoveAlongLeftWallRG(Pose(235,0,0), 25),

    // // 第二列尽头校准移动到第三列
    // CarMoveStatus(),
    // ExtremityCalibAction(),
    // Second2ThirdMove(),
    // LidarReadCalibDG(20),
    // createRotateCommand(180),
    // createZeroOdomCommand(),

    // // 第三列开始抓水果
    // VisionStatus(),
    // // MovePickAction(Pose(235,0,0), 0, 3),   //苹果
    // MoveAlongLeftWallRG(Pose(120,0,0), 25),
    // VisionPick_S(0, 1),
    // PutFruit2BasketAction_1(3,0),          //将水果放入篮子

    // VisionStatus(),
    // // MovePickAction(Pose(235,0,0), 0, 3),    //苹果
    // MoveAlongLeftWallRG(Pose(180,0,0), 25),
    // VisionPick_S(0, 1),
    // PutFruit2BasketAction_1(3,1),          //将水果放入篮子

    // VisionStatus(),
    // // MovePickAction(Pose(235,0,0), 0, 3),
    // MoveAlongLeftWallRG(Pose(235,0,0), 25),

    // // 第三列尽头校准离开草地
    // CarMoveStatus(),
    // ExtremityCalibAction(),
    // LeaveGrassAction(),     //离开草地坐标移动



    //在草地的时候返回起点
    CarGoHomeCalibAction(),
    createRotateCommand(180),
    createZeroOdomCommand()
    // //移动到放篮子
    // createTrackingPointCommand(Pose(320,0,0),move_speed),
    // LidarReadCalibDG(20),
    // createRotateCommand(-90),
    // createZeroOdomCommand(),
    // MoveAlongRightWallRG(Pose(120,0,0),35),
    // LidarReadCalibDG(115),


    // FangQuGuoLan(),

    // //回原点
    // CarMoveStatusPG(),
    // createZeroOdomCommand(),
    // createTrackingPointCommand(Pose(90,0,0),move_speed)

  );

  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  DG->AddCommands(
    createTurnMotorPIDCommand(),
    createLiftMotorPIDCommand(),
    createLeftMotorPIDCommand(),
    createRightMotorPIDCommand(),
    // createLidarReadCommand(),
    createUpdataOdomCommand()
  );
  DG->setDeadlineCommand(sequential);
  DG->schedule();

  Vision::GetInstance();    //开启摄像头，预加载识别模型


  sleep(3);
  Scheduler::GetInstance().stop();

  return 0;
}


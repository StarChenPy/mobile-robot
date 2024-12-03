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
    // ResetLiftAndTurn(-8),

    // TurnAnglePIDCommand(180),
    // createClampServoCommand(24.2),
    // LiftDistancePIDCommand(-35),

    LidarReadCalibDG(20),
    createRotateCommand(-90),
    LidarReadCalibDG(35),
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
    createRaiseServoCommand(98),
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
    createRaiseServoCommand(88),
    createcTelescopicServoCommand(-2),

    // //抓第二个果篮
    ZhuaQuGuoLanKaiShi(),

    createRaiseServoCommand(110),
    TurnAnglePIDCommand(178),
    createRaiseServoCommand(30),
    createcTelescopicServoCommand(2),
    LiftDistancePIDCommand(-28.5),
    createClampServoCommand(24.2),

    //复位
    LiftDistancePIDCommand(-1),
    createRaiseServoCommand(88),
    createcTelescopicServoCommand(-2),

    //抓第三个果篮
    ZhuaQuGuoLanKaiShi(),

    createRaiseServoCommand(110),
    TurnAnglePIDCommand(178),
    createRaiseServoCommand(53),
    createcTelescopicServoCommand(3.5),
    LiftDistancePIDCommand(-28),
    createClampServoCommand(24.2)

    // //复位
    // LiftDistancePIDCommand(-1),
    // createRaiseServoCommand(88),
    // createcTelescopicServoCommand(-2)


  );
  return sequential;
}

Command::Ptr FangQuGuoLanFuWei(){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    LiftDistancePIDCommand(-1),       //
    createcTelescopicServoCommand(9),
    createRaiseServoCommand(130),
    TurnAnglePIDCommand(90),
    createRaiseServoCommand(100),
    createClampServoCommand(24.2),
    createRaiseServoCommand(90),
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
    TurnAnglePIDCommand(179),
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
    LiftDistancePIDCommand(-31),
    createClampServoCommand(18),

      // 放果篮
    FangQuGuoLanFuWei(),


    //第一个果篮取出
    TurnAnglePIDCommand(180),
    createClampServoCommand(24.2),
    createcTelescopicServoCommand(1),
    createRaiseServoCommand(0),
    LiftDistancePIDCommand(-35.8),
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

    // TurnAnglePIDCommand(178),
    // createClampServoCommand(24.2),
    // createcTelescopicServoCommand(9),
    // createRaiseServoCommand(0),
    // LiftDistancePIDCommand(-25),

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
    createTrackingPointCommand(Pose(65,0,0),30),
    LidarReadCalibDG(20),
    // createZeroOdomCommand(),
    // createTrackingPointCommand(Pose(12,0,0),20),
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
    createTrackingPointCommand(Pose(-67,0,-90),30),
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



//摘高处水果前
Command::Ptr PrePickHighApplePG(double turn_angle){
  ParallelCommandGroup::Ptr PG = std::make_shared<ParallelCommandGroup>();
  PG->AddCommands(
    LiftAndTurnPG(-20, turn_angle),
    GripperServoPG(-3, 90, 20)
  );
  SequentialCommandGroup::Ptr SG = std::make_shared<SequentialCommandGroup>();
  SG->AddCommands(
    PG,
    LiftDistancePIDCommand(-47)
  );
  return SG;
}

//摘高处苹果
Command::Ptr PickHighAppleSG(){
  SequentialCommandGroup::Ptr SG = std::make_shared<SequentialCommandGroup>();
  SG->AddCommands(
    createcTelescopicServoCommand(-3),
    createClampServoCommand(6),
    createRaiseServoCommand(45),
    LiftDistancePIDCommand(-20)
  );
  return SG;
}

//摘低处水果前
Command::Ptr PrePickLowApplePG(double turn_angle){
  ParallelCommandGroup::Ptr PG = std::make_shared<ParallelCommandGroup>();
  PG->AddCommands(
    LiftAndTurnPG(-20, turn_angle),
    GripperServoPG(-3, 90, 20)
  );
  SequentialCommandGroup::Ptr SG = std::make_shared<SequentialCommandGroup>();
  SG->AddCommands(
    PG,
    LiftDistancePIDCommand(-57)
  );
  return SG;
}

//摘低处苹果
Command::Ptr PickLowAppleSG(){
  ParallelCommandGroup::Ptr Lift_PG = std::make_shared<ParallelCommandGroup>();
  Lift_PG->AddCommands(
    createRaiseServoCommand(45),
    LiftDistancePIDCommand(-20)
  );
  SequentialCommandGroup::Ptr SG = std::make_shared<SequentialCommandGroup>();
  SG->AddCommands(
    createcTelescopicServoCommand(7),
    createClampServoCommand(6),
    createcTelescopicServoCommand(-3),
    Lift_PG
  );
  return SG;
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
    // CarMoveStatusPG(),

    // createZeroOdomCommand(),
    // createTrackingPointCommand(Pose(-185,0,90), move_speed),

    // LidarReadCalibDG(20),
    // createZeroOdomCommand(),
    // createTrackingPointCommand(Pose(-320,0,0),move_speed),



    //进草地前
    CarGrassCalibAction(),
    MoveAlongRightWallRG(Pose(150,0,-90),30),
    createRotateCommand(-90),
    createTrackingPointCommand(Pose(149,-34,-90), move_speed),

    //第一面
    // LiftDistancePIDCommand(hold_high),
    PrePickHighApplePG(0),
    PickHighAppleSG(),
    PutFruit2BasketPG(1,0),          //将水果放入篮子
    CarMoveStatusPG(),

    // LiftDistancePIDCommand(hold_high),
    PrePickLowApplePG(0),
    PickLowAppleSG(),
    PutFruit2BasketPG(1,1),          //将水果放入篮子
    CarMoveStatusPG(),

    //
    createTrackingPointCommand(Pose(150,-55,-90), move_speed),
    createTrackingXYCommand(Pose(85,-95,-180), move_speed),

    //校准
    LidarReadCalibDG(125),
    createRotateCommand(90),
    LidarReadCalibDG(70),
    // createZeroOdomCommand(),

    //第二面
    // LiftDistancePIDCommand(hold_high),
    PrePickHighApplePG(90),
    PickHighAppleSG(),
    PutFruit2BasketPG(2,0),          //将水果放入篮子
    CarMoveStatusPG(),

    // LiftDistancePIDCommand(hold_high),
    PrePickLowApplePG(90),
    PickLowAppleSG(),
    PutFruit2BasketPG(2,1),          //将水果放入篮子
    CarMoveStatusPG(),

    //移动
    createTrackingPointCommand(Pose(0,-130,-90), move_speed),
    //校准
    LidarReadCalibDG(20),
    createRotateCommand(-90),
    LidarReadCalibDG(20),
    createRotateCommand(-180),
    createZeroOdomCommand(),
    MoveAlongLeftWallRG(Pose(141,0,0),24),

    //第三面
    // LiftDistancePIDCommand(hold_high),
    PrePickHighApplePG(90),
    PickHighAppleSG(),
    PutFruit2BasketPG(3,0),          //将水果放入篮子
    CarMoveStatusPG(),

    // LiftDistancePIDCommand(hold_high),
    PrePickLowApplePG(90),
    PickLowAppleSG(),
    PutFruit2BasketPG(3,1),          //将水果放入篮子
    CarMoveStatusPG(),

    //移动
    createTrackingPointCommand(Pose(210,10,0), move_speed),
    //校准
    LidarReadCalibDG(61),
    createRotateCommand(90),
    LidarReadCalibDG(50),

    //第四面
    // LiftDistancePIDCommand(hold_high),
    PrePickHighApplePG(90),
    PickHighAppleSG(),
    PutFruit2BasketPG(2,0),          //将水果放入篮子
    CarMoveStatusPG(),

    // LiftDistancePIDCommand(hold_high),
    PrePickLowApplePG(90),
    PickLowAppleSG(),
    PutFruit2BasketPG(2,1),          //将水果放入篮子
    CarMoveStatusPG(),

    //离开
    createZeroOdomCommand(),
    createTrackingXYCommand(Pose(60,50,0), move_speed),
    LidarReadCalibDG(20),
    createRotateCommand(90),
    createZeroOdomCommand(),
    MoveAlongRightWallRG(Pose(150,0,0),35),
    LidarReadCalibDG(20),
    
    
    //在草地的时候返回起点
    CarGoHomeCalibAction(),
    createRotateCommand(180),
    createZeroOdomCommand(),
    //移动到放篮子
    createTrackingPointCommand(Pose(320,0,0),move_speed),
    LidarReadCalibDG(20),
    createRotateCommand(-90),
    createZeroOdomCommand(),
    MoveAlongRightWallRG(Pose(120,0,0),35),
    LidarReadCalibDG(115),


    FangQuGuoLan(),

    //回原点
    CarMoveStatusPG(),
    createZeroOdomCommand(),
    createTrackingPointCommand(Pose(90,0,0),move_speed)
    // LidarReadCalibDG(30)

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


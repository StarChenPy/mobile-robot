#include "system/SysParams.h"
#include "system/Robot.h"
using namespace std;

// 电机速度环
PIDCtrlParams LeftMotorPIDParams;
PIDCtrlParams RightMotorPIDParams;
PIDCtrlParams TurnMotorPIDParams;
PIDCtrlParams LiftMotorPIDParams;

// 电机位置环
PIDCtrlParams TurnDisPIDParams;
PIDCtrlParams LiftDisPIDParams;

// 红外校准pid
PIDCtrlParams IRCalAnglePIDParams;
PIDCtrlParams IRCalDisPIDParams;

// 单红外校准pid
PIDCtrlParams SingleIRCalAnglePIDParams;
PIDCtrlParams SingleIRCalDisPIDParams;

// 超声波校准pid
PIDCtrlParams USCalAnglePIDParams;
PIDCtrlParams USCalDisPIDParams;

// 雷达校准pid
PIDCtrlParams LidarCalAnglePIDParams;
PIDCtrlParams LidarCalDisPIDParams;

// 雷达沿墙行驶pid
PIDCtrlParams AlongWallPIDParams;

// 坐标移动pid
CtrlPID VxPIDParams;
CtrlPID VyPIDParams;
CtrlPID VzPIDParams;

//雷达校准pid误差配置
LidarCalibError LidarCalibE;
//红外校准pid误差配置
SensorCalibError IRCalibE;
//超声波校准pid误差配置
SensorCalibError USCalibE;
//单红外校准pid误差配置
SensorCalibError SingleIRCalibE;

//标定数据
CalImg ImgCalData;
//雷达参数
LidarInitParams LidarParams;

void VisionCalInit(){
  //标定数据
  ImgCalData.Image.P1 = {83, 125};
  ImgCalData.Image.P2 = {249, 127};
  ImgCalData.Image.P3 = {260, 228};
  ImgCalData.Image.P4 = {68, 229};
  ImgCalData.Object.P1 = {5.0f, 14.0f};
  ImgCalData.Object.P2 = {-5.0f, 14.0f};
  ImgCalData.Object.P3 = {-10.0f, 0.0f};
  ImgCalData.Object.P4 = {10.0f, 0.0f};
}

void writeParams2Share(){
  LABVIEW::AllPID AllPIDParams;
  // 左轮
  AllPIDParams.Motor.Left.pid = LEFTMOTORPID;
  AllPIDParams.Motor.Left.limit = LEFTMOTOROUTPUTLIMITS;
  AllPIDParams.Motor.Left.dt = PIDDT;
  // 右轮
  AllPIDParams.Motor.Right.pid = RIGHTMOTORPID;
  AllPIDParams.Motor.Right.limit = RIGHTMOTOROUTPUTLIMITS;
  AllPIDParams.Motor.Right.dt = PIDDT;
  // 旋转
  AllPIDParams.Motor.Turn.pid = TURNMOTORPID;
  AllPIDParams.Motor.Turn.limit = TURNMOTOROUTPUTLIMITS;
  AllPIDParams.Motor.Turn.dt = PIDDT;
  // 升降
  AllPIDParams.Motor.Lift.pid = LIFTMOTORPID;
  AllPIDParams.Motor.Lift.limit = LIFTMOTOROUTPUTLIMITS;
  AllPIDParams.Motor.Lift.dt = PIDDT;
  //电机位置环pid
  // 旋转
  AllPIDParams.MotorDis.TurnDis.pid = TURNMOTORDISTANCEPIDPARAMS;
  AllPIDParams.MotorDis.TurnDis.limit = TURNMOTODISTANCELIMITS;
  AllPIDParams.MotorDis.TurnDis.dt = PIDDT;
  // 升降
  AllPIDParams.MotorDis.LiftDis.pid = LIFTMOTORDISTANCEPIDPARAMS;
  AllPIDParams.MotorDis.LiftDis.limit = LIFTMOTODISTANCELIMITS;
  AllPIDParams.MotorDis.LiftDis.dt = PIDDT;
  // 红外校准pid
  AllPIDParams.IR.Angle.pid = IRANGLEPID;
  AllPIDParams.IR.Angle.limit = IRANGLEOUTPUTLIMITS;
  AllPIDParams.IR.Angle.dt = PIDDT;
  AllPIDParams.IR.Dis.pid = IRDISTANCEPID;
  AllPIDParams.IR.Dis.limit = IRDISTANCEOUTPUTLIMITS;
  AllPIDParams.IR.Dis.dt = PIDDT;
  // 单红外校准pid
  AllPIDParams.SingeIR.Angle.pid = SINGLEIRANGLEPID;
  AllPIDParams.SingeIR.Angle.limit = SINGLEIRANGLEOUTPUTLIMITS;
  AllPIDParams.SingeIR.Angle.dt = PIDDT;
  AllPIDParams.SingeIR.Dis.pid = SINGLEIRDISTANCEPID;
  AllPIDParams.SingeIR.Dis.limit = SINGLEIRDISTANCEOUTPUTLIMITS;
  AllPIDParams.SingeIR.Dis.dt = PIDDT;
  // 超声波校准pid
  AllPIDParams.US.Angle.pid = USANGLEPID;
  AllPIDParams.US.Angle.limit = USANGLEOUTPUTLIMITS;
  AllPIDParams.US.Angle.dt = CTRLDT;
  AllPIDParams.US.Dis.pid = USDISTANCEPID;
  AllPIDParams.US.Dis.limit = USDISTANCEOUTPUTLIMITS;
  AllPIDParams.US.Dis.dt = CTRLDT;
  // 雷达校准pid
  AllPIDParams.Lidar.Angle.pid = LIDARANGLEPID;
  AllPIDParams.Lidar.Angle.limit = LIDARANGLEPIDOUTPUTLIMITS;
  AllPIDParams.Lidar.Angle.dt = CTRLDT;
  AllPIDParams.Lidar.Dis.pid = LIDARDISTANCEPID;
  AllPIDParams.Lidar.Dis.limit = LIDARDISTANCEPIDOUTPUTLIMITS;
  AllPIDParams.Lidar.Dis.dt = CTRLDT;
  // 雷达沿墙行驶pid
  AllPIDParams.AlongWall.pid = ALONGWALLPID;
  AllPIDParams.AlongWall.limit = ALONGWALLPIDLIMITS;
  AllPIDParams.AlongWall.dt = CTRLDT;
  // 坐标移动pid
  AllPIDParams.Chassis.Vx = VXPID;
  AllPIDParams.Chassis.Vy = VYPID;
  AllPIDParams.Chassis.Vz = VZPID;
  //写入共享内存
  LABVIEW::AllPIDShareAddress->write(AllPIDParams);

  //雷达*********************************************************
  LABVIEW::LidarInitParams writeLidarParams;
  writeLidarParams.InitAngle = -90;
  writeLidarParams.CalAngle = 10;
  LABVIEW::LidarInitParamsShareAddress->write(writeLidarParams);
}


void ParamsInit(){
  writeParams2Share();
  //电机速度环pid
  // 左轮
  LeftMotorPIDParams.pid = LEFTMOTORPID;
  LeftMotorPIDParams.limit = LEFTMOTOROUTPUTLIMITS;
  LeftMotorPIDParams.dt = PIDDT;
  // 右轮
  RightMotorPIDParams.pid = RIGHTMOTORPID;
  RightMotorPIDParams.limit = RIGHTMOTOROUTPUTLIMITS;
  RightMotorPIDParams.dt = PIDDT;
  // 旋转
  TurnMotorPIDParams.pid = TURNMOTORPID;
  TurnMotorPIDParams.limit = TURNMOTOROUTPUTLIMITS;
  TurnMotorPIDParams.dt = PIDDT;
  // 升降
  LiftMotorPIDParams.pid = LIFTMOTORPID;
  LiftMotorPIDParams.limit = LIFTMOTOROUTPUTLIMITS;
  LiftMotorPIDParams.dt = PIDDT;
  //电机位置环pid
  // 旋转
  TurnDisPIDParams.pid = TURNMOTORDISTANCEPIDPARAMS;
  TurnDisPIDParams.limit = TURNMOTODISTANCELIMITS;
  TurnDisPIDParams.dt = PIDDT;
  // 升降
  LiftDisPIDParams.pid = LIFTMOTORDISTANCEPIDPARAMS;
  LiftDisPIDParams.limit = LIFTMOTODISTANCELIMITS;
  LiftDisPIDParams.dt = PIDDT;
  // 红外校准pid
  IRCalAnglePIDParams.pid = IRANGLEPID;
  IRCalAnglePIDParams.limit = IRANGLEOUTPUTLIMITS;
  IRCalAnglePIDParams.dt = PIDDT;
  IRCalDisPIDParams.pid = IRDISTANCEPID;
  IRCalDisPIDParams.limit = IRDISTANCEOUTPUTLIMITS;
  IRCalDisPIDParams.dt = PIDDT;
  // 单红外校准pid
  SingleIRCalAnglePIDParams.pid = SINGLEIRANGLEPID;
  SingleIRCalAnglePIDParams.limit = SINGLEIRANGLEOUTPUTLIMITS;
  SingleIRCalAnglePIDParams.dt = PIDDT;
  SingleIRCalDisPIDParams.pid = SINGLEIRDISTANCEPID;
  SingleIRCalDisPIDParams.limit = SINGLEIRDISTANCEOUTPUTLIMITS;
  SingleIRCalDisPIDParams.dt = PIDDT;

  SingleIRCalibE.Angle = 0.5;
  SingleIRCalibE.Dis = 0.5;
  SingleIRCalibE.CNT = 3;
  SingleIRCalibE.LeftRightE = 1;

  // 超声波校准pid
  USCalAnglePIDParams.pid = USANGLEPID;
  USCalAnglePIDParams.limit = USANGLEOUTPUTLIMITS;
  USCalAnglePIDParams.dt = PIDDT;
  USCalDisPIDParams.pid = USDISTANCEPID;
  USCalDisPIDParams.limit = USDISTANCEOUTPUTLIMITS;
  USCalDisPIDParams.dt = PIDDT;
  // 雷达校准pid
  LidarCalAnglePIDParams.pid = LIDARANGLEPID;
  LidarCalAnglePIDParams.limit = LIDARANGLEPIDOUTPUTLIMITS;
  LidarCalAnglePIDParams.dt = CTRLDT;
  LidarCalDisPIDParams.pid = LIDARDISTANCEPID;
  LidarCalDisPIDParams.limit = LIDARDISTANCEPIDOUTPUTLIMITS;
  LidarCalDisPIDParams.dt = CTRLDT;
  // 雷达沿墙行驶pid
  AlongWallPIDParams.pid = ALONGWALLPID;
  AlongWallPIDParams.limit = ALONGWALLPIDLIMITS;
  AlongWallPIDParams.dt = CTRLDT;
  // 坐标移动pid
  VxPIDParams = VXPID;
  VyPIDParams = VYPID;
  VzPIDParams = VZPID;



  ////////////////////////////////////////////////////

  //雷达校准pid误差配置
  LidarCalibE.Angle.E_Range = LIDAR_ANGLE_E_MAX;
  LidarCalibE.Angle.CNT = LIDAR_ANGLE_CNT_MIN;
  LidarCalibE.Dis.E_Range = LIDAR_DISTANCE_E_MAX;
  LidarCalibE.Dis.CNT = LIDAR_DISTANCE_CNT_MIN;
  LidarCalibE.LeftRightE = LIDAR_ANGLE_PID_E_MAX;

  //红外校准pid误差配置
  IRCalibE.Dis = 0.5;
  IRCalibE.Angle = 0.5;
  IRCalibE.CNT = 3;
  IRCalibE.LeftRightE = 2;

  //超声波校准pid误差配置
  USCalibE.Dis = 0.5;
  USCalibE.Angle = 0.5;
  USCalibE.CNT = 3;
  USCalibE.LeftRightE = 2;

  //标定数据
  VisionCalInit();
  // Robot::GetInstance().chassis_ctrl->set_v_pidlimit(VxPIDParams.limit);

  LidarParams.InitAngle = LIDAR_INIT_ANGLE;
  LidarParams.CalAngle = LIDAR_CALIB_ANGLE;
}

void updataLidarParams(){
  LABVIEW::LidarInitParams readLidarParams;
  LABVIEW::LidarInitParamsShareAddress->read(readLidarParams);
  LidarParams.InitAngle = readLidarParams.InitAngle;
  LidarParams.CalAngle = readLidarParams.CalAngle;
}

void updataPIDParams(){
  //读取共享内存
  LABVIEW::AllPID readPIDParams;
  LABVIEW::AllPIDShareAddress->read(readPIDParams);
  //电机速度环pid
  // 左轮
  LeftMotorPIDParams.pid.kp = readPIDParams.Motor.Left.pid.kp;
  LeftMotorPIDParams.pid.ki = readPIDParams.Motor.Left.pid.ki;
  LeftMotorPIDParams.pid.kd = readPIDParams.Motor.Left.pid.kd;
  LeftMotorPIDParams.limit.max = readPIDParams.Motor.Left.limit.max;
  LeftMotorPIDParams.limit.min = readPIDParams.Motor.Left.limit.min;
  LeftMotorPIDParams.dt = PIDDT;
  // 右轮
  RightMotorPIDParams.pid.kp = readPIDParams.Motor.Right.pid.kp;
  RightMotorPIDParams.pid.ki = readPIDParams.Motor.Right.pid.ki;
  RightMotorPIDParams.pid.kd = readPIDParams.Motor.Right.pid.kd;
  RightMotorPIDParams.limit.max = readPIDParams.Motor.Right.limit.max;
  RightMotorPIDParams.limit.min = readPIDParams.Motor.Right.limit.min;
  RightMotorPIDParams.dt = PIDDT;
  cout << "left pid:" << LeftMotorPIDParams.pid.kp << ", " << LeftMotorPIDParams.pid.ki << ", " << LeftMotorPIDParams.pid.kd << endl;
  cout << "right pid:" << RightMotorPIDParams.pid.kp << ", " << RightMotorPIDParams.pid.ki << ", " << RightMotorPIDParams.pid.kd << endl;
  // 旋转
  TurnMotorPIDParams.pid.kp = readPIDParams.Motor.Turn.pid.kp;
  TurnMotorPIDParams.pid.ki = readPIDParams.Motor.Turn.pid.ki;
  TurnMotorPIDParams.pid.kd = readPIDParams.Motor.Turn.pid.kd;
  TurnMotorPIDParams.limit.max = readPIDParams.Motor.Turn.limit.max;
  TurnMotorPIDParams.limit.min = readPIDParams.Motor.Turn.limit.min;
  TurnMotorPIDParams.dt = PIDDT;
  // 升降
  LiftMotorPIDParams.pid.kp = readPIDParams.Motor.Lift.pid.kp;
  LiftMotorPIDParams.pid.ki = readPIDParams.Motor.Lift.pid.ki;
  LiftMotorPIDParams.pid.kd = readPIDParams.Motor.Lift.pid.kd;
  LiftMotorPIDParams.limit.max = readPIDParams.Motor.Lift.limit.max;
  LiftMotorPIDParams.limit.min = readPIDParams.Motor.Lift.limit.min;
  LiftMotorPIDParams.dt = PIDDT;
  //电机位置环pid
  // 旋转
  TurnDisPIDParams.pid.kp = readPIDParams.MotorDis.TurnDis.pid.kp;
  TurnDisPIDParams.pid.ki = readPIDParams.MotorDis.TurnDis.pid.ki;
  TurnDisPIDParams.pid.kd = readPIDParams.MotorDis.TurnDis.pid.kd;
  TurnDisPIDParams.limit.max = readPIDParams.MotorDis.TurnDis.limit.max;
  TurnDisPIDParams.limit.min = readPIDParams.MotorDis.TurnDis.limit.min;
  TurnDisPIDParams.dt = PIDDT;
  // 升降
  LiftDisPIDParams.pid.kp = readPIDParams.MotorDis.LiftDis.pid.kp;
  LiftDisPIDParams.pid.ki = readPIDParams.MotorDis.LiftDis.pid.ki;
  LiftDisPIDParams.pid.kd = readPIDParams.MotorDis.LiftDis.pid.kd;
  LiftDisPIDParams.limit.max = readPIDParams.MotorDis.LiftDis.limit.max;
  LiftDisPIDParams.limit.min = readPIDParams.MotorDis.LiftDis.limit.min;
  LiftDisPIDParams.dt = PIDDT;
  // 红外校准pid
  IRCalAnglePIDParams.pid.kp = readPIDParams.IR.Angle.pid.kp;
  IRCalAnglePIDParams.pid.ki = readPIDParams.IR.Angle.pid.ki;
  IRCalAnglePIDParams.pid.kd = readPIDParams.IR.Angle.pid.kd;
  IRCalAnglePIDParams.limit.max = readPIDParams.IR.Angle.limit.max;
  IRCalAnglePIDParams.limit.min = readPIDParams.IR.Angle.limit.min;
  IRCalAnglePIDParams.dt = PIDDT;
  IRCalDisPIDParams.pid.kp = readPIDParams.IR.Dis.pid.kp;
  IRCalDisPIDParams.pid.ki = readPIDParams.IR.Dis.pid.ki;
  IRCalDisPIDParams.pid.kd = readPIDParams.IR.Dis.pid.kd;
  IRCalDisPIDParams.limit.max = readPIDParams.IR.Dis.limit.max;
  IRCalDisPIDParams.limit.min = readPIDParams.IR.Dis.limit.min;
  IRCalDisPIDParams.dt = PIDDT;
  // 单红外校准pid
  SingleIRCalAnglePIDParams.pid.kp = readPIDParams.SingeIR.Angle.pid.kp;
  SingleIRCalAnglePIDParams.pid.ki = readPIDParams.SingeIR.Angle.pid.ki;
  SingleIRCalAnglePIDParams.pid.kd = readPIDParams.SingeIR.Angle.pid.kd;
  SingleIRCalAnglePIDParams.limit.max = readPIDParams.SingeIR.Angle.limit.max;
  SingleIRCalAnglePIDParams.limit.min = readPIDParams.SingeIR.Angle.limit.min;
  SingleIRCalAnglePIDParams.dt = PIDDT;
  SingleIRCalDisPIDParams.pid.kp = readPIDParams.SingeIR.Dis.pid.kp;
  SingleIRCalDisPIDParams.pid.ki = readPIDParams.SingeIR.Dis.pid.ki;
  SingleIRCalDisPIDParams.pid.kd = readPIDParams.SingeIR.Dis.pid.kd;
  SingleIRCalDisPIDParams.limit.max = readPIDParams.SingeIR.Dis.limit.max;
  SingleIRCalDisPIDParams.limit.min = readPIDParams.SingeIR.Dis.limit.min;
  SingleIRCalDisPIDParams.dt = PIDDT;
  // 超声波校准pid
  USCalAnglePIDParams.pid.kp = readPIDParams.US.Angle.pid.kp;
  USCalAnglePIDParams.pid.ki = readPIDParams.US.Angle.pid.ki;
  USCalAnglePIDParams.pid.kd = readPIDParams.US.Angle.pid.kd;
  USCalAnglePIDParams.limit.max = readPIDParams.US.Angle.limit.max;
  USCalAnglePIDParams.limit.min = readPIDParams.US.Angle.limit.min;
  USCalAnglePIDParams.dt = PIDDT;
  USCalDisPIDParams.pid.kp = readPIDParams.US.Dis.pid.kp;
  USCalDisPIDParams.pid.ki = readPIDParams.US.Dis.pid.ki;
  USCalDisPIDParams.pid.kd = readPIDParams.US.Dis.pid.kd;
  USCalDisPIDParams.limit.max = readPIDParams.US.Dis.limit.max;
  USCalDisPIDParams.limit.min = readPIDParams.US.Dis.limit.min;
  USCalDisPIDParams.dt = PIDDT;
  // 雷达校准pid
  LidarCalAnglePIDParams.pid.kp = readPIDParams.Lidar.Angle.pid.kp;
  LidarCalAnglePIDParams.pid.ki = readPIDParams.Lidar.Angle.pid.ki;
  LidarCalAnglePIDParams.pid.kd = readPIDParams.Lidar.Angle.pid.kd;
  LidarCalAnglePIDParams.limit.max = readPIDParams.Lidar.Angle.limit.max;
  LidarCalAnglePIDParams.limit.min = readPIDParams.Lidar.Angle.limit.min;
  LidarCalAnglePIDParams.dt = CTRLDT;
  LidarCalDisPIDParams.pid.kp = readPIDParams.Lidar.Dis.pid.kp;
  LidarCalDisPIDParams.pid.ki = readPIDParams.Lidar.Dis.pid.ki;
  LidarCalDisPIDParams.pid.kd = readPIDParams.Lidar.Dis.pid.kd;
  LidarCalDisPIDParams.limit.max = readPIDParams.Lidar.Dis.limit.max;
  LidarCalDisPIDParams.limit.min = readPIDParams.Lidar.Dis.limit.min;
  LidarCalDisPIDParams.dt = CTRLDT;
  // 雷达沿墙行驶pid
  AlongWallPIDParams.pid.kp = readPIDParams.AlongWall.pid.kp;
  AlongWallPIDParams.pid.ki = readPIDParams.AlongWall.pid.ki;
  AlongWallPIDParams.pid.kd = readPIDParams.AlongWall.pid.kd;
  AlongWallPIDParams.limit.max = readPIDParams.AlongWall.limit.max;
  AlongWallPIDParams.limit.min = readPIDParams.AlongWall.limit.min;
  AlongWallPIDParams.dt = CTRLDT;
  // 坐标移动pid
  VxPIDParams.P = readPIDParams.Chassis.Vx.P;
  VxPIDParams.I = readPIDParams.Chassis.Vx.I;
  VxPIDParams.D = readPIDParams.Chassis.Vx.D;
  VxPIDParams.limit = readPIDParams.Chassis.Vx.limit;
  VyPIDParams.P = readPIDParams.Chassis.Vy.P;
  VyPIDParams.I = readPIDParams.Chassis.Vy.I;
  VyPIDParams.D = readPIDParams.Chassis.Vy.D;
  VyPIDParams.limit = readPIDParams.Chassis.Vy.limit;
  VzPIDParams.P = readPIDParams.Chassis.Vz.P;
  VzPIDParams.I = readPIDParams.Chassis.Vz.I;
  VzPIDParams.D = readPIDParams.Chassis.Vz.D;
  VzPIDParams.limit = readPIDParams.Chassis.Vz.limit;

  uint8_t status = COMMEND_END;
  LABVIEW::updateAllPIDStatusShareAddress->read(status);
}

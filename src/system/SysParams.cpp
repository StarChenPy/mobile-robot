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

void VisionCalInit() {
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

void ParamsInit() {
    //电机速度环pid
    // 左轮
    LeftMotorPIDParams.pid = LEFT_MOTOR_PID;
    LeftMotorPIDParams.limit = LEFT_MOTOR_OUTPUT_LIMITS;
    LeftMotorPIDParams.dt = PID_DT;
    // 右轮
    RightMotorPIDParams.pid = RIGHT_MOTOR_PID;
    RightMotorPIDParams.limit = RIGHT_MOTOR_OUTPUT_LIMITS;
    RightMotorPIDParams.dt = PID_DT;
    // 旋转
    TurnMotorPIDParams.pid = TURN_MOTOR_PID;
    TurnMotorPIDParams.limit = TURN_MOTOR_OUTPUT_LIMITS;
    TurnMotorPIDParams.dt = PID_DT;
    // 升降
    LiftMotorPIDParams.pid = LIFT_MOTOR_PID;
    LiftMotorPIDParams.limit = LIFT_MOTOR_OUTPUT_LIMITS;
    LiftMotorPIDParams.dt = PID_DT;
    //电机位置环pid
    // 旋转
    TurnDisPIDParams.pid = TURN_MOTOR_DISTANCE_PID_PARAMS;
    TurnDisPIDParams.limit = TURN_MOTOR_DISTANCE_LIMITS;
    TurnDisPIDParams.dt = PID_DT;
    // 升降
    LiftDisPIDParams.pid = LIFT_MOTOR_DISTANCE_PID_PARAMS;
    LiftDisPIDParams.limit = LIFT_MOTOR_DISTANCE_LIMITS;
    LiftDisPIDParams.dt = PID_DT;
    // 红外校准pid
    IRCalAnglePIDParams.pid = IR_ANGLE_PID;
    IRCalAnglePIDParams.limit = IR_ANGLE_OUTPUT_LIMITS;
    IRCalAnglePIDParams.dt = PID_DT;
    IRCalDisPIDParams.pid = IR_DISTANCE_PID;
    IRCalDisPIDParams.limit = IR_DISTANCE_OUTPUT_LIMITS;
    IRCalDisPIDParams.dt = PID_DT;
    // 单红外校准pid
    SingleIRCalAnglePIDParams.pid = SINGLE_IR_ANGLE_PID;
    SingleIRCalAnglePIDParams.limit = SINGLE_IR_ANGLE_OUTPUT_LIMITS;
    SingleIRCalAnglePIDParams.dt = PID_DT;
    SingleIRCalDisPIDParams.pid = SINGLE_IR_DISTANCE_PID;
    SingleIRCalDisPIDParams.limit = SINGLE_IR_DISTANCE_OUTPUT_LIMITS;
    SingleIRCalDisPIDParams.dt = PID_DT;

    SingleIRCalibE.Angle = 0.5;
    SingleIRCalibE.Dis = 0.5;
    SingleIRCalibE.CNT = 3;
    SingleIRCalibE.LeftRightE = 1;

    // 超声波校准pid
    USCalAnglePIDParams.pid = US_ANGLE_PID;
    USCalAnglePIDParams.limit = US_ANGLE_OUTPUT_LIMITS;
    USCalAnglePIDParams.dt = PID_DT;
    USCalDisPIDParams.pid = US_DISTANCE_PID;
    USCalDisPIDParams.limit = US_DISTANCE_OUTPUT_LIMITS;
    USCalDisPIDParams.dt = PID_DT;
    // 雷达校准pid
    LidarCalAnglePIDParams.pid = LIDAR_ANGLE_PID;
    LidarCalAnglePIDParams.limit = LIDAR_ANGLE_PID_OUTPUT_LIMITS;
    LidarCalAnglePIDParams.dt = CTRL_DT;
    LidarCalDisPIDParams.pid = LIDAR_DISTANCE_PID;
    LidarCalDisPIDParams.limit = LIDAR_DISTANCE_PID_OUTPUT_LIMITS;
    LidarCalDisPIDParams.dt = CTRL_DT;
    // 雷达沿墙行驶pid
    AlongWallPIDParams.pid = ALONG_WALL_PID;
    AlongWallPIDParams.limit = ALONG_WALL_PID_LIMITS;
    AlongWallPIDParams.dt = CTRL_DT;
    // 坐标移动pid
    VxPIDParams = VX_PID;
    VyPIDParams = VY_PID;
    VzPIDParams = VZ_PID;

    ////////////////////////////////////////////////////

    //雷达校准pid误差配置
    LidarCalibE.Angle.E_Range = LIDAR_ANGLE_E_MAX;
    LidarCalibE.Angle.CNT = LIDAR_ANGLE_CNT_MIN;
    LidarCalibE.Dis.E_Range = LIDAR_DISTANCE_E_MAX;
    LidarCalibE.Dis.CNT = LIDAR_DISTANCE_CNT_MIN;
    LidarCalibE.leftRightE = LIDAR_ANGLE_PID_E_MAX;

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
    // Robot::getInstance().chassis_ctrl->set_v_pidlimit(VxPIDParams.limit);

    LidarParams.InitAngle = LIDAR_INIT_ANGLE;
    LidarParams.CalAngle = LIDAR_CALIB_ANGLE;
}

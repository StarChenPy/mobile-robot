#pragma once
#include "util/params.h"

struct LidarCalibError {
    PIDErrorParams Dis;
    PIDErrorParams Angle;
    double leftRightE{};
};

struct SensorCalibError {
    double Angle;
    double Dis;
    int32_t CNT;
    double LeftRightE;
    SensorCalibError() : Angle(0), Dis(0), CNT(0), LeftRightE(0) {}
    SensorCalibError(double angle, double dis, int32_t cnt, double LeftRightE)
        : Angle(angle), Dis(dis), CNT(cnt), LeftRightE(LeftRightE) {}
};

//标定数据
struct Point {
    float x;
    float y;
    Point() : x(0.0), y(0.0) {}
    Point(float x, float y) : x(x), y(y) {}
};
struct ImgCalPoints {
    Point P1;
    Point P2;
    Point P3;
    Point P4;
};
struct CalImg {
    ImgCalPoints Image;
    ImgCalPoints Object;
};

//雷达参数
struct LidarInitParams {
    double InitAngle; //安装位置
    double CalAngle;  //校准所采样的雷达角度，默认-10°~10°
    LidarInitParams() : InitAngle(-90), CalAngle(10) {}
    LidarInitParams(double initAngle, double calAngle) : InitAngle(initAngle), CalAngle(calAngle) {}
};

// 底盘左轮电机
extern PIDCtrlParams LeftMotorPIDParams;
// 底盘右轮电机
extern PIDCtrlParams RightMotorPIDParams;
// OMS旋转电机
extern PIDCtrlParams TurnMotorPIDParams;
// OMS升降电机
extern PIDCtrlParams LiftMotorPIDParams;

// 电机位置环
extern PIDCtrlParams TurnDisPIDParams;
extern PIDCtrlParams LiftDisPIDParams;

// 红外校准pid
extern PIDCtrlParams IRCalAnglePIDParams;
extern PIDCtrlParams IRCalDisPIDParams;

// 单红外校准pid
extern PIDCtrlParams SingleIRCalAnglePIDParams;
extern PIDCtrlParams SingleIRCalDisPIDParams;

// 超声波校准pid
extern PIDCtrlParams USCalAnglePIDParams;
extern PIDCtrlParams USCalDisPIDParams;

// 雷达校准pid
extern PIDCtrlParams LidarCalAnglePIDParams;
extern PIDCtrlParams LidarCalDisPIDParams;

// 雷达沿墙行驶pid
extern PIDCtrlParams AlongWallPIDParams;

// 坐标移动pid
extern CtrlPID VxPIDParams;
extern CtrlPID VyPIDParams;
extern CtrlPID VzPIDParams;

//雷达校准pid误差配置
extern LidarCalibError LidarCalibE;
//红外校准pid误差配置
extern SensorCalibError IRCalibE;
//超声波校准pid误差配置
extern SensorCalibError USCalibE;
//单红外校准pid误差配置
extern SensorCalibError SingleIRCalibE;

//标定数据
extern CalImg ImgCalData;
//雷达参数
extern LidarInitParams LidarParams;

void VisionCalInit();
void ParamsInit();

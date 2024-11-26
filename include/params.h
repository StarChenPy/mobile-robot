#pragma once

#include <bits/stdint-intn.h>

// 机器人参数
// 测试精度比例 参考(测量值/计算值)
#define ACCURACY_K 95.1 / 98.2
// 轮子直径
#define WHEEL_DIAMETER 12.5 * ACCURACY_K
// 车身宽度，轮子间距
#define CAR_WIDTH 32.5
// 电机光栅数
#define GRATING_NUM 1440.0
// 轮子控制周期
#define MOTOR_CTRL_PERIOD 20.0
// 数据采样周期
#define ODOM_PERIOD 20.0

// 雷达安装初始角度
#define LIDAR_INIT_ANGLE -90
// 雷达校准采样角度范围 参考-10°到10°
#define LIDAR_CALIB_ANGLE 10.0
// 雷达校准pid参数 {p, i, d, limit}
#define LIDAR_CALIB_PID                                                                                                \
    { 0.5, 0.3, 0, 10 }

// 舵机
// 夹手舵机 最小值 合
#define CLAMP_SERVO_MIN 0.05
// 夹手舵机 最大值 开
#define CLAMP_SERVO_MAX 0.255
// 抬手舵机 最小值 低
#define RAISE_SERVO_MIN 0.05
// 抬手舵机 最大值 高
#define RAISE_SERVO_MAX 0.19
// 伸缩舵机 最小值 后, 0.1最后
#define TELESCOPIC_SERVO_MIN 0.095
// 伸缩舵机 最大值 前
#define TELESCOPIC_SERVO_MAX 0.25
// 旋转舵机 最小值 后, 0.1最后
#define ROTATING_SERVO_MIN 0.24
// 旋转舵机 最大值 前
#define ROTATING_SERVO_MAX 0.11

// 夹手舵机  默认 安装值
#define CLAMP_SERVO_DEFAULE 0.10
// 伸缩舵机  默认 安装位
#define TELESCOPIC_SERVO_DEFAULE 0.10
// 抬手舵机  默认 安装位
#define RAISE_SERVO_DEFAULE 0.05
// 旋转舵机  默认 安装位
#define ROTATING_SERVO_DEFAULE 0.18 //对应0°

// 舵机标定参数设置
// 夹手舵机宽度 最小值 合  2cm
#define CLAMP_LEN_MIN 1.3
// 夹手舵机宽度 最大值 开  24.2cm
#define CLAMP_LEN_MAX 24.2
// 夹手舵机A
#define CLAMP_LEN_A (CLAMP_SERVO_MAX - CLAMP_SERVO_MIN) / (CLAMP_LEN_MAX - CLAMP_LEN_MIN)
// 夹手舵机B
#define CLAMP_LEN_B                                                                                                    \
    (CLAMP_LEN_MAX * CLAMP_SERVO_MIN - CLAMP_LEN_MIN * CLAMP_SERVO_MAX) / (CLAMP_LEN_MAX - CLAMP_LEN_MIN)

// 抬手舵机角度 最小值 0°
#define RAISE_ANGLE_MIN 0.0
// 抬手舵机角度 最大值 90°
#define RAISE_ANGLE_MAX 90.0
// 抬手舵机A
#define RAISE_ANGLE_A (RAISE_SERVO_MAX - RAISE_SERVO_MIN) / (RAISE_ANGLE_MAX - RAISE_ANGLE_MIN)
// 抬手舵机B
#define RAISE_ANGLE_B                                                                                                  \
    (RAISE_ANGLE_MAX * RAISE_SERVO_MIN - RAISE_ANGLE_MIN * RAISE_SERVO_MAX) / (RAISE_ANGLE_MAX - RAISE_ANGLE_MIN)

// 伸缩舵机距离 最小值 单位cm
#define TELESCOPIC_DIS_MIN -3.3
// 伸缩舵机距离 最大值 单位cm
#define TELESCOPIC_DIS_MAX 9
// 抬手舵机A
#define TELESCOPIC_DIS_A (TELESCOPIC_SERVO_MAX - TELESCOPIC_SERVO_MIN) / (TELESCOPIC_DIS_MAX - TELESCOPIC_DIS_MIN)
// 抬手舵机B
#define TELESCOPIC_DIS_B                                                                                               \
    (TELESCOPIC_DIS_MAX * TELESCOPIC_SERVO_MIN - TELESCOPIC_DIS_MIN * TELESCOPIC_SERVO_MAX) /                          \
        (TELESCOPIC_DIS_MAX - TELESCOPIC_DIS_MIN)

// 旋转舵机角度 最小值 0°
#define ROTATING_ANGLE_MIN -90.0
// 旋转舵机角度 最大值 90°
#define ROTATING_ANGLE_MAX 90.0
// 旋转舵机A
#define ROTATING_ANGLE_A (ROTATING_SERVO_MAX - ROTATING_SERVO_MIN) / (ROTATING_ANGLE_MAX - ROTATING_ANGLE_MIN)
// 旋转舵机B
#define ROTATING_ANGLE_B                                                                                               \
    (ROTATING_ANGLE_MAX * ROTATING_SERVO_MIN - ROTATING_ANGLE_MIN * ROTATING_SERVO_MAX) /                              \
        (ROTATING_ANGLE_MAX - ROTATING_ANGLE_MIN)

// 水果标签号和大小
#define APPLE_LABEL 0
#define APPLE_SIZE 10

// PID参数
//电机速度环
#define LEFTMOTORPID                                                                                                   \
    { 0.2, 0.005, 0 }
#define RIGHTMOTORPID                                                                                                  \
    { 0.2, 0.005, 0 }
#define TURNMOTORPID                                                                                                   \
    { 0.2, 0.005, 0 }
#define LIFTMOTORPID                                                                                                   \
    { 0.2, 0.005, 0 }
#define LEFTMOTOROUTPUTLIMITS                                                                                          \
    { -100, 100 }
#define RIGHTMOTOROUTPUTLIMITS                                                                                         \
    { -100, 100 }
#define TURNMOTOROUTPUTLIMITS                                                                                          \
    { -100, 100 }
#define LIFTMOTOROUTPUTLIMITS                                                                                          \
    { -100, 100 }

//电机位置环
#define LIFTMOTORDISTANCEPIDPARAMS                                                                                     \
    { 0.05, 0.0, 0.01 }
#define TURNMOTORDISTANCEPIDPARAMS                                                                                     \
    { 0.05, 0.0, 0.01 }
#define LIFTMOTODISTANCELIMITS                                                                                         \
    { -30, 30 }
#define TURNMOTODISTANCELIMITS                                                                                         \
    { -30, 30 }
//误差配置
#define LIFTMOTORDISTANCEERROR 10
#define LIFTMOTORDISTANCECounter 5
#define TURNMOTORDISTANCEERROR 10
#define TURNMOTORDISTANCECounter 5

// IR校准角度和距离的PID参数
#define IRDISTANCEPID                                                                                                  \
    { 1.5, 0.0, 0 }
#define IRANGLEPID                                                                                                     \
    { 0.5, 0.0, 0.0 }
#define IRDISTANCEOUTPUTLIMITS                                                                                         \
    { -20, 20 }
#define IRANGLEOUTPUTLIMITS                                                                                            \
    { -10, 10 }

// 单IR校准角度和距离的PID参数
#define SINGLEIRDISTANCEPID                                                                                            \
    { 1.5, 0.0, 0 }
#define SINGLEIRANGLEPID                                                                                               \
    { 1.0, 0.0, 0.0 }
#define SINGLEIRDISTANCEOUTPUTLIMITS                                                                                   \
    { -20, 20 }
#define SINGLEIRANGLEOUTPUTLIMITS                                                                                      \
    { -10, 10 }

// US校准角度和距离的PID参数
#define USDISTANCEPID                                                                                                  \
    { 1.5, 0.0, 0 }
#define USANGLEPID                                                                                                     \
    { 0.5, 0.0, 0.0 }
#define USDISTANCEOUTPUTLIMITS                                                                                         \
    { -20, 20 }
#define USANGLEOUTPUTLIMITS                                                                                            \
    { -10, 10 }

// 雷达校准
#define LIDARANGLEPID                                                                                                  \
    { 1.5, 0.0, 0.1 }
#define LIDARDISTANCEPID                                                                                               \
    { 1.5, 0.0, 0.1 }
#define LIDARANGLEPIDOUTPUTLIMITS                                                                                      \
    { -10, 10 }
#define LIDARDISTANCEPIDOUTPUTLIMITS                                                                                   \
    { -20, 20 }
//误差配置
#define LIDAR_ANGLE_PID_E_MAX 0.5
#define LIDAR_ANGLE_CNT_MIN 5
#define LIDAR_DISTANCE_CNT_MIN 5
#define LIDAR_ANGLE_E_MAX 0.3
#define LIDAR_DISTANCE_E_MAX 0.5

// 雷达沿墙行驶
#define ALONGWALLPID                                                                                                   \
    { 1, 0.0, 0.1 }
#define ALONGWALLPIDLIMITS                                                                                             \
    { -5, 5 }

// 坐标移动pid
#define VXPID                                                                                                          \
    { 2.5, 1.5, 0, 60 }
#define VYPID                                                                                                          \
    { 2.5, 3.5, 0, 40 }
#define VZPID                                                                                                          \
    { 1.0, 4.0, 0, 180 }
// 控制参数
#define V_MAX 100.0    //限制最高速度
#define W_MAX 360.0    //限制最高角速度
#define V_MIN 0.5      //低于某个值判定为0
#define W_MIN 0.5      //低于某个值判定为0
#define V_INC_MAX 30.0 //限制最大加速度
#define W_INC_MAX 60.0 //限制最大角加速度

//控制周期
#define PIDDT 0.02
#define CTRLDT 0.1

struct PIDParams {
    double kp;
    double ki;
    double kd;
    PIDParams() : kp(0), ki(0), kd(0) {}
    PIDParams(double p, double i, double d) : kp(p), ki(i), kd(d) {}
};
struct PIDOutputLimits {
    double min;
    double max;
    PIDOutputLimits() : min(0), max(0) {}
    PIDOutputLimits(double min, double max) : min(min), max(max) {}
};
struct PIDCtrlParams {
    PIDParams pid;
    PIDOutputLimits limit;
    double dt;
};

struct PIDErrorParams {
    double E_Range;
    int32_t CNT;
    PIDErrorParams() : E_Range(0), CNT(0) {}
    PIDErrorParams(double e, int32_t cnt) : E_Range(e), CNT(cnt) {}
};

//位姿
struct Pose {
    double x_;
    double y_;
    double theta_;
    Pose() : x_(0.0), y_(0.0), theta_(0.0) {}
    Pose(double x, double y, double theta) : x_(x), y_(y), theta_(theta) {}
};

//控制量
struct CarCtrlVal {
    double v = 0; //速度
    double w = 0; //角速度
    CarCtrlVal() : v(0), w(0) {}
    CarCtrlVal(double v, double w) : v(v), w(w) {}
};

// pid结构体
struct CtrlPID {
    double P, I, D, limit;
    CtrlPID() : P(0.0), I(0.0), D(0.0), limit(0.0) {}
    CtrlPID(double p, double i, double d, double limit) : P(p), I(i), D(d), limit(limit) {}
};
struct Error {
    double Current_Error;  //当起误差
    double Last_Error;     //上次误差
    double Previous_Error; //上上次误差
    Error() : Current_Error(0.0), Last_Error(0.0), Previous_Error(0.0) {}
    Error(double cur, double last, double pre) : Current_Error(cur), Last_Error(last), Previous_Error(pre) {}
};

// lidar数据结构体
struct LidarData {
    double Range;     //距离,m
    double Angle;     //角度，弧度制
    double Intensity; //质量
    LidarData() : Range(0.0), Angle(0.0), Intensity(0.0) {}
    LidarData(double r, double ang, double intensity) : Range(r), Angle(ang), Intensity(intensity) {}
};
// 极坐标结构体
struct Polar {
    double range; //距离
    double angle; //角度，角度制
    Polar() : range(0), angle(0) {}
    Polar(double r, double ang) : range(r), angle(ang) {}
};
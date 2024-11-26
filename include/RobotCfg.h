#pragma once
#include "RobotGenius.h"
#include "share.h"
#include "system/LidarDrive.h"

#include <hal/Titan.h>
#include <sensor/ultrasound.h>
#include <unistd.h>

#include "system/SysParams.h"

using namespace std;
using namespace robot;
using namespace VMX;
using namespace Titan;
using namespace Sensor;

// 底盘左轮电机
extern Motor::ptr LeftMotor;
extern ENC::ptr LeftENC;
// 底盘右轮电机
extern Motor::ptr RightMotor;
extern ENC::ptr RightENC;
// OMS旋转电机
extern Motor::ptr TurnMotor;
extern ENC::ptr TurnENC;
// OMS升降电机
extern Motor::ptr LiftMotor;
extern ENC::ptr LiftENC;

// 夹手舵机
extern PWM::ptr ClampServo;
// 抬手舵机
extern PWM::ptr RaiseServo;
// 伸缩舵机
extern PWM::ptr TelescopicServo;
// 旋转舵机
extern PWM::ptr RotatingServo;

// 升降上限位
extern DI::ptr LiftLimit;
// 升降下限位
extern AI::ptr LiftDownLimit;
// 播种触发限位
extern DI::ptr SeedingLimit;
// 旋转触发限位
extern AI::ptr TurningLimit;

// 急停触发限位
extern DI::ptr StopLimit;
// 红外线读取
extern AI::ptr IR_left;
extern AI::ptr IR_right;

// 激光雷达
extern Sensor::LiDAR::Ptr lidar;

// Start按钮灯光
extern PWM::ptr StartLed;
// ResetLed按钮灯光
extern PWM::ptr ResetLed;
// StopLed按钮灯光
extern PWM::ptr StopLed;
//按钮信号
extern TitanQuandLimit::ptr Button;

//右超声波
extern Ultrasound::Ptr Rightus;
//左超声波
extern Ultrasound::Ptr Leftus;

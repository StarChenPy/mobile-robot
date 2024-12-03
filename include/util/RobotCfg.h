#include "RobotGenius.h"
#include "system/LidarDrive.h"

#include "hal/Titan.h"
#include "system/Ultrasound.h"
#include <shared_mutex>
#include <unistd.h>

#include "system/SysParams.h"

#pragma once

using namespace std;
using namespace robot;
using namespace VMX;
using namespace Titan;
using namespace robot_sensor;

// 底盘左轮电机
extern Motor::ptr leftMotor;
extern ENC::ptr leftEnc;
// 底盘右轮电机
extern Motor::ptr rightMotor;
extern ENC::ptr rightEnc;
// OMS旋转电机
extern Motor::ptr turnMotor;
extern ENC::ptr turnEnc;
// OMS升降电机
extern Motor::ptr liftMotor;
extern ENC::ptr liftEnc;

// 夹手舵机
extern PWM::ptr clampServo;
// 抬手舵机
extern PWM::ptr raiseServo;
// 伸缩舵机
extern PWM::ptr telescopicServo;
// 旋转舵机
extern PWM::ptr rotatingServo;

// 升降上限位
extern DI::ptr liftLimit;
// 升降下限位
extern AI::ptr liftDownLimit;
// 播种触发限位
extern DI::ptr seedingLimit;
// 旋转触发限位
extern AI::ptr turningLimit;

// 急停触发限位
extern DI::ptr stopLimit;
// 红外线读取
extern AI::ptr irLeft;
extern AI::ptr irRight;

// 激光雷达
extern robot_sensor::LiDAR::ptr lidar;

// Start按钮灯光
extern PWM::ptr startLed;
// ResetLed按钮灯光
extern PWM::ptr resetLed;
// StopLed按钮灯光
extern PWM::ptr stopLed;
//按钮信号
extern TitanQuandLimit::ptr titanButton;

//右超声波
extern Ultrasound::ptr rightUs;
//左超声波
extern Ultrasound::ptr leftUs;

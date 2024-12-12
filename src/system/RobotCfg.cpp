#include "system/RobotCfg.h"
using namespace std;
using namespace robot;
using namespace VMX;
using namespace Titan;

// 底盘左轮电机
Motor::ptr leftMotor = std::make_shared<Motor>(0);
ENC::ptr leftEnc = std::make_shared<ENC>(0);
// 底盘右轮电机
Motor::ptr rightMotor = std::make_shared<Motor>(1);
ENC::ptr rightEnc = std::make_shared<ENC>(1);
// OMS旋转电机
Motor::ptr turnMotor = std::make_shared<Motor>(3);
ENC::ptr turnEnc = std::make_shared<ENC>(3);
// OMS升降电机
Motor::ptr liftMotor = std::make_shared<Motor>(2);
ENC::ptr liftEnc = std::make_shared<ENC>(2);

// 夹手舵机
PWM::ptr clampServo = std::make_shared<PWM>(14, 100);
// 抬手舵机
PWM::ptr raiseServo = std::make_shared<PWM>(15, 100);
// 伸缩舵机
PWM::ptr telescopicServo = std::make_shared<PWM>(16, 100);
// 旋转舵机
PWM::ptr rotatingServo = std::make_shared<PWM>(17, 100);

// 升降上限位
DI::ptr liftLimit = std::make_shared<DI>(9);
// 升降下限位
AI::ptr liftDownLimit = std::make_shared<AI>(24);
// 播种触发限位
DI::ptr seedingLimit = std::make_shared<DI>(27);
// 旋转限位
AI::ptr turningLimit = std::make_shared<AI>(25);

// 急停触发限位
DI::ptr stopLimit = std::make_shared<DI>(11);

// 红外数据读取
AI::ptr irLeft = std::make_shared<AI>(22);
AI::ptr irRight = std::make_shared<AI>(23);

// 激光雷达
robot_sensor::LiDAR::ptr lidar = std::make_shared<robot_sensor::LiDAR>("/dev/ttyUSB0");

// Start按钮灯光
PWM::ptr startLed = std::make_shared<PWM>(21);
// RestLed按钮灯光
PWM::ptr resetLed = std::make_shared<PWM>(20);
// StopLed按钮灯光
PWM::ptr stopLed = std::make_shared<PWM>(19);
//按钮信号
TitanQuandLimit::ptr titanButton = std::make_shared<TitanQuandLimit>();

//右超声波
Ultrasound::ptr rightUs = std::make_shared<Ultrasound>(12, 8);
//左超声波
Ultrasound::ptr leftUs = std::make_shared<Ultrasound>(13, 10);
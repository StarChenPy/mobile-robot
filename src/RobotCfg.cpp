
#include "RobotCfg.h"
using namespace std;
using namespace RobotGenius;
using namespace VMX;
using namespace Titan;

// 底盘左轮电机
Motor::ptr LeftMotor = std::make_shared<Motor>(1);
ENC::ptr LeftENC = std::make_shared<ENC>(1);
// 底盘右轮电机
Motor::ptr RightMotor = std::make_shared<Motor>(2);
ENC::ptr RightENC = std::make_shared<ENC>(2);
// OMS旋转电机
Motor::ptr TurnMotor = std::make_shared<Motor>(3);
ENC::ptr TurnENC = std::make_shared<ENC>(3);
// OMS升降电机
Motor::ptr LiftMotor = std::make_shared<Motor>(0);
ENC::ptr LiftENC = std::make_shared<ENC>(0);


// 夹手舵机
PWM::ptr ClampServo = std::make_shared<PWM>(14,100);
// 抬手舵机
PWM::ptr RaiseServo = std::make_shared<PWM>(15,100);
// 伸缩舵机
PWM::ptr TelescopicServo = std::make_shared<PWM>(16,100);
// 旋转舵机
PWM::ptr RotatingServo = std::make_shared<PWM>(17,100);

// 升降上限位
DI::ptr LiftLimit = std::make_shared<DI>(9);
// 升降下限位
AI::ptr LiftDownLimit = std::make_shared<AI>(24);
// 播种触发限位
DI::ptr SeedingLimit = std::make_shared<DI>(27);
// 旋转限位
AI::ptr TurningLimit = std::make_shared<AI>(25);

// 急停触发限位
DI::ptr StopLimit = std::make_shared<DI>(11);

// 红外数据读取
AI::ptr IR_left = std::make_shared<AI>(22);
AI::ptr IR_right = std::make_shared<AI>(23);

// 激光雷达
Sensor::LiDAR::Ptr  lidar = std::make_shared<Sensor::LiDAR>("/dev/ttyUSB0");




//Start按钮灯光
PWM::ptr StartLed = std::make_shared<PWM>(19);
//RestLed按钮灯光
PWM::ptr ResetLed = std::make_shared<PWM>(20);
//StopLed按钮灯光
PWM::ptr StopLed = std::make_shared<PWM>(21);
//按钮信号
TitanQuandLimit::ptr Button = std::make_shared<TitanQuandLimit>();

//右超声波
Ultrasound::Ptr Rightus = std::make_shared<Ultrasound>(12, 8);
//左超声波
Ultrasound::Ptr Leftus = std::make_shared<Ultrasound>(13,10);
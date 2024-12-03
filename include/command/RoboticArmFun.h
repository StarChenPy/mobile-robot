/**
 * @file RoboticArmFun.hpp
 * @author Zijian.Yan (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "system/Robot.h"
#include "util/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

#include "command/sensor/ENCComand.h"
#include "command/ServoCommand.h"
#include "command/motor/MotorPIDCommand.h"
#include "command/motor/MotorDistancePIDCommand.h"

using namespace std;
using namespace robot;

//伸缩舵机逐步运行动作
ICommand::ptr TelescopicCtrlAction(double start, double end);

//夹手舵机逐步运行动作
ICommand::ptr ClampCtrlAction(double start, double end);

//抬手舵机逐步运行动作
ICommand::ptr RaiseCtrlAction(double start, double end);

//升降旋转置零动作
ICommand::ptr ResetLiftAndTurn(double turn_speed);

//抓手并行
ICommand::ptr GripperServoPG(double telescopic, double raise, double clamp);

//升降旋转并行
ICommand::ptr LiftAndTurnPG(double h, double angle);

//重置机械臂
ICommand::ptr ResetRoboticArm();

//车辆移动机械臂状态
ICommand::ptr CarMoveStatus();

//车辆移动机械臂状态并行
ICommand::ptr CarMoveStatusPG();

//初始状态
ICommand::ptr InitStatus();

// 限高
ICommand::ptr HeightLimitStatus();

//看水果抓手状态
ICommand::ptr VisionServoStatus();

//看水果机械臂状态
ICommand::ptr VisionStatus();

ICommand::ptr VisionStatusPG();

//抓水果动作
ICommand::ptr PickFruitStatus(int layer);

//抓水果动作
ICommand::ptr PickFruitSG();

//抓篮子
ICommand::ptr PickBasketAction();

//放篮子
ICommand::ptr PutBasketAction();

//放篮子
ICommand::ptr PutFruit2BasketAction();

//放水果进篮子
ICommand::ptr PutFruit2BasketAction_1(int DiJiGe , int ZuoYou);

//放水果进篮子并行
ICommand::ptr PutFruit2BasketPG(int DiJiGe , int ZuoYou);
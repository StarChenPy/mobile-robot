#pragma once
#include "system/Robot.h"
#include "system/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"
#include "command/sensor/ENCComand.h"
#include "command/ServoCommand.h"
#include "command/motor/MotorPIDCommand.h"
#include "command/motor/MotorDistancePIDCommand.h"

//-------------------------------- Reset Functions --------------------------------//

/**
 * 重置机械臂的抬升和旋转部件
 * @return 并行命令组
 */
ICommand::ptr resetLiftAndTurn();

/**
 * 重置整个机械臂，包括抬升、旋转和抓手部件
 * @return 并行命令组
 */
ICommand::ptr resetRoboticArm();

/**
 * 将机械臂设置为默认位置
 * @return 顺序命令组
 */
ICommand::ptr moveToDefault();

//-------------------------------- Control Functions --------------------------------//

/**
 * 控制机械臂的伸缩动作
 * @param start 起始位置
 * @param end 结束位置
 * @return 顺序命令组
 */
ICommand::ptr telescopicControlAction(double start, double end);

/**
 * 控制机械臂夹持器的动作
 * @param start 起始长度
 * @param end 结束长度
 * @return 顺序命令组
 */
ICommand::ptr clampControlAction(double start, double end);

/**
 * 控制机械臂的升降动作
 * @param start 起始角度
 * @param end 结束角度
 * @return 顺序命令组
 */
ICommand::ptr raiseControlAction(double start, double end);

/**
 * 组合控制机械臂的抓手动作，包括升降、伸缩和夹持
 * @param telescopic 伸缩位置
 * @param raise 升降角度
 * @param clamp 夹持长度
 * @return 并行命令组
 */
ICommand::ptr gripperServoAction(double telescopic, double raise, double clamp);

/**
 * 控制机械臂的抬升和旋转动作
 * @param height 抬升高度
 * @param angle 旋转角度
 * @return 并行命令组
 */
ICommand::ptr liftAndTurnAction(double height, double angle);

//-------------------------------- Complex Actions --------------------------------//

/**
 * 将机械臂调整为车辆移动状态
 * @return 顺序命令组
 */
ICommand::ptr carMoveStatus();

/**
 * 将机械臂调整为车辆移动状态（并行方式）
 * @return 并行命令组
 */
ICommand::ptr carMoveStatusParallel();

/**
 * 根据层数调整机械臂以采摘果实
 * @param layer 果实所在层数
 * @return 顺序命令组
 */
ICommand::ptr pickFruitStatus(int layer);

/**
 * 执行采摘果篮的动作
 * @return 顺序命令组
 */
ICommand::ptr pickBasketAction();

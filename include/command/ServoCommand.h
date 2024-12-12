#pragma once
#include "system/Robot.h"
#include "system/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

class ClampServoCommand : public ICommand {
public:
    /**
     * 夹爪舵机控制命令
     * @param angle 夹持距离（单位：厘米）
     */
    explicit ClampServoCommand(double len);

    void execute() override;

    static ICommand::ptr create(double len);
private:
    double clampLen_;
};

class TelescopicServoCommand : public ICommand {
public:
    /**
     * 伸缩舵机控制命令
     * @param angle 伸出距离（单位：厘米）
     */
    explicit TelescopicServoCommand(double len);

    void execute() override;

    static ICommand::ptr create(double dis);
private:
    double telescopicLen_;
};

class RaiseServoCommand : public ICommand {
public:
    /**
     * 点头舵机控制命令
     * @param angle 旋转角度（单位：度）
     */
    explicit RaiseServoCommand(double angle);

    void execute() override;

    static ICommand::ptr create(double angle);
private:
    double raiseAngle_;
};

class RotatingServoCommand : public ICommand {
public:
    /**
     * 旋转舵机控制命令
     * @param angle 旋转角度（单位：度）
     */
    explicit RotatingServoCommand(double angle);

    void execute() override;

    static ICommand::ptr create(double angle);
private:
    double rotatingAngle_;
};

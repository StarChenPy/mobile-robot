/**
 * @file ServoCommand.h
 * @author Zijian.Yan (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-10-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "system/Robot.h"
#include "util/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"
using namespace std;
using namespace robot;

class ClampServoCommand : public ICommand {
  public:
    typedef std::shared_ptr<ClampServoCommand> Ptr;
    
    ClampServoCommand(double val) : ClampServo_val(val) {}
    ClampServoCommand(double val, double cnt_limit) : ClampServo_val(val), counter_limit(cnt_limit) {}
    ~ClampServoCommand() override = default;

    void execute() override;
    void end() override;

  private:
    int64_t m_last_time = 0;

    int counter = 0;
    int counter_limit = 10;
    double ClampServo_val = CLAMP_SERVO_MIN;
};

// 指令封装  len:打开宽度，单位cm
ICommand::ptr createClampServoCommand(double len);
ICommand::ptr createClampServoCommand(double len, double cnt_limit);

class TelescopicServoCommand : public ICommand {
  public:
    typedef std::shared_ptr<TelescopicServoCommand> Ptr;
    explicit TelescopicServoCommand(double val) : TelescopicServo_val(val) {}
    TelescopicServoCommand(double val, double cnt_limit) : TelescopicServo_val(val), counter_limit(cnt_limit) {}
    ~TelescopicServoCommand() override = default;

    void execute() override;
    void end() override;

  private:
    int64_t m_last_time = 0;

    int counter = 0;
    double counter_limit = 10;
    double TelescopicServo_val = TELESCOPIC_SERVO_MIN;
};

// 指令封装  dis:伸缩距离，单位cm
ICommand::ptr createTelescopicServoCommand(double dis);
ICommand::ptr createTelescopicServoCommand(double dis, double cnt_limit);

class RaiseServoCommand : public ICommand {
  public:
    typedef std::shared_ptr<RaiseServoCommand> Ptr;
    RaiseServoCommand(double val) : RaiseServo_val(val) {}
    RaiseServoCommand(double val, double cnt_limit) : RaiseServo_val(val), counter_limit(cnt_limit) {}
    ~RaiseServoCommand() {}

    void execute() override;
    void end() override;

  private:
    int64_t m_last_time = 0;

    int counter = 0;
    double counter_limit = 10;
    double RaiseServo_val = RAISE_SERVO_MIN;
};

// 指令封装  angle:抬起角度，单位：度
ICommand::ptr createRaiseServoCommand(double angle);
ICommand::ptr createRaiseServoCommand(double angle, double cnt_limit);

class RotatingServoCommand : public ICommand {
  public:
    typedef std::shared_ptr<RotatingServoCommand> Ptr;
    RotatingServoCommand(double val) : RotatingServo_val(val) {}
    RotatingServoCommand(double val, double cnt_limit) : RotatingServo_val(val), counter_limit(cnt_limit) {}
    ~RotatingServoCommand() override = default;

    void execute() override;
    void end() override;

  private:
    int64_t m_last_time = 0;

    int counter = 0;
    double counter_limit = 10;
    double RotatingServo_val = RAISE_SERVO_MIN;
};

// 指令封装  angle:角度，单位：度
ICommand::ptr createRotatingServoCommand(double angle);
ICommand::ptr createRotatingServoCommand(double angle, double cnt_limit);
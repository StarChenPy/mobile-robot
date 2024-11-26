/**
 * @file RotateCommand.h
 * @author Zijian.Yan (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "RobotCfg.h"
#include "RobotGenius.h"
#include "params.h"
#include "system/Robot.h"
using namespace std;
using namespace robot;

class RotateCommand : public CommandBase {
  public:
    typedef std::shared_ptr<RotateCommand> Ptr;
    RotateCommand(double angle) : target_angle(angle) {}
    // RotateCommand(double angle, double w) : target_angle(angle), Vz_max(w) {}
    ~RotateCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;

    double Init_Phi;
    double target_angle;
    // double Vz_max = 360;
};

Command::ptr createRotateCommand(double angle);

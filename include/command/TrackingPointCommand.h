/**
 * @file UpdateOdomCommand.h
 * @author Zijian.Yan (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-07
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

class TrackingPointCommand : public ICommand {
  public:
    typedef std::shared_ptr<TrackingPointCommand> Ptr;
    TrackingPointCommand(Pose target_pose) : target(target_pose) {}
    TrackingPointCommand(Pose target_pose, double v) : target(target_pose), set_v(v) {}
    TrackingPointCommand(Pose target_pose, double v, double e_dis, double e_phi)
        : target(target_pose), set_v(v), E_dis(e_dis), E_phi(e_phi) {}
    ~TrackingPointCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;
    Pose target;
    double set_v = 20;
    double E_dis = 2;
    double E_phi = 0.5;
};

class TrackingXYCommand : public ICommand {
  public:
    typedef std::shared_ptr<TrackingXYCommand> Ptr;
    TrackingXYCommand(Pose target_pose) : target(target_pose) {}
    TrackingXYCommand(Pose target_pose, double v) : target(target_pose), set_v(v) {}
    TrackingXYCommand(Pose target_pose, double v, double e_dis, double e_phi)
        : target(target_pose), set_v(v), E_dis(e_dis), E_phi(e_phi) {}
    ~TrackingXYCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;
    Pose target;
    double Init_PHi;
    double set_v = 20;
    double E_dis = 2;
    double E_phi = 0.5;

    int step = 0;
    bool flag = false;
};

ICommand::ptr createTrackingPointCommand(Pose target_pose);
// ICommand::ptr createTrackingPointCommand(Pose target_pose, double v);
ICommand::ptr createTrackingPointCommand(Pose target_pose, double v, double e_dis = 2, double e_phi = 0.5);
ICommand::ptr createTrackingVectorCommand(const vector<Pose> &points);
ICommand::ptr createTrackingVectorCommand(const vector<Pose> &points, double v);

ICommand::ptr createTrackingXYCommand(Pose target_pose);
// ICommand::ptr createTrackingXYCommand(Pose target_pose, double v);
ICommand::ptr createTrackingXYCommand(Pose target_pose, double v, double e_dis = 2, double e_phi = 0.5);
ICommand::ptr createTrackingVectorXYCommand(const vector<Pose> &points);
ICommand::ptr createTrackingVectorXYCommand(const vector<Pose> &points, double v);
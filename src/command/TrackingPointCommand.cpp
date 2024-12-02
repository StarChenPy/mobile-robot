#include "command/TrackingPointCommand.h"

// 坐标优先，走弧线
void TrackingPointCommand::initialize() {
    is_finished = false;
    Robot::getInstance().chassis_ctrl->set_v_pidlimit(set_v);
    Robot::getInstance().chassis_ctrl->set_delta_d_max(E_dis);
    Robot::getInstance().chassis_ctrl->set_d_PHi_min(E_phi);
}
void TrackingPointCommand::execute() {
    Pose cur = Robot::getInstance().odom->getPose();
    is_finished = Robot::getInstance().chassis_ctrl->TrackingPointTask(target, cur);
    double R_setpoint = Robot::getInstance().chassis_ctrl->get_R_setpoint();
    double L_setpoint = Robot::getInstance().chassis_ctrl->get_L_setpoint();
    Robot::getInstance().setRightMotorSpeed(R_setpoint);
    Robot::getInstance().setLeftMotorSpeed(L_setpoint);

    // 打印
    Robot::getInstance().odom->print();

    int time = robot::getCurrentMs();
    m_last_time = time;
}
void TrackingPointCommand::end() {
    std::cout << "TrackingPointCommand end!" << std::endl;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
    Pose cur = Robot::getInstance().odom->getPose();
    std::cout << "End pose x = " << cur.x_ << " y = " << cur.y_ << " theta_ = " << cur.theta_ << std::endl;
}
bool TrackingPointCommand::isFinished() {
    if (Robot::getInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getInstance().getStopSignal();
}
// 指令封装
Command::ptr createTrackingPointCommand(Pose target_pose) {
    return std::make_shared<TrackingPointCommand>(target_pose)->withTimer(100);
}
Command::ptr createTrackingPointCommand(Pose target_pose,
                                        double v) { //第二个参数是设定速度
    return std::make_shared<TrackingPointCommand>(target_pose, v)->withTimer(100);
}
Command::ptr createTrackingPointCommand(Pose target_pose, double v, double e_dis,
                                        double e_phi) { //第二个参数是设定速度
    return std::make_shared<TrackingPointCommand>(target_pose, v, e_dis, e_phi)->withTimer(100);
}
Command::ptr createTrackingVectorCommand(const vector<Pose> &points) {
    SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
    for (int i = 0; i < points.size(); i++) {
        S->addCommand(std::make_shared<TrackingPointCommand>(points[i])->withTimer(100));
    }
    return S;
}
Command::ptr createTrackingVectorCommand(const vector<Pose> &points,
                                         double v) { //第二个参数是设定速度
    SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
    for (int i = 0; i < points.size(); i++) {
        S->addCommand(std::make_shared<TrackingPointCommand>(points[i], v)->withTimer(100));
    }
    return S;
}

// 角度优先，先旋转到目标点的航向角
void TrackingXYCommand::initialize() {
    is_finished = false;
    Pose InitPose = Robot::getInstance().odom->getPose();
    double dy = target.y_ - InitPose.y_;
    double dx = target.x_ - InitPose.x_;
    double targetPHi = std::atan2(dy, dx) * (180 / M_PI);
    double curPHi = InitPose.theta_;

    double temp_deltaPHi[3] = {std::abs(targetPHi - curPHi), std::abs(targetPHi - (curPHi + 360)),
                               std::abs(targetPHi - (curPHi - 360))};
    //寻找最近的旋转方向
    int minIndex = 0;
    for (int i = 1; i < 3; i++) {
        if (temp_deltaPHi[i] < temp_deltaPHi[minIndex]) {
            minIndex = i;
        }
    }
    //确定合适的取值
    if (minIndex == 1) {
        curPHi = curPHi + 360;
    } else if (minIndex == 2) {
        curPHi = curPHi - 360;
    }

    if (std::fabs(targetPHi - curPHi) > 90) { //目标点在车身后方
        Init_PHi = targetPHi + 180;
    } else {
        Init_PHi = targetPHi;
    }

    if (Init_PHi > 180) {
        Init_PHi = Init_PHi - 360;
    } else if (Init_PHi < -180) {
        Init_PHi = Init_PHi + 360;
    }

    std::cout << "Init_PHi = " << Init_PHi << std::endl;

    Robot::getInstance().chassis_ctrl->set_v_pidlimit(set_v);
    Robot::getInstance().chassis_ctrl->set_delta_d_max(E_dis);
    Robot::getInstance().chassis_ctrl->set_d_PHi_min(E_phi);
}
void TrackingXYCommand::execute() {
    Pose cur = Robot::getInstance().odom->getPose();
    if (step == 0) {
        flag = Robot::getInstance().chassis_ctrl->RotateTask(Init_PHi, cur.theta_);
        if (flag) {
            step = 1;
        }
    } else if (step == 1) {
        is_finished = Robot::getInstance().chassis_ctrl->TrackingPointTask(target, cur);
    }

    double R_setpoint = Robot::getInstance().chassis_ctrl->get_R_setpoint();
    double L_setpoint = Robot::getInstance().chassis_ctrl->get_L_setpoint();
    Robot::getInstance().setRightMotorSpeed(R_setpoint);
    Robot::getInstance().setLeftMotorSpeed(L_setpoint);

    int time = robot::getCurrentMs();
    m_last_time = time;
}
void TrackingXYCommand::end() {
    std::cout << "TrackingXYCommand end!" << std::endl;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
    Pose cur = Robot::getInstance().odom->getPose();
    std::cout << "End pose x = " << cur.x_ << " y = " << cur.y_ << " theta_ = " << cur.theta_ << std::endl;
}
bool TrackingXYCommand::isFinished() {
    if (Robot::getInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getInstance().getStopSignal();
}
// 指令封装
Command::ptr createTrackingXYCommand(Pose target_pose) {
    return std::make_shared<TrackingXYCommand>(target_pose)->withTimer(100);
}
Command::ptr createTrackingXYCommand(Pose target_pose,
                                     double v) { //第二个参数是设定速度
    return std::make_shared<TrackingXYCommand>(target_pose, v)->withTimer(100);
}
Command::ptr createTrackingXYCommand(Pose target_pose, double v, double e_dis,
                                     double e_phi) { //第二个参数是设定速度
    return std::make_shared<TrackingXYCommand>(target_pose, v, e_dis, e_phi)->withTimer(100);
}
Command::ptr createTrackingVectorXYCommand(const vector<Pose> &points) {
    SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
    for (int i = 0; i < points.size(); i++) {
        S->addCommand(std::make_shared<TrackingXYCommand>(points[i])->withTimer(100));
    }
    return S;
}
Command::ptr createTrackingVectorXYCommand(const vector<Pose> &points,
                                           double v) { //第二个参数是设定速度
    SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
    for (int i = 0; i < points.size(); i++) {
        S->addCommand(std::make_shared<TrackingXYCommand>(points[i], v)->withTimer(100));
    }
    return S;
}
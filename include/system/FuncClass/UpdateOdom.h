/**
 * @file UpdateOdomCommand.h
 * @author Zijian.Yan
 * @brief
 * @version 0.1
 * @date 2024-08-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
// #include "robotgenius/robot.h"
#include "RobotCfg.h"
#include "params.h"
// #include "system/Robot.h"
using namespace std;
// using namespace robot;

class UpdateOdom {
  public:
    typedef std::shared_ptr<UpdateOdom> Ptr;
    UpdateOdom() {
        //车辆底盘参数初始化
        round2len_k = M_PI * WHEEL_DIAMETER;
        len2round_k = 1 / round2len_k;
        round2grating_k = GRATING_NUM;
        grating2round_k = 1 / round2grating_k;
        grating2len_k = round2grating_k / round2len_k;
        len2grating_k = 1 / grating2len_k;
    }
    ~UpdateOdom() {}

    //轮子数据
    struct Whell {
        double enc_last = 0;
        double enc_new = 0;
        double enc_process = 0;
        double realSpeed = 0; //实际速度
        // double setpoint = 0;         //设定光栅速度
        // double setSpeed = 0;         //设定轮子速度
    };

    //四轮底盘模型运动学模型
    void KinematicsFunc(double v_R, double v_L) {
        CtrlValOutput.v = (v_R + v_L) * 0.5;
        CtrlValOutput.w = (v_R - v_L) / CAR_WIDTH;
    }
    //编码器速度转轮子速度
    double calWhellSpeed(double enc_process, double period) {
        // std::cout << "len2grating_k = " << len2grating_k << " enc_process = "
        // << enc_process << " (1000 / period) = " << (1000 / period) <<
        // std::endl;
        return len2grating_k * enc_process * (1000 / period);
    }
    // 更新轮子编码数据
    void UpdateWhellData(double L_emc, double R_enc, double period) {
        // double ms2s_k =
        //获取编码器
        Whell_L.enc_new = L_emc;
        Whell_R.enc_new = R_enc;
        Whell_R.enc_process = Whell_R.enc_new - Whell_R.enc_last;
        Whell_L.enc_process = Whell_L.enc_new - Whell_L.enc_last;
        Whell_R.enc_last = Whell_R.enc_new;
        Whell_L.enc_last = Whell_L.enc_new;
        Whell_R.realSpeed = calWhellSpeed(Whell_R.enc_process, period);
        Whell_L.realSpeed = calWhellSpeed(Whell_L.enc_process, period);

        // std::cout << "Whell_R.enc_process = " << Whell_R.enc_process << "
        // Whell_L.enc_process = " << Whell_L.enc_process << std::endl;
    }
    void calOdom(double gyro, double delta_t_ms) {
        //计算坐标和航向角
        Pose delta;
        double delta_t_s = delta_t_ms / 1000;
        m_pose.theta_ = gyro + Offset_gyro; //
        delta.x_ = CtrlValOutput.v * cos(m_pose.theta_ * (M_PI / 180));
        delta.y_ = CtrlValOutput.v * sin(m_pose.theta_ * (M_PI / 180));
        m_pose.x_ = m_pose.x_ + delta.x_ * delta_t_s;
        m_pose.y_ = m_pose.y_ + delta.y_ * delta_t_s;

        if (m_pose.theta_ > 180) {
            m_pose.theta_ = m_pose.theta_ - 360;
        } else if (m_pose.theta_ < -180) {
            m_pose.theta_ = m_pose.theta_ + 360;
        }
    }
    //里程计
    void CarClassisOdometer(double L_enc, double R_enc, double gyro, double delta_t_ms) {
        UpdateWhellData(L_enc, R_enc, delta_t_ms);
        KinematicsFunc(-Whell_R.realSpeed, Whell_L.realSpeed);
        calOdom(gyro, delta_t_ms);
    }
    Pose getPose() { return m_pose; }
    CarCtrlVal getSpeed() { return CtrlValOutput; }

    void zeroPose() {
        m_pose.x_ = 0;
        m_pose.y_ = 0;
        m_pose.theta_ = 0;
        zeroYaw();
        Offset_gyro = 0;
    }
    void setPose(double x, double y, double theta) {
        zeroYaw();
        Offset_gyro = theta;
        m_pose.x_ = x;
        m_pose.y_ = y;
        m_pose.theta_ = theta;
    }
    void print() {
        static int cnt = 0;
        cnt++;
        if (cnt % 50 == 0) {
            std::cout << "pose x = " << m_pose.x_ << " y = " << m_pose.y_ << " theta_ = " << m_pose.theta_ << std::endl;
            cnt = 0;
        }
    }

  private:
    Pose m_pose;
    double Offset_gyro = 0;   //陀螺仪补偿值
    Whell Whell_R;            //右轮
    Whell Whell_L;            //左轮
    CarCtrlVal CtrlValOutput; //运动控制量输出

  private:
    double len2round_k;     // 1cm对应的转数
    double round2len_k;     //一转对于多少cm
    double round2grating_k; //一转对应多少光栅数
    double grating2round_k; //一个光栅对应多少转
    double grating2len_k;   //一个光栅对应多少cm
    double len2grating_k;   // 1cm对应多少光栅数
};

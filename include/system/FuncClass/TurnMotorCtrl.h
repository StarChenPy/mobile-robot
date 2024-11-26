/**
 * @file TurnCtrl.h
 * @author Zijian.Yan
 * @brief
 * @version 0.1
 * @date 2024-08-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "RobotCfg.h"
#include "params.h"

using namespace std;

class TurnMotorCtrl {
  public:
    typedef std::shared_ptr<TurnMotorCtrl> Ptr;
    TurnMotorCtrl() {}
    ~TurnMotorCtrl() {}

    double PID_Increase(Error *sptr, CtrlPID *pid, float NowPlace, float Point) {
        double Increase; //最后得出的实际增量
        double Inc_limit = pid->limit;
        sptr->Current_Error = Point - NowPlace; // 计算当前误差

        Increase = pid->P * (sptr->Current_Error - sptr->Last_Error)                               //比例P
                   + pid->I * sptr->Current_Error                                                  //积分I
                   + pid->D * (sptr->Current_Error - 2 * sptr->Last_Error + sptr->Previous_Error); //微分D

        sptr->Previous_Error = sptr->Last_Error; // 更新前次误差
        sptr->Last_Error = sptr->Current_Error;  // 更新上次误差

        if (Increase > Inc_limit) //防止变化过大
        {
            Increase = Inc_limit;
        } else if (Increase < -Inc_limit) {
            Increase = -Inc_limit;
        }

        return Increase; // 返回增量
    }

    void updateTurnAngle(double turn_enc) {
        double k = 0.4;
        double cur_enc = turn_enc;
        double delta_angle = (cur_enc - m_enc_) * k * grating2angle;
        m_enc_ = cur_enc;
        TurnAngle += delta_angle;
    }

    double getTurnAngle() { return TurnAngle; }

    void resetTurnAngle() { TurnAngle = 0; }

    void print() { std::cout << "TurnAngle = " << TurnAngle << std::endl; }

    bool ctrlTurnAngle(double target_angle, double curAngle) {
        bool result = false;
        double delta_angle = std::fabs(target_angle - curAngle);
        if (delta_angle < delta_angle_min) {
            A_setpoint = 0;
            result = true;
        } else {
            A_setpoint = PID_Increase(&angle_error, &angle_pid, curAngle, target_angle);
        }
        return result;
    }

    double getA_setpoint() { return A_setpoint; }

  private:
    int32_t m_enc_ = 0;
    double TurnAngle = 0;

    const double grating2angle = 360.0 / static_cast<double>(GRATING_NUM);
    const double gear_num = 64;

    double A_setpoint = 0;
    double delta_angle_min = 0.1;
    CtrlPID angle_pid = {0.2, 0.9, 0, 5};
    Error angle_error = {0, 0, 0};
};

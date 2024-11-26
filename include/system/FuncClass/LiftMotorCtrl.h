/**
 * @file LiftCtrl.h
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


class LiftMotorCtrl {
 public:
  typedef std::shared_ptr<LiftMotorCtrl> Ptr;
  LiftMotorCtrl() {}
  ~LiftMotorCtrl() {}

  double PID_Increase(Error *sptr, CtrlPID *pid, float NowPlace, float Point){
    double Increase;	      //最后得出的实际增量
    double Inc_limit = pid->limit;
    sptr->Current_Error = Point - NowPlace;	  // 计算当前误差
  
    Increase =  pid->P * (sptr->Current_Error - sptr->Last_Error)   //比例P
              + pid->I *  sptr->Current_Error                       //积分I
              + pid->D * (sptr->Current_Error - 2 * sptr->Last_Error + sptr->Previous_Error);  //微分D
    
    sptr->Previous_Error = sptr->Last_Error;  // 更新前次误差
    sptr->Last_Error = sptr->Current_Error;	  // 更新上次误差
    
    if(Increase > Inc_limit)          //防止变化过大
    {
      Increase = Inc_limit;
    }else if(Increase < -Inc_limit){
      Increase = -Inc_limit;
    }
    
    return Increase;	// 返回增量
  }

  void updateHeight(double lift_enc) {
    double k = 1*0.78;
    double cur_enc = lift_enc;
    double delta_H = (cur_enc - m_enc_) * k * diameter * M_PI / static_cast<double>(GRATING_NUM);
    m_enc_ = cur_enc;
    Height += delta_H;
  }

  double getHeight(){
    return Height;
  }

  void resetHeight(){
    Height = 0;
  }

  void print(){
    std::cout << "Lift Height = " << Height << std::endl;
  }



  bool ctrlHeight(double target_height, double curHeight){
    bool result = false;
    double delta_target_H = std::fabs(target_height - curHeight);
    if(delta_target_H < delta_H_min){
      H_setpoint = 0;
      result = true;
    }else{
      H_setpoint = PID_Increase(&h_error, &h_pid, curHeight, target_height);
    }
    return result;
  }

  double getH_setpoint(){
    return H_setpoint;
  }

  

 private:
  int32_t m_enc_ = 0;
  double Height = 0;

  const double diameter = 6.4;
  const double gear_num = 64;

  double H_setpoint = 0;
  double delta_H_min = 0.1;
  CtrlPID h_pid = {2, 1, 0, 10};
  Error h_error= {0, 0, 0};
  
  
};




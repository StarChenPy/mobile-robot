/**
 * @file LidarCalibrate.h
 * @author Zijian.Yan
 * @brief 
 * @version 0.1
 * @date 2024-08-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once
#include "RobotCfg.h"
#include "params.h"


using namespace std;


class LidarCalibrate {
 public:
  typedef std::shared_ptr<LidarCalibrate> Ptr;
  LidarCalibrate() {
    angle_pid = std::make_shared<PID>(CTRLDT);
    distance_pid = std::make_shared<PID>(CTRLDT);
    R_F_pid = std::make_shared<PID>(CTRLDT);
    R_B_pid = std::make_shared<PID>(CTRLDT);
    L_F_pid = std::make_shared<PID>(CTRLDT);
    L_B_pid = std::make_shared<PID>(CTRLDT);
  }
  ~LidarCalibrate() {}

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

  std::vector<Polar> Sampling(const vector<LidarData>& lidar_buf, double angle_min, double angle_max){
    std::vector<Polar> sample;
    for(int i = 0; i < lidar_buf.size(); i++){
      LidarData data = lidar_buf[i];
      // cout << "Range = " << data.Range * 100 << " Angle = " << data.Angle * (180/M_PI) << " i = " << i << endl;
      if(data.Angle > angle_min * (M_PI/180) && data.Angle < angle_max * (M_PI/180)){
        Polar lidar_polar;
        lidar_polar.range = data.Range * 100;
        lidar_polar.angle = data.Angle * (180/M_PI);
        sample.push_back(lidar_polar);
        // cout << "Range = " << lidar_polar.range << " Angle = " << lidar_polar.angle << " i = " << i << endl;
      }
    }

    return sample;
  }

  bool LidarCalib(std::vector<Polar> lidar_sample){
    double R_sum = 0;   //右方误差和
    double L_sum = 0;   //左方误差和
    double R_L = 0; // 右方测量的长度
    double L_L = 0; // 左方测量的长度
    int R_cnt = 0;    //右边点计数
    int L_cnt = 0;    //左边点计数

    // 
    for(int i = 0; i < lidar_sample.size(); i++){
      Polar temp = lidar_sample[i];
      if(temp.angle > 0){   //在雷达右方的点
        double R_ref = temp.range * std::cos(temp.angle * (M_PI/180));  //与墙保持一定距离的参考长距离
        R_sum = R_sum + R_ref;     //累计误差和并且进行平均
        R_cnt++;
      }else{                //在雷达左方的点
        double L_ref = temp.range * std::cos(temp.angle * (M_PI/180)); 
        L_sum = L_sum + L_ref;
        L_cnt++;
      }
    }
    R_L = R_sum / R_cnt;
    L_L = L_sum / L_cnt;
    if(R_cnt == 0 || L_cnt == 0){
      R_L = 0;
      L_L = 0;
    }
    angle_pid_param = LidarCalAnglePIDParams.pid;
    angle_pid_limits = LidarCalAnglePIDParams.limit;
      angle_pid->set_point(0);
      angle_pid->set_gains(angle_pid_param.kp, angle_pid_param.ki, angle_pid_param.kd);
      angle_pid->set_output_limits(angle_pid_limits.min, angle_pid_limits.max);
      angle_pid->set_process(R_L - L_L);
    angle_pid->calculate();

    disatance_pid_param = LidarCalDisPIDParams.pid;
    disatance_pid_limits = LidarCalDisPIDParams.limit;
      distance_pid->set_gains(disatance_pid_param.kp, disatance_pid_param.ki, disatance_pid_param.kd);
      distance_pid->set_output_limits(disatance_pid_limits.min, disatance_pid_limits.max);
      distance_pid->set_point(d_wall);
      distance_pid->set_process((R_L + L_L) / 2);
    distance_pid->calculate();

    // if (distance_pid->output_ > disatance_pid_limits.max) {
    //   distance_pid->output_ = disatance_pid_limits.max;
    // }
    // if (distance_pid->output_ < disatance_pid_limits.min) {
    //   distance_pid->output_ = disatance_pid_limits.min;
    // }
    // if (angle_pid->output_ > angle_pid_limits.max) {
    //   angle_pid->output_ = angle_pid_limits.max;
    // }
    // if (angle_pid->output_ < angle_pid_limits.min) {
    //   angle_pid->output_ = angle_pid_limits.min;
    // }

    if (abs(R_L- L_L) > LidarCalibE.LeftRightE) distance_pid->output_ = 0;
    R_Setpoint = -angle_pid->output_ - distance_pid->output_ ;
    L_Setpoint = angle_pid->output_ - distance_pid->output_ ;

    if(lidar_sample.size() < 2){    //雷达扫不到数据让机器人前进，使雷达能扫到点
      R_Setpoint = -3;
      L_Setpoint = -3;
      std::cout << "lidar_sample size < 2" << std::endl;
    }

    // //雷达扫不到数据让机器人前进，使雷达能扫到点
    // if(R_cnt < 1){
    //   R_Setpoint = disatance_pid_limits.min / 2;
    // }
    // if(L_cnt < 1){
    //   L_Setpoint = disatance_pid_limits.min / 2;
    // }
    static int angle_counter = 0;
    static int distance_counter = 0; 
    // std::cout << "abs(d_wall - (R_Setpoint+L_Setpoint)/2):" << abs(d_wall - (R_L+ L_L)/2) << std::endl;
    if (abs(d_wall - R_L) < LidarCalibE.Dis.E_Range && abs(d_wall - L_L) < LidarCalibE.Dis.E_Range) {
      distance_counter++;
    } else {
      distance_counter = 0;
    }
    if (abs(R_L- L_L) < LidarCalibE.Angle.E_Range) {
      angle_counter ++;
      if (angle_counter > LidarCalibE.Angle.CNT && distance_counter > LidarCalibE.Dis.CNT) {
        angle_counter = 0;
        distance_counter = 0;
        return true;
      }
    } else {
      angle_counter = 0;
    }
    return false;
  }

  bool LidarCalibTask(const vector<LidarData>& lidar_data, double calib_d){
    bool result = false;
    d_wall = calib_d;
    std::vector<Polar> lidar_sample = Sampling(lidar_data, -LidarParams.CalAngle, LidarParams.CalAngle);
    result = LidarCalib(lidar_sample);
    return result;
  }

  std::vector<Polar> LidarFilter(std::vector<Polar> lidar_sample, double e){
    std::vector<Polar> lidar_temp;
    int len = lidar_sample.size();
    for(int i = 0; i < len; i++){
      Polar temp = lidar_sample[i];
      double dy = temp.range * std::sin(-temp.angle * (M_PI/180));  //与墙保持一定距离的参考长距离
      if(dy < e){
        lidar_temp.push_back(temp);
      }
    }
    return lidar_temp;
  }

  bool LidarAlongRightWall(double speed, std::vector<Polar> lidar_sample){
    double F_sum = 0;   //前误差和
    double B_sum = 0;   //后误差和

    double F_L = 0; // 前方测量的长度
    double B_L = 0; // 后方测量的长度

    int F_cnt = 0;    //前边点计数
    int B_cnt = 0;    //后边点计数

    int len = lidar_sample.size();
    if(len == 0) {return false;}
    // std::cout << "len: " << len <<std::endl;
    // if(len < 10){    //当没有雷达数据
    //   R_Setpoint = 0;
    //   L_Setpoint = 0;
    //   return true;
    // }
    for(int i = 0; i < len; i++){
      Polar temp = lidar_sample[i];
      if(i < len / 2){   //在雷达前方的点
        double F_ref = temp.range * std::sin(temp.angle * (M_PI/180));  //与墙保持一定距离的参考长距离
        if(std::fabs(F_ref - d_wall) > 30){   //当扫到的点与墙大于一定的距离
          R_Setpoint = 0;
          L_Setpoint = 0;
          return true;
        }
        F_sum = F_sum + F_ref;     //累计误差和并且进行平均
        F_cnt++;
      }else{                //在雷达前方的点
        double B_ref = temp.range * std::sin(temp.angle * (M_PI/180));
        if(std::fabs(B_ref - d_wall) > 30){   //当扫到的点与墙大于一定的距离
          R_Setpoint = 0;
          L_Setpoint = 0;
          return true;
        }
        B_sum = B_sum + B_ref;
        B_cnt++;
      }
    }
    F_L = F_sum / F_cnt;
    B_L = B_sum / B_cnt;

    R_F_pid_param = AlongWallPIDParams.pid;
    R_F_pid_limits = AlongWallPIDParams.limit;
      R_F_pid->set_point(d_wall);
      R_F_pid->set_gains(R_F_pid_param.kp, R_F_pid_param.ki, R_F_pid_param.kd);
      R_F_pid->set_output_limits(R_F_pid_limits.min, R_F_pid_limits.max);
      R_F_pid->set_process(F_L);
    R_F_pid->calculate();

    R_B_pid_param = AlongWallPIDParams.pid;
    R_B_pid_limits = AlongWallPIDParams.limit;
      R_B_pid->set_gains(R_B_pid_param.kp, R_B_pid_param.ki, R_B_pid_param.kd);
      R_B_pid->set_output_limits(R_B_pid_limits.min, R_B_pid_limits.max);
      R_B_pid->set_point(d_wall);
      R_B_pid->set_process(B_L);
    R_B_pid->calculate();

    R_Setpoint = speed + R_F_pid->output_;
    L_Setpoint = speed - R_B_pid->output_;
    
    if(len < 2){    //雷达扫不到数据让机器人前进，使雷达能扫到点
      R_Setpoint = 0;
      L_Setpoint = 0;
    }
    return false;
  }

  bool LidarAlongRightWallTask(double speed, const vector<LidarData>& lidar_data, double calib_d){
    bool result = false;
    d_wall = calib_d;
    std::vector<Polar> lidar_sample = Sampling(lidar_data, 45, 80);
    result = LidarAlongRightWall(speed, lidar_sample);
    // std::vector<Polar> lidar_filter = LidarFilter(lidar_sample, 20);
    // result = LidarAlongRightWall(speed, lidar_filter);
    return result;
  }

  

  bool LidarLeftAlongWall(double speed, std::vector<Polar> lidar_sample){
    double F_sum = 0;   //前误差和
    double B_sum = 0;   //后误差和

    double F_L = 0; // 前方测量的长度
    double B_L = 0; // 后方测量的长度

    int F_cnt = 0;    //前边点计数
    int B_cnt = 0;    //后边点计数

    int len = lidar_sample.size();
    if(len == 0) {return false;}
    // std::cout << "len: " << len <<std::endl;
    // if(len < 10){    //当没有雷达数据
    //   R_Setpoint = 0;
    //   L_Setpoint = 0;
    //   return true;
    // }
    for(int i = 0; i < len; i++){
      Polar temp = lidar_sample[i];
      if(i > len / 2){   //在雷达前方的点
        double F_ref = temp.range * std::sin(-temp.angle * (M_PI/180));  //与墙保持一定距离的参考长距离
        if(std::fabs(F_ref - d_wall) > 30){   //当扫到的点与墙大于一定的距离
          R_Setpoint = 0;
          L_Setpoint = 0;
          return true;
        }
        F_sum = F_sum + F_ref;     //累计误差和并且进行平均
        F_cnt++;
      }else{                //在雷达前方的点
        double B_ref = temp.range * std::sin(-temp.angle * (M_PI/180));
        if(std::fabs(B_ref - d_wall) > 30){   //当扫到的点与墙大于一定的距离
          R_Setpoint = 0;
          L_Setpoint = 0;
          return true;
        }
        B_sum = B_sum + B_ref;
        B_cnt++;
      }
    }
    F_L = F_sum / F_cnt;
    B_L = B_sum / B_cnt;

    L_F_pid_param = AlongWallPIDParams.pid;
    L_F_pid_limits = AlongWallPIDParams.limit;
      L_F_pid->set_point(d_wall);
      L_F_pid->set_gains(L_F_pid_param.kp, L_F_pid_param.ki, L_F_pid_param.kd);
      L_F_pid->set_output_limits(L_F_pid_limits.min, L_F_pid_limits.max);
      L_F_pid->set_process(F_L);
    L_F_pid->calculate();

    L_B_pid_param = AlongWallPIDParams.pid;
    L_B_pid_limits = AlongWallPIDParams.limit;
      L_B_pid->set_gains(L_B_pid_param.kp, L_B_pid_param.ki, L_B_pid_param.kd);
      L_B_pid->set_output_limits(L_B_pid_limits.min, L_B_pid_limits.max);
      L_B_pid->set_point(d_wall);
      L_B_pid->set_process(B_L);
    L_B_pid->calculate();
    
    R_Setpoint = speed - L_F_pid->output_;
    L_Setpoint = speed + L_B_pid->output_;
    
    if(len < 2){    //雷达扫不到数据
      R_Setpoint = 0;
      L_Setpoint = 0;
    }
    return false;
  }

  bool LidarAlongLeftWallTask(double speed, const vector<LidarData>& lidar_data, double calib_d){
    bool result = false;
    d_wall = calib_d;
    std::vector<Polar> lidar_sample = Sampling(lidar_data, -80, -45);
    result = LidarLeftAlongWall(speed, lidar_sample);
    // std::vector<Polar> lidar_filter = LidarFilter(lidar_sample, 20);
    // result = LidarLeftAlongWall(speed, lidar_filter);
    return result;
  }

  

  double get_R_setpoint(){
    return R_Setpoint;
  }
  double get_L_setpoint(){
    return L_Setpoint;
  }

 private:
  double d_wall = 30;
  std::vector<Polar> LidarSample;
  double R_Setpoint = 0, L_Setpoint = 0;
  
  int reach_cnt = 0;
  int cnt_max = 8;
  double E_min = 0.3;
  double Setpoint_min = 0.2;

  PID::ptr angle_pid;
  PIDParams angle_pid_param;
  PIDOutputLimits angle_pid_limits;
  PID::ptr distance_pid;
  PIDParams disatance_pid_param;
  PIDOutputLimits disatance_pid_limits;
  PID::ptr R_F_pid;
  PIDParams R_F_pid_param;
  PIDOutputLimits R_F_pid_limits;
  PID::ptr R_B_pid;
  PIDParams R_B_pid_param;
  PIDOutputLimits R_B_pid_limits;
  PID::ptr L_F_pid;
  PIDParams L_F_pid_param;
  PIDOutputLimits L_F_pid_limits;
  PID::ptr L_B_pid;
  PIDParams L_B_pid_param;
  PIDOutputLimits L_B_pid_limits;

  // CtrlPID R_pid = LIDAR_CALIB_PID;
  // Error R_error= {0, 0, 0};
  // CtrlPID L_pid = LIDAR_CALIB_PID;
  // Error L_error = {0, 0, 0};


  // CtrlPID L_F_pid = {0.5, 0.3, 0, 5};
  // Error L_F_error= {0, 0, 0};
  // CtrlPID L_B_pid = {0.5, 0.3, 0, 5};
  // Error L_B_error = {0, 0, 0};
};




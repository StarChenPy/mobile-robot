/**
 * @file ChassisCtrl.h
 * @author Zijian.Yan
 * @brief 
 * @version 0.1
 * @date 2024-08-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once
#include "RobotCfg.h"
#include "params.h"
using namespace std;

class ChassisCtrl {
 public:
  typedef std::shared_ptr<ChassisCtrl> Ptr;
  ChassisCtrl() {
    n2Speed_k = M_PI * WHEEL_DIAMETER;
    Speed2n_k = 1 / n2Speed_k;
    n2m_setpoint_k = GRATING_NUM * static_cast<double>(MOTOR_CTRL_PERIOD)/1000;
    m_setpoint2n_k = 1 / n2m_setpoint_k;
    Speed2m_setpoint_k = n2m_setpoint_k / n2Speed_k;
    m_setpoint2Speed_k = 1 / Speed2m_setpoint_k;
  }
  ~ChassisCtrl() {}

  //轮子数据
  struct Whell{
    double setpoint = 0;         //设定光栅速度
    double setSpeed = 0;         //设定轮子速度
    // double enc_last = 0;
    // double enc_new = 0;
    // double enc_process = 0;
    // double realSpeed = 0;        //实际速度
  };

  //增量PID
  double PID_Increase(Error *sptr, CtrlPID *pid, double NowPlace, double Point)
  {
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


  /**
   * @brief 世界坐标系中的点转换为机器人坐标系中的点
   * @details
   *    worldPoint: 世界坐标系中的点
   *    robotPose: 机器人当前位姿
   * @return   
   *    robotPoint: 返回转换后的机器人坐标系中的点
   */
  Pose worldToRobotCoordinates(const Pose& worldPoint, const Pose& robotPose) {
    Pose robotPoint;
    double worldpoint_theta;
    double cosTheta = std::cos(robotPose.theta_ * (M_PI/180));
    double sinTheta = std::sin(robotPose.theta_ * (M_PI/180));
    // 平移
    double dx = worldPoint.x_ - robotPose.x_;
    double dy = worldPoint.y_ - robotPose.y_;
    // 旋转
    robotPoint.x_ = dx * cosTheta + dy * sinTheta;
    robotPoint.y_ = -dx * sinTheta + dy * cosTheta;
    if(dx == 0) {dx = 0.0000001;}
    if(dx > 0){
      worldpoint_theta = std::atan(dy/dx) * (180/M_PI);
    }else{
      worldpoint_theta = std::atan(dy/dx) * (180/M_PI) + 180;
    }
    robotPoint.theta_ = worldpoint_theta - robotPose.theta_;
    
    if(robotPoint.theta_ > 180){
      robotPoint.theta_ = robotPoint.theta_ - 360;
    }
    else if(robotPoint.theta_ < -180){
      robotPoint.theta_ = robotPoint.theta_ + 360;
    }
    return robotPoint;
  }

  //位置到达后角度旋转
  CarCtrlVal theta_rotate(double targetPHi, double curPHi, double delta_PHi_min){
    CarCtrlVal val;
    double temp_deltaPHi[3] = {std::abs(targetPHi - curPHi),
                              std::abs(targetPHi - (curPHi + 360)),
                              std::abs(targetPHi - (curPHi - 360))};
    //寻找最近的旋转方向
    int minIndex = 0;
    for(int i = 1; i < 3; i++){
      if(temp_deltaPHi[i] < temp_deltaPHi[minIndex]){
        minIndex = i;
      }
    }
    //确定合适的取值
    if(minIndex == 1) {curPHi = curPHi + 360;}
    else if(minIndex == 2) {curPHi = curPHi - 360;}
    //PID控制
    val.w = -PID_Increase(&w_error, &w_pid, curPHi, targetPHi);
    val.v = 0;

    //到达判定
    if((curPHi-targetPHi < delta_PHi_min)&&(curPHi-targetPHi > -delta_PHi_min)){
      val.w = 0;
      reachAngle_flag = true;
      // std::cout << "Reach target angle!" << std::endl;
    }else{
      reachAngle_flag = false;
    }

    return val;
  }

  // 跟踪算法
  CarCtrlVal PurePursuit(const Pose& target, const Pose& cur){
    double ld_y, ld_x;
    CarCtrlVal ctrl_val;
    Pose targetRobotCoorPoint = worldToRobotCoordinates(target, cur);   //目标坐标的机器人坐标
    double delta_d = sqrt(targetRobotCoorPoint.x_ * targetRobotCoorPoint.x_ + targetRobotCoorPoint.y_ * targetRobotCoorPoint.y_);
    // double l_d = 200;
    // if(delta_d > l_d){delta_d = l_d;}
    ld_y = delta_d * sin(targetRobotCoorPoint.theta_ * (M_PI/180));
    ld_x = delta_d * cos(targetRobotCoorPoint.theta_ * (M_PI/180));
    ctrl_val.v = PID_Increase(&v_error, &v_pid, -ld_x, 0);
    if(ld_x > 0){
      ctrl_val.w = PID_Increase(&ld_y_error, &ld_y_pid, ld_y, 0);
    }else{
      ctrl_val.w = PID_Increase(&ld_y_error, &ld_y_pid, -ld_y, 0);
    }

    if(std::fabs(ld_x) < delta_d_max){
      std::cout << "ld_x = " << ld_x << " ld_y = " << ld_y << " delta_d = " << delta_d << std::endl;
    }

    // 到达判定
    if(std::fabs(delta_d) < delta_d_max || ((std::fabs(ld_x) < (2/3)*delta_d_max) && (std::fabs(ld_y) < (5/2)*delta_d_max)))    //当相对距离小于某个值判定为到达
    {
      // reachPoint_cnt++;
      ctrl_val = theta_rotate(target.theta_, cur.theta_, d_PHi_min);
      reachPoint_flag = true;
    }else{
      // reachPoint_cnt = 0;
      reachPoint_flag = false;
    }

    if(reachPoint_flag == true && reachAngle_flag == true){
      reachPoint_cnt = 0;
      reachAngle_cnt = 0;
      reachTarget_flag = true;
      // std::cout <<"Reach target!"  << std::endl;
    }else{
      reachTarget_flag = false;
    }
    // std::cout << "v = " << ctrl_val.v << " w = " << ctrl_val.w  << std::endl;
    // std::cout << "car(" << cur.x_ << ", " << cur.y_ << ") -> "
    //           << "PathPoint(" << target.x_ << ", " << target.y_ << "), angle = " << cur.theta_ << std::endl;
    return ctrl_val;    //控制量输出
  }

  //控制量约束
  void CtrlConstraint(){
    //速度增量约束
    if(CtrlValInput.v > val_last.v + V_INC_MAX){
      CtrlValInput.v = val_last.v + V_INC_MAX;
    }else if(CtrlValInput.v < val_last.v - V_INC_MAX){
      CtrlValInput.v = val_last.v - V_INC_MAX;
    }
    //角速度增量约束
    if(CtrlValInput.w > val_last.w + W_INC_MAX){
      CtrlValInput.w = val_last.w + W_INC_MAX;
    }else if(CtrlValInput.w < val_last.w - W_INC_MAX){
      CtrlValInput.w = val_last.w - W_INC_MAX;
    }
    //最大最小值约束
    if(CtrlValInput.v > V_MAX) {CtrlValInput.v = V_MAX;}
    else if(CtrlValInput.v < V_MIN && CtrlValInput.v > -V_MIN){CtrlValInput.v = 0;}
    // else if(CtrlValInput.v < V_MIN){CtrlValInput.v = 0;}
    if(CtrlValInput.w > W_MAX) {CtrlValInput.w = W_MAX;}
    else if(CtrlValInput.w < -W_MAX) {CtrlValInput.w = -W_MAX;}
    else if(CtrlValInput.w < W_MIN && CtrlValInput.w > -W_MIN){CtrlValInput.w = 0;}

    if(std::isnan(CtrlValInput.v) || std::isnan(CtrlValInput.w)){
      CtrlValInput.v = 0;
      CtrlValInput.w = 0;
      // std::cout << "CtrlConstraint v = "<< CtrlValInput.v << " w = "<< CtrlValInput.w << std::endl;
    }
    //保存上一次控制量数值
    val_last = CtrlValInput;
  }

  //四轮底盘模型逆运动学模型
  void InverseKinematicsFunc(double v, double w){
    Whell_R.setSpeed = v + w * (M_PI/180) * CAR_WIDTH/2;
    Whell_L.setSpeed = v - w * (M_PI/180) * CAR_WIDTH/2;
  }
  //轮子速度转编码器速度
  double Speed2m_setpoint(double v_speed){
    return Speed2m_setpoint_k * v_speed;
  }

  void Whell_setpoint(){
    Whell_R.setpoint = Speed2m_setpoint(Whell_R.setSpeed);
    Whell_L.setpoint = Speed2m_setpoint(Whell_L.setSpeed);
  }

  bool TrackingPointTask(Pose target, Pose cur){
    bool result = false;
    // Pose curPose = bot_->getPose();
    updataVxPIDParams();
    updataVyPIDParams();
    updataVzPIDParams();
    CtrlValInput = PurePursuit(target, cur);
    CtrlConstraint();
    InverseKinematicsFunc(CtrlValInput.v, CtrlValInput.w);
    Whell_setpoint();
    // 到达判定
    if(reachTarget_flag == true){
      Whell_R.setpoint = 0;
      Whell_L.setpoint = 0;
      reachTarget_cnt = 0;
      result = true;
    }

    return result;
  }

  bool RotateTask(double target_PHi, double curPHi){
    bool result = false;
    updataVzPIDParams();
    CtrlValInput = theta_rotate(target_PHi, curPHi, d_PHi_min);
    // std::cout << "CtrlValInput.w = " << CtrlValInput.w << std::endl;
    // std::cout << "target_PHi = " << target_PHi << ", curPHi = " << curPHi << std::endl;
    CtrlConstraint();
    InverseKinematicsFunc(CtrlValInput.v, CtrlValInput.w);
    Whell_setpoint();
    // std::cout << "Whell_R.setpoint = " << Whell_R.setpoint << ", Whell_L.setpoint = " << Whell_L.setpoint << std::endl;

    // 到达判定
    if(reachAngle_flag == true){
      Whell_R.setpoint = 0;
      Whell_L.setpoint = 0;
      result = true;
    }
    return result;
  }

  bool VisionCtrlTask(int fruit_cx, int cap_cx){
    bool result = false;
    double d_cx = cap_cx - fruit_cx;
    double setpoint = PID_Increase(&cx_error, &cx_pid, d_cx, 0);
    
    if(std::fabs(d_cx) < d_cx_min){
      setpoint = 0;
      result = true;
    }

    std::cout << "fruit_cx = " << fruit_cx << " cap_cx = " << cap_cx << " setpoint = " << setpoint << std::endl;

    Whell_R.setpoint = setpoint;
    Whell_L.setpoint = setpoint;
    return result;
  }

  void updataVxPIDParams(){
    v_pid.P = VxPIDParams.P;
    v_pid.I = VxPIDParams.I;
    v_pid.D = VxPIDParams.D;
  }
  void updataVyPIDParams(){
    ld_y_pid.P = VxPIDParams.P;
    ld_y_pid.I = VyPIDParams.I;
    ld_y_pid.D = VyPIDParams.D;
  }
  void updataVzPIDParams(){
    w_pid.P = VzPIDParams.P;
    w_pid.I = VzPIDParams.I;
    w_pid.D = VzPIDParams.D;
  }


  void set_v_pidlimit(double v_limit){v_pid.limit = v_limit;}
  void set_w_pidlimit(double w_limit){w_pid.limit = w_limit;}
  void set_ld_y_pidlimit(double ld_y_limit){ld_y_pid.limit = ld_y_limit;}
  void set_delta_d_max(double E_dis){delta_d_max = E_dis;}
  void set_d_PHi_min(double E_phi){d_PHi_min = E_phi;}

  bool get_reachPoint_flag(){return reachPoint_flag;}
  bool get_reachAngle_flag(){return reachAngle_flag;}
  bool get_reachTarget_flag(){return reachTarget_flag;}

  double get_R_setpoint(){return Whell_R.setpoint;}
  double get_L_setpoint(){return Whell_L.setpoint;}

 private:
  bool reachPoint_flag = false;
  bool reachAngle_flag = false;
  bool reachTarget_flag = false;
  int reachPoint_cnt = 0;
  int reachAngle_cnt = 0;
  int reachTarget_cnt = 0;
  int reach_cnt_max = 3;
  double delta_d_max = 2;
  double d_PHi_min = 0.5;
  double d_cx_min = 2;

  CarCtrlVal CtrlValInput;
  CarCtrlVal val_last;
  Whell Whell_R;
  Whell Whell_L;

  double Speed2n_k;            //轮子运动速度对于轮子转速的比值, n = k * Speed
  double n2Speed_k;            //轮子转速对于轮子运动速度的比值, Speed = k * n
  double n2m_setpoint_k;       //轮子转速对于光栅速度的比值, set_point_ = k * n
  double m_setpoint2n_k;       //光栅速度对于轮子转速的比值, n = k * set_point_
  double Speed2m_setpoint_k;   //轮子运动速度对于光栅速度的比值, set_point_ = k * Speed
  double m_setpoint2Speed_k;   //光栅速度对于轮子运动速度的比值, Speed= k * m _setpoint 

  CtrlPID v_pid = VXPID;
  Error v_error= {0, 0, 0};
  CtrlPID w_pid = VZPID;
  Error w_error = {0, 0, 0};
  CtrlPID ld_y_pid = VYPID;
  Error ld_y_error = {0, 0, 0};
  CtrlPID cx_pid = {0.0, 0.04, 0, 3};
  Error cx_error= {0, 0, 0};

};


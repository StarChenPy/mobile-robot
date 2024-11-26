/**
 * @file share.h
 * @author jiapeng.lin (jiapeng.lin@high-genius,com)
 * @brief 创建共享内存
 * @version 0.1
 * @date 2023-06-26
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
// #ifndef INCLUDE_SHARE_H_
// #define INCLUDE_SHARE_H_

#include <errno.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#include <sys/epoll.h>
#include <sys/inotify.h>
#include <unistd.h>
#include <memory>

// #include "params.h"
#define COMMEND_FREE 0
#define COMMEND_CREATE 1
#define COMMEND_WAIT 2
#define COMMEND_END 3
#define COMMEND_CANCEL 10

namespace LABVIEW{

  extern "C" {
    class Semaphore {
    public:
      /// @brief 创建信号量
      /// @param key
      Semaphore(int key);
      /// @brief 摧毁信号量
      ~Semaphore();
      /// @brief P 操作，申请资源
      void P();
      /// @brief V 操作，释放资源
      void V();

    private:
      int sem_id_;
      bool semaphore_status_ = false;
    };
  }

  template <class T>
  class ShareMemory {
  public:
    typedef std::shared_ptr<ShareMemory<T>> ptr;
    ShareMemory (int key);
    ~ShareMemory();
    /// @brief 创建共享内存
    /// @param key
    /// @param data_size
    /// @return true
    /// @return false
    bool createSharedMemory(int key);
    /// @brief 摧毁共享内存
    /// @return true
    /// @return false
    bool destroySharedMemory();
    /// @brief 获取共享内存
    /// @return T*
    void read(T& data);
    void write(T data);
    /// @brief 获取共享内存状态
    /// @return SharedMemoryStatus*
    void getSharedMemoryStatus(int& shm_id, int& key);
    /// @brief 打印共享内存状态
    void printStatus();

  private:
    int shm_id_;
    T* buffer_;
    Semaphore* sem_;
    int key_;
  };
  template <class T>
  ShareMemory<T>::ShareMemory (int key) : key_(key) {
    std::cout << "createSharedMemory: " << createSharedMemory(key_) << std::endl;
  }
  template <class T>
  ShareMemory<T>::~ShareMemory() {
    destroySharedMemory();
  }
  template <class T>
  bool ShareMemory<T>::createSharedMemory(int key) {
      /// @brief 创建内存信号量
    sem_ = new Semaphore(key_);
    /// 尝试打开已存在的共享内存
    shm_id_ =
        shmget(key_, sizeof(T), 0);
    if (shm_id_ == -1) {
      // 队列状态共享内存不存在，创建共享内存
      shm_id_ = shmget(
          key_, sizeof(T), IPC_CREAT | 0666);
      if (shm_id_ == -1) {
        return false;
      }
    }
    // 连接共享内存
    buffer_ = static_cast<T*>(
        shmat(shm_id_, nullptr, 0));
    if (buffer_ == nullptr) {
      return false;
    }
    return true;
  }
  template <class T>
  bool ShareMemory<T>::destroySharedMemory() {
    if (buffer_ != nullptr) {
      // 解除连接共享内存
      if (shmdt(buffer_) == -1) {
        return false;
      }
      buffer_ = nullptr;
    }

    if (shm_id_ != -1) {
      // 销毁共享内存
      if (shmctl(shm_id_, IPC_RMID, nullptr) == -1) {
        return false;
      }
      shm_id_ = -1;
    }
    delete sem_;
    return true;
  }
  template <class T>
  void ShareMemory<T>::read(T& data) {
    sem_->P();
    data = *buffer_;
    sem_->V();
  }
  template <class T>
  void ShareMemory<T>::write(T data) {
    sem_->P();
    *buffer_ = data;
    sem_->V();
  }
  template <class T>
  void ShareMemory<T>::getSharedMemoryStatus(int& shm_id, int& key) {
    shm_id = shm_id_;
    key = key_;
  }
  template <class T>
  void ShareMemory<T>::printStatus() {
    std::cout << "shared_memory shm_id:" << shm_id_ << " key:" << key_ << std::endl;
  }
  
template<typename T>
class SharedMemoryArray {
 public:
  typedef std::shared_ptr<SharedMemoryArray<T>> ptr;
  SharedMemoryArray(int key, int rows)
      : rows_(rows), sharedMemory_(NULL), key_(key) {
         /// @brief 创建内存信号量
    /// @brief 创建内存信号量
    sem_ = new Semaphore(key_);
    size_t size = sizeof(T) * rows_ ;
    shm_id_ = shmget(key_, size, 0);
    if (shm_id_ == -1) {
      shm_id_ = shmget(key_, size, IPC_CREAT | 0666);
      if (shm_id_ == -1) {
        std::cerr << "Failed to get existing shared memory segment."
                  << std::endl;
      }
    }

    sharedMemory_ = static_cast<T*>(shmat(shm_id_, NULL, 0));
    if (sharedMemory_ == reinterpret_cast<void*>(-1)) {
      std::cerr << "Failed to attach shared memory segment." << std::endl;
    }
  }
  ~SharedMemoryArray() {
    if (shmdt(sharedMemory_) == -1) {
      std::cerr << "Failed to detach shared memory segment." << std::endl;
    }
    if (shmctl(shm_id_, IPC_RMID, NULL) == -1) {
      std::cerr << "Failed to delete shared memory segment." << std::endl;
    }
    // delete sem_;
  }
  void write(T* data) {
    sem_->P();
    std::copy(data , data + rows_, sharedMemory_);
    sem_->V();
  }

  void read(T* data) const {
    sem_->P();
    std::copy(sharedMemory_, sharedMemory_ + rows_, data);
    sem_->V();
  }

 private:
  int rows_;
  int shm_id_;
  T* sharedMemory_;
  // ShareMemory<int>::ptr rows_readed_ptr;
  // ShareMemory<int>::ptr cols_readed_ptr;
  Semaphore* sem_;
  int key_;
};




  extern "C" {
    //坐标
    struct Pose {
      double x_;
      double y_;
      double theta_;
      Pose() : x_(0.0), y_(0.0), theta_(0.0) {}
      Pose(double x, double y, double theta) : x_(x), y_(y), theta_(theta) {} 
    };
    struct PoseCtrlParams {
      double v;
      double point_err;
      double angle_err;
    };
    struct PoseCtrl {
      Pose pose;
      PoseCtrlParams params;
    };

    //升降电机
    struct LiftCtrlParams {
      double enc_err;
      uint8_t cnt;
    };
    struct LiftCtrl {
      double height;
      LiftCtrlParams params;
    };
    //升降置零
    struct ResetLiftCtrl {
      bool reset_flag;
      double speed;
    };

    //旋转电机
    struct TurnCtrlParams {
      double enc_err;
      uint8_t cnt;
    };
    struct TurnCtrl {
      double angle;
      TurnCtrlParams params;
    };
    //旋转置零
    struct ResetTurnCtrl {
      bool reset_flag;
      double speed;
    };


    //舵机
    struct ServoCtrl {
      double val;
      uint16_t time;
    };


    //雷达校准
    struct LidarCalibCtrlParams {
      double d_err;       //距离误差范围
      double d_err_cnt;   //距离误差计数
      double angle_err;       //角度误差范围
      double angle_err_cnt;   //角度误差计数
      double left_right_e;    //左右两边误差大于这个值时直接先调整角度
    };
    struct LidarCalibCtrl {
      double dis;
      LidarCalibCtrlParams params;
      // bool stop_flag;
    };

    //沿墙行驶
    struct AlongWall {
      Pose pose;
      double dis;
      double v;
    };
    struct AlongWallCtrl {
      AlongWall input;
      bool stop_flag;
    };


    //超声波校准
    // struct USCalibCtrlParams {
    //   double v;
    //   double d_err;
    // };
    struct USCalibCtrl {
      double dis;
      double angle_e;
      double dis_e;
      double left_right_e;
      uint8_t CNT;
    };

    //红外校准
    // struct IRCalibCtrlParams {
    //   double v;
    //   double d_err;
    // };
    struct IRCalibCtrl {
      double dis;
      double angle_e;
      double dis_e;
      double left_right_e;
      uint8_t CNT;
    };

    //单红外校准
    struct SingleIRCalibCtrl {
      double dis;
      double angle_e;
      double dis_e;
      uint8_t CNT;
    };


    struct MotorParams {
      bool start_flag;
      uint8_t speed;
      bool dir; 
    };
    struct Motor {
      MotorParams Lift;   //端口0
      MotorParams Left;   //端口1
      MotorParams Right;  //端口2
      MotorParams Turn;   //端口3
    };

    struct ServoParams {
      bool start_flag;
      double val; 
    };
    struct Servo {
      ServoParams Clamp;        //端口14
      ServoParams Raise;        //端口15
      ServoParams Telescopic;   //端口16
      ServoParams Rotating;     //端口17
    };

    struct LED {
      bool StartLED;  //端口19
      bool ResetLED;  //端口20
      bool StopLED;   //端口21
    };

    struct ResetENC {
      bool ResetLiftENC;   //端口0
      bool ResetLeftENC;   //端口1
      bool ResetRightENC;  //端口2
      bool ResetTurnENC;   //端口3
    };

    struct WriteIO {
      Motor motor;
      Servo servo;
      LED led;
      ResetENC enc_reset;
      bool imu_reset;
    };
    
    struct ENC {
      int32_t LiftENC;   //端口0
      int32_t LeftENC;   //端口1
      int32_t RightENC;  //端口2
      int32_t TurnENC;   //端口3
    };

    struct Limit {
      bool LiftUpLimit;     //端口9
      bool LiftDownLimit;   //端口24
      bool TurnLimit;       //端口25
    };

    struct Button {
      bool StartButton;   //端口9
      bool ResetButton;   //端口25
      bool StopButton;
      bool EStopButton;
    };

    struct Sensor {
      double UltrasoundLeft;   //端口(13,10)
      double UltrasoundRight;   //端口(12,8)
      double IRLeft;  //端口22
      double IRRight;   //端口23
    };

    struct LidarData {
      double angle;   //角度
      double r;   //距离
      float intensity;   //质量
    };

    struct ReadIO {
      ENC enc;   //
      Limit limit;   //
      Button button;   //
      Sensor sensor;
      double imu_data;
      // LidarData lidar_data;
    };

    
    //视觉
    struct BoxInfo {
      float x1;   //图片边框左上
      float y1;
      float x2;   //图片边框右下
      float y2;
      float score;  //置信度
      int32_t label;    //标签
      BoxInfo() : x1(0),y1(0),x2(0),y2(0),score(0),label(-1) {}
      BoxInfo(float x1, float y1, float x2,float y2,float score, int32_t label) : x1(x1),y1(y1),x2(x2),y2(y2),score(score),label(label) {}
    };
    struct RealXH {
      double X;   //实际相对坐标X
      double H;   //实际高度相对位置
      RealXH() : X(0), H(0) {}
      RealXH(double X, double H) : X(X), H(H) {}
    };
    struct IdentifyInfo {
      BoxInfo box_info;
      RealXH XH;
      IdentifyInfo() = default;
      IdentifyInfo(const BoxInfo& temp1, const RealXH& temp2) : box_info(temp1), XH(temp2){}
      IdentifyInfo(const IdentifyInfo& temp) : box_info(temp.box_info), XH(temp.XH){}
    };
    struct FruitsInfo {   //多个水果
      uint8_t num;        //水果数量
      IdentifyInfo fruits_1;
      IdentifyInfo fruits_2;
      IdentifyInfo fruits_3;
      IdentifyInfo fruits_4;
      IdentifyInfo fruits_5;
      FruitsInfo() = default;
      FruitsInfo(uint8_t num, const IdentifyInfo& fruits_1, const IdentifyInfo& fruits_2, const IdentifyInfo& fruits_3, const IdentifyInfo& fruits_4, const IdentifyInfo& fruits_5) : num(num), fruits_1(fruits_1), fruits_2(fruits_2), fruits_3(fruits_3), fruits_4(fruits_4), fruits_5(fruits_5) {}
      FruitsInfo(const FruitsInfo& temp) : num(temp.num), fruits_1(temp.fruits_1), fruits_2(temp.fruits_2), fruits_3(temp.fruits_3), fruits_4(temp.fruits_4), fruits_5(temp.fruits_5) {}
    };

    //标定数据
    struct Point {
      float x;
      float y;
    };
    struct ImgCalPoints {
      Point P1;
      Point P2;
      Point P3;
      Point P4;
    };
    struct CalImg {
      ImgCalPoints Image;
      ImgCalPoints Object;
    };


        
    //pid
    struct PIDParams {
      double kp;
      double ki;
      double kd;
      PIDParams() : kp(0), ki(0), kd(0) {}
      PIDParams(double p, double i, double d) : kp(p), ki(i), kd(d) {} 
    };
    struct PIDOutputLimits {
      double min;
      double max;
      PIDOutputLimits() : min(0), max(0) {}
      PIDOutputLimits(double min, double max) : min(min), max(max){} 
    };
    struct PIDCtrlParams {
      PIDParams pid;
      PIDOutputLimits limit;
      double dt;      //不可调
    };

    //电机速度环pid
    struct MotorPID {
      PIDCtrlParams Left;
      PIDCtrlParams Right;
      PIDCtrlParams Turn;
      PIDCtrlParams Lift;
    };
    //电机位置环pid
    struct MotorDisPID {
      PIDCtrlParams TurnDis;
      PIDCtrlParams LiftDis;
    };
    //红外
    struct IRCalibPID {
      PIDCtrlParams Angle;
      PIDCtrlParams Dis;
    };
    //单红外
    struct SingeIRCalibPID {
      PIDCtrlParams Angle;
      PIDCtrlParams Dis;
    };
    //超声波
    struct USCalibPID {
      PIDCtrlParams Angle;
      PIDCtrlParams Dis;
    };
    //雷达
    struct LidarCalibPID {
      PIDCtrlParams Angle;
      PIDCtrlParams Dis;
    };

    struct CtrlPID
    { 
      double P;
      double I;
      double D;
      double limit;     //不用调
      CtrlPID() : P(0.0), I(0.0), D(0.0), limit(0.0) {}
      CtrlPID(double p, double i, double d, double limit) :P(p), I(i), D(d), limit(limit) {} 
    };
    //底盘
    struct ChassisPID {
      CtrlPID Vx;
      CtrlPID Vy;
      CtrlPID Vz;
    };
    //所有pid参数
    struct AllPID {
      MotorPID Motor;
      MotorDisPID MotorDis;
      IRCalibPID IR;
      SingeIRCalibPID SingeIR;
      USCalibPID US;
      LidarCalibPID Lidar;
      PIDCtrlParams AlongWall;
      ChassisPID Chassis;
    };


    // //pid误差
    // struct PIDErrorParams {
    //   double E_Range;
    //   int32_t CNT;
    //   PIDErrorParams() : E_Range(0), CNT(0) {}
    //   PIDErrorParams(double e, int32_t cnt) : E_Range(e), CNT(cnt){} 
    // };

    // //雷达
    // struct LidarCalibError {
    //   PIDErrorParams Dis;
    //   PIDErrorParams Angle;
    //   double LeftRightE;
    // };

    // struct SensorCalibError {
    //   double Angle;
    //   double Dis;
    //   int32_t CNT;
    //   double LeftRightE;
    //   SensorCalibError() : Angle(0), Dis(0), CNT(0), LeftRightE(0){}
    //   SensorCalibError(double angle, double dis, int32_t cnt, double LeftRightE) : Angle(angle), Dis(dis), CNT(cnt), LeftRightE(LeftRightE){} 
    // };


    struct LidarInitParams {
      double InitAngle;   //安装位置
      double CalAngle;    //校准所采样的雷达角度，默认-10°~10°
      LidarInitParams() : InitAngle(-90), CalAngle(10) {}
      LidarInitParams(double initAngle, double calAngle) : InitAngle(initAngle), CalAngle(calAngle) {} 
    };


    //定义地址
    //labview状态
    extern ShareMemory<uint8_t>::ptr LabviewStatusShareAddress;
    //labview运行时间
    extern ShareMemory<double>::ptr LabviewTimeShareAddress;
    //坐标移动
    extern ShareMemory<PoseCtrl>::ptr TrackingPointShareAddress;
    extern ShareMemory<uint8_t>::ptr TrackingPointStatusShareAddress;
    extern ShareMemory<uint8_t>::ptr TrackingXYStatusShareAddress;
    //坐标移动里程计
    extern ShareMemory<Pose>::ptr UpdateOdomShareAddress;

    //底盘旋转
    extern ShareMemory<double>::ptr RotateShareAddress;
    extern ShareMemory<uint8_t>::ptr RotateStatusShareAddress;

    //坐标清零
    extern ShareMemory<bool>::ptr ZeroOdomShareAddress;
    extern ShareMemory<uint8_t>::ptr ZeroOdomStatusShareAddress;

    //设定位姿
    extern ShareMemory<Pose>::ptr SetOdomShareAddress;
    extern ShareMemory<uint8_t>::ptr SetOdomStatusShareAddress;

    //升降控制
    extern ShareMemory<LiftCtrl>::ptr LiftDistanceShareAddress;
    extern ShareMemory<uint8_t>::ptr LiftDistanceStatusShareAddress;
    //升降复位控制
    extern ShareMemory<ResetLiftCtrl>::ptr ResetLiftShareAddress;
    extern ShareMemory<uint8_t>::ptr ResetLiftStatusShareAddress;
    //升降限位开关
    extern ShareMemory<bool>::ptr ResetLiftLimitShareAddress;

    //旋转控制
    extern ShareMemory<TurnCtrl>::ptr TurnAngleShareAddress;
    extern ShareMemory<uint8_t>::ptr TurnAngleStatusShareAddress;
    //旋转复位控制
    extern ShareMemory<ResetTurnCtrl>::ptr ResetTurnAngleShareAddress;
    extern ShareMemory<uint8_t>::ptr ResetTurnAngleStatusShareAddress;
    //旋转限位开关
    extern ShareMemory<bool>::ptr ResetTurnLimitShareAddress;
    
    //夹手舵机
    extern ShareMemory<ServoCtrl>::ptr ClampServoShareAddress;
    extern ShareMemory<uint8_t>::ptr ClampServoStatusShareAddress;
    //伸缩舵机
    extern ShareMemory<ServoCtrl>::ptr TelescopicServoShareAddress;
    extern ShareMemory<uint8_t>::ptr TelescopicServoStatusShareAddress;
    //摆手舵机
    extern ShareMemory<ServoCtrl>::ptr RaiseServoShareAddress;
    extern ShareMemory<uint8_t>::ptr RaiseServoStatusShareAddress;
    //旋转舵机
    extern ShareMemory<ServoCtrl>::ptr RotatingServoShareAddress;
    extern ShareMemory<uint8_t>::ptr RotatingServoStatusShareAddress;

    //雷达校准
    extern ShareMemory<LidarCalibCtrl>::ptr LidarCalibShareAddress;
    extern ShareMemory<uint8_t>::ptr LidarCalibStatusShareAddress;
    //沿右墙行驶
    extern ShareMemory<AlongWallCtrl>::ptr AlongRightWallShareAddress;
    extern ShareMemory<uint8_t>::ptr AlongRightWallStatusShareAddress;
    //沿左墙行驶
    extern ShareMemory<AlongWallCtrl>::ptr AlongLeftWallShareAddress;
    extern ShareMemory<uint8_t>::ptr AlongLeftWallStatusShareAddress;

    //测试IO控制指令状态
    extern ShareMemory<uint8_t>::ptr TestIOStatusShareAddress;
    extern ShareMemory<WriteIO>::ptr WriteIOShareAddress;
    extern ShareMemory<ReadIO>::ptr ReadIOShareAddress;

    //超声波校准
    extern ShareMemory<USCalibCtrl>::ptr USCalibCtrlShareAddress;
    extern ShareMemory<uint8_t>::ptr USCalibCtrlStatusShareAddress;

    //红外校准
    extern ShareMemory<IRCalibCtrl>::ptr IRCalibCtrlShareAddress;
    extern ShareMemory<uint8_t>::ptr IRCalibCtrlStatusShareAddress;

    //单红外校准
    extern ShareMemory<SingleIRCalibCtrl>::ptr SingleIRCalibCtrlShareAddress;
    extern ShareMemory<uint8_t>::ptr SingleIRCalibCtrlStatusShareAddress;

    //视觉识别指令状态共享内存
    extern ShareMemory<uint8_t>::ptr IdentifyStatusShareAddress;
    //读取视觉识别数据
    extern ShareMemory<IdentifyInfo>::ptr IdentifyDataShareAddress;
    //读取视觉识别多个水果数据
    extern ShareMemory<FruitsInfo>::ptr FruitsDataShareAddress;

    //标定数据
    extern ShareMemory<CalImg>::ptr CalImgShareAddress;
    extern ShareMemory<uint8_t>::ptr CalImgStatusShareAddress;

    //PID参数数据
    extern ShareMemory<AllPID>::ptr AllPIDShareAddress;
    extern ShareMemory<uint8_t>::ptr updateAllPIDStatusShareAddress;

    //雷达参数
    extern ShareMemory<LidarInitParams>::ptr LidarInitParamsShareAddress;
    extern ShareMemory<uint8_t>::ptr LidarInitParamsStatusShareAddress;

    //图像数据数组
    extern SharedMemoryArray<uint8_t>::ptr ImageDataShareAddress;

    //雷达数据数组
    extern SharedMemoryArray<double>::ptr LidarAngleDataShareAddress;
    extern SharedMemoryArray<double>::ptr LidarRangeDataShareAddress;
    extern SharedMemoryArray<float>::ptr LidarIntensityDataShareAddress;
  }
// #endif  //  INCLUDE_SHARE_H_
}
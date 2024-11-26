/**
 * @file share.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius,com)
 * @brief 创建共享内存
 * @version 0.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "share.h"
#include <cerrno>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <unistd.h>
#include <cstdint>
using namespace LABVIEW;
extern "C"
{
  
  Semaphore::Semaphore(int key) {
    sem_id_ = semget(key, 1, IPC_CREAT | IPC_EXCL | 0666);
    if (sem_id_ == -1) {
      if (errno == EEXIST) {
        // 信号量已经存在，尝试直接打开
        sem_id_ = semget(key, 1, 0);
        if (sem_id_ == -1) {
          semaphore_status_ = false;
        }
      } else {
        semaphore_status_ = false;
      }
    }

    // 初始化信号量
    union semun {
      int val;
      struct sem_id_ds* buf;
      unsigned short* array;
    } arg{};
    arg.val = 1;  // 固定值为1，表示只允许一个进程访问

    if (semctl(sem_id_, 0, SETVAL, arg) == -1) {
      semaphore_status_ = false;
    } else {
      semaphore_status_ = true;
    }
  }
  Semaphore::~Semaphore() {
    if (sem_id_ != -1) {
      // 销毁信号量
      if (semctl(sem_id_, 0, IPC_RMID) == -1) {
        semaphore_status_ = false;
      } else {
        semaphore_status_ = true;
      }
      sem_id_ = -1;
    } else {
      semaphore_status_ = true;
    }
  }
  void Semaphore::P() {
    struct sembuf sb{};
    sb.sem_num = 0;
    sb.sem_op = -1;  // P 操作，申请资源
    sb.sem_flg = SEM_UNDO;

    if (semop(sem_id_, &sb, 1) == -1) {
      semaphore_status_ = false;
    }
  }
  void Semaphore::V() {
    struct sembuf sb{};
    sb.sem_num = 0;
    sb.sem_op = 1;  // V 操作，释放资源
    sb.sem_flg = SEM_UNDO;

    if (semop(sem_id_, &sb, 1) == -1) {
      semaphore_status_ = false;
    }
  }



  //定义地址
  //labview状态
  ShareMemory<uint8_t>::ptr LabviewStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x0050);
  //labview运行时间
  ShareMemory<double>::ptr LabviewTimeShareAddress = std::make_shared<ShareMemory<double>>(0x00100);
  //坐标移动
  ShareMemory<PoseCtrl>::ptr TrackingPointShareAddress = std::make_shared<ShareMemory<PoseCtrl>>(0x1000);
  ShareMemory<uint8_t>::ptr TrackingPointStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2000);
  ShareMemory<uint8_t>::ptr TrackingXYStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2005);
  //坐标移动里程计
  ShareMemory<Pose>::ptr UpdateOdomShareAddress = std::make_shared<ShareMemory<Pose>>(0x3000);

  //底盘旋转
  ShareMemory<double>::ptr RotateShareAddress = std::make_shared<ShareMemory<double>>(0x1010);
  ShareMemory<uint8_t>::ptr RotateStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2010);

  //坐标清零
  ShareMemory<bool>::ptr ZeroOdomShareAddress = std::make_shared<ShareMemory<bool>>(0x1020);
  ShareMemory<uint8_t>::ptr ZeroOdomStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2020);

  //设定位姿
  ShareMemory<Pose>::ptr SetOdomShareAddress = std::make_shared<ShareMemory<Pose>>(0x1025);
  ShareMemory<uint8_t>::ptr SetOdomStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2025);

  //升降控制
  ShareMemory<LiftCtrl>::ptr LiftDistanceShareAddress = std::make_shared<ShareMemory<LiftCtrl>>(0x1030);
  ShareMemory<uint8_t>::ptr LiftDistanceStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2030);
  //升降复位控制
  ShareMemory<ResetLiftCtrl>::ptr ResetLiftShareAddress = std::make_shared<ShareMemory<ResetLiftCtrl>>(0x1040);
  ShareMemory<uint8_t>::ptr ResetLiftStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2040);
  //升降限位开关
  ShareMemory<bool>::ptr ResetLiftLimitShareAddress = std::make_shared<ShareMemory<bool>>(0x3010);

  //旋转控制
  ShareMemory<TurnCtrl>::ptr TurnAngleShareAddress = std::make_shared<ShareMemory<TurnCtrl>>(0x1050);
  ShareMemory<uint8_t>::ptr TurnAngleStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2050);
  //旋转复位控制
  ShareMemory<ResetTurnCtrl>::ptr ResetTurnAngleShareAddress = std::make_shared<ShareMemory<ResetTurnCtrl>>(0x1060);
  ShareMemory<uint8_t>::ptr ResetTurnAngleStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2060);
  //旋转限位开关
  ShareMemory<bool>::ptr ResetTurnLimitShareAddress = std::make_shared<ShareMemory<bool>>(0x3020);

  //夹手舵机
  ShareMemory<ServoCtrl>::ptr ClampServoShareAddress = std::make_shared<ShareMemory<ServoCtrl>>(0x1070);
  ShareMemory<uint8_t>::ptr ClampServoStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2070);

  //伸缩舵机
  ShareMemory<ServoCtrl>::ptr TelescopicServoShareAddress = std::make_shared<ShareMemory<ServoCtrl>>(0x1080);
  ShareMemory<uint8_t>::ptr TelescopicServoStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2080);

  //摆手舵机
  ShareMemory<ServoCtrl>::ptr RaiseServoShareAddress = std::make_shared<ShareMemory<ServoCtrl>>(0x1090);
  ShareMemory<uint8_t>::ptr RaiseServoStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2090);

  //旋转舵机
  ShareMemory<ServoCtrl>::ptr RotatingServoShareAddress = std::make_shared<ShareMemory<ServoCtrl>>(0x1095);
  ShareMemory<uint8_t>::ptr RotatingServoStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2095);

  //雷达校准
  ShareMemory<LidarCalibCtrl>::ptr LidarCalibShareAddress = std::make_shared<ShareMemory<LidarCalibCtrl>>(0x1100);
  ShareMemory<uint8_t>::ptr LidarCalibStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2100);

  //沿右墙行驶
  ShareMemory<AlongWallCtrl>::ptr AlongRightWallShareAddress = std::make_shared<ShareMemory<AlongWallCtrl>>(0x1110);
  ShareMemory<uint8_t>::ptr AlongRightWallStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2110);

  //沿左墙行驶
  ShareMemory<AlongWallCtrl>::ptr AlongLeftWallShareAddress = std::make_shared<ShareMemory<AlongWallCtrl>>(0x1120);
  ShareMemory<uint8_t>::ptr AlongLeftWallStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2120);

  //测试IO控制指令状态
  ShareMemory<uint8_t>::ptr TestIOStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2800);
  ShareMemory<WriteIO>::ptr WriteIOShareAddress = std::make_shared<ShareMemory<WriteIO>>(0x1800);
  ShareMemory<ReadIO>::ptr ReadIOShareAddress = std::make_shared<ShareMemory<ReadIO>>(0x3800);

  //超声波校准
  ShareMemory<USCalibCtrl>::ptr USCalibCtrlShareAddress = std::make_shared<ShareMemory<USCalibCtrl>>(0x1130);
  ShareMemory<uint8_t>::ptr USCalibCtrlStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2130);

  //红外校准
  ShareMemory<IRCalibCtrl>::ptr IRCalibCtrlShareAddress = std::make_shared<ShareMemory<IRCalibCtrl>>(0x1140);
  ShareMemory<uint8_t>::ptr IRCalibCtrlStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2140);

  //单红外校准
  ShareMemory<SingleIRCalibCtrl>::ptr SingleIRCalibCtrlShareAddress = std::make_shared<ShareMemory<SingleIRCalibCtrl>>(0x1145);
  ShareMemory<uint8_t>::ptr SingleIRCalibCtrlStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2145);

  //视觉识别指令状态共享内存
  ShareMemory<uint8_t>::ptr IdentifyStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2150);
  //读取视觉识别置信度最高物品数据
  ShareMemory<IdentifyInfo>::ptr IdentifyDataShareAddress = std::make_shared<ShareMemory<IdentifyInfo>>(0x3150);
  //读取视觉识别多个水果数据
  ShareMemory<FruitsInfo>::ptr FruitsDataShareAddress = std::make_shared<ShareMemory<FruitsInfo>>(0x3155);

  //标定数据
  ShareMemory<CalImg>::ptr CalImgShareAddress = std::make_shared<ShareMemory<CalImg>>(0x1500);
  ShareMemory<uint8_t>::ptr CalImgStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2500);

  //雷达参数
  ShareMemory<LidarInitParams>::ptr LidarInitParamsShareAddress = std::make_shared<ShareMemory<LidarInitParams>>(0x1510);
  ShareMemory<uint8_t>::ptr LidarInitParamsStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2510);

  //PID参数数据
  ShareMemory<AllPID>::ptr AllPIDShareAddress = std::make_shared<ShareMemory<AllPID>>(0x1600);
  ShareMemory<uint8_t>::ptr updateAllPIDStatusShareAddress = std::make_shared<ShareMemory<uint8_t>>(0x2600);

  //图像数据数组
  SharedMemoryArray<uint8_t>::ptr ImageDataShareAddress = std::make_shared<SharedMemoryArray<uint8_t>>(0x00010000, 640*480*3);

  //雷达数据数组
  SharedMemoryArray<double>::ptr LidarAngleDataShareAddress = std::make_shared<SharedMemoryArray<double>>(0x9000, 360);
  SharedMemoryArray<double>::ptr LidarRangeDataShareAddress = std::make_shared<SharedMemoryArray<double>>(0x9100, 360);
  SharedMemoryArray<float>::ptr LidarIntensityDataShareAddress = std::make_shared<SharedMemoryArray<float>>(0x9200, 360);
}
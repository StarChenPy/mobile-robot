/**
 * @file LabviewCommand.h
 * @author Zijian.Yan (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-10-08
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

class LabviewCommand : public CommandBase {
  public:
    typedef std::shared_ptr<LabviewCommand> Ptr;
    LabviewCommand() {}
    ~LabviewCommand() {}

    void InitCommandStatus();

    void readShareMemory();
    void TestIOStatusShareCommand();
    void TrackingPointShareCommand();
    void TrackingXYShareCommand();
    void RotateShareCommand();
    void ZeroOdomShareCommand();
    void SetOdomShareCommand();
    void LiftDistanceShareCommand();
    void ResetLiftShareCommand();
    void TurnAngleShareCommand();
    void ResetTurnAngleShareCommand();
    void ClampServoShareCommand();
    void TelescopicServoShareCommand();
    void RaiseServoShareCommand();
    void RotatingServoShareCommand();
    void LidarCalibShareCommand();
    void IRCalibShareCommand();
    void SingleIRCalibShareCommand();
    void USCalibShareCommand();
    void AlongRightWallShareCommand();
    void AlongLeftWallShareCommand();
    void IdentifyFruitshareCommand();
    void updataImgCalParams();
    void updataShareLidarParams();
    void updataSharePIDParams();

    void createShareMemoryListCommand();

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;
    int init_time;
    double cur_time_s;

    // labview状态
    uint8_t LabviewStatusShare;
    //测试IO
    uint8_t TestIOStatusShare;

    //坐标移动
    uint8_t TrackingPointStatusShare;
    uint8_t TrackingXYStatusShare;
    //底盘旋转
    uint8_t RotateStatusShare;
    //坐标清零
    uint8_t ZeroOdomStatusShare;
    //设定位姿
    uint8_t SetOdomStatusShare;
    //升降控制
    uint8_t LiftDistanceStatusShare;
    //升降复位控制
    uint8_t ResetLiftStatusShare;
    //旋转控制
    uint8_t TurnAngleStatusShare;
    //旋转复位控制
    uint8_t ResetTurnAngleStatusShare;
    //夹手舵机
    uint8_t ClampServoStatusShare;
    //伸缩舵机
    uint8_t TelescopicServoStatusShare;
    //摆手舵机
    uint8_t RaiseServoStatusShare;
    //旋转舵机
    uint8_t RotatingServoStatusShare;
    //雷达校准
    uint8_t LidarCalibStatusShare;
    //沿右墙行驶
    uint8_t AlongRightWallStatusShare;
    //沿左墙行驶
    uint8_t AlongLeftWallStatusShare;
    //超声波校准
    uint8_t USCalibStatusShare;
    //红外校准
    uint8_t IRCalibStatusShare;
    //红外校准
    uint8_t SingleIRCalibStatusShare;
    //视觉识别
    uint8_t IdentifyStatusShare;
    //视觉标定
    uint8_t ImgCalStatusShare;
    //雷达参数
    uint8_t LidarInitParamsStatusShare;
    //更新pid参数
    uint8_t updataPIDStatusShare;
};

Command::ptr createLabviewCommand();

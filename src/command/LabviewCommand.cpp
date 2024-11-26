#include "command/LabviewCommand.h"

#include "command/RotateCommand.h"
#include "command/TrackingPointCommand.h"
#include "command/ZeroOdomCommand.h"
// #include "command/VisionCtrlCommand.h"
#include "command/MotorPIDCommand.h"
#include "command/ServoCommand.h"
// #include "command/LidarReadCommand.h"
#include "command/CalCommand.h"
#include "command/IOTestCommand.h"
#include "command/LidarAlongWallCommand.h"
#include "command/LidarCalibCommand.h"
#include "command/Vision/VisionCommand.h"
// #include "command/ButtonCommand.h"
// #include "command/UpdataOdomCommand.h"

//初始化指令状态
void LabviewCommand::InitCommandStatus() {
    uint8_t free_status = COMMEND_FREE;
    //测试IO
    LABVIEW::TestIOStatusShareAddress->write(free_status);
    //坐标移动
    LABVIEW::TrackingPointStatusShareAddress->write(free_status);
    LABVIEW::TrackingXYStatusShareAddress->write(free_status);
    //底盘旋转
    LABVIEW::RotateStatusShareAddress->write(free_status);
    //坐标清零
    LABVIEW::ZeroOdomStatusShareAddress->write(free_status);
    //坐标清零
    LABVIEW::SetOdomStatusShareAddress->write(free_status);
    //升降控制
    LABVIEW::LiftDistanceStatusShareAddress->write(free_status);
    //升降复位控制
    LABVIEW::ResetLiftStatusShareAddress->write(free_status);
    //旋转控制
    LABVIEW::TurnAngleStatusShareAddress->write(free_status);
    //旋转复位控制
    LABVIEW::ResetTurnAngleStatusShareAddress->write(free_status);
    //夹手舵机
    LABVIEW::ClampServoStatusShareAddress->write(free_status);
    //伸缩舵机
    LABVIEW::TelescopicServoStatusShareAddress->write(free_status);
    //摆手舵机
    LABVIEW::RaiseServoStatusShareAddress->write(free_status);
    //旋转舵机
    LABVIEW::RotatingServoStatusShareAddress->write(free_status);
    //雷达校准
    LABVIEW::LidarCalibStatusShareAddress->write(free_status);
    //沿右墙行驶
    LABVIEW::AlongRightWallStatusShareAddress->write(free_status);
    //沿左墙行驶
    LABVIEW::AlongLeftWallStatusShareAddress->write(free_status);
    //视觉识别
    LABVIEW::IdentifyStatusShareAddress->write(free_status);
    //超声波校准
    LABVIEW::USCalibCtrlStatusShareAddress->write(free_status);
    //红外校准
    LABVIEW::IRCalibCtrlStatusShareAddress->write(free_status);
    //单红外校准
    LABVIEW::SingleIRCalibCtrlStatusShareAddress->write(free_status);
    //视觉标定
    LABVIEW::CalImgStatusShareAddress->write(free_status);
    //雷达参数
    LABVIEW::LidarInitParamsStatusShareAddress->write(free_status);
    //更新pid参数
    LABVIEW::updateAllPIDStatusShareAddress->write(free_status);
}

//读取指令状态
void LabviewCommand::readShareMemory() {
    // labview状态
    LABVIEW::LabviewStatusShareAddress->read(LabviewStatusShare);
    //测试IO
    LABVIEW::TestIOStatusShareAddress->read(TestIOStatusShare);
    //坐标移动
    LABVIEW::TrackingPointStatusShareAddress->read(TrackingPointStatusShare);
    LABVIEW::TrackingXYStatusShareAddress->read(TrackingXYStatusShare);
    //底盘旋转
    LABVIEW::RotateStatusShareAddress->read(RotateStatusShare);
    //坐标清零
    LABVIEW::ZeroOdomStatusShareAddress->read(ZeroOdomStatusShare);
    //设定位姿
    LABVIEW::SetOdomStatusShareAddress->read(SetOdomStatusShare);
    //升降控制
    LABVIEW::LiftDistanceStatusShareAddress->read(LiftDistanceStatusShare);
    //升降复位控制
    LABVIEW::ResetLiftStatusShareAddress->read(ResetLiftStatusShare);
    //旋转控制
    LABVIEW::TurnAngleStatusShareAddress->read(TurnAngleStatusShare);
    //旋转复位控制
    LABVIEW::ResetTurnAngleStatusShareAddress->read(ResetTurnAngleStatusShare);
    //夹手舵机
    LABVIEW::ClampServoStatusShareAddress->read(ClampServoStatusShare);
    //伸缩舵机
    LABVIEW::TelescopicServoStatusShareAddress->read(TelescopicServoStatusShare);
    //摆手舵机
    LABVIEW::RaiseServoStatusShareAddress->read(RaiseServoStatusShare);
    //旋转舵机
    LABVIEW::RotatingServoStatusShareAddress->read(RotatingServoStatusShare);
    //雷达校准
    LABVIEW::LidarCalibStatusShareAddress->read(LidarCalibStatusShare);
    //沿右墙行驶
    LABVIEW::AlongRightWallStatusShareAddress->read(AlongRightWallStatusShare);
    //沿左墙行驶
    LABVIEW::AlongLeftWallStatusShareAddress->read(AlongLeftWallStatusShare);
    //视觉识别
    LABVIEW::IdentifyStatusShareAddress->read(IdentifyStatusShare);
    //超声波校准
    LABVIEW::USCalibCtrlStatusShareAddress->read(USCalibStatusShare);
    //红外校准
    LABVIEW::IRCalibCtrlStatusShareAddress->read(IRCalibStatusShare);
    //单红外校准
    LABVIEW::SingleIRCalibCtrlStatusShareAddress->read(SingleIRCalibStatusShare);
    //视觉标定
    LABVIEW::CalImgStatusShareAddress->read(ImgCalStatusShare);
    //雷达参数
    LABVIEW::LidarInitParamsStatusShareAddress->read(LidarInitParamsStatusShare);
    //更新pid参数
    LABVIEW::updateAllPIDStatusShareAddress->read(updataPIDStatusShare);
}

//创建指令
void LabviewCommand::TestIOStatusShareCommand() { //测试IO
    if (TestIOStatusShare == COMMEND_CREATE) {
        createIOTestCommand()->schedule();
    }
}
void LabviewCommand::TrackingPointShareCommand() { //坐标移动
    if (TrackingPointStatusShare == COMMEND_CREATE) {
        LABVIEW::PoseCtrl TrackingPointShare;
        LABVIEW::TrackingPointShareAddress->read(TrackingPointShare);
        Pose target(TrackingPointShare.pose.x_, TrackingPointShare.pose.y_, TrackingPointShare.pose.theta_);
        createTrackingPointCommand(target, TrackingPointShare.params.v, TrackingPointShare.params.point_err,
                                   TrackingPointShare.params.angle_err)
            ->schedule();
    }
}
void LabviewCommand::TrackingXYShareCommand() { //坐标移动XY
    if (TrackingXYStatusShare == COMMEND_CREATE) {
        LABVIEW::PoseCtrl TrackingPointShare;
        LABVIEW::TrackingPointShareAddress->read(TrackingPointShare);
        Pose target(TrackingPointShare.pose.x_, TrackingPointShare.pose.y_, TrackingPointShare.pose.theta_);
        createTrackingXYCommand(target, TrackingPointShare.params.v, TrackingPointShare.params.point_err,
                                TrackingPointShare.params.angle_err)
            ->schedule();
    }
}
void LabviewCommand::RotateShareCommand() { //底盘旋转
    if (RotateStatusShare == COMMEND_CREATE) {
        double RotateShare;
        LABVIEW::RotateShareAddress->read(RotateShare);
        createRotateCommand(RotateShare)->schedule();
    }
}
void LabviewCommand::ZeroOdomShareCommand() { //坐标清零
    if (ZeroOdomStatusShare == COMMEND_CREATE) {
        bool ZeroOdomShare;
        LABVIEW::ZeroOdomShareAddress->read(ZeroOdomShare);
        if (ZeroOdomShare == true) {
            createZeroOdomCommand()->schedule();
        }
    }
}
void LabviewCommand::SetOdomShareCommand() { //设定位姿
    if (SetOdomStatusShare == COMMEND_CREATE) {
        LABVIEW::Pose setPose;
        LABVIEW::SetOdomShareAddress->read(setPose);
        createSetOdomCommand(setPose.x_, setPose.y_, setPose.theta_)->schedule();
    }
}
void LabviewCommand::LiftDistanceShareCommand() { //升降控制
    if (LiftDistanceStatusShare == COMMEND_CREATE) {
        LABVIEW::LiftCtrl LiftDistanceShare;
        LABVIEW::LiftDistanceShareAddress->read(LiftDistanceShare);
        double h = LiftDistanceShare.height;
        double e_enc = LiftDistanceShare.params.enc_err;
        double cnt = LiftDistanceShare.params.cnt;
        LiftDistancePIDCommand(h, e_enc, cnt)->schedule();
    }
}
void LabviewCommand::ResetLiftShareCommand() { //升降复位控制
    if (ResetLiftStatusShare == COMMEND_CREATE) {
        LABVIEW::ResetLiftCtrl ResetLiftShare;
        LABVIEW::ResetLiftShareAddress->read(ResetLiftShare);
        if (ResetLiftShare.reset_flag == true) {
            double speed = ResetLiftShare.speed;
            ResetLiftMotorDistance(speed)->schedule();
        }
    }
}
void LabviewCommand::TurnAngleShareCommand() { //旋转控制
    if (TurnAngleStatusShare == COMMEND_CREATE) {
        LABVIEW::TurnCtrl TurnAngleShare;
        LABVIEW::TurnAngleShareAddress->read(TurnAngleShare);
        double angle = TurnAngleShare.angle;
        double e_enc = TurnAngleShare.params.enc_err;
        double cnt = TurnAngleShare.params.cnt;
        TurnAnglePIDCommand(angle, e_enc, cnt)->schedule();
    }
}
void LabviewCommand::ResetTurnAngleShareCommand() { //旋转复位控制
    if (ResetTurnAngleStatusShare == COMMEND_CREATE) {
        LABVIEW::ResetTurnCtrl ResetTurnAngleShare;
        LABVIEW::ResetTurnAngleShareAddress->read(ResetTurnAngleShare);
        if (ResetTurnAngleShare.reset_flag == true) {
            double speed = ResetTurnAngleShare.speed;
            ResetTurnMotorAngle(speed)->schedule();
        }
    }
}
void LabviewCommand::ClampServoShareCommand() { //夹手舵机
    if (ClampServoStatusShare == COMMEND_CREATE) {
        LABVIEW::ServoCtrl ClampServoShare;
        LABVIEW::ClampServoShareAddress->read(ClampServoShare);
        double len = ClampServoShare.val;
        uint16_t time = ClampServoShare.time;
        createClampServoCommand(len, time)->schedule();
    }
}
void LabviewCommand::TelescopicServoShareCommand() { //伸缩舵机
    if (TelescopicServoStatusShare == COMMEND_CREATE) {
        LABVIEW::ServoCtrl TelescopicServoShare;
        LABVIEW::TelescopicServoShareAddress->read(TelescopicServoShare);
        double dis = TelescopicServoShare.val;
        uint16_t time = TelescopicServoShare.time;
        createcTelescopicServoCommand(dis, time)->schedule();
    }
}
void LabviewCommand::RaiseServoShareCommand() { //摆手舵机
    if (RaiseServoStatusShare == COMMEND_CREATE) {
        LABVIEW::ServoCtrl RaiseServoShare;
        LABVIEW::RaiseServoShareAddress->read(RaiseServoShare);
        double angle = RaiseServoShare.val;
        uint16_t time = RaiseServoShare.time;
        createRaiseServoCommand(angle, time)->schedule();
    }
}
void LabviewCommand::RotatingServoShareCommand() { //旋转舵机
    if (RotatingServoStatusShare == COMMEND_CREATE) {
        LABVIEW::ServoCtrl RotatingServoShare;
        LABVIEW::RotatingServoShareAddress->read(RotatingServoShare);
        double angle = RotatingServoShare.val;
        uint16_t time = RotatingServoShare.time;
        createRotatingServoCommand(angle, time)->schedule();
    }
}
void LabviewCommand::LidarCalibShareCommand() { //雷达校准
    if (LidarCalibStatusShare == COMMEND_CREATE) {
        LABVIEW::LidarCalibCtrl LidarCalibShare;
        LABVIEW::LidarCalibShareAddress->read(LidarCalibShare);
        double dis = LidarCalibShare.dis;
        LidarCalibE.Dis.E_Range = LidarCalibShare.params.d_err;
        LidarCalibE.Dis.CNT = LidarCalibShare.params.d_err_cnt;
        LidarCalibE.Angle.E_Range = LidarCalibShare.params.angle_err;
        LidarCalibE.Angle.CNT = LidarCalibShare.params.angle_err_cnt;
        LidarCalibE.LeftRightE = LidarCalibShare.params.left_right_e;
        LidarReadCalibDG(dis)->schedule();
        // createLidarCalibCommand(dis)->schedule();
    }
}
void LabviewCommand::IRCalibShareCommand() { //红外校准
    if (IRCalibStatusShare == COMMEND_CREATE) {
        LABVIEW::IRCalibCtrl IRCalibShare;
        LABVIEW::IRCalibCtrlShareAddress->read(IRCalibShare);
        double dis = IRCalibShare.dis;
        double angle_e = IRCalibShare.angle_e;
        double dis_e = IRCalibShare.dis_e;
        double cnt = IRCalibShare.CNT;
        double left_right_e = IRCalibShare.left_right_e;
        IRCalCommandAssistance(dis, angle_e, dis_e, cnt, left_right_e, 0)->schedule();
    }
}
void LabviewCommand::SingleIRCalibShareCommand() { //单红外校准
    if (SingleIRCalibStatusShare == COMMEND_CREATE) {
        LABVIEW::SingleIRCalibCtrl SingleIRCalibCtrlShare;
        LABVIEW::SingleIRCalibCtrlShareAddress->read(SingleIRCalibCtrlShare);
        double dis = SingleIRCalibCtrlShare.dis;
        double angle_e = SingleIRCalibCtrlShare.angle_e;
        double dis_e = SingleIRCalibCtrlShare.dis_e;
        double cnt = SingleIRCalibCtrlShare.CNT;
        SingleIRCalCommandAssistance(dis, angle_e, dis_e, cnt)->schedule();
    }
}
void LabviewCommand::USCalibShareCommand() { //超声波校准
    if (USCalibStatusShare == COMMEND_CREATE) {
        LABVIEW::USCalibCtrl USCalibShare;
        LABVIEW::USCalibCtrlShareAddress->read(USCalibShare);
        double dis = USCalibShare.dis;
        double angle_e = USCalibShare.angle_e;
        double dis_e = USCalibShare.dis_e;
        double cnt = USCalibShare.CNT;
        double left_right_e = USCalibShare.left_right_e;
        USCalCommandAssistance(dis, angle_e, dis_e, cnt, left_right_e, 0)->schedule();
    }
}
void LabviewCommand::AlongRightWallShareCommand() { //沿右墙行驶
    if (AlongRightWallStatusShare == COMMEND_CREATE) {
        LABVIEW::AlongWallCtrl AlongRightWallShare;
        LABVIEW::AlongRightWallShareAddress->read(AlongRightWallShare);
        LABVIEW::Pose labview_pose = AlongRightWallShare.input.pose;
        Pose pose(labview_pose.x_, labview_pose.y_, labview_pose.theta_);
        double dis = AlongRightWallShare.input.dis;
        double speed = AlongRightWallShare.input.v;
        MoveAlongRightWallRG(pose, dis, speed)->schedule();
    }
}
void LabviewCommand::AlongLeftWallShareCommand() { //沿左墙行驶
    if (AlongLeftWallStatusShare == COMMEND_CREATE) {
        LABVIEW::AlongWallCtrl AlongLeftWallShare;
        LABVIEW::AlongLeftWallShareAddress->read(AlongLeftWallShare);
        LABVIEW::Pose labview_pose = AlongLeftWallShare.input.pose;
        Pose pose(labview_pose.x_, labview_pose.y_, labview_pose.theta_);
        double dis = AlongLeftWallShare.input.dis;
        double speed = AlongLeftWallShare.input.v;
        MoveAlongLeftWallRG(pose, dis, speed)->schedule();
    }
}
void LabviewCommand::IdentifyFruitshareCommand() { //视觉识别
    if (IdentifyStatusShare == COMMEND_CREATE) {
        createIdentifyFruitCommand()->schedule();
    }
}

void LabviewCommand::updataImgCalParams() { //视觉标定
    if (ImgCalStatusShare == COMMEND_CREATE) {
        uint8_t updata_status = COMMEND_WAIT;
        LABVIEW::CalImgStatusShareAddress->write(updata_status);
        LABVIEW::CalImg CalImgShare;
        LABVIEW::CalImgShareAddress->read(CalImgShare);
        //标定数据
        ImgCalData.Image.P1 = {CalImgShare.Image.P1.x, CalImgShare.Image.P1.y};
        ImgCalData.Image.P2 = {CalImgShare.Image.P2.x, CalImgShare.Image.P2.y};
        ImgCalData.Image.P3 = {CalImgShare.Image.P3.x, CalImgShare.Image.P3.y};
        ImgCalData.Image.P4 = {CalImgShare.Image.P4.x, CalImgShare.Image.P4.y};
        ImgCalData.Object.P1 = {CalImgShare.Object.P1.x, CalImgShare.Object.P1.y};
        ImgCalData.Object.P2 = {CalImgShare.Object.P2.x, CalImgShare.Object.P2.y};
        ImgCalData.Object.P3 = {CalImgShare.Object.P3.x, CalImgShare.Object.P3.y};
        ImgCalData.Object.P4 = {CalImgShare.Object.P4.x, CalImgShare.Object.P4.y};
        updata_status = COMMEND_END;
        LABVIEW::CalImgStatusShareAddress->write(updata_status);
    }
}
void LabviewCommand::updataShareLidarParams() { //更新lidar参数
    if (LidarInitParamsStatusShare == COMMEND_CREATE) {
        uint8_t updata_status = COMMEND_WAIT;
        LABVIEW::LidarInitParamsStatusShareAddress->write(updata_status);
        updataLidarParams();
        updata_status = COMMEND_END;
        LABVIEW::LidarInitParamsStatusShareAddress->write(updata_status);
    }
}
void LabviewCommand::updataSharePIDParams() { //更新pid参数
    if (updataPIDStatusShare == COMMEND_CREATE) {
        uint8_t updata_status = COMMEND_WAIT;
        LABVIEW::updateAllPIDStatusShareAddress->write(updata_status);
        updataPIDParams();
        updata_status = COMMEND_END;
        LABVIEW::updateAllPIDStatusShareAddress->write(updata_status);
    }
}

//创建指令列表
void LabviewCommand::createShareMemoryListCommand() {
    TestIOStatusShareCommand();    //测试IO
    TrackingPointShareCommand();   //坐标移动
    TrackingXYShareCommand();      //坐标移动XY
    RotateShareCommand();          //底盘旋转
    ZeroOdomShareCommand();        //坐标清零
    SetOdomShareCommand();         //设定位姿
    LiftDistanceShareCommand();    //升降控制
    ResetLiftShareCommand();       //升降复位控制
    TurnAngleShareCommand();       //旋转控制
    ResetTurnAngleShareCommand();  //旋转复位控制
    ClampServoShareCommand();      //夹手舵机
    TelescopicServoShareCommand(); //伸缩舵机
    RaiseServoShareCommand();      //摆手舵机
    RotatingServoShareCommand();   //旋转舵机
    LidarCalibShareCommand();      //雷达校准
    IRCalibShareCommand();         //红外校准
    SingleIRCalibShareCommand();   //单红外校准
    USCalibShareCommand();         //超声波校准
    AlongRightWallShareCommand();  //沿右墙行驶
    AlongLeftWallShareCommand();   //沿左墙行驶
    IdentifyFruitshareCommand();   //视觉识别

    updataImgCalParams();     //视觉标定
    updataShareLidarParams(); //更新lidar参数
    updataSharePIDParams();   //更新pid参数
}

void LabviewCommand::initialize() {
    InitCommandStatus();
    uint8_t updata_status = COMMEND_WAIT;
    LABVIEW::LabviewStatusShareAddress->write(updata_status);

    init_time = RobotGenius::getCurrentMs();
    std::cout << "LabviewCommand initialize!" << std::endl;
    is_finished = false;
}
void LabviewCommand::execute() {
    int time = RobotGenius::getCurrentMs();
    int dt = time - m_last_time;
    m_last_time = time;
    cur_time_s = static_cast<double>(time - init_time) / 1000.0;
    LABVIEW::LabviewTimeShareAddress->write(cur_time_s);

    readShareMemory();
    createShareMemoryListCommand();

    // std::cout << "LabviewCommand execute dt = " << static_cast<double>(dt) <<
    // std::endl;

    // static uint8_t Labview_cnt = 0;
    // Labview_cnt++;
    // if(Labview_cnt > 9){
    //   std::cout << "RobotGenius Current Time: " << cur_time_s << std::endl;
    //   Labview_cnt = 0;
    // }
}
void LabviewCommand::end() {
    InitCommandStatus();
    uint8_t updata_status = COMMEND_END;
    LABVIEW::LabviewStatusShareAddress->write(updata_status);
    std::cout << "LabviewCommand end!" << std::endl;
}
bool LabviewCommand::isFinished() {
    uint8_t command_status;
    LABVIEW::LabviewStatusShareAddress->read(command_status);
    if (command_status == COMMEND_CANCEL) {
        is_finished = true;
    }
    if (Robot::GetInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::GetInstance().getStopSignal();
}

Command::ptr createLabviewCommand() { return std::make_shared<LabviewCommand>()->withTimer(100); }

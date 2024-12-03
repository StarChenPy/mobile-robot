#include "command/vision/VisionTrackingAction.h"
#include "command/lidar/LidarAlongWallCommand.h"

namespace robot {
    ICommand::ptr VisionCalibAction() {
        ICommandGroup::ptr sequential = SequentialCommandGroup::create();
        sequential->addCommand(
                RotateCommand::create(90),
                LidarCalibCommand::create(24),
                createZeroOdomCommand(),
                createTrackingPointCommand(Pose(13,0,0),10),
                RotateCommand::create(-90),
                LidarCalibCommand::create(22),
                createZeroOdomCommand()
        );
        return sequential;
    }

    ICommand::ptr VisionCalibAction_2(){
        ICommandGroup::ptr sequential = SequentialCommandGroup::create();
        sequential->addCommand(
                RotateCommand::create(-90),
                // LidarReadCalibDG(23),
                LidarCalibCommand::create(23),
                // createZeroOdomCommand(),
                // createTrackingPointCommand(Pose(10,0,0),10),
                RotateCommand::create(90),
                LidarCalibCommand::create(22),
                // LidarReadCalibDG(22),
                createZeroOdomCommand()
        );
        return sequential;
    }

    ICommand::ptr VisionPick_S(int fruit){
        ICommandGroup::ptr sequential = SequentialCommandGroup::create();
        sequential->addCommand(
                createVisionIdentifyCommand(fruit),
                createVisionMoveCommand(fruit),
                // createVisionCtrlCommand(fruit),   //移动机器人到水果正前方位置
                createRaiseServoCommand(90, 5),
                createVisionHeightCtrlCommand(fruit),      //
                PickFruitSG()               //捉水果动作
        );
        return sequential;
    }

    ICommand::ptr IdentifyTracking_R(int column, int fruit){
        ICommand::ptr AlongWall;
        double d_wall;
        switch (column) {
            case 1: {
                d_wall = 42;
                AlongWall = AlongRightWallCommand::create(d_wall, 5);
                break;
            }
            case 2: {
                d_wall = 23;
                AlongWall = AlongLeftWallCommand::create(d_wall, 5);
                break;
            }
            case 3: {
                d_wall = 42;
                AlongWall = AlongRightWallCommand::create(d_wall, 5);
                break;
            }
            default: {
                break;
            }
        }

        ParallelRaceGroup::ptr r = std::make_shared<ParallelRaceGroup>();
        r->addCommand(
                createVisionIdentifyCommand(fruit),     //识别前方是否有指定水果
                AlongWall
                // createTrackingPointCommand(pose, 10)    //坐标移动
        );
        return r;
    }

    ICommand::ptr TrackingIdentifyPick_S(Pose pose, int fruit, int column){
        SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
        sequential->addCommand(
                VisionStatus(),
                IdentifyTracking_R(column, fruit),
                VisionPick_S(fruit)
        );
        return sequential;
    }

    ICommand::ptr TrackingIdentifyPick_DG(Pose pose, int fruit, int column){
        ParallelDeadlineGroup::ptr DG = std::make_shared<ParallelDeadlineGroup>();
        SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
        ParallelRaceGroup::ptr RG = std::make_shared<ParallelRaceGroup>();
        auto identifyAction = createVisionIdentifyCommand(fruit);
        auto trackingAction = createTrackingPointCommand(pose, 2.5);

        auto servoAction = VisionStatus();
        auto pickAction = VisionPick_S(fruit);

        RG->addCommand(identifyAction, trackingAction);
        sequential->addCommand(
                servoAction,
                RG,
                pickAction
        );
        DG->addCommand(sequential);
        DG->setDeadlineCommand(trackingAction);
        DG->disableScheduleDeadlineCommand();
        return DG;
    }

    VisionTrackingTackCommand::VisionTrackingTackCommand(Pose pose, int fruit, int col) {
        endPose_ = pose;
        pickFruit_ = fruit;
        column_ = col;
        isFinished_ = false;
    }
    void VisionTrackingTackCommand::execute() {
        std::cout << "isFinishedDec:" << isFinished() << std::endl;
        if ((state_ != ICommand::State::FINISHED || state_ != ICommand::State::STOP) && !isFinished()) {
            state_ = ICommand::State::PAUSED;
            action_ = TrackingIdentifyPick_S(endPose_, pickFruit_, column_);
            action_->parent_ = getPtr();
            action_->schedule();
        }
    }
    void VisionTrackingTackCommand::end() {
        action_->cancel();
        Robot::getInstance().setRightMotorSpeed(0);
        Robot::getInstance().setLeftMotorSpeed(0);
        std::cout << "VisionTrackingTackCommand end!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }

    isReachPointCommand::isReachPointCommand(Pose pose) {
        endPose_ = pose;
        dMin = 4;
    }
    void isReachPointCommand::execute() {
        Pose cur = Robot::getInstance().odom->getPose();
        double d = std::sqrt((cur.x_ - endPose_.x_) * (cur.x_ - endPose_.x_) + (cur.y_ - endPose_.y_) * (cur.y_ - endPose_.y_));
        if(d < dMin){
            isFinished_ = true;
        }else{
            isFinished_ = false;
        }
    }
    void isReachPointCommand::end() {
        Robot::getInstance().setRightMotorSpeed(0);
        Robot::getInstance().setLeftMotorSpeed(0);
        std::cout << "isReachPointCommand end!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }
} // namespace robot
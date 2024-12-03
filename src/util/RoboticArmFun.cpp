/**
 * @file RoboticArmFun.hpp
 * @author Zijian.Yan (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "command/RoboticArmFun.h"

using namespace std;
using namespace robot;

//伸缩舵机逐步运行动作
ICommand::ptr TelescopicCtrlAction(double start, double end){
    double d = 0.5;
    double cnt_limit = 1;
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    if(end > start){
        for(double dis = start; dis <= end; dis += d){
            sequential->addCommand(createTelescopicServoCommand(dis, cnt_limit));
        }
    }else{
        for(double dis = start; dis >= end; dis -= d){
            sequential->addCommand(createTelescopicServoCommand(dis, cnt_limit));
        }
    }
    return sequential;
}

//夹手舵机逐步运行动作
ICommand::ptr ClampCtrlAction(double start, double end){
    double d_l = 1;
    double cnt_limit = 1;
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    if(end > start){
        for(double len = start; len <= end; len += d_l){
            sequential->addCommand(createClampServoCommand(len, cnt_limit));
        }
    }else{
        for(double len = start; len >= end; len -= d_l){
            sequential->addCommand(createClampServoCommand(len, cnt_limit));
        }
    }
    return sequential;
}

//抬手舵机逐步运行动作
ICommand::ptr RaiseCtrlAction(double start, double end){
    double d_angle = 5;
    double cnt_limit = 1;
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    if(end > start){
        for(double angle = start; angle <= end; angle += d_angle){
            sequential->addCommand(createRaiseServoCommand(angle, cnt_limit));
        }
    }else{
        for(double angle = start; angle >= end; angle -= d_angle){
            sequential->addCommand(createRaiseServoCommand(angle, cnt_limit));
        }
    }
    return sequential;
}

//升降旋转置零动作
ICommand::ptr ResetLiftAndTurn(double turn_speed){
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            ResetLiftMotorCommand::create(10),         //重置升降
            LiftMotorDistancePIDCommand::create(-1),       //下降1cm
            ResetLiftMotorCommand::create(turn_speed)   //旋转复位
    );
    return sequential;
}

//抓手并行
ICommand::ptr GripperServoPG(double telescopic, double raise, double clamp){
    ParallelCommandGroup::ptr PG = std::make_shared<ParallelCommandGroup>();
    PG->addCommand(
            createRaiseServoCommand(raise),               //RAISE_ANGLE_MIN = 0.0   RAISE_ANGLE_MAX = 90.0
            createTelescopicServoCommand(telescopic),    //TELESCOPIC_DIS_MIN = 0.0   TELESCOPIC_DIS_MAX = 9.0
            createClampServoCommand(clamp)                //CLAMP_LEN_MIN = 2.0    CLAMP_LEN_MAX = 24.5
    );
    return PG;
}

//升降旋转并行
ICommand::ptr LiftAndTurnPG(double h, double angle){
    ParallelCommandGroup::ptr PG = std::make_shared<ParallelCommandGroup>();
    PG->addCommand(
            LiftMotorDistancePIDCommand::create(h),
            TurnMotorDistancePIDCommand::create(angle)
    );
    return PG;
}

//重置机械臂
ICommand::ptr ResetRoboticArm(){
    SequentialCommandGroup::ptr LiftS = std::make_shared<SequentialCommandGroup>();
    LiftS->addCommand(
            ResetLiftMotorCommand::create(15),         //重置升降
            LiftMotorDistancePIDCommand::create(-1)       //下降1cm
    );
    ParallelCommandGroup::ptr PG = std::make_shared<ParallelCommandGroup>();
    PG->addCommand(
            LiftS,
            ResetLiftMotorCommand::create(-10),   //旋转复位
            GripperServoPG(3, 0, 22)
    );
    return PG;
}


//车辆移动机械臂状态
ICommand::ptr CarMoveStatus(){
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            LiftMotorDistancePIDCommand::create(-1),       //升高最高
            createClampServoCommand(CLAMP_LEN_MAX),   //抓手，打开到最大
            createTelescopicServoCommand(5),         //伸缩，伸到5cm位置
            createRaiseServoCommand(0),               //摆手，垂直：0°
            TurnMotorDistancePIDCommand::create(178),                 //旋转180°
            LiftMotorDistancePIDCommand::create(-25)               //下降25cm
    );
    return sequential;
}

//车辆移动机械臂状态并行
ICommand::ptr CarMoveStatusPG(){
    ParallelCommandGroup::ptr PG = std::make_shared<ParallelCommandGroup>();
    PG->addCommand(
            GripperServoPG(5, 0, CLAMP_LEN_MAX),
            TurnMotorDistancePIDCommand::create(178),                 //旋转180°
            LiftMotorDistancePIDCommand::create(-20)               //下降20cm
    );
    return PG;
}



//初始状态
ICommand::ptr InitStatus(){
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            LiftMotorDistancePIDCommand::create(-20),
            TurnMotorDistancePIDCommand::create(180),
            createRaiseServoCommand(90),       //RAISE_ANGLE_MIN = 0.0   RAISE_ANGLE_MAX = 90.0
            createTelescopicServoCommand(0),    //TELESCOPIC_DIS_MIN = 0.0   TELESCOPIC_DIS_MAX = 9.0
            createClampServoCommand(10)         //CLAMP_LEN_MIN = 2.0    CLAMP_LEN_MAX = 19.5
    );
    return sequential;
}

// 限高
ICommand::ptr HeightLimitStatus(){
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            LiftMotorDistancePIDCommand::create(-1),
            createTelescopicServoCommand(5),    //TELESCOPIC_DIS_MIN = 0.0   TELESCOPIC_DIS_MAX = 9.0
            createRaiseServoCommand(90),       //RAISE_ANGLE_MIN = 0.0   RAISE_ANGLE_MAX = 90.0
            createClampServoCommand(10),         //CLAMP_LEN_MIN = 2.0    CLAMP_LEN_MAX = 19.5
            TurnMotorDistancePIDCommand::create(0),
            LiftMotorDistancePIDCommand::create(-60)
    );
    return sequential;
}

//看水果抓手状态
ICommand::ptr VisionServoStatus(){
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            createRaiseServoCommand(45),       //RAISE_ANGLE_MIN = 0.0   RAISE_ANGLE_MAX = 90.0
            createTelescopicServoCommand(-3.6),    //TELESCOPIC_DIS_MIN = 0.0   TELESCOPIC_DIS_MAX = 9.0
            createClampServoCommand(CLAMP_LEN_MAX)         //CLAMP_LEN_MIN = 2.0    CLAMP_LEN_MAX = 24.5
    );
    return sequential;
}


//看水果机械臂状态
ICommand::ptr VisionStatus(){
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            LiftMotorDistancePIDCommand::create(-1),
            TurnMotorDistancePIDCommand::create(90),
            VisionServoStatus(),    //三个抓手舵机
            LiftMotorDistancePIDCommand::create(-40)
    );
    return sequential;
}

ICommand::ptr VisionStatusPG(){
    ParallelCommandGroup::ptr PG = std::make_shared<ParallelCommandGroup>();
    PG->addCommand(
            GripperServoPG(-3.8, 50, CLAMP_LEN_MAX),
            TurnMotorDistancePIDCommand::create(90),                 //旋转180°
            LiftMotorDistancePIDCommand::create(-20)               //下降20cm
    );
    SequentialCommandGroup::ptr SG = std::make_shared<SequentialCommandGroup>();
    SG->addCommand(
            PG,
            LiftMotorDistancePIDCommand::create(-38)
    );
    return SG;
}


//抓水果动作
ICommand::ptr PickFruitStatus(int layer){
    double h;
    double raise = 90;
    double telescopic = TELESCOPIC_DIS_MIN;
    double clamp = 7;
    switch (layer){
        case 1:
            h = -25;
            break;
        case 2:
            h = -35;
            break;
        case 3:
            h = -46;
            break;
        default:
            h = 0;
            std::cout << "layer over" << std::endl;
            break;
    }

    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();

    sequential->addCommand(
            LiftMotorDistancePIDCommand::create(h),
            createRaiseServoCommand(raise),       //RAISE_ANGLE_MIN = 0.0   RAISE_ANGLE_MAX = 90.0
            createTelescopicServoCommand(telescopic),    //TELESCOPIC_DIS_MIN = 0.0   TELESCOPIC_DIS_MAX = 9.0
            createClampServoCommand(clamp),         //CLAMP_LEN_MIN = 2.0    CLAMP_LEN_MAX = 19.5
            // createTelescopicServoCommand(0)
            LiftMotorDistancePIDCommand::create(-20)
    );
    return sequential;
}

//抓水果动作
ICommand::ptr PickFruitSG(){
    double telescopic = -2;
    double clamp = 7;

    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            // createVisionHeightCtrlCommand(0),
            // createRaiseServoCommand(raise),
            createTelescopicServoCommand(telescopic, 5),
            createClampServoCommand(clamp),
            // createTelescopicServoCommand(0)
            LiftMotorDistancePIDCommand::create(-20)
    );
    return sequential;
}



//抓篮子
ICommand::ptr PickBasketAction(){
    double pick_h = -2.0;
    double down_h = pick_h - 21;
    double raise = 115;
    double pick_raise = 100;
    double pick_telescopic = 9;
    double clamp = 18;
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            LiftMotorDistancePIDCommand::create(pick_h),
            createRaiseServoCommand(pick_raise),
            createTelescopicServoCommand(TELESCOPIC_DIS_MIN),

            createClampServoCommand(CLAMP_LEN_MAX),
            TurnMotorDistancePIDCommand::create(-90),
            TelescopicCtrlAction(TELESCOPIC_DIS_MIN, pick_telescopic),
            // createRaiseServoCommand(90),
            // createTelescopicServoCommand(pick_telescopic),
            createClampServoCommand(clamp),
            createRaiseServoCommand(raise),
            TurnMotorDistancePIDCommand::create(0),
            createRaiseServoCommand(0),
            TurnMotorDistancePIDCommand::create(180),

            createRaiseServoCommand(14),
            LiftMotorDistancePIDCommand::create(down_h),
            createClampServoCommand(CLAMP_LEN_MAX)
    );
    return sequential;
}

//放篮子
ICommand::ptr PutBasketAction(){
    double h = -24;
    double raise = 14;
    double telescopic = 9;
    double clamp = 18.2;
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            TurnMotorDistancePIDCommand::create(180),
            createClampServoCommand(CLAMP_LEN_MAX),
            TelescopicCtrlAction(0, telescopic),
            createRaiseServoCommand(raise),
            LiftMotorDistancePIDCommand::create(h),
            createClampServoCommand(clamp),
            LiftMotorDistancePIDCommand::create(-1),
            TurnMotorDistancePIDCommand::create(0),
            createRaiseServoCommand(115),
            TurnMotorDistancePIDCommand::create(-90),
            createRaiseServoCommand(100),
            LiftMotorDistancePIDCommand::create(-1),
            createClampServoCommand(CLAMP_LEN_MAX),
            createRaiseServoCommand(115,20),
            TelescopicCtrlAction(telescopic,TELESCOPIC_DIS_MIN),
            TurnMotorDistancePIDCommand::create(0)
            // createTelescopicServoCommand(0),
            // TurnMotorDistancePIDCommand::create(0),

    );
    return sequential;
}

//放篮子
ICommand::ptr PutFruit2BasketAction(){
    double h = -15;
    double raise = 14;
    double telescopic = 9;
    // double clamp = 18.2;
    double turn_angle = 170;
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            // TelescopicCtrlAction(0, telescopic),
            createTelescopicServoCommand(telescopic),
            createRaiseServoCommand(raise),
            TurnMotorDistancePIDCommand::create(turn_angle),
            LiftMotorDistancePIDCommand::create(h),
            createClampServoCommand(CLAMP_LEN_MAX)
    );
    return sequential;
}



//放水果进篮子
ICommand::ptr PutFruit2BasketAction_1(int DiJiGe , int ZuoYou){
    double turn_angle;
    double raise;
    double telescopic;

    switch (DiJiGe){
        case 1:
            switch (ZuoYou){
                case 0:
                    turn_angle = 190;
                    break;

                case 1:
                    turn_angle = 160;
                    break;
            }
            raise = 0;
            telescopic = 0;
            break;

        case 2:
            switch (ZuoYou){
                case 0:
                    turn_angle = 188;
                    break;

                case 1:
                    turn_angle = 170;
                    break;
            }
            raise = 10;
            telescopic = 9;
            break;

        case 3:
            switch (ZuoYou){
                case 0:
                    turn_angle = 185;
                    break;

                case 1:
                    turn_angle = 170;
                    break;
            }
            raise = 38;
            telescopic = 8;
            break;
    }
    double h = -22;
    // double raise = 14;
    // double telescopic = 9;
    // double clamp = 18.2;
    // double turn_angle = 170;
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            // TelescopicCtrlAction(0, telescopic),
            createTelescopicServoCommand(telescopic),
            createRaiseServoCommand(raise),
            TurnMotorDistancePIDCommand::create(turn_angle),
            LiftMotorDistancePIDCommand::create(h),
            createClampServoCommand(CLAMP_LEN_MAX)
    );
    return sequential;
}


//放水果进篮子并行
ICommand::ptr PutFruit2BasketPG(int DiJiGe , int ZuoYou){
    double turn_angle;
    double raise;
    double telescopic;

    switch (DiJiGe){
        case 1:
            switch (ZuoYou){
                case 0:
                    turn_angle = 190;
                    break;

                case 1:
                    turn_angle = 160;
                    break;
            }
            raise = 0;
            telescopic = 0;
            break;

        case 2:
            switch (ZuoYou){
                case 0:
                    turn_angle = 188;
                    break;

                case 1:
                    turn_angle = 170;
                    break;
            }
            raise = 10;
            telescopic = 9;
            break;

        case 3:
            switch (ZuoYou){
                case 0:
                    turn_angle = 185;
                    break;

                case 1:
                    turn_angle = 170;
                    break;
            }
            raise = 38;
            telescopic = 8;
            break;
    }
    double h = -19;
    ParallelCommandGroup::ptr ArmPG = std::make_shared<ParallelCommandGroup>();
    ArmPG->addCommand(
            // TelescopicCtrlAction(0, telescopic),
            createTelescopicServoCommand(telescopic),
            createRaiseServoCommand(raise),
            TurnMotorDistancePIDCommand::create(turn_angle),
            LiftMotorDistancePIDCommand::create(h)
    );
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            ArmPG,
            createClampServoCommand(CLAMP_LEN_MAX)
    );
    return sequential;
}
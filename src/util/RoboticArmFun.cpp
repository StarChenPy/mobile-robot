#include "util/RoboticArmFun.h"

//-------------------------------- 重置功能 --------------------------------//

/**
 * 重置机械臂的抬升和旋转部件。
 * @return 并行命令组。
 */
ICommand::ptr resetLiftAndTurn() {
    ICommandGroup::ptr root = ParallelCommandGroup::create();
    SequentialCommandGroup::ptr liftReset = std::make_shared<SequentialCommandGroup>();
    liftReset->addCommand(
            ResetLiftMotorCommand::create(10),
            LiftMotorDistancePIDCommand::create(-1)
    );
    root->addCommand(
            liftReset,
            ResetLiftMotorCommand::create(10)
    );
    return root;
}

/**
 * 重置整个机械臂，包括抬升、旋转和抓手部件。
 * @return 并行命令组。
 */
ICommand::ptr resetRoboticArm() {
    ICommandGroup::ptr root = ParallelCommandGroup::create();
    root->addCommand(
            resetLiftAndTurn(),
            gripperServoAction(3, 0, 22)
    );
    return root;
}

/**
 * 将机械臂设置为默认位置。
 * @return 顺序命令组。
 */
ICommand::ptr moveToDefault() {
    ICommandGroup::ptr root = SequentialCommandGroup::create();
    root->addCommand(
            LiftMotorDistancePIDCommand::create(-20),
            TurnMotorDistancePIDCommand::create(90),
            RaiseServoCommand::create(90),
            TelescopicServoCommand::create(0),
            ClampServoCommand::create(10)
    );
    return root;
}

//-------------------------------- 控制功能 --------------------------------//

/**
 * 控制机械臂的伸缩动作。
 * @param start 起始位置。
 * @param end 结束位置。
 * @return 顺序命令组。
 */
ICommand::ptr telescopicControlAction(double start, double end) {
    double step = 0.5;
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    if (end > start) {
        for (double position = start; position <= end; position += step) {
            sequential->addCommand(TelescopicServoCommand::create(position));
        }
    } else {
        for (double position = start; position >= end; position -= step) {
            sequential->addCommand(TelescopicServoCommand::create(position));
        }
    }
    return sequential;
}

/**
 * 控制机械臂夹持器的动作。
 * @param start 起始长度。
 * @param end 结束长度。
 * @return 顺序命令组。
 */
ICommand::ptr clampControlAction(double start, double end) {
    double step = 1;
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    if (end > start) {
        for (double length = start; length <= end; length += step) {
            sequential->addCommand(ClampServoCommand::create(length));
        }
    } else {
        for (double length = start; length >= end; length -= step) {
            sequential->addCommand(ClampServoCommand::create(length));
        }
    }
    return sequential;
}

/**
 * 控制机械臂的升降动作。
 * @param start 起始角度。
 * @param end 结束角度。
 * @return 顺序命令组。
 */
ICommand::ptr raiseControlAction(double start, double end) {
    double step = 5;
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    if (end > start) {
        for (double angle = start; angle <= end; angle += step) {
            sequential->addCommand(RaiseServoCommand::create(angle));
        }
    } else {
        for (double angle = start; angle >= end; angle -= step) {
            sequential->addCommand(RaiseServoCommand::create(angle));
        }
    }
    return sequential;
}

/**
 * 组合控制机械臂的抓手动作，包括升降、伸缩和夹持。
 * @param telescopic 伸缩位置。
 * @param raise 升降角度。
 * @param clamp 夹持长度。
 * @return 并行命令组。
 */
ICommand::ptr gripperServoAction(double telescopic, double raise, double clamp) {
    ParallelCommandGroup::ptr parallelGroup = std::make_shared<ParallelCommandGroup>();
    parallelGroup->addCommand(
            RaiseServoCommand::create(raise),
            TelescopicServoCommand::create(telescopic),
            ClampServoCommand::create(clamp)
    );
    return parallelGroup;
}

/**
 * 控制机械臂的抬升和旋转动作。
 * @param height 抬升高度。
 * @param angle 旋转角度。
 * @return 并行命令组。
 */
ICommand::ptr liftAndTurnAction(double height, double angle) {
    ParallelCommandGroup::ptr parallelGroup = std::make_shared<ParallelCommandGroup>();
    parallelGroup->addCommand(
            LiftMotorDistancePIDCommand::create(height),
            TurnMotorDistancePIDCommand::create(angle)
    );
    return parallelGroup;
}

//-------------------------------- 复杂操作 --------------------------------//

/**
 * 将机械臂调整为车辆移动状态。
 * @return 顺序命令组。
 */
ICommand::ptr carMoveStatus() {
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            LiftMotorDistancePIDCommand::create(-1),
            ClampServoCommand::create(CLAMP_LEN_MAX),
            TelescopicServoCommand::create(5),
            RaiseServoCommand::create(0),
            TurnMotorDistancePIDCommand::create(178),
            LiftMotorDistancePIDCommand::create(-25)
    );
    return sequential;
}

/**
 * 将机械臂调整为车辆移动状态（并行方式）。
 * @return 并行命令组。
 */
ICommand::ptr carMoveStatusParallel() {
    ParallelCommandGroup::ptr parallelGroup = std::make_shared<ParallelCommandGroup>();
    parallelGroup->addCommand(
            gripperServoAction(5, 0, CLAMP_LEN_MAX),
            TurnMotorDistancePIDCommand::create(178),
            LiftMotorDistancePIDCommand::create(-20)
    );
    return parallelGroup;
}

/**
 * 根据层数调整机械臂以采摘果实。
 * @param layer 果实所在层数。
 * @return 顺序命令组。
 */
ICommand::ptr pickFruitStatus(int layer) {
    double height;
    switch (layer) {
        case 1: height = -25; break;
        case 2: height = -35; break;
        case 3: height = -46; break;
        default: height = 0; std::cout << "Invalid layer" << std::endl; break;
    }
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            LiftMotorDistancePIDCommand::create(height),
            RaiseServoCommand::create(90),
            TelescopicServoCommand::create(0),
            ClampServoCommand::create(7),
            LiftMotorDistancePIDCommand::create(-20)
    );
    return sequential;
}

/**
 * 执行采摘果篮的动作。
 * @return 顺序命令组。
 */
ICommand::ptr pickBasketAction() {
    double pickHeight = -2.0;
    double downHeight = pickHeight - 21;
    SequentialCommandGroup::ptr sequential = std::make_shared<SequentialCommandGroup>();
    sequential->addCommand(
            LiftMotorDistancePIDCommand::create(pickHeight),
            RaiseServoCommand::create(100),
            TelescopicServoCommand::create(TELESCOPIC_DIS_MIN),
            ClampServoCommand::create(CLAMP_LEN_MAX),
            telescopicControlAction(TELESCOPIC_DIS_MIN, 9),
            ClampServoCommand::create(18),
            LiftMotorDistancePIDCommand::create(downHeight),
            ClampServoCommand::create(CLAMP_LEN_MAX)
    );
    return sequential;
}

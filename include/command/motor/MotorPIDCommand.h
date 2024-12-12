#pragma once
#include "system/Robot.h"
#include "system/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

namespace robot {
/**
 * 左电机PID命令类
 */
    class LeftMotorPIDCommand : public ICommand {
    public:
        void execute() override;
        void end() override;

        static ICommand::ptr create();
    };

/**
 * 右电机PID命令类
 */
    class RightMotorPIDCommand : public ICommand {
    public:
        void execute() override;
        void end() override;

        static ICommand::ptr create();
    };

/**
 * 旋转电机PID命令类
 */
    class TurnMotorPIDCommand : public ICommand {
    public:
        void execute() override;
        void end() override;

        static ICommand::ptr create();
    };

/**
 * 升降电机PID命令类
 */
    class LiftMotorPIDCommand : public ICommand {
    public:
        void execute() override;
        void end() override;

        static ICommand::ptr create();
    };
} // namespace robot

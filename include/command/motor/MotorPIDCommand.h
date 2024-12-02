/**
 * @file MotorPIDCommand.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "system/Robot.h"
#include "util/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

using namespace robot;

namespace robot {
/**
 * 电机PID命令类
 */
    class MotorPIDCommand : public CommandBase {
    public:
        MotorPIDCommand();

        typedef std::shared_ptr<MotorPIDCommand> ptr;

        void initialize() override;
        bool isFinished() override;
    protected:
        bool isFinished_;
    };

/**
 * 左电机PID命令类
 */
    class LeftMotorPIDCommand : public MotorPIDCommand {
    public:
        LeftMotorPIDCommand();

        void execute() override;
        void end() override;

        static Command::ptr create();
    };

/**
 * 右电机PID命令类
 */
    class RightMotorPIDCommand : public MotorPIDCommand {
    public:
        RightMotorPIDCommand();

        void execute() override;
        void end() override;

        static Command::ptr create();
    };

/**
 * 旋转电机PID命令类
 */
    class TurnMotorPIDCommand : public MotorPIDCommand {
    public:
        TurnMotorPIDCommand();

        void execute() override;
        void end() override;

        static Command::ptr create();
    };

/**
 * 升降电机PID命令类
 */
    class LiftMotorPIDCommand : public MotorPIDCommand {
    public:
        LiftMotorPIDCommand();

        void execute() override;
        void end() override;

        static Command::ptr create();
    };
} // namespace robot

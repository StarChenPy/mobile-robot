/**
 * @file ScramCommand.h
 * @author Longping.Huang
 * @brief
 * @version 0.1
 * @date 2024-09-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "system/Robot.h"
#include "system/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

using namespace std;
using namespace robot;
using namespace VMX;

namespace robot {
    class EStopCommand : public ICommand {
    public:
        typedef std::shared_ptr<EStopCommand> ptr;

        /**
         * 急停按钮
         */
        EStopCommand() = default;
        ~EStopCommand() override = default;

        void initialize() override;
        void execute() override;
        void end() override;

        static ICommand::ptr create();
    };

    class StartCommand : public ICommand {
    public:
        typedef std::shared_ptr<StartCommand> ptr;

        /**
         * 启动按钮
         */
        StartCommand() = default;
        ~StartCommand() override = default;

        void initialize() override;
        void execute() override;
        void end() override;

        static ICommand::ptr create();
    };
}

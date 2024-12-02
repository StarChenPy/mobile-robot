/**
 * @file CommandBase.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "Command.h"
#include "command/group/CommandGroupBase.h"
#include "util/Util.h"
#include <iostream>
#include <memory>
namespace robot {
    class CommandBase : public Command {
    protected:
        typedef std::shared_ptr<CommandBase> ptr;
        CommandBase();
        ~CommandBase();

        /**
         * @brief 将命令送进调度器队列
         *
         * @return true
         * @return false
         */
        void initialize() override;
        void execute() override {};
        void end() override{};
        bool isFinished() override;

        bool isScheduled;
        bool isFinished_;
    };

} // namespace robot
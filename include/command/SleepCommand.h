#pragma once
#ifndef MOBILE_ROBOT_SLEEPCOMMAND_H
#define MOBILE_ROBOT_SLEEPCOMMAND_H

#include "ICommand.h"

namespace robot {
    class SleepCommand: public ICommand {
    public:
        void execute() override;

        static ICommand::ptr create(int time);
    private:
        int count;
    };
} // namespace robot

#endif //MOBILE_ROBOT_SLEEPCOMMAND_H

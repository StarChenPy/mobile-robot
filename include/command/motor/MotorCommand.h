#pragma once

#include "command/ICommand.h"
#include "util/params.h"

namespace robot {
    class LeftMotorCommand : public ICommand {
        public:
        LeftMotorCommand();

        void execute() override;

        static ICommand::ptr create();
    };

    class RightMotorCommand : public ICommand {
        public:
        RightMotorCommand();

        void execute() override;

        static ICommand::ptr create();
    };

    class TurnMotorCommand : public ICommand {
        public:
        TurnMotorCommand();

        void execute() override;

        static ICommand::ptr create();
    };

    class LiftMotorCommand : public ICommand {
        public:
        LiftMotorCommand();

        void execute() override;

        static ICommand::ptr create();
    };
} // namespace robot


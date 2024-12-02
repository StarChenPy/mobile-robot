#pragma once

#include "command/CommandBase.h"
#include "util/params.h"

namespace robot {
    class MotorCommand : public CommandBase {
        public:
        typedef std::shared_ptr<MotorCommand> ptr;

        MotorCommand();

        void initialize() override;
        bool isFinished() override;

        static Command::ptr create();

        protected:
        bool isFinished_ = false;
    };

    class LeftMotorCommand : public MotorCommand {
        public:
        LeftMotorCommand();

        void execute() override;
    };

    class RightMotorCommand : public MotorCommand {
        public:
        RightMotorCommand();

        void execute() override;
    };

    class TurnMotorCommand : public MotorCommand {
        public:
        TurnMotorCommand();

        void execute() override;
    };

    class LiftMotorCommand : public MotorCommand {
        public:
        LiftMotorCommand();

        void execute() override;
    };
} // namespace robot


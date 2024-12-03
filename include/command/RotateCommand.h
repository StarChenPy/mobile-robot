#pragma once
#include "system/Robot.h"
#include "util/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

namespace robot {
    class RotateCommand : public ICommand {
    public:
        typedef std::shared_ptr<RotateCommand> Ptr;
        RotateCommand(double angle) : targetAngle_(angle) {}
        // RotateCommand(double angle, double w) : targetAngle_(angle), Vz_max(w) {}
        ~RotateCommand() {}

        void initialize() override;
        void execute() override;
        void end() override;

        static ICommand::ptr create(double angle);
    private:
        int64_t lastTime_ = 0;

        double initPhi_ = 0;
        double targetAngle_;
        // double Vz_max = 360;
    };
} // namespace robot
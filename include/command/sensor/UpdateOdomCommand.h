#pragma once
#include "system/Robot.h"
#include "system/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

namespace robot {
    class UpdateOdomCommand : public ICommand {
    public:
        typedef std::shared_ptr<UpdateOdomCommand> ptr;

        void initialize() override;
        void execute() override;

        static ICommand::ptr create();
    private:
        int64_t lastTime_ = 0;
    };
} // namespace robot

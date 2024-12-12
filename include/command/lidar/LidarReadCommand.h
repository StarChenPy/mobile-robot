#pragma once
#include "system/Robot.h"
#include "system/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"
using namespace std;
using namespace robot;

class LidarReadCommand : public ICommand {
  public:
    typedef std::shared_ptr<LidarReadCommand> Ptr;
    LidarReadCommand() = default;
    ~LidarReadCommand() override = default;

    void execute() override;
    void end() override;

    static ICommand::ptr create();
};

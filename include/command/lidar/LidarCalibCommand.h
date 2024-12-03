#pragma once
#include "system/Robot.h"
#include "util/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

class LidarCalibCommand : public ICommand {
  public:
    typedef std::shared_ptr<LidarCalibCommand> Ptr;
    explicit LidarCalibCommand(double d) : calib_d(d) {}
    ~LidarCalibCommand() override = default;

    void initialize() override;
    void execute() override;
    void end() override;

    static ICommand::ptr create(double d);
  private:
    double calib_d = 30;
};

ICommand::ptr LidarReadCalibDG(double d);
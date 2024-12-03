#pragma once
#include "system/Robot.h"
#include "util/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"
using namespace std;
using namespace robot;

class ZeroOdomCommand : public ICommand {
public:
    ZeroOdomCommand() = default;
    ~ZeroOdomCommand() override = default;

    void initialize() override;
    void execute() override;
    void end() override;
};
ICommand::ptr createZeroOdomCommand();

class SetOdomCommand : public ICommand {
public:
    SetOdomCommand(double x, double y, double theta) : set_x(x), set_y(y), set_theta(theta) {}
    ~SetOdomCommand() override = default;

    void initialize() override;
    void execute() override;
    void end() override;
private:
    double set_x = 0;
    double set_y = 0;
    double set_theta = 0;
};
ICommand::ptr createSetOdomCommand(double x, double y, double theta);
//
// Created by 34253 on 2024/11/28.
//
#pragma once

#include "command/CommandBase.h"
namespace robot {
class MotorSpeedCommand: public CommandBase {
    public:
    typedef std::shared_ptr<MotorSpeedCommand> ptr;

    explicit MotorSpeedCommand(double speed = 10, uint32_t counter = 5);

    void initialize() override;
    bool isFinished() override;

    static Command::ptr create();

    protected:
    bool isFinished_ = false;
    double speed_;
    uint32_t counter_;
    uint32_t currentCounter_;
};

class LeftMotorSpeedCommand : public MotorSpeedCommand {
    public:
    LeftMotorSpeedCommand(double speed, uint32_t counter);

    void execute() override;
    void end() override;
};
class RightMotorSpeedCommand : public MotorSpeedCommand {
    public:
    RightMotorSpeedCommand(double speed, uint32_t counter);

    void execute() override;
    void end() override;
};
} // namespace robot

//
// Created by 34253 on 2024/11/28.
//
#pragma once

#include "command/ICommand.h"
namespace robot {
class MotorSpeedCommand: public ICommand {
    public:
    typedef std::shared_ptr<MotorSpeedCommand> ptr;

    explicit MotorSpeedCommand(double speed, uint32_t counter): speed_(speed), counter_(counter) {};

    protected:
    bool isFinished_ = false;
    double speed_;
    uint32_t counter_;
    uint32_t currentCounter_ = 0;
};

class LeftMotorSpeedCommand : public MotorSpeedCommand {
    public:
    LeftMotorSpeedCommand(double speed, uint32_t counter) : MotorSpeedCommand(speed, counter) {}

    void execute() override;
    void end() override;

    static ICommand::ptr create(double speed = 10, uint32_t counter = 5);
};
class RightMotorSpeedCommand : public MotorSpeedCommand {
    public:
    RightMotorSpeedCommand(double speed, uint32_t counter) : MotorSpeedCommand(speed, counter) {}

    void execute() override;
    void end() override;

    static ICommand::ptr create(double speed = 10, uint32_t counter = 5);
};
} // namespace robot

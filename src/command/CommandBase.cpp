/**
 * @file CommandBase.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "command/CommandBase.h"
#include "system/Robot.h"

namespace robot {
    CommandBase::CommandBase() {
        isGroup_ = false;
        isScheduled = false;
        isFinished_ = false;
    }
    CommandBase::~CommandBase() {
        if (isScheduled) {
            cancel();
        }
    }
    void CommandBase::initialize() {
        isFinished_ = false;
    }
    bool CommandBase::isFinished() {
        if (Robot::getStopSignal()) {
            stopAll();
        }
        return Robot::getStopSignal() || isFinished_;;
    }
} // namespace robot
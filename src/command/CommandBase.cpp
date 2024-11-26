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
#include "command/Scheduler.h"

namespace robot {
CommandBase::~CommandBase() {
    if (isScheduled) {
        cancel();
    }
}
bool CommandBase::isFinished() { return true; }

} // namespace robot
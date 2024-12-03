/**
 * @file SequentialCommandGroup.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "command/group/SequentialCommandGroup.h"

namespace robot {
    void SequentialCommandGroup::initialize() {
        isFinished_ = false;
        if (commands_.empty()) {
            isFinished_ = true;
            return;
        }
        currentCommandIndex_ = 0;
        currentCommand_ = commands_[currentCommandIndex_];
        currentCommand_->schedule();
    }

    void SequentialCommandGroup::execute() {
        if (!currentCommand_)
            return;
        if (currentCommand_->isFinished()) {

            if (currentCommandIndex_ + 1 < commands_.size()) {
                currentCommandIndex_++;
                currentCommand_ = commands_[currentCommandIndex_];
                if (currentCommand_ && currentCommand_->state_ != ICommand::State::STOP) {
                    currentCommand_->schedule();
                }
            } else {
                isFinished_ = true;
            }
        }
        if (state_ != ICommand::State::STOP && !isFinished_) {
            state_ = ICommand::State::PAUSED;
        }
    }

    void SequentialCommandGroup::end() {
        if (commands_[currentCommandIndex_]->state_ != ICommand::State::STOP &&
            commands_[currentCommandIndex_]->state_ != ICommand::State::FINISHED) {
            commands_[currentCommandIndex_]->cancel();
        }
        commands_.clear();
        if (nextCommand_.get()) {
            nextCommand_->schedule();
        }
    }

    ICommandGroup::ptr SequentialCommandGroup::create() {
        return std::make_shared<SequentialCommandGroup>();
    }
} //  namespace robot
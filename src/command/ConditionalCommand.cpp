/**
 * @file ConditionalCommand.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "command/ConditionalCommand.h"
namespace robot {

ConditionalCommand::ConditionalCommand(std::function<bool()> condition, CommandBase::ptr on_true_command,
                                       CommandBase::ptr on_false_command)
    : m_condition_(condition), m_on_true_command_(on_true_command), m_on_false_command_(on_false_command) {}

void ConditionalCommand::initialize() {
    workCommand_ = m_condition_() ? m_on_true_command_ : m_on_false_command_;
    workCommand_->initialize();
}
void ConditionalCommand::execute() { workCommand_->execute(); }
void ConditionalCommand::end() { workCommand_->end(); }

bool ConditionalCommand::isFinished() { return workCommand_->isFinished(); }

} // namespace robot
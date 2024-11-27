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

ConditionalComamand::ConditionalComamand(std::function<bool()> condition, CommandBase::Ptr on_true_command,
                                         CommandBase::Ptr on_false_command)
    : m_condition_(condition), m_on_true_command_(on_true_command), m_on_false_command_(on_false_command) {}

void ConditionalComamand::initialize() {
    workCommand_ = m_condition_() ? m_on_true_command_ : m_on_false_command_;
    workCommand_->initialize();
}
void ConditionalComamand::execute() { workCommand_->execute(); }
void ConditionalComamand::end() { workCommand_->end(); }

bool ConditionalComamand::isFinished() { return workCommand_->isFinished(); }

} // namespace robot
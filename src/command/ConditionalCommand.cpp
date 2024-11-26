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
namespace RobotGenius {

ConditionalComamand::ConditionalComamand(std::function<bool()> condition, CommandBase::Ptr on_true_command,
                                         CommandBase::Ptr on_false_command)
    : m_condition_(condition), m_on_true_command_(on_true_command), m_on_false_command_(on_false_command) {}

void ConditionalComamand::initialize() {
    m_work_command_ = m_condition_() ? m_on_true_command_ : m_on_false_command_;
    m_work_command_->initialize();
}
void ConditionalComamand::execute() { m_work_command_->execute(); }
void ConditionalComamand::end() { m_work_command_->end(); }

bool ConditionalComamand::isFinished() { return m_work_command_->isFinished(); }

} // namespace RobotGenius
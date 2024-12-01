/**
 * @file InterruptibleCommand.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-30
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "command/InterruptibleCommand.h"
#include "command/Scheduler.h"
namespace robot {
InterruptibleCommand::InterruptibleCommand(Command::ptr primary_command, Command::ptr interrupt_condition,
                                           Command::ptr fallback_command) {

    m_primary_command_ = primary_command;
    m_interrupt_condition_ = interrupt_condition;
    m_fallback_command_ = fallback_command;
}
void InterruptibleCommand::initialize() {
    if (!m_interrupt_condition_.get()) {
        std::cout << "Warn:interrupt_condition is null" << std::endl;
        return;
    }
    //
    m_primary_command_->parent_ = getPtr();
    m_interrupt_condition_->parent_ = getPtr();
    m_fallback_command_->parent_ = getPtr();
}
void InterruptibleCommand::execute() {
    if (!m_interrupt_condition_.get() && !m_primary_command_.get() && m_fallback_command_.get()) {
        std::cout << "Warn:interrupt command is null" << std::endl;
        return;
    }

    switch (m_status) {
    case Status::BEFORE:
        if (m_primary_command_->state_ == State::PAUSED)
            m_primary_command_->state_ = State::RUNNING;
        if (m_primary_command_->getTimer().get())
            m_primary_command_->getTimer()->setPause(false);
        else {
            // if (m_primary_command_->state_ == State::WAIT)
            m_primary_command_->schedule();
        }
        if (m_interrupt_condition_->getTimer().get())
            m_interrupt_condition_->getTimer()->setPause(false);
        else {
            if (m_interrupt_condition_->state_ == State::WAIT)
                m_interrupt_condition_->schedule();
        }
        state_ = Command::State::PAUSED;
        m_status = Status::InterruptB;
        break;
    case Status::InterruptB:
        m_primary_command_->state_ = State::PAUSED;
        if (m_primary_command_->getTimer().get())
            m_primary_command_->getTimer()->setPause();
        if (m_interrupt_condition_->isFinishedDec()) {
            m_status = Status::Interrupt;
        }
        break;
    case Status::Interrupt:
        if (m_interrupt_condition_->getTimer().get())
            m_interrupt_condition_->getTimer()->setPause();
        m_fallback_command_->schedule();
        m_status = Status::InterruptA;
        state_ = Command::State::PAUSED;
        break;
    case Status::InterruptA:
        if (m_fallback_command_->isFinishedDec()) {
            m_interrupt_condition_ = m_interrupt_condition_->reset();
            m_fallback_command_ = m_fallback_command_->reset();
            m_status = Status::BEFORE;
            ;
            m_interrupt_condition_->parent_ = getPtr();
            m_fallback_command_->parent_ = getPtr();
        }
        break;
    // case Status::AFTER:
    //   m_primary_command_->state_ = State::RUNNING;
    //   if (m_primary_command_->getTimer().get())
    //   m_primary_command_->getTimer()->setPause(false);
    //   m_primary_command_->schedule();
    //   state_ = Command::State::PAUSED;
    //   m_status = Status::BEFORE;
    //   break;
    default:
        break;
    }
}
void InterruptibleCommand::end() {
    if (!m_interrupt_condition_.get() && !m_primary_command_.get() && m_fallback_command_.get()) {
        std::cout << "Warn:interrupt command is null" << std::endl;
        return;
    }
    if (!m_primary_command_->isFinishedDec())
        m_primary_command_->cancel();
    if (!m_interrupt_condition_->isFinishedDec())
        m_interrupt_condition_->cancel();
    if (!m_fallback_command_->isFinishedDec() && m_fallback_command_->state_ != State::WAIT)
        m_fallback_command_->cancel();
}
bool InterruptibleCommand::isFinished() {
    if (!m_interrupt_condition_.get() && !m_primary_command_.get() && m_fallback_command_.get()) {
        std::cout << "Warn:interrupt command is null" << std::endl;
        return true;
    }
    return m_primary_command_->isFinishedDec();
}
} // namespace robot
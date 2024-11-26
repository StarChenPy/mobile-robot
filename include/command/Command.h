/**
 * @file Command.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "Timer.h"
#include "Util.h"
#include <memory>
namespace RobotGenius {

class Command : public std::enable_shared_from_this<Command> {
  public:
    typedef std::shared_ptr<Command> ptr;
    Command() = default;
    virtual ~Command();

    Command(const Command &);
    Command &operator=(const Command &);
    Command(Command &&) = default;
    Command &operator=(Command &&) = default;
    enum State { WAIT = 0, INIT = 1, RUNNING = 2, HOLDON = 3, FINISHED = 4, CANCELED = 5, STOP = 6 };

  public:
    // virtual bool schedule() = 0;
    virtual void initialize() = 0;
    virtual void execute() = 0;
    virtual void end() = 0;
    void cancel();
    virtual bool isFinished() = 0;
    bool isFinisheddec();
    Command::ptr getPtr();
    virtual bool schedule();
    virtual Command::ptr reset() { return nullptr; }
    Command::ptr withTimer(uint64_t ms);
    std::shared_ptr<Timer> getTimer() { return m_timer_; }

    Command::State getWorkCommandState() { return m_work_command_->m_state; }
    uint32_t getPointer() { return reinterpret_cast<uint64_t>(getPtr().get()); }
    int getThreadId();
    void stopAll() { gloabl_stop_ = true; }
    void setScheduleStatus(bool state) { m_isscheduled_ = state; }

  protected:
    bool m_isscheduled_ = false;
    bool m_is_group_ = false;
    bool m_is_schedule = false;
    std::shared_ptr<Timer> m_timer_;

  public:
    bool has_timer_ = false;

  public:
    State m_state = State::WAIT;
    Command::ptr m_parent;
    Command::ptr m_work_command_;
    bool m_stop_flag_ = false;
    bool gloabl_stop_ = false;
};

} // namespace RobotGenius
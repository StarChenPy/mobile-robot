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
namespace robot {

class Command : public std::enable_shared_from_this<Command> {
  public:
    typedef std::shared_ptr<Command> ptr;
    Command() = default;
    virtual ~Command();

    Command(const Command &);
    Command &operator=(const Command &);
    Command(Command &&) = default;
    Command &operator=(Command &&) = default;
    enum State { WAIT = 0, INIT = 1, RUNNING = 2, PAUSED = 3, FINISHED = 4, CANCELED = 5, STOP = 6 };

  public:
    // virtual bool schedule() = 0;
    virtual void initialize() = 0;
    virtual void execute() = 0;
    virtual void end() = 0;
    void cancel();
    virtual bool isFinished() = 0;
    bool isFinishedDec();
    Command::ptr getPtr();

    /**
     * @brief 将命令送进调度器队列
     *
     * @return true
     * @return false
     */
    virtual bool schedule();

    virtual Command::ptr reset() { return nullptr; }
    Command::ptr withTimer(uint64_t ms);
    std::shared_ptr<Timer> getTimer() { return timer_; }

    Command::State getWorkCommandState() { return workCommand_->state_; }
    uint32_t getPointer() { return reinterpret_cast<uint64_t>(getPtr().get()); }
    void stopAll() { globalStop_ = true; }
    void setScheduleStatus(bool state) { isScheduled_ = state; }

  protected:
    bool isScheduled_ = false;
    bool isGroup_ = false;
    bool isSchedule_ = false;
    std::shared_ptr<Timer> timer_;

  public:
    bool hasTimer_ = false;

  public:
    State state_ = State::WAIT;
    Command::ptr parent_;
    Command::ptr workCommand_;
    bool stopFlag_ = false;
    bool globalStop_ = false;
};

} // namespace robot
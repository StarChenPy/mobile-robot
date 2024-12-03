/**
 * @file ICommand.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "util/Timer.h"
#include "util/Util.h"
namespace robot {
    class ICommand : public std::enable_shared_from_this<ICommand> {
    public:
        enum State { WAIT = 0, INIT = 1, RUNNING = 2, PAUSED = 3, FINISHED = 4, CANCELED = 5, STOP = 6 };

        typedef std::shared_ptr<ICommand> ptr;

        ICommand();
        virtual ~ICommand();

        virtual void initialize() {};
        virtual void execute() = 0;
        virtual void end() {};
        virtual void cancel();
        virtual bool isFinished();

        ICommand::ptr getPtr();

        /**
         * @brief 将命令送进调度器队列
         *
         * @return true
         * @return false
         */
        void schedule();
        ICommand::ptr withTimer(uint64_t ms);
        Timer::ptr getTimer() const { return timer_; }
        ICommand::State getWorkCommandState() { return workCommand_->state_; }
    protected:
        bool isFinished_;
    public:
        bool hasTimer_;
        ICommand::ptr parent_;
        ICommand::ptr workCommand_;
        Timer::ptr timer_;
        State state_;
    };
} // namespace robot
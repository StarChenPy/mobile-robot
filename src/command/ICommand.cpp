#include "command/ICommand.h"
#include "system/Scheduler.h"
#include "command/TimerCommand.h"
#include "system/Robot.h"

ICommand::ICommand() {
    isFinished_ = false;
    hasTimer_ = false;
    state_ = State::WAIT;
}
ICommand::~ICommand() = default;

void ICommand::schedule() {
    // 如果没有工作命令，返回false
    // 子类有可能没有给m_work_command_赋值
    Scheduler::getInstance().schedule(getPtr());
    Scheduler::getInstance().tickle();
}
ICommand::ptr ICommand::getPtr() {
    try {
        std::shared_ptr<ICommand> selfPtr = shared_from_this();
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        return std::shared_ptr<ICommand>(this);
    }
}

ICommand::ptr ICommand::withTimer(uint64_t ms) {
    return std::make_shared<TimerCommand>(ms, getPtr());
}

void ICommand::initialize() {
    LOG(INFO) << abi::__cxa_demangle(typeid(*this).name(), nullptr, nullptr, nullptr) << " 正在初始化";
    isFinished_ = false;
}

void ICommand::end() {
    LOG(INFO) << abi::__cxa_demangle(typeid(*this).name(), nullptr, nullptr, nullptr) << " 正在结束";
}

void ICommand::cancel() {
    isFinished_ = true;
    state_ = ICommand::State::FINISHED;
    if (!hasTimer_) {
        schedule();
    } else {
        Scheduler::getInstance().tickle();
    }
}

bool ICommand::isFinished() {
    return Robot::getStopSignal() || isFinished_;;
}

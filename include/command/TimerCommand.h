#pragma once
#include "command/ICommand.h"
#include "util/Timer.h"
#include <memory>
#include <string>

namespace robot {

class TimerCommand : public ICommand {

  public:
    typedef std::shared_ptr<TimerCommand> ptr;
    TimerCommand(uint64_t ms, ICommand::ptr command);
    virtual ~TimerCommand() {}

  public:
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    uint64_t m_ms_;
};

} // namespace robot
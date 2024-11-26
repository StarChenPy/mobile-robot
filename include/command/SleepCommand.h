//
// Created by 34253 on 2024/11/26.
//

#pragma once

#include "CommandBase.h"

namespace robot {
class SleepCommand: public CommandBase {
  public:
    typedef std::shared_ptr<SleepCommand> ptr;
    explicit SleepCommand(int sleepTime);

    void initialize() override;
    void execute() override;
    void end() override {};
    bool isFinished() override;

    static Command::ptr create(int sleepTime);

  private:
    int sleepTime;
};
}


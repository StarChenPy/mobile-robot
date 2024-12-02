/**
 * @file UpdateOdomCommand.h
 * @author Zijian.Yan (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "system/Robot.h"
#include "util/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"
using namespace std;
using namespace robot;

class UpdateOdomCommand : public CommandBase {
  public:
    typedef std::shared_ptr<UpdateOdomCommand> Ptr;
    UpdateOdomCommand() {}
    ~UpdateOdomCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;
};

Command::ptr createUpdataOdomCommand();
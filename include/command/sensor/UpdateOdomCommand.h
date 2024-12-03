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

class UpdateOdomCommand : public ICommand {
  public:
    typedef std::shared_ptr<UpdateOdomCommand> Ptr;
    UpdateOdomCommand() = default;
    ~UpdateOdomCommand() override = default;

    void initialize() override;
    void execute() override;
    void end() override;

    static ICommand::ptr create();
  private:
    int64_t lastTime_ = 0;
};
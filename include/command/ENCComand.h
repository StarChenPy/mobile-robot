/**
 * @file readLeftENCComand.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-07
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "util/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

using namespace std;
using namespace robot;

class readLeftENCCommand : public CommandBase {
  public:
    typedef std::shared_ptr<readLeftENCCommand> Ptr;
    readLeftENCCommand();
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
};

class readRightENCCommand : public CommandBase {
  public:
    typedef std::shared_ptr<readRightENCCommand> Ptr;
    readRightENCCommand();
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
};

class readTurnENCCommand : public CommandBase {
  public:
    typedef std::shared_ptr<readTurnENCCommand> Ptr;
    readTurnENCCommand();
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
};

class readLiftENCCommand : public CommandBase {
  public:
    typedef std::shared_ptr<readLiftENCCommand> Ptr;
    readLiftENCCommand();
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
};

class readOdomENCCommand : public CommandBase {
  public:
    typedef std::shared_ptr<readOdomENCCommand> Ptr;
    readOdomENCCommand();
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
};

class readAllENCCommand : public CommandBase {
  public:
    typedef std::shared_ptr<readAllENCCommand> Ptr;
    readAllENCCommand();
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
};
class resetLeftENCCommand : public CommandBase {
  public:
    typedef std::shared_ptr<resetLeftENCCommand> Ptr;
    resetLeftENCCommand();
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
};

class resetRightENCCommand : public CommandBase {
  public:
    typedef std::shared_ptr<resetRightENCCommand> Ptr;
    resetRightENCCommand();
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
};

class resetTurnENCCommand : public CommandBase {
  public:
    typedef std::shared_ptr<resetTurnENCCommand> Ptr;
    resetTurnENCCommand();
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
};

class resetLiftENCCommand : public CommandBase {
  public:
    typedef std::shared_ptr<resetLiftENCCommand> Ptr;
    resetLiftENCCommand();
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
};

class resetOdomENCCommand : public CommandBase {
  public:
    typedef std::shared_ptr<resetOdomENCCommand> Ptr;
    resetOdomENCCommand();
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
};
class resetAllENCCommand : public CommandBase {
  public:
    typedef std::shared_ptr<resetAllENCCommand> Ptr;
    resetAllENCCommand();
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
};

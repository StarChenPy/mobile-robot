/**
 * @file VisionCtrlCommand.h
 * @author Zijian.Yan (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "RobotCfg.h"
#include "RobotGenius.h"
#include "params.h"
#include "system/Robot.h"

#include "command/Vision/VisionCommand.h"
using namespace std;
using namespace RobotGenius;

class VisionCtrlCommand : public CommandBase {
  public:
    typedef std::shared_ptr<VisionCtrlCommand> Ptr;
    VisionCtrlCommand(int label) : fruit_label(label) {}
    ~VisionCtrlCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;

    int fruit_label;

    const char
        *Class_names[18] =
            {"banana",       "bell pepper",  "chili pepper",
             "fig",          "green apple",  "red apple",
             "green grape",  "mangosteen",   "momordica charantia",
             "watermelon",   "potato",       "egg",
             "red egg",      "green egg",    "green grape",
             "purple grape", "yellow grape", "lianwuguo"}; //只供打印用，改变这个数组并不会改变输出结果label
};

class VisionIdentifyCommand : public CommandBase {
  public:
    typedef std::shared_ptr<VisionIdentifyCommand> Ptr;
    VisionIdentifyCommand(int label) : fruit_label(label) {}
    ~VisionIdentifyCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;

    int fruit_label;

    const char
        *Class_names[18] =
            {"banana",       "bell pepper",  "chili pepper",
             "fig",          "green apple",  "red apple",
             "green grape",  "mangosteen",   "momordica charantia",
             "watermelon",   "potato",       "egg",
             "red egg",      "green egg",    "green grape",
             "purple grape", "yellow grape", "lianwuguo"}; //只供打印用，改变这个数组并不会改变输出结果label
};

class VisionMoveCommand : public CommandBase {
  public:
    typedef std::shared_ptr<VisionMoveCommand> Ptr;
    VisionMoveCommand(int label) : fruit_label(label) {}
    ~VisionMoveCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;

    int fruit_label;
    Pose InitPose;
    Pose target;

    const char
        *Class_names[18] =
            {"banana",       "bell pepper",  "chili pepper",
             "fig",          "green apple",  "red apple",
             "green grape",  "mangosteen",   "momordica charantia",
             "watermelon",   "potato",       "egg",
             "red egg",      "green egg",    "green grape",
             "purple grape", "yellow grape", "lianwuguo"}; //只供打印用，改变这个数组并不会改变输出结果label
};

class VisionHeightCtrlCommand : public CommandBase {
  public:
    typedef std::shared_ptr<VisionHeightCtrlCommand> Ptr;
    VisionHeightCtrlCommand(int label) : fruit_label(label) {}
    ~VisionHeightCtrlCommand() {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;

    int fruit_label;
    double Height_target;
    int32_t m_setpoint = 0;
    int32_t m_conter = 0;

    const char
        *Class_names[18] =
            {"banana",       "bell pepper",  "chili pepper",
             "fig",          "green apple",  "red apple",
             "green grape",  "mangosteen",   "momordica charantia",
             "watermelon",   "potato",       "egg",
             "red egg",      "green egg",    "green grape",
             "purple grape", "yellow grape", "lianwuguo"}; //只供打印用，改变这个数组并不会改变输出结果label
};

Command::ptr createVisionCtrlCommand(int label);
Command::ptr createVisionIdentifyCommand(int label);
Command::ptr createVisionMoveCommand(int label);
Command::ptr createVisionHeightCtrlCommand(int label);

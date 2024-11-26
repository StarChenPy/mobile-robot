/**
 * @file IOTestCommand.h
 * @author Zijian.Yan (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-10-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "RobotCfg.h"
#include "RobotGenius.h"
#include "params.h"
#include "system/Robot.h"

using namespace std;
using namespace RobotGenius;

class IOTestCommand : public CommandBase {
  public:
    typedef std::shared_ptr<IOTestCommand> Ptr;
    IOTestCommand() {}
    ~IOTestCommand() {}

    void updataShareMemory();

    void readENC();
    void readButton();
    double IRDistance(double input);
    void readSensor();
    void readLimit();
    void readIMU();
    void readLidar();

    void readData();

    void ctrlMotor();
    void ctrlServo();
    void ctrlLED();
    void ResetIMU();
    void ResetENC();

    void ctrlIO();

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;

    //坐标移动
    LABVIEW::WriteIO writrIO;
    LABVIEW::ReadIO readIO;
    // uint8_t TestIOStatusShare;
};

Command::ptr createIOTestCommand();

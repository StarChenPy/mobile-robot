/**
 * @file testLabview.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius,com)
 * @brief 测试共享内存
 * @version 0.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream>
#include <string>
#include "share.h"
#include "RobotCfg.h"
#include "command/MotorPIDCommand.h"
#include "command/TrackingPointCommand.h"
#include "command/UpdataOdomCommand.h"
#include "command/LabviewCommand.h"
#include "command/ButtonCommand.h"
#include "command/Vision/VisionCommand.h"

void LEDshar(){
  ResetLed->setDutyCycle(1);
  StopLed->setDutyCycle(1);
  StartLed->setDutyCycle(1);
  sleep(1);
  ResetLed->setDutyCycle(0);
  StopLed->setDutyCycle(0);
  StartLed->setDutyCycle(0);
  sleep(1);
  ResetLed->setDutyCycle(1);
  StopLed->setDutyCycle(1);
  StartLed->setDutyCycle(1);
  sleep(1);
  ResetLed->setDutyCycle(0);
  StopLed->setDutyCycle(0);
  StartLed->setDutyCycle(0);
  sleep(1);
  ResetLed->setDutyCycle(1);
  StopLed->setDutyCycle(1);
  StartLed->setDutyCycle(1);
  sleep(1);
  ResetLed->setDutyCycle(0);
  StopLed->setDutyCycle(0);
  StartLed->setDutyCycle(0);
}

int main() {
  // LEDshar();
  Scheduler::GetInstance(5, false).start();
  ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
  RG->AddCommands(
    createLabviewCommand(),
    createEStopCommand()
  );

  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  DG->AddCommands(
    createTurnMotorPIDCommand(),
    createLiftMotorPIDCommand(),
    createLeftMotorPIDCommand(),
    createRightMotorPIDCommand(),
    // createLidarReadCommand(),
    createUpdataOdomCommand()
  );
  DG->setDeadlineCommand(RG);
  DG->schedule();

  // vision::GetInstance();    //开启摄像头，预加载识别模型

  sleep(1);
  Scheduler::GetInstance().stop();

  return 0;
}
  
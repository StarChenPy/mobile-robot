/**
 * @file robot.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 包含常用的头文件
 * @version 0.1
 * @date 2024-05-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "system/Scheduler.h"
#include "command/ConditionalCommand.h"
#include "command/SelectCommand.h"
#include "command/TimerCommand.h"
#include "command/group/ParallelCommandGroup.h"
#include "command/group/ParallelDeadlineGroup.h"
#include "command/group/ParallelRaceGroup.h"
#include "command/group/SequentialCommandGroup.h"
#include "util/PID.h"
#include "hal/Titan.h"
#include "hal/vmx.h"
#include "system//Ultrasound.h"
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>
#include <glog/logging.h>
#define MAKE_SHARED(Type, ...) std::make_shared<Type>(__VA_ARGS__)

/**
 * @file LidarCalibrate.h
 * @author Zijian.Yan
 * @brief
 * @version 0.1
 * @date 2024-10-30
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "RobotCfg.h"
#include "params.h"

using namespace std;

class MovingAverageFilter {
  public:
    typedef std::shared_ptr<MovingAverageFilter> Ptr;
    // 构造函数，设置窗口大小
    MovingAverageFilter(int size) : windowSize(size), sum(0.0), count(0) {
        // 确保最大窗口大小足够
        if (windowSize > maxBufferSize) {
            windowSize = maxBufferSize;
        }
        buffer.fill(0.0);
    }

    double filter(double newValue) {
        // 更新环形缓冲区
        sum -= buffer[count];             // 减去旧值
        sum += newValue;                  // 加上新值
        buffer[count] = newValue;         // 更新缓冲区
        count = (count + 1) % windowSize; // 更新索引

        // 返回当前窗口的平均值
        // return sum / std::min(count + 1, windowSize);
        return sum / windowSize;
    }

  private:
    static const int maxBufferSize = 10;      // 最大窗口大小
    int windowSize;                           // 实际窗口大小
    double sum;                               // 当前窗口的数据总和
    int count;                                // 当前缓冲区索引
    std::array<double, maxBufferSize> buffer; // 用于存储数据的缓冲区
};
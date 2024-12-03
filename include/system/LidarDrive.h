/**
 * @file LidarDrive.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 测试激光雷达数据读取
 * @version 0.1
 * @date 2024-05-01
 *
 * @copyright Copyright (c) 2024
 *
 */
// #pragma once

#ifndef LIDAR_DRIVE
#define LIDAR_DRIVE

#include <algorithm>
#include <cassert>
#include <cctype>
#include <iostream>
#include <memory>
#include <src/CYdLidar.h>
#include <string>
#include <unistd.h>
#include <vector>

namespace robot_sensor {
class LiDAR {
  public:
    typedef std::shared_ptr<LiDAR> ptr;
    LiDAR(std::string port);
    LiDAR();
    ~LiDAR();

    std::vector<LaserPoint> read();
    std::vector<LaserPoint> get() { return m_scan_.points; }
    bool isOk();

  private:
    void init();

  private:
    CYdLidar m_laser_;
    std::string m_port_;
    int m_baudrate_;
    LaserScan m_scan_;
    float frequency = 8.0;
};

} //  namespace robot_sensor

#endif //
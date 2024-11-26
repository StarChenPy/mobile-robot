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

#include <iostream>
#include <unistd.h>
#include <src/CYdLidar.h>
#include <memory>
#include <string>
#include <algorithm>
#include <cctype>
#include <cassert>
#include <vector>

using namespace std;
using namespace ydlidar;
namespace Sensor {

class LiDAR {
 public:
  typedef std::shared_ptr<LiDAR> Ptr;
  LiDAR(std::string port);
  LiDAR();
  ~LiDAR();
  
  std::vector<LaserPoint> read();
  std::vector<LaserPoint> get() {return m_scan_.points;}
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



}  //  namespace Sensor

#endif  //
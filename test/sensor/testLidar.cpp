#include "robotgenius/sensor/lidar.h"
// #include "system/LidarDrive.h"
// /home/pi/Pick/
using namespace Sensor;


int main() {
  LiDAR lidar_sensor;
  std::vector<LaserPoint> datas = lidar_sensor.read();
  for (auto data : datas) {
    std::cout << "angle: " << data.angle << " distance: " << data.range << std::endl;
  }
  
  return 0;
}
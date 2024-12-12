/**
 * @file LidarRead.h
 * @author Zijian.Yan
 * @brief
 * @version 0.1
 * @date 2024-08-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "system/LidarDrive.h"
#include "system/SysParams.h"
#include "system/RobotCfg.h"
#include "util/params.h"
#include <mutex>

using namespace std;

class LidarRead {
  public:
    typedef std::shared_ptr<LidarRead> Ptr;
    LidarRead(const double init_angle) : InitAngle(init_angle) {}
    ~LidarRead() {}

  private:
    std::vector<LidarData> readLidarData() {
        std::vector<LidarData> lidardata_buf;
        InitAngle = LidarParams.InitAngle;
        double init_angle_2 = InitAngle * (M_PI / 180);
        // lidar_data.clear();
        for (auto it : lidar->read()) {
            LidarData data;
            if (it.range < lidar_range_max && it.range > lidar_range_min) //激光雷达扫描距离少于一定值
            {
                if (it.angle < lidar_angle_max - init_angle_2 && it.angle > lidar_angle_min - init_angle_2) //角度
                {
                    // std::cout << "intensity = " << it.intensity << endl;
                    if (it.intensity == 1008) { //激光雷达数据质量
                        data.Range = it.range;
                        data.Angle = it.angle + init_angle_2;
                        data.Intensity = it.intensity;
                        lidardata_buf.push_back(data);
                        // cout << "Range = " << data.Range << " Angle = " <<
                        // data.Angle << " lidar_data size = " <<
                        // lidardata_buf.size() << endl;
                    }
                }
            }
        }
        return lidardata_buf;
    }

    void saveLidarData(std::vector<LidarData> &output, const std::vector<LidarData> &input) {
        output.clear();
        for (int i = 0; i < input.size(); i++) {
            output.push_back(input[i]);
        }
    }

  public:
    void Task() {
        std::vector<LidarData> temp = readLidarData();
        I.lock();
        saveLidarData(lidar_data, temp);
        I.unlock();
    }

    void getLidarData(std::vector<LidarData> &output) {
        output.clear();
        for (int i = 0; i < lidar_data.size(); i++) {
            output.push_back(lidar_data[i]);
        }
    }

  private:
    double InitAngle = 0;
    std::mutex I;
    std::vector<LidarData> lidar_data;

    const double lidar_angle_max = M_PI_2; //雷达扫描范围
    const double lidar_angle_min = -M_PI_2;
    const double lidar_range_max = 4.0; //单位m
    const double lidar_range_min = 0;
};

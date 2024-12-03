#include "system/LidarDrive.h"

namespace robot_sensor {

LiDAR::LiDAR(std::string port) : m_port_(port) {
    ydlidar::os_init();
    m_baudrate_ = 115200;
    assert(ydlidar::os_isOk());
    init();
}

LiDAR::LiDAR() {
    ydlidar::os_init();
    m_baudrate_ = 115200;
    assert(ydlidar::os_isOk());
    std::map<std::string, std::string> ports = ydlidar::lidarPortList();
    std::map<std::string, std::string>::iterator it;

    if (ports.size() == 1) {
        m_port_ = ports.begin()->second;
    } else {
        int id = 0;

        for (it = ports.begin(); it != ports.end(); it++) {
            printf("%d. %s\n", id, it->first.c_str());
            id++;
        }

        if (ports.empty()) {
            printf("Not lidar was detected. Please enter the lidar serial port.");
        } else {
            printf("Please enter the lidar port.");
        }
        assert(false);
    }
    init();
}

LiDAR::~LiDAR() {
    m_laser_.turnOff();
    m_laser_.disconnecting();
}

void LiDAR::init() {
    m_laser_.setlidaropt(LidarPropSerialPort, m_port_.c_str(), m_port_.size());
    m_laser_.setlidaropt(LidarPropSerialBaudrate, &m_baudrate_, sizeof(int));
    int optval = TYPE_TRIANGLE;
    m_laser_.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
    /// device type
    optval = YDLIDAR_TYPE_SERIAL;
    m_laser_.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
    /// sample rate
    optval = 3;
    m_laser_.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
    optval = 4;
    m_laser_.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
    /// fixed angle resolution
    bool b_optvalue = false;
    m_laser_.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
    /// rotate 180
    m_laser_.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
    /// Counterclockwise
    m_laser_.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
    b_optvalue = true;
    m_laser_.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
    /// one-way communication
    bool isSingleChannel = true;
    m_laser_.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
    /// intensity
    b_optvalue = false;
    m_laser_.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
    /// Motor DTR
    b_optvalue = true;
    m_laser_.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
    /// HeartBeat
    b_optvalue = false;
    m_laser_.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

    //////////////////////float property/////////////////
    /// unit: °
    float f_optvalue = 180.0f;
    m_laser_.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
    f_optvalue = -180.0f;
    m_laser_.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
    /// unit: m
    f_optvalue = 64.f;
    m_laser_.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
    f_optvalue = 0.05f;
    m_laser_.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
    /// unit: Hz
    m_laser_.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));
    bool ret = m_laser_.initialize();

    if (ret) {
        ret = m_laser_.turnOn();
    } else {
        fprintf(stderr, "%s\n", m_laser_.DescribeError());
        fflush(stderr);
    }
}

std::vector<LaserPoint> LiDAR::read() {
    if (ydlidar::os_isOk()) {
        if (m_laser_.doProcessSimple(m_scan_)) {
            return m_scan_.points;
        } else {
            fprintf(stderr, "Failed to get lidar Data\n");
            fflush(stderr);
        }
    }
    return std::vector<LaserPoint>();
}

bool LiDAR::isOk() { return ydlidar::os_isOk(); }

} //  namespace robot_sensor
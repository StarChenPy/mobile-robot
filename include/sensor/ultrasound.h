/**
 * @file ultrasound.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 超声波模块
 * @version 0.1
 * @date 2024-05-24
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "hal/vmx.h"
using namespace VMX;
namespace Sensor {

class Ultrasound {
  public:
    typedef std::shared_ptr<Ultrasound> Ptr;
    Ultrasound(int trig, int echo);
    Ultrasound(DO::ptr trig, Timer::ptr echo);
    ~Ultrasound() {}

    uint32_t echo();
    void trig(int32_t time = 100);
    uint32_t get() { return m_data_; }

  private:
    uint32_t m_data_;
    DO::ptr m_trig_;
    Timer::ptr m_echo_;
};

} //  namespace Sensor
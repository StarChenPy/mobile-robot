/**
 * @file ultrasound.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 超声波模块
 * @version 0.1
 * @date 2024-05-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "sensor/ultrasound.h"
using namespace VMX;
namespace Sensor {

Ultrasound::Ultrasound(int trig, int echo) {
    m_trig_ = std::make_shared<DO>(trig);
    m_echo_ = std::make_shared<Timer>(echo);
}
Ultrasound::Ultrasound(DO::ptr trig, Timer::ptr echo) {
    m_trig_ = trig;
    m_echo_ = echo;
}
uint32_t Ultrasound::echo() {
    m_data_ = m_echo_->read();
    return m_data_;
}
void Ultrasound::trig(int32_t time) { m_trig_->setPulse(time); }

} //  namespace Sensor
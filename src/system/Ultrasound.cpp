#include "system/Ultrasound.h"

namespace robot_sensor {

Ultrasound::Ultrasound(int trig, int echo) {
  m_trig_ = std::make_shared<VMX::DO>(trig);
  m_echo_ = std::make_shared<VMX::Timer>(echo);
}
Ultrasound::Ultrasound(VMX::DO::ptr trig, VMX::Timer::ptr echo) {
  m_trig_ = trig;
  m_echo_ = echo;
}
uint32_t Ultrasound::echo() {
  m_data_ = m_echo_->read();
  return m_data_;
}
void Ultrasound::trig(int32_t time) {
  m_trig_->setPulse(time);
}

}  //  namespace robot_sensor
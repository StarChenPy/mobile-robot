#pragma once
#include "hal/vmx.h"

namespace robot_sensor {
    class Ultrasound {
    public:
        typedef std::shared_ptr<Ultrasound> ptr;
        Ultrasound(int trig, int echo);
        Ultrasound(VMX::DO::ptr trig, VMX::Timer::ptr echo);
        ~Ultrasound() = default;

        uint32_t echo();
        void trig(int32_t time = 100);
        uint32_t get() {return m_data_;}
    private:
        uint32_t m_data_;
        VMX::DO::ptr m_trig_;
        VMX::Timer::ptr m_echo_;
    };
}  //  namespace robot_sensor
/**
 * @file PID.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief PID控制算法
 * @version 0.1
 * @date 2024-05-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "control/PID.h"
#include <iostream>
namespace robot {

PID::PID(double dt_s) : dt_s_(dt_s) {
    assert(dt_s > 0);
    output_ = 0;
    max_output_ = 0;
    min_output_ = 0;
    set_point_ = 0;
    process_ = 0;
    kp_ = 0;
    ki_ = 0;
    kd_ = 0;
    kp_last_ = 0;
    ki_last_ = 0;
    kd_last_ = 0;
    integral_err_ = 0;
    error_ = 0;
    last_process_ = 0;
    diff_ = 0;
    first_ = true;
}
void PID::set_gains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
void PID::set_output_limits(double min_output, double max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
}
void PID::set_point(double setpoint) { set_point_ = setpoint; }
void PID::set_process(double process) { process_ = process; }

void PID::calculate() {
    error_ = set_point_ - process_;

    if (!is_gains_changed() && !first_) {
        output_ = last_output_;
        kp_last_ = kp_;
        ki_last_ = ki_;
        kd_last_ = kd_;
        last_output_ = output_;
        first_ = false;
        return;
    }
    diff_calculate();
    integral_calculate();

    output_ = kp_ * error_ + integral_output_ + diff_output_;
    output_ = output_ > max_output_ ? max_output_ : output_;
    output_ = output_ < min_output_ ? min_output_ : output_;

    last_process_ = process_;
    kp_last_ = kp_;
    ki_last_ = ki_;
    kd_last_ = kd_;
    last_output_ = output_;
    first_ = false;
}

void PID::diff_calculate() {
    if (first_) {
        diff_output_ = 0;
        last_process_ = process_;
        return;
    }
    diff_output_ = (last_process_ - process_) * kd_ * kp_ / dt_s_;
}
double PID::get_p() const { return kp_ * error_; }
double PID::get_i() const { return integral_output_; }
double PID::get_d() const { return diff_output_; }
int PID::get_output_int() const { return static_cast<int>(output_); }
/**
 * @brief 通过最大输出和最小输出限制积分项
 *
 */
void PID::integral_calculate() {
    if (ki_ == 0) {
        integral_output_ = 0;
        return;
    }
    integral_output_ = (integral_err_ + error_) * kp_ * dt_s_ / ki_;

    if (integral_output_ > max_output_) {
        integral_output_ = max_output_;
    } else if (integral_output_ < min_output_) {
        integral_output_ = min_output_;
    } else {
        integral_err_ += error_;
    }
}

bool PID::is_gains_changed() const { return kp_ == kp_last_ && ki_ == ki_last_ && kd_ == kd_last_; }

void PID::reset() {
    process_ = 0;
    set_point_ = 0;
    output_ = 0;
    error_ = 0;
    integral_err_ = 0;
    diff_output_ = 0;
    integral_output_ = 0;
    last_process_ = 0;
    diff_ = 0;
    first_ = true;
}

PidFilter::PidFilter(uint32_t max_size) : m_max_size_(max_size) {
    assert(max_size > 0);
    m_data_array_ = new double[m_max_size_];
    m_data_index_ = 0;
}

double PidFilter::filter(double input) {
    double sum = 0;
    if (m_data_index_ < m_max_size_) {
        m_data_array_[m_data_index_] = input;
        for (int i = 0; i <= m_data_index_; i++) {
            sum += m_data_array_[i];
        }
        m_data_index_++;
        return sum / (m_data_index_);
    }

    for (int i = 0; i < m_max_size_ - 1; i++) {
        m_data_array_[i] = m_data_array_[i + 1];
    }

    m_data_array_[m_max_size_ - 1] = input;
    for (int i = 0; i < m_max_size_; i++) {
        sum += m_data_array_[i];
    }

    return sum / m_max_size_;
}
} //  namespace robot
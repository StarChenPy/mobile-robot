/**
 * @file PID.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief PID控制算法
 * @version 0.1
 * @date 2024-05-01
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <cassert>
#include <memory>
#include <queue>

namespace robot {
class PID {
  public:
    typedef std::shared_ptr<PID> ptr;

    PID(double dt_s);
    ~PID() {}

    void set_gains(double kp, double ki, double kd);
    void set_output_limits(double min_output, double max_output);
    void set_point(double setpoint);
    void set_process(double process);
    void calculate();
    int get_output_int() const;
    void reset();
    double get_p() const;
    double get_i() const;
    double get_d() const;

  private:
    void diff_calculate();
    void integral_calculate();
    bool is_gains_changed() const;

  public:
    double output_;
    double max_output_;
    double min_output_;
    double set_point_;
    double process_;

  private:
    double dt_s_;
    double kp_;
    double ki_;
    double kd_;
    double kp_last_;
    double ki_last_;
    double kd_last_;
    double integral_err_;
    double integral_output_;
    double diff_output_;
    double error_;
    double last_process_;
    double diff_;
    bool first_ = true;
    double last_output_;
};

class PidFilter {
  public:
    typedef std::shared_ptr<PidFilter> Ptr;
    PidFilter(uint32_t max_size = 5);
    ~PidFilter() {}
    double filter(double input);

  private:
    uint32_t m_max_size_;
    double *m_data_array_;
    int32_t m_data_index_;
};

} //  namespace robot
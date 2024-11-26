/**
 * @file IRCalCommand.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-08-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "RobotCfg.h"
#include "RobotGenius.h"
#include "params.h"
#include "system/Robot.h"
using namespace std;
using namespace robot;

class IRCalCommand : public CommandBase {
  public:
    typedef std::shared_ptr<IRCalCommand> Ptr;
    IRCalCommand(double distance, double angle_error = 1, double disatance_error = 1, uint32_t counter = 3,
                 double left_right_e = 2, double offset = 0)
        : m_distance(distance), m_offset(offset), m_angle_error(angle_error), m_disatance_error(disatance_error),
          m_counter(counter), m_left_right_e(left_right_e) {}
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;
    double m_distance;
    double m_offset;
    double m_angle_error;
    double m_disatance_error;
    double m_left_right_e;
    uint32_t m_counter;
    uint32_t m_current_counter = 0;
};

Command::ptr IRCalCommandAssistance(double distance, double angle_error = 0.5, double disatance_error = 0.5,
                                    uint32_t counter = 3, double left_right_e = 1, double offset = 0);

class SingleIRCalCommand : public CommandBase {
  public:
    typedef std::shared_ptr<SingleIRCalCommand> Ptr;
    SingleIRCalCommand(double distance, double angle_error = 1, double disatance_error = 1, uint32_t counter = 3)
        : m_distance(distance), m_angle_error(angle_error), m_disatance_error(disatance_error), m_counter(counter) {}
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;
    double m_distance;
    // double m_offset;
    double m_holdphi;
    double m_angle_error;
    double m_disatance_error;
    uint32_t m_counter;
    uint32_t m_current_counter = 0;
};

Command::ptr SingleIRCalCommandAssistance(double distance);
Command::ptr SingleIRCalCommandAssistance(double distance, double angle_error = 0.5, double disatance_error = 0.5,
                                          uint32_t counter = 3);

class USCalCommand : public CommandBase {
  public:
    typedef std::shared_ptr<USCalCommand> Ptr;
    USCalCommand(double distance, double angle_error = 1, double disatance_error = 1, uint32_t counter = 3,
                 double left_right_e = 2, double offset = 0)
        : m_distance(distance), m_offset(offset), m_angle_error(angle_error), m_disatance_error(disatance_error),
          m_counter(counter), m_left_right_e(left_right_e) {}
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    bool is_finished = false;
    int64_t m_last_time;
    double m_distance;
    double m_offset;
    double m_angle_error;
    double m_disatance_error;
    double m_left_right_e;
    uint32_t m_counter;
    uint32_t m_current_counter = 0;
};

Command::ptr USCalCommandAssistance(double distance, double angle_error = 0.5, double disatance_error = 0.5,
                                    uint32_t counter = 3, double left_right_e = 2, double offset = 0);
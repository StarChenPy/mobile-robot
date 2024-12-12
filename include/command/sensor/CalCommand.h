#pragma once
#include "system/Robot.h"
#include "system/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

class IRCalCommand : public ICommand {
  public:
    typedef std::shared_ptr<IRCalCommand> ptr;
    IRCalCommand(double distance, double angle_error = 1, double distance_error = 1, uint32_t counter = 3,
                 double left_right_e = 2, double offset = 0)
        : m_distance(distance), m_offset(offset), m_angle_error(angle_error), m_distance_error(distance_error),
          m_counter(counter), m_left_right_e(left_right_e) {}
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    double m_distance;
    double m_offset;
    double m_angle_error;
    double m_distance_error;
    double m_left_right_e;
    uint32_t m_counter;
    uint32_t m_current_counter = 0;
};

ICommand::ptr IRCalCommandAssistance(double distance, double angle_error = 0.5, double distance_error = 0.5,
                                     uint32_t counter = 3, double left_right_e = 1, double offset = 0);

class SingleIRCalCommand : public ICommand {
  public:
    typedef std::shared_ptr<SingleIRCalCommand> ptr;
    SingleIRCalCommand(double distance, double angle_error = 1, double distance_error = 1, uint32_t counter = 3)
        : m_distance(distance), m_angle_error(angle_error), m_distance_error(distance_error), m_counter(counter) {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    double m_distance;
    double m_holdphi;
    double m_angle_error;
    double m_distance_error;
    uint32_t m_counter;
    uint32_t m_current_counter = 0;
};

ICommand::ptr SingleIRCalCommandAssistance(double distance);
ICommand::ptr SingleIRCalCommandAssistance(double distance, double angle_error = 0.5, double distance_error = 0.5,
                                           uint32_t counter = 3);

class USCalCommand : public ICommand {
  public:
    typedef std::shared_ptr<USCalCommand> ptr;

    USCalCommand(double distance, double angle_error = 1, double distance_error = 1, uint32_t counter = 3,
                 double left_right_e = 2, double offset = 0)
        : m_distance(distance), m_offset(offset), m_angle_error(angle_error), m_distance_error(distance_error),
          m_counter(counter), m_left_right_e(left_right_e) {}

    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;

  private:
    double m_distance;
    double m_offset;
    double m_angle_error;
    double m_distance_error;
    double m_left_right_e;
    uint32_t m_counter;
    uint32_t m_current_counter = 0;
};

ICommand::ptr USCalCommandAssistance(double distance, double angle_error = 0.5, double distance_error = 0.5,
                                     uint32_t counter = 3, double left_right_e = 2, double offset = 0);
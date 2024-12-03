#include "command/sensor/CalCommand.h"
#include "command/sensor/ENCComand.h"

void IRCalCommand::initialize() {
    isFinished_ = false;
    m_current_counter = 0;
    irLeft->read();
    irRight->read();
}
void IRCalCommand::execute() {
    double error =
            Robot::getInstance().calibrationIR(m_distance, m_angle_error, m_distance_error, m_left_right_e, m_offset);
    if (!error) {
        m_current_counter = 0;
    } else {
        m_current_counter++;
    }
}
void IRCalCommand::end() {
    Robot::getInstance().setLeftMotorSpeed(0);
    Robot::getInstance().setRightMotorSpeed(0);
    std::cout << "IRCalCommand end!" << std::endl;
}
bool IRCalCommand::isFinished() {
    isFinished_ = m_current_counter > m_counter;
    return isFinished_ || Robot::getStopSignal();
}

ICommand::ptr IRCalCommandAssistance(double distance) {
    IRCalCommand::ptr command =
            std::make_shared<IRCalCommand>(distance, IRCalibE.Angle, IRCalibE.Dis, IRCalibE.CNT, IRCalibE.LeftRightE, 0);
    return command->withTimer(100);
}
ICommand::ptr IRCalCommandAssistance(double distance, double angle_error, double distance_error, uint32_t counter,
                                     double left_right_e, double offset) {
    IRCalCommand::ptr command =
            std::make_shared<IRCalCommand>(distance, angle_error, distance_error, counter, left_right_e, offset);
    return command->withTimer(100);
}

void SingleIRCalCommand::initialize() {
    isFinished_ = false;
    m_current_counter = 0;
    irLeft->read();
    irRight->read();
    m_holdphi = Robot::getInstance().odom->getPose().theta_;
}
void SingleIRCalCommand::execute() {
    double error = Robot::getInstance().calibrationSingleIR(m_distance, m_holdphi, m_angle_error, m_distance_error);
    if (!error) {
        m_current_counter = 0;
    } else {
        m_current_counter++;
    }
}
void SingleIRCalCommand::end() {
    Robot::getInstance().setLeftMotorSpeed(0);
    Robot::getInstance().setRightMotorSpeed(0);
    std::cout << "SingleIRCalCommand end!" << std::endl;
}
bool SingleIRCalCommand::isFinished() {
    isFinished_ = m_current_counter > m_counter;
    return isFinished_ || Robot::getStopSignal();
}

ICommand::ptr SingleIRCalCommandAssistance(double distance) {
    SingleIRCalCommand::ptr command =
            std::make_shared<SingleIRCalCommand>(distance, IRCalibE.Angle, IRCalibE.Dis, IRCalibE.CNT);
    return command->withTimer(100);
}
ICommand::ptr SingleIRCalCommandAssistance(double distance, double angle_error, double distance_error,
                                           uint32_t counter) {
    SingleIRCalCommand::ptr command =
            std::make_shared<SingleIRCalCommand>(distance, angle_error, distance_error, counter);
    return command->withTimer(100);
}

void USCalCommand::initialize() {
    isFinished_ = false;
    m_current_counter = 0;
    irLeft->read();
    irRight->read();
}
void USCalCommand::execute() {
    double error =
            Robot::getInstance().calibrationUS(m_distance, m_angle_error, m_distance_error, m_left_right_e, m_offset);
    if (!error) {
        m_current_counter = 0;
    } else {
        m_current_counter++;
    }
}
void USCalCommand::end() {
    Robot::getInstance().setLeftMotorSpeed(0);
    Robot::getInstance().setRightMotorSpeed(0);
    std::cout << "USCalCommand end!" << std::endl;
}
bool USCalCommand::isFinished() {
    isFinished_ = m_current_counter > m_counter;
    return isFinished_ || Robot::getStopSignal();
}

ICommand::ptr USCalCommandAssistance(double distance) {
    USCalCommand::ptr command =
            std::make_shared<USCalCommand>(distance, USCalibE.Angle, USCalibE.Dis, USCalibE.CNT, USCalibE.LeftRightE, 0);
    return command->withTimer(100);
}
ICommand::ptr USCalCommandAssistance(double distance, double angle_error, double distance_error, uint32_t counter,
                                     double left_right_e, double offset) {
    USCalCommand::ptr command =
            std::make_shared<USCalCommand>(distance, angle_error, distance_error, counter, left_right_e, offset);
    return command->withTimer(100);
}
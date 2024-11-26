#include "command/CalCommand.h"
#include "command/ENCComand.h"
#include "command/MotorPIDCommand.h"
#include "params.h"

void IRCalCommand::initialize() {
    uint8_t updata_status = COMMEND_WAIT;
    LABVIEW::IRCalibCtrlStatusShareAddress->write(updata_status);

    is_finished = false;
    m_current_counter = 0;
    IR_left->read();
    IR_right->read();
}
void IRCalCommand::execute() {
    double error =
        Robot::GetInstance().calibrationIR(m_distance, m_angle_error, m_disatance_error, m_left_right_e, m_offset);
    if (!error) {
        m_current_counter = 0;
    } else {
        m_current_counter++;
    }
}
void IRCalCommand::end() {
    Robot::GetInstance().setLeftMotorSpeed(0);
    Robot::GetInstance().setRightMotorSpeed(0);
    std::cout << "IRCalCommand end!" << std::endl;

    uint8_t updata_status = COMMEND_END;
    LABVIEW::IRCalibCtrlStatusShareAddress->write(updata_status);
}
bool IRCalCommand::isFinished() {
    uint8_t command_status;
    LABVIEW::IRCalibCtrlStatusShareAddress->read(command_status);
    if (command_status == COMMEND_CANCEL) {
        is_finished = true;
        return is_finished;
    }
    is_finished = m_current_counter > m_counter;
    return is_finished || Robot::GetInstance().getStopSignal();
}

Command::ptr IRCalCommandAssistance(double distance) {
    IRCalCommand::Ptr command =
        std::make_shared<IRCalCommand>(distance, IRCalibE.Angle, IRCalibE.Dis, IRCalibE.CNT, IRCalibE.LeftRightE, 0);
    return command->withTimer(100);
}
Command::ptr IRCalCommandAssistance(double distance, double angle_error, double disatance_error, uint32_t counter,
                                    double left_right_e, double offset) {
    IRCalCommand::Ptr command =
        std::make_shared<IRCalCommand>(distance, angle_error, disatance_error, counter, left_right_e, offset);
    return command->withTimer(100);
}

void SingleIRCalCommand::initialize() {
    uint8_t updata_status = COMMEND_WAIT;
    LABVIEW::SingleIRCalibCtrlStatusShareAddress->write(updata_status);

    is_finished = false;
    m_current_counter = 0;
    IR_left->read();
    IR_right->read();
    m_holdphi = Robot::GetInstance().odom->getPose().theta_;
}
void SingleIRCalCommand::execute() {
    double error = Robot::GetInstance().calibrationSingleIR(m_distance, m_holdphi, m_angle_error, m_disatance_error);
    if (!error) {
        m_current_counter = 0;
    } else {
        m_current_counter++;
    }
}
void SingleIRCalCommand::end() {
    Robot::GetInstance().setLeftMotorSpeed(0);
    Robot::GetInstance().setRightMotorSpeed(0);
    std::cout << "SingleIRCalCommand end!" << std::endl;

    uint8_t updata_status = COMMEND_END;
    LABVIEW::SingleIRCalibCtrlStatusShareAddress->write(updata_status);
}
bool SingleIRCalCommand::isFinished() {
    uint8_t command_status;
    LABVIEW::SingleIRCalibCtrlStatusShareAddress->read(command_status);
    if (command_status == COMMEND_CANCEL) {
        is_finished = true;
        return is_finished;
    }
    is_finished = m_current_counter > m_counter;
    return is_finished || Robot::GetInstance().getStopSignal();
}

Command::ptr SingleIRCalCommandAssistance(double distance) {
    SingleIRCalCommand::Ptr command =
        std::make_shared<SingleIRCalCommand>(distance, IRCalibE.Angle, IRCalibE.Dis, IRCalibE.CNT);
    return command->withTimer(100);
}
Command::ptr SingleIRCalCommandAssistance(double distance, double angle_error, double disatance_error,
                                          uint32_t counter) {
    SingleIRCalCommand::Ptr command =
        std::make_shared<SingleIRCalCommand>(distance, angle_error, disatance_error, counter);
    return command->withTimer(100);
}

void USCalCommand::initialize() {
    uint8_t updata_status = COMMEND_WAIT;
    LABVIEW::USCalibCtrlStatusShareAddress->write(updata_status);

    is_finished = false;
    m_current_counter = 0;
    IR_left->read();
    IR_right->read();
}
void USCalCommand::execute() {
    double error =
        Robot::GetInstance().calibrationUS(m_distance, m_angle_error, m_disatance_error, m_left_right_e, m_offset);
    if (!error) {
        m_current_counter = 0;
    } else {
        m_current_counter++;
    }
}
void USCalCommand::end() {
    Robot::GetInstance().setLeftMotorSpeed(0);
    Robot::GetInstance().setRightMotorSpeed(0);
    std::cout << "USCalCommand end!" << std::endl;

    uint8_t updata_status = COMMEND_END;
    LABVIEW::USCalibCtrlStatusShareAddress->write(updata_status);
}
bool USCalCommand::isFinished() {
    uint8_t command_status;
    LABVIEW::USCalibCtrlStatusShareAddress->read(command_status);
    if (command_status == COMMEND_CANCEL) {
        is_finished = true;
        return is_finished;
    }

    is_finished = m_current_counter > m_counter;
    return is_finished || Robot::GetInstance().getStopSignal();
}

Command::ptr USCalCommandAssistance(double distance) {
    USCalCommand::Ptr command =
        std::make_shared<USCalCommand>(distance, USCalibE.Angle, USCalibE.Dis, USCalibE.CNT, USCalibE.LeftRightE, 0);
    return command->withTimer(100);
}
Command::ptr USCalCommandAssistance(double distance, double angle_error, double disatance_error, uint32_t counter,
                                    double left_right_e, double offset) {
    USCalCommand::Ptr command =
        std::make_shared<USCalCommand>(distance, angle_error, disatance_error, counter, left_right_e, offset);
    return command->withTimer(100);
}
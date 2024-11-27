#include "command/ZeroOdomCommand.h"

void ZeroOdomCommand::initialize() {
    uint8_t updata_status = COMMEND_WAIT;
    LABVIEW::ZeroOdomStatusShareAddress->write(updata_status);

    is_finished = false;
    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);
}
void ZeroOdomCommand::execute() {
    int time = robot::getCurrentMs();
    int dt = time - m_last_time;
    m_last_time = time;

    Robot::GetInstance().odom->zeroPose();

    // 打印
    Robot::GetInstance().odom->print();

    // sleep(3);
    is_finished = true;

    // std::cout << "ZeroOdomCommand execute dt = " << static_cast<double>(dt)
    // << std::endl; isFinished_ = true;
}
void ZeroOdomCommand::end() {
    uint8_t updata_status = COMMEND_END;
    LABVIEW::ZeroOdomStatusShareAddress->write(updata_status);

    std::cout << "ZeroOdomCommand end!" << std::endl;
}
bool ZeroOdomCommand::isFinished() {
    if (Robot::GetInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::GetInstance().getStopSignal();
}
Command::ptr createZeroOdomCommand() { return std::make_shared<ZeroOdomCommand>()->withTimer(100); }

void SetOdomCommand::initialize() {
    uint8_t updata_status = COMMEND_WAIT;
    LABVIEW::SetOdomStatusShareAddress->write(updata_status);

    is_finished = false;
    Robot::GetInstance().setRightMotorSpeed(0);
    Robot::GetInstance().setLeftMotorSpeed(0);
}
void SetOdomCommand::execute() {
    int time = robot::getCurrentMs();
    int dt = time - m_last_time;
    m_last_time = time;

    Robot::GetInstance().odom->setPose(set_x, set_y, set_theta);
    // 打印
    Robot::GetInstance().odom->print();

    // sleep(3);
    is_finished = true;

    // std::cout << "SetOdomCommand execute dt = " << static_cast<double>(dt) <<
    // std::endl; isFinished_ = true;
}
void SetOdomCommand::end() {
    uint8_t updata_status = COMMEND_END;
    LABVIEW::SetOdomStatusShareAddress->write(updata_status);

    std::cout << "SetOdomCommand end!" << std::endl;
}
bool SetOdomCommand::isFinished() {
    if (Robot::GetInstance().getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::GetInstance().getStopSignal();
}
Command::ptr createSetOdomCommand(double x, double y, double theta) {
    return std::make_shared<SetOdomCommand>(x, y, theta)->withTimer(100);
}

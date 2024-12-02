#include "command/ServoCommand.h"

//**********************************夹手*****************************************
void ClampServoCommand::initialize() {
    is_finished = false;
}
void ClampServoCommand::execute() {
    int time = robot::getCurrentMs();
    m_last_time = time;

    clampServo->setDutyCycle(ClampServo_val);
    counter++;
    if (counter > counter_limit) {
        is_finished = true;
        counter = 0;
    }
}
void ClampServoCommand::end() {
    std::cout << "ClampServoCommand end" << std::endl;
}
bool ClampServoCommand::isFinished() {
    if (Robot::getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getStopSignal();
}

// 指令封装  len:打开宽度，单位cm
Command::ptr createClampServoCommand(double len) {
    double target = CLAMP_LEN_A * len + CLAMP_LEN_B;
    return std::make_shared<ClampServoCommand>(target)->withTimer(100);
}
Command::ptr createClampServoCommand(double len, double cnt_limit) {
    double target = CLAMP_LEN_A * len + CLAMP_LEN_B;
    return std::make_shared<ClampServoCommand>(target, cnt_limit)->withTimer(100);
}

//**********************************伸缩*****************************************
void TelescopicServoCommand::initialize() {
    is_finished = false;
}
void TelescopicServoCommand::execute() {
    int time = robot::getCurrentMs();
    m_last_time = time;

    telescopicServo->setDutyCycle(TelescopicServo_val);
    counter++;
    if (counter > counter_limit) {
        is_finished = true;
        counter = 0;
    }
}
void TelescopicServoCommand::end() {
    std::cout << "TelescopicServoCommand end" << std::endl;
}
bool TelescopicServoCommand::isFinished() {
    if (Robot::getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getStopSignal();
}

// 指令封装  dis:伸缩距离，单位cm
Command::ptr createTelescopicServoCommand(double dis) {
    double target = TELESCOPIC_DIS_A * dis + TELESCOPIC_DIS_B;
    return std::make_shared<TelescopicServoCommand>(target)->withTimer(100);
}
Command::ptr createTelescopicServoCommand(double dis, double cnt_limit) {
    double target = TELESCOPIC_DIS_A * dis + TELESCOPIC_DIS_B;
    return std::make_shared<TelescopicServoCommand>(target, cnt_limit)->withTimer(100);
}

//**********************************摆手*****************************************
void RaiseServoCommand::initialize() {
    is_finished = false;
}
void RaiseServoCommand::execute() {
    int time = robot::getCurrentMs();
    m_last_time = time;

    raiseServo->setDutyCycle(RaiseServo_val);
    counter++;
    if (counter > counter_limit) {
        is_finished = true;
        counter = 0;
    }
}
void RaiseServoCommand::end() {
    std::cout << "RaiseServoCommand end" << std::endl;
}
bool RaiseServoCommand::isFinished() {
    if (Robot::getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getStopSignal();
}

// 指令封装  angle:抬起角度，单位：度
Command::ptr createRaiseServoCommand(double angle) {
    double target = RAISE_ANGLE_A * angle + RAISE_ANGLE_B;
    std::cout << "Raise target = " << target << std::endl;
    return std::make_shared<RaiseServoCommand>(target)->withTimer(100);
}
Command::ptr createRaiseServoCommand(double angle, double cnt_limit) {
    double target = RAISE_ANGLE_A * angle + RAISE_ANGLE_B;
    return std::make_shared<RaiseServoCommand>(target, cnt_limit)->withTimer(100);
}

//**********************************旋转*****************************************
void RotatingServoCommand::initialize() {
    is_finished = false;
}
void RotatingServoCommand::execute() {
    int time = robot::getCurrentMs();
    m_last_time = time;

    rotatingServo->setDutyCycle(RotatingServo_val);
    counter++;
    if (counter > counter_limit) {
        is_finished = true;
        counter = 0;
    }
}
void RotatingServoCommand::end() {
    std::cout << "RotatingServoCommand end" << std::endl;
}
bool RotatingServoCommand::isFinished() {
    if (Robot::getStopSignal()) {
        stopAll();
    }
    return is_finished || Robot::getStopSignal();
}

// 指令封装  angle:角度，单位：度
Command::ptr createRotatingServoCommand(double angle) {
    double target = ROTATING_ANGLE_A * angle + ROTATING_ANGLE_B;
    std::cout << "Rotating target = " << target << std::endl;
    return std::make_shared<RotatingServoCommand>(target)->withTimer(100);
}
Command::ptr createRotatingServoCommand(double angle, double cnt_limit) {
    double target = ROTATING_ANGLE_A * angle + ROTATING_ANGLE_B;
    return std::make_shared<RotatingServoCommand>(target, cnt_limit)->withTimer(100);
}
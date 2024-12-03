#include "command/ServoCommand.h"

//**********************************夹手*****************************************
void ClampServoCommand::execute() {
    int time = getCurrentMs();
    m_last_time = time;

    clampServo->setDutyCycle(ClampServo_val);
    counter++;
    if (counter > counter_limit) {
        isFinished_ = true;
        counter = 0;
    }
}
void ClampServoCommand::end() {
    std::cout << "ClampServoCommand end" << std::endl;
}

// 指令封装  len:打开宽度，单位cm
ICommand::ptr createClampServoCommand(double len) {
    double target = CLAMP_LEN_A * len + CLAMP_LEN_B;
    return std::make_shared<ClampServoCommand>(target)->withTimer(100);
}
ICommand::ptr createClampServoCommand(double len, double cnt_limit) {
    double target = CLAMP_LEN_A * len + CLAMP_LEN_B;
    return std::make_shared<ClampServoCommand>(target, cnt_limit)->withTimer(100);
}

//**********************************伸缩*****************************************
void TelescopicServoCommand::execute() {
    int time = robot::getCurrentMs();
    m_last_time = time;

    telescopicServo->setDutyCycle(TelescopicServo_val);
    counter++;
    if (counter > counter_limit) {
        isFinished_ = true;
        counter = 0;
    }
}
void TelescopicServoCommand::end() {
    std::cout << "TelescopicServoCommand end" << std::endl;
}

// 指令封装  dis:伸缩距离，单位cm
ICommand::ptr createTelescopicServoCommand(double dis) {
    double target = TELESCOPIC_DIS_A * dis + TELESCOPIC_DIS_B;
    return std::make_shared<TelescopicServoCommand>(target)->withTimer(100);
}
ICommand::ptr createTelescopicServoCommand(double dis, double cnt_limit) {
    double target = TELESCOPIC_DIS_A * dis + TELESCOPIC_DIS_B;
    return std::make_shared<TelescopicServoCommand>(target, cnt_limit)->withTimer(100);
}

//**********************************摆手*****************************************
void RaiseServoCommand::execute() {
    int time = robot::getCurrentMs();
    m_last_time = time;

    raiseServo->setDutyCycle(RaiseServo_val);
    counter++;
    if (counter > counter_limit) {
        isFinished_ = true;
        counter = 0;
    }
}
void RaiseServoCommand::end() {
    std::cout << "RaiseServoCommand end" << std::endl;
}

// 指令封装  angle:抬起角度，单位：度
ICommand::ptr createRaiseServoCommand(double angle) {
    double target = RAISE_ANGLE_A * angle + RAISE_ANGLE_B;
    std::cout << "Raise target = " << target << std::endl;
    return std::make_shared<RaiseServoCommand>(target)->withTimer(100);
}
ICommand::ptr createRaiseServoCommand(double angle, double cnt_limit) {
    double target = RAISE_ANGLE_A * angle + RAISE_ANGLE_B;
    return std::make_shared<RaiseServoCommand>(target, cnt_limit)->withTimer(100);
}

//**********************************旋转*****************************************
void RotatingServoCommand::execute() {
    int time = robot::getCurrentMs();
    m_last_time = time;

    rotatingServo->setDutyCycle(RotatingServo_val);
    counter++;
    if (counter > counter_limit) {
        isFinished_ = true;
        counter = 0;
    }
}
void RotatingServoCommand::end() {
    std::cout << "RotatingServoCommand end" << std::endl;
}

// 指令封装  angle:角度，单位：度
ICommand::ptr createRotatingServoCommand(double angle) {
    double target = ROTATING_ANGLE_A * angle + ROTATING_ANGLE_B;
    std::cout << "Rotating target = " << target << std::endl;
    return std::make_shared<RotatingServoCommand>(target)->withTimer(100);
}
ICommand::ptr createRotatingServoCommand(double angle, double cnt_limit) {
    double target = ROTATING_ANGLE_A * angle + ROTATING_ANGLE_B;
    return std::make_shared<RotatingServoCommand>(target, cnt_limit)->withTimer(100);
}
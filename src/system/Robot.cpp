#include "system/Robot.h"
#include "params.h"
// #include "RobotCfg.h"


Robot::Robot() {
    ParamsInit();

    left_pid_ = std::make_shared<PID>(PIDDT);
    right_pid_ = std::make_shared<PID>(PIDDT);
    turn_pid_ = std::make_shared<PID>(PIDDT);
    lift_pid_ = std::make_shared<PID>(PIDDT);
    lift_distance_pid_ = std::make_shared<PID>(PIDDT);
    turn_distance_pid_ = std::make_shared<PID>(PIDDT);
    ir_angle_pid_ = std::make_shared<PID>(PIDDT);
    ir_distance_pid_ = std::make_shared<PID>(PIDDT);
    us_angle_pid_ = std::make_shared<PID>(PIDDT);
    us_distance_pid_ = std::make_shared<PID>(PIDDT);

    single_ir_angle_pid_ = std::make_shared<PID>(PIDDT);
    single_ir_distance_pid_ = std::make_shared<PID>(PIDDT);
}

int32_t Robot::read_left_enc() {
    return LeftENC->read();
}

int32_t Robot::read_right_enc() {
    return RightENC->read();
}

int32_t Robot::read_turn_enc() {
    return TurnENC->read();
}

int32_t Robot::read_lift_enc() {
    return LiftENC->read();
}

void Robot::reset_left_enc() {
    LeftENC->reset();
}

void Robot::reset_right_enc() {
    RightENC->reset();
}

void Robot::reset_turn_enc() {
    TurnENC->reset();
}

void Robot::reset_lift_enc() {
    LiftENC->reset();
}

void Robot::reset_all_enc() {
    reset_left_enc();
    reset_right_enc();
    reset_turn_enc();
    reset_lift_enc();
}

Robot& Robot::GetInstance() {
    static Robot robot;
    return robot;
}

void Robot::controlLeftMotor() {
    left_pid_param = LeftMotorPIDParams.pid;
    left_output_limits = LeftMotorPIDParams.limit;
    left_pid_->set_gains(left_pid_param.kp, left_pid_param.ki, left_pid_param.kd);
    left_pid_->set_output_limits(left_output_limits.min, left_output_limits.max);

    left_pid_->calculate();

    LeftMotor->setSpeedAndDir((abs(left_pid_->output_)), left_pid_->output_ > 0 , left_pid_->output_ < 0);
}
void Robot::controlRightMotor() {
    right_pid_param = RightMotorPIDParams.pid;
    right_output_limits = RightMotorPIDParams.limit;
    right_pid_->set_gains(right_pid_param.kp, right_pid_param.ki, right_pid_param.kd);
    right_pid_->set_output_limits(right_output_limits.min, right_output_limits.max);

    right_pid_->calculate();

    RightMotor->setSpeedAndDir((abs(right_pid_->output_)), right_pid_->output_ > 0 , right_pid_->output_ < 0);
}
void Robot::controlTurnMotor() {
    turn_pid_param = TurnMotorPIDParams.pid;
    turn_output_limits = TurnMotorPIDParams.limit;
    turn_pid_->set_gains(turn_pid_param.kp, turn_pid_param.ki, turn_pid_param.kd);
    turn_pid_->set_output_limits(turn_output_limits.min, turn_output_limits.max);

    turn_pid_->calculate();

    TurnMotor->setSpeedAndDir((abs(turn_pid_->output_)), turn_pid_->output_ > 0 , turn_pid_->output_ < 0);
}
void Robot::controlLiftMotor() {
    lift_pid_param = LiftMotorPIDParams.pid;
    lift_output_limits = LiftMotorPIDParams.limit;
    lift_pid_->set_gains(lift_pid_param.kp, lift_pid_param.ki, lift_pid_param.kd);
    lift_pid_->set_output_limits(lift_output_limits.min, lift_output_limits.max);

    lift_pid_->calculate();

    LiftMotor->setSpeedAndDir((abs(lift_pid_->output_)), lift_pid_->output_ > 0 , lift_pid_->output_ < 0);
}
void Robot::resetMotorsPID() {
    left_pid_->reset();
    right_pid_->reset();
    turn_pid_->reset();
    lift_pid_->reset();
}

void Robot::resetLeftMotorPID() {
    left_pid_->reset();
    left_pid_temp_.last_enc_counter = 0;
}

void Robot::resetRightMotorPID() {
    right_pid_->reset();
    right_pid_temp_.last_enc_counter = 0;
}

void Robot::resetTurnMotorPID() {
    turn_pid_->reset();
    turn_pid_temp_.last_enc_counter = 0;
}

void Robot::resetLiftMotorPID() {
    lift_pid_->reset();
    lift_pid_temp_.last_enc_counter = 0;
}


bool Robot::getStopSignal() {
    return !StopLimit->read();
}

void Robot::setLeftMotorSpeed(double speed) {
    left_pid_->set_point(speed);
}
void Robot::setRightMotorSpeed(double speed){
    right_pid_->set_point(-speed);
}
void Robot::setTurnMotorSpeed(double speed) {
    turn_pid_->set_point(speed);
}
void Robot::setLiftMotorSpeed(double speed) {
    lift_pid_->set_point(speed);
}

void Robot::setLeftMotorProcess(int32_t process) {
    left_pid_->set_process(process - left_pid_temp_.last_enc_counter);
}
void Robot::setRightMotorProcess(int32_t process) {
    right_pid_->set_process(process - right_pid_temp_.last_enc_counter);
}

void Robot::setTurnMotorProcess(int32_t process) {
    turn_pid_->set_process(process - turn_pid_temp_.last_enc_counter);
}

void Robot::setLiftMotorProcess(int32_t process) {
    lift_pid_->set_process(process - lift_pid_temp_.last_enc_counter);
}

void Robot::setLeftMotorLastENCCounter(int32_t last_enc_counter) {
    left_pid_temp_.last_enc_counter = last_enc_counter;
}

void Robot::setRightMotorLastENCCounter(int32_t last_enc_counter) {
    right_pid_temp_.last_enc_counter = last_enc_counter;
}

void Robot::setTurnMotorLastENCCounter(int32_t last_enc_counter) {
    turn_pid_temp_.last_enc_counter = last_enc_counter;
}

void Robot::setLiftMotorLastENCCounter(int32_t last_enc_counter) {
    lift_pid_temp_.last_enc_counter = last_enc_counter;
}

void Robot::setLeftMotorSpeedWithoutPID() const {
LeftMotor->setSpeedAndDir(  static_cast<uint8_t>(abs(left_speed_without_PID_)), left_speed_without_PID_ > 0, left_speed_without_PID_ < 0);
}

void Robot::setRightMotorSpeedWithoutPID() const {
    RightMotor->setSpeedAndDir(static_cast<uint8_t>(abs(right_speed_without_PID_)), right_speed_without_PID_ > 0, right_speed_without_PID_ < 0);
}

void Robot::setTurnMotorSpeedWithoutPID() const {
    TurnMotor->setSpeedAndDir(static_cast<uint8_t>(abs(turn_speed_without_PID_)), turn_speed_without_PID_ > 0, turn_speed_without_PID_ < 0);
}

void Robot::setLiftMotorSpeedWithoutPID() const {
    LiftMotor->setSpeedAndDir(static_cast<uint8_t>(abs(lift_speed_without_PID_)), lift_speed_without_PID_ > 0, lift_speed_without_PID_ < 0);
}

void Robot::LiftMotorDistancePID(int32_t setpoint) {
    lift_distance_pid_param = LiftDisPIDParams.pid;
    lift_distance_limits = LiftDisPIDParams.limit;
    lift_distance_pid_->set_gains(lift_distance_pid_param.kp, lift_distance_pid_param.ki, lift_distance_pid_param.kd);
    lift_distance_pid_->set_output_limits(lift_distance_limits.min, lift_distance_limits.max);
    lift_distance_pid_->set_point(setpoint);
    lift_distance_pid_->set_process(LiftENC->get());

    lift_distance_pid_->calculate();

    lift_pid_->set_point(lift_distance_pid_->output_);
}

void Robot::setTurnMotorDistance(int32_t setpoint) {
    turn_distance_pid_param = TurnDisPIDParams.pid;
    turn_distance_limits = TurnDisPIDParams.limit;
    turn_distance_pid_->set_gains(turn_distance_pid_param.kp, turn_distance_pid_param.ki, turn_distance_pid_param.kd);
    turn_distance_pid_->set_output_limits(turn_distance_limits.min, turn_distance_limits.max);
    turn_distance_pid_->set_point(setpoint);
    turn_distance_pid_->set_process(TurnENC->get());

    turn_distance_pid_->calculate();

    turn_pid_->set_point(turn_distance_pid_->output_);
}

double irChange(double x) {
    return 101.012 - 166.289*x + 141.969* std::pow(x, 2) - 66.134* std::pow(x, 3) + 15.868* std::pow(x, 4) - 1.532* std::pow(x, 5);
}
//红外校准
bool  Robot::calibrationIR(double distance, double angle_error, double distance_error, double left_right_e, double offset) {
    double right = ir_right_filter->filter(irChange(IR_right->read()));
    double left = ir_left_filter->filter(irChange(IR_left->read()));

    ir_angle_pid_param = IRCalAnglePIDParams.pid;
    ir_angle_limits = IRCalAnglePIDParams.limit;
    ir_angle_pid_->set_gains(ir_angle_pid_param.kp, ir_angle_pid_param.ki, ir_angle_pid_param.kd);
    ir_angle_pid_->set_output_limits(ir_angle_limits.min, ir_angle_limits.max);
    ir_angle_pid_->set_point(offset);

    std::cout << "right = " << irChange(IR_right->read()) << " filter right = " << right << std::endl;
    std::cout << "left = " << irChange(IR_left->read()) << " filter left = " << left << std::endl;
    ir_angle_pid_->set_process(right - left);
    ir_angle_pid_->calculate();
    ir_distance_pid_param = IRCalDisPIDParams.pid;
    ir_distance_limits = IRCalDisPIDParams.limit;
    ir_distance_pid_->set_gains(ir_distance_pid_param.kp, ir_distance_pid_param.ki, ir_distance_pid_param.kd);
    ir_distance_pid_->set_output_limits(ir_distance_limits.min, ir_distance_limits.max);
    ir_distance_pid_->set_point(distance);
    ir_distance_pid_->set_process((left + right) / 2);
    ir_distance_pid_->calculate();

    if (abs(left - right) > left_right_e) {
        ir_distance_pid_->output_ = 0;
    }


    double L_speed = ir_distance_pid_->output_ - ir_angle_pid_->output_;
    double R_speed = ir_distance_pid_->output_ + ir_angle_pid_->output_;
    std::cout << "L_speed: " << L_speed  <<  " R_speed:" << R_speed << std::endl;

    Robot::GetInstance().setLeftMotorSpeed(-L_speed);       //传感器装在车前为-L_speed
    Robot::GetInstance().setRightMotorSpeed(-R_speed);
    bool angle_in_limit = abs(left - right) < angle_error;
    bool distance_in_limit = abs((left + right) / 2 - distance) < distance_error;
    return angle_in_limit & distance_in_limit;
}

//超声波校准
bool  Robot::calibrationUS(double distance, double angle_error,double distance_error, double left_right_e, double offset) {
    us_angle_pid_param = USCalAnglePIDParams.pid;
    us_angle_limits = USCalAnglePIDParams.limit;

    us_angle_pid_->set_gains(us_angle_pid_param.kp, us_angle_pid_param.ki, us_angle_pid_param.kd);
    us_angle_pid_->set_output_limits(us_angle_limits.min, us_angle_limits.max);
    us_angle_pid_->set_point(offset);

    Leftus->trig();
    Rightus->trig();

    double right = us_right_filter->filter(Rightus->echo() * 0.017);
    double left = us_left_filter->filter(Leftus->echo() * 0.017);

    std::cout << "left = " << left <<  " right = " << right << std::endl;
    us_angle_pid_->set_process(right - left);
    us_angle_pid_->calculate();

    us_distance_pid_param = USCalDisPIDParams.pid;
    us_distance_limits = USCalDisPIDParams.limit;

    us_distance_pid_->set_gains(us_distance_pid_param.kp, us_distance_pid_param.ki, us_distance_pid_param.kd);
    us_distance_pid_->set_output_limits(us_distance_limits.min, us_distance_limits.max);
    us_distance_pid_->set_point(distance);
    us_distance_pid_->set_process((left + right) / 2);
    us_distance_pid_->calculate();
    if (abs(left - right) > left_right_e) {
        us_distance_pid_->output_ = 0;
    }


    double L_speed = us_distance_pid_->output_ - us_angle_pid_->output_;
    double R_speed = us_distance_pid_->output_ + us_angle_pid_->output_;
    std::cout << "L_speed: " << L_speed  <<  " R_speed:" << R_speed << std::endl;

    Robot::GetInstance().setLeftMotorSpeed(L_speed);    //传感器装在车前为-L_speed
    Robot::GetInstance().setRightMotorSpeed(R_speed);
    bool angle_in_limit = abs(left - right) < angle_error;
    bool distance_in_limit = abs((left + right) / 2 - distance) < distance_error;
    return angle_in_limit & distance_in_limit;
}




//单红外校准
bool Robot::calibrationSingleIR(double distance, double hold_phi, double angle_error,double distance_error) {
    double cur_phi = VMX::getYaw();
    double temp_deltaPHi[3] = {std::abs(hold_phi - cur_phi),
                               std::abs(hold_phi - (cur_phi + 360)),
                               std::abs(hold_phi - (cur_phi - 360))};
    //寻找最近的旋转方向
    int minIndex = 0;
    for(int i = 1; i < 3; i++){
    if(temp_deltaPHi[i] < temp_deltaPHi[minIndex]){
      minIndex = i;
    }
    }
    //确定合适的取值
    if(minIndex == 1) {cur_phi = cur_phi + 360;}
    else if(minIndex == 2) {cur_phi = cur_phi - 360;}

    single_ir_angle_pid_param = SingleIRCalAnglePIDParams.pid;
    single_ir_angle_limits = SingleIRCalAnglePIDParams.limit;
    single_ir_angle_pid_->set_gains(single_ir_angle_pid_param.kp, single_ir_angle_pid_param.ki,
                                    single_ir_angle_pid_param.kd);
    single_ir_angle_pid_->set_output_limits(single_ir_angle_limits.min, single_ir_angle_limits.max);
    single_ir_angle_pid_->set_point(hold_phi);
    single_ir_angle_pid_->set_process(cur_phi);
    single_ir_angle_pid_->calculate();

    double right = ir_right_filter->filter(irChange(IR_right->read()));
    std::cout << " right = " << irChange(IR_right->read()) << " filter right = " << right << std::endl;

    single_ir_distance_pid_param = SingleIRCalDisPIDParams.pid;
    single_ir_distance_limits = SingleIRCalDisPIDParams.limit;
    single_ir_distance_pid_->set_gains(single_ir_distance_pid_param.kp, single_ir_distance_pid_param.ki,
                                       single_ir_distance_pid_param.kd);
    single_ir_distance_pid_->set_output_limits(single_ir_distance_limits.min, single_ir_distance_limits.max);
    single_ir_distance_pid_->set_point(distance);
    single_ir_distance_pid_->set_process(right);
    single_ir_distance_pid_->calculate();
    if (abs(hold_phi - cur_phi) > SingleIRCalibE.LeftRightE) {
      single_ir_distance_pid_->output_ = 0;
    }

    double L_speed = single_ir_distance_pid_->output_ - single_ir_angle_pid_->output_;
    double R_speed = single_ir_distance_pid_->output_ + single_ir_angle_pid_->output_;

    Robot::GetInstance().setLeftMotorSpeed(-L_speed);       //传感器装在车前为-L_speed
    Robot::GetInstance().setRightMotorSpeed(-R_speed);
    bool angle_in_limit = abs(hold_phi - cur_phi) < angle_error;
    bool distance_in_limit = abs(right - distance) < distance_error;
    return angle_in_limit & distance_in_limit;
}
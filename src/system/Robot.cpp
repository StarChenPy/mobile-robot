#include "system/Robot.h"
#include "util/params.h"

namespace robot {
    Robot::Robot() {
        ParamsInit();

        left_pid_ = std::make_shared<PID>(PID_DT);
        right_pid_ = std::make_shared<PID>(PID_DT);
        turn_pid_ = std::make_shared<PID>(PID_DT);
        lift_pid_ = std::make_shared<PID>(PID_DT);
        lift_distance_pid_ = std::make_shared<PID>(PID_DT);
        turn_distance_pid_ = std::make_shared<PID>(PID_DT);
        ir_angle_pid_ = std::make_shared<PID>(PID_DT);
        ir_distance_pid_ = std::make_shared<PID>(PID_DT);
        us_angle_pid_ = std::make_shared<PID>(PID_DT);
        us_distance_pid_ = std::make_shared<PID>(PID_DT);

        single_ir_angle_pid_ = std::make_shared<PID>(PID_DT);
        single_ir_distance_pid_ = std::make_shared<PID>(PID_DT);
    }

    int32_t Robot::readLeftEnc() { return leftEnc->read(); }

    int32_t Robot::readRightEnc() { return rightEnc->read(); }

    int32_t Robot::readTurnEnc() { return turnEnc->read(); }

    int32_t Robot::readLiftEnc() { return liftEnc->read(); }

    void Robot::resetLeftEnc() { leftEnc->reset(); }

    void Robot::resetRightEnc() { rightEnc->reset(); }

    void Robot::resetTurnEnc() { turnEnc->reset(); }

    void Robot::resetLiftEnc() { liftEnc->reset(); }

    void Robot::resetAllEnc() {
        resetLeftEnc();
        resetRightEnc();
        resetTurnEnc();
        resetLiftEnc();
    }

    Robot &Robot::getInstance() {
        static Robot Robot;
        return Robot;
    }

    void Robot::controlLeftMotor() {
        leftPidParam = LeftMotorPIDParams.pid;
        leftOutputLimits = LeftMotorPIDParams.limit;
        left_pid_->set_gains(leftPidParam.kp, leftPidParam.ki, leftPidParam.kd);
        left_pid_->set_output_limits(leftOutputLimits.min, leftOutputLimits.max);

        left_pid_->calculate();

        leftMotor->setSpeedAndDir((abs(left_pid_->output_)), left_pid_->output_ > 0, left_pid_->output_ < 0);
    }
    void Robot::controlRightMotor() {
        rightPidParam = RightMotorPIDParams.pid;
        rightOutputLimits = RightMotorPIDParams.limit;
        right_pid_->set_gains(rightPidParam.kp, rightPidParam.ki, rightPidParam.kd);
        right_pid_->set_output_limits(rightOutputLimits.min, rightOutputLimits.max);

        right_pid_->calculate();

        rightMotor->setSpeedAndDir((abs(right_pid_->output_)), right_pid_->output_ > 0, right_pid_->output_ < 0);
    }
    void Robot::controlTurnMotor() {
        turnPidParam = TurnMotorPIDParams.pid;
        turnOutputLimits = TurnMotorPIDParams.limit;
        turn_pid_->set_gains(turnPidParam.kp, turnPidParam.ki, turnPidParam.kd);
        turn_pid_->set_output_limits(turnOutputLimits.min, turnOutputLimits.max);

        turn_pid_->calculate();

        turnMotor->setSpeedAndDir((abs(turn_pid_->output_)), turn_pid_->output_ > 0, turn_pid_->output_ < 0);
    }
    void Robot::controlLiftMotor() {
        liftPidParam = LiftMotorPIDParams.pid;
        liftOutputLimits = LiftMotorPIDParams.limit;
        lift_pid_->set_gains(liftPidParam.kp, liftPidParam.ki, liftPidParam.kd);
        lift_pid_->set_output_limits(liftOutputLimits.min, liftOutputLimits.max);

        lift_pid_->calculate();

        liftMotor->setSpeedAndDir((abs(lift_pid_->output_)), lift_pid_->output_ > 0, lift_pid_->output_ < 0);
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
        return !stopLimit->read();
    }

    void Robot::setLeftMotorSpeed(double speed) { left_pid_->set_point(speed); }
    void Robot::setRightMotorSpeed(double speed) { right_pid_->set_point(-speed); }
    void Robot::setTurnMotorSpeed(double speed) { turn_pid_->set_point(speed); }
    void Robot::setLiftMotorSpeed(double speed) { lift_pid_->set_point(speed); }

    void Robot::setLeftMotorProcess(int32_t process) { left_pid_->set_process(process - left_pid_temp_.last_enc_counter); }
    void Robot::setRightMotorProcess(int32_t process) {
        right_pid_->set_process(process - right_pid_temp_.last_enc_counter);
    }

    void Robot::setTurnMotorProcess(int32_t process) { turn_pid_->set_process(process - turn_pid_temp_.last_enc_counter); }

    void Robot::setLiftMotorProcess(int32_t process) { lift_pid_->set_process(process - lift_pid_temp_.last_enc_counter); }

    void Robot::setLeftMotorLastENCCounter(int32_t last_enc_counter) { left_pid_temp_.last_enc_counter = last_enc_counter; }

    void Robot::setRightMotorLastENCCounter(int32_t last_enc_counter) {
        right_pid_temp_.last_enc_counter = last_enc_counter;
    }

    void Robot::setTurnMotorLastENCCounter(int32_t last_enc_counter) { turn_pid_temp_.last_enc_counter = last_enc_counter; }

    void Robot::setLiftMotorLastENCCounter(int32_t last_enc_counter) { lift_pid_temp_.last_enc_counter = last_enc_counter; }

    void Robot::setLeftMotorSpeedWithoutPID() const {
        leftMotor->setSpeedAndDir(static_cast<uint8_t>(abs(left_speed_without_PID_)), left_speed_without_PID_ > 0,
                                  left_speed_without_PID_ < 0);
    }

    void Robot::setRightMotorSpeedWithoutPID() const {
        rightMotor->setSpeedAndDir(static_cast<uint8_t>(abs(right_speed_without_PID_)), right_speed_without_PID_ > 0,
                                   right_speed_without_PID_ < 0);
    }

    void Robot::setTurnMotorSpeedWithoutPID() const {
        turnMotor->setSpeedAndDir(static_cast<uint8_t>(abs(turn_speed_without_PID_)), turn_speed_without_PID_ > 0,
                                  turn_speed_without_PID_ < 0);
    }

    void Robot::setLiftMotorSpeedWithoutPID() const {
        liftMotor->setSpeedAndDir(static_cast<uint8_t>(abs(lift_speed_without_PID_)), lift_speed_without_PID_ > 0,
                                  lift_speed_without_PID_ < 0);
    }

    void Robot::LiftMotorDistancePID(int32_t setpoint) {
        liftDistancePidParam = LiftDisPIDParams.pid;
        liftDistanceLimits = LiftDisPIDParams.limit;
        lift_distance_pid_->set_gains(liftDistancePidParam.kp, liftDistancePidParam.ki, liftDistancePidParam.kd);
        lift_distance_pid_->set_output_limits(liftDistanceLimits.min, liftDistanceLimits.max);
        lift_distance_pid_->set_point(setpoint);
        lift_distance_pid_->set_process(liftEnc->get());

        lift_distance_pid_->calculate();

        lift_pid_->set_point(lift_distance_pid_->output_);
    }

    void Robot::setTurnMotorDistance(int32_t setpoint) {
        turnDistancePidParam = TurnDisPIDParams.pid;
        turnDistanceLimits = TurnDisPIDParams.limit;
        turn_distance_pid_->set_gains(turnDistancePidParam.kp, turnDistancePidParam.ki, turnDistancePidParam.kd);
        turn_distance_pid_->set_output_limits(turnDistanceLimits.min, turnDistanceLimits.max);
        turn_distance_pid_->set_point(setpoint);
        turn_distance_pid_->set_process(turnEnc->get());

        turn_distance_pid_->calculate();

        turn_pid_->set_point(turn_distance_pid_->output_);
    }

    double irChange(double x) {
        return 101.012 - 166.289 * x + 141.969 * std::pow(x, 2) - 66.134 * std::pow(x, 3) + 15.868 * std::pow(x, 4) -
               1.532 * std::pow(x, 5);
    }
//红外校准
    bool Robot::calibrationIR(double distance, double angle_error, double distance_error, double left_right_e,
                              double offset) {
        double right = ir_right_filter->filter(irChange(irRight->read()));
        double left = ir_left_filter->filter(irChange(irLeft->read()));

        irAnglePidParam = IRCalAnglePIDParams.pid;
        irAngleLimits = IRCalAnglePIDParams.limit;
        ir_angle_pid_->set_gains(irAnglePidParam.kp, irAnglePidParam.ki, irAnglePidParam.kd);
        ir_angle_pid_->set_output_limits(irAngleLimits.min, irAngleLimits.max);
        ir_angle_pid_->set_point(offset);

        std::cout << "right = " << irChange(irRight->read()) << " filter right = " << right << std::endl;
        std::cout << "left = " << irChange(irLeft->read()) << " filter left = " << left << std::endl;
        ir_angle_pid_->set_process(right - left);
        ir_angle_pid_->calculate();
        irDistancePidParam = IRCalDisPIDParams.pid;
        irDistanceLimits = IRCalDisPIDParams.limit;
        ir_distance_pid_->set_gains(irDistancePidParam.kp, irDistancePidParam.ki, irDistancePidParam.kd);
        ir_distance_pid_->set_output_limits(irDistanceLimits.min, irDistanceLimits.max);
        ir_distance_pid_->set_point(distance);
        ir_distance_pid_->set_process((left + right) / 2);
        ir_distance_pid_->calculate();

        if (abs(left - right) > left_right_e) {
            ir_distance_pid_->output_ = 0;
        }

        double L_speed = ir_distance_pid_->output_ - ir_angle_pid_->output_;
        double R_speed = ir_distance_pid_->output_ + ir_angle_pid_->output_;
        std::cout << "L_speed: " << L_speed << " R_speed:" << R_speed << std::endl;

        Robot::getInstance().setLeftMotorSpeed(-L_speed); //传感器装在车前为-L_speed
        Robot::getInstance().setRightMotorSpeed(-R_speed);
        bool angle_in_limit = abs(left - right) < angle_error;
        bool distance_in_limit = abs((left + right) / 2 - distance) < distance_error;
        return angle_in_limit & distance_in_limit;
    }

//超声波校准
    bool Robot::calibrationUS(double distance, double angle_error, double distance_error, double left_right_e,
                              double offset) {
        usAnglePidParam = USCalAnglePIDParams.pid;
        usAngleLimits = USCalAnglePIDParams.limit;

        us_angle_pid_->set_gains(usAnglePidParam.kp, usAnglePidParam.ki, usAnglePidParam.kd);
        us_angle_pid_->set_output_limits(usAngleLimits.min, usAngleLimits.max);
        us_angle_pid_->set_point(offset);

        leftUs->trig();
        rightUs->trig();

        double right = us_right_filter->filter(rightUs->echo() * 0.017);
        double left = us_left_filter->filter(leftUs->echo() * 0.017);

        std::cout << "left = " << left << " right = " << right << std::endl;
        us_angle_pid_->set_process(right - left);
        us_angle_pid_->calculate();

        usDistancePidParam = USCalDisPIDParams.pid;
        usDistanceLimits = USCalDisPIDParams.limit;

        us_distance_pid_->set_gains(usDistancePidParam.kp, usDistancePidParam.ki, usDistancePidParam.kd);
        us_distance_pid_->set_output_limits(usDistanceLimits.min, usDistanceLimits.max);
        us_distance_pid_->set_point(distance);
        us_distance_pid_->set_process((left + right) / 2);
        us_distance_pid_->calculate();
        if (abs(left - right) > left_right_e) {
            us_distance_pid_->output_ = 0;
        }

        double L_speed = us_distance_pid_->output_ - us_angle_pid_->output_;
        double R_speed = us_distance_pid_->output_ + us_angle_pid_->output_;
        std::cout << "L_speed: " << L_speed << " R_speed:" << R_speed << std::endl;

        Robot::getInstance().setLeftMotorSpeed(L_speed); //传感器装在车前为-L_speed
        Robot::getInstance().setRightMotorSpeed(R_speed);
        bool angle_in_limit = abs(left - right) < angle_error;
        bool distance_in_limit = abs((left + right) / 2 - distance) < distance_error;
        return angle_in_limit & distance_in_limit;
    }

//单红外校准
    bool Robot::calibrationSingleIR(double distance, double hold_phi, double angle_error, double distance_error) {
        double cur_phi = VMX::getYaw();
        double temp_deltaPHi[3] = {std::abs(hold_phi - cur_phi), std::abs(hold_phi - (cur_phi + 360)),
                                   std::abs(hold_phi - (cur_phi - 360))};
        //寻找最近的旋转方向
        int minIndex = 0;
        for (int i = 1; i < 3; i++) {
            if (temp_deltaPHi[i] < temp_deltaPHi[minIndex]) {
                minIndex = i;
            }
        }
        //确定合适的取值
        if (minIndex == 1) {
            cur_phi = cur_phi + 360;
        } else if (minIndex == 2) {
            cur_phi = cur_phi - 360;
        }

        singleIrAnglePidParam = SingleIRCalAnglePIDParams.pid;
        singleIrAngleLimits = SingleIRCalAnglePIDParams.limit;
        single_ir_angle_pid_->set_gains(singleIrAnglePidParam.kp, singleIrAnglePidParam.ki, singleIrAnglePidParam.kd);
        single_ir_angle_pid_->set_output_limits(singleIrAngleLimits.min, singleIrAngleLimits.max);
        single_ir_angle_pid_->set_point(hold_phi);
        single_ir_angle_pid_->set_process(cur_phi);
        single_ir_angle_pid_->calculate();

        double right = ir_right_filter->filter(irChange(irRight->read()));
        std::cout << " right = " << irChange(irRight->read()) << " filter right = " << right << std::endl;

        singleIrDistancePidParam = SingleIRCalDisPIDParams.pid;
        singleIrDistanceLimits = SingleIRCalDisPIDParams.limit;
        single_ir_distance_pid_->set_gains(singleIrDistancePidParam.kp, singleIrDistancePidParam.ki,
                                           singleIrDistancePidParam.kd);
        single_ir_distance_pid_->set_output_limits(singleIrDistanceLimits.min, singleIrDistanceLimits.max);
        single_ir_distance_pid_->set_point(distance);
        single_ir_distance_pid_->set_process(right);
        single_ir_distance_pid_->calculate();
        if (abs(hold_phi - cur_phi) > SingleIRCalibE.LeftRightE) {
            single_ir_distance_pid_->output_ = 0;
        }

        double L_speed = single_ir_distance_pid_->output_ - single_ir_angle_pid_->output_;
        double R_speed = single_ir_distance_pid_->output_ + single_ir_angle_pid_->output_;

        Robot::getInstance().setLeftMotorSpeed(-L_speed); //传感器装在车前为-L_speed
        Robot::getInstance().setRightMotorSpeed(-R_speed);
        bool angle_in_limit = abs(hold_phi - cur_phi) < angle_error;
        bool distance_in_limit = abs(right - distance) < distance_error;
        return angle_in_limit & distance_in_limit;
    }
} // namespace Robot

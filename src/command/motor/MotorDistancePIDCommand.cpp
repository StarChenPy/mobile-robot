#include "command/motor/MotorDistancePIDCommand.h"
#include "system/Robot.h"

namespace robot {
    LiftMotorDistancePIDCommand::LiftMotorDistancePIDCommand(int32_t distance, double distanceError, uint8_t verificationTimes) {
        isFinished_ = false;
        counter_ = 0;
        distance_ = distance;
        distanceError_ = distanceError;
        verificationTimes_ = verificationTimes;
    }
    void LiftMotorDistancePIDCommand::execute() {
        Robot::getInstance().LiftMotorDistancePID(distance_);
        std::cout << "Lift ENC: " << liftEnc->get() << " Lift set_point_: " << distance_ << std::endl;
    }
    void LiftMotorDistancePIDCommand::end() {
        Robot::getInstance().setLiftMotorSpeed(0);
        std::cout << "LiftMotorDistancePIDCommand end!" << std::endl;
    }
    bool LiftMotorDistancePIDCommand::isFinished() {
        if (abs(distance_ - liftEnc->get()) < distanceError_) {
            counter_++;
        } else {
            counter_ = 0;
        }
        return Robot::getStopSignal() || counter_ > verificationTimes_ || isFinished_;
    }
    ICommand::ptr LiftMotorDistancePIDCommand::create(double distance, double distanceError, uint8_t verificationTimes) {
        if (distance > 0) {
            distance = 0;
            std::cout << "升降电机输入不可为正数" << std::endl;
        }
        distance = static_cast<int32_t>(distance * (1000 / 10.7 / 2));
        return std::make_shared<LiftMotorDistancePIDCommand>(distance, distanceError, verificationTimes)->withTimer(100);
    }

    TurnMotorDistancePIDCommand::TurnMotorDistancePIDCommand(int32_t distance, double distanceError, uint8_t verificationTimes) {
        isFinished_ = false;
        counter_ = 0;
        distance_ = distance;
        distanceError_ = distanceError;
        verificationTimes_ = verificationTimes;
    }
    void TurnMotorDistancePIDCommand::execute() {
        Robot::getInstance().setTurnMotorDistance(distance_);
        std::cout << "Turn ENC: " << turnEnc->get() << " Turn set_point_: " << distance_ << std::endl;
    }
    void TurnMotorDistancePIDCommand::end() {
        Robot::getInstance().setTurnMotorSpeed(0);
        std::cout << "TurnMotorDistancePIDCommand end!" << std::endl;
    }
    bool TurnMotorDistancePIDCommand::isFinished() {
        if (abs(distance_ - turnEnc->get()) < distanceError_) {
            counter_++;
        } else {
            counter_ = 0;
        }
        return Robot::getStopSignal() || counter_ > verificationTimes_ || isFinished_;
    }
    ICommand::ptr TurnMotorDistancePIDCommand::create(double angle, double distanceError, uint8_t verificationTimes) {
        if (angle > 270 || angle < -270) {
            angle = 0;
            std::cout << "旋转电机输入不可大于270°，小于-270°!" << std::endl;
        }
        angle = static_cast<int32_t>(angle * (1670.0 / 90.0) - 2);
        return std::make_shared<TurnMotorDistancePIDCommand>(angle, distanceError, verificationTimes)->withTimer(100);
    }

// *------------------------------------ 下面是 Reset 命令 ------------------------------------* //

    ResetLiftMotorCommand::ResetLiftMotorCommand(int32_t speed) {
        isFinished_ = false;
        speed_ = speed;
        maxCounter_ = 10;
        counter_ = 0;
    }
    void ResetLiftMotorCommand::execute() {
        Robot::getInstance().setLiftMotorSpeed(speed_);
        std::cout << "liftLimit->read():" << liftLimit->read() << std::endl;

        static bool finished = false;
        if (!finished) {
            finished = !liftLimit->read();
        }

        if (finished) {
            counter_++;
            speed_ = 0;
        } else {
            counter_ = 0;
        }
        isFinished_ = counter_ > maxCounter_;
        if (isFinished_) {
            finished = false;
        }
    }
    void ResetLiftMotorCommand::end() {
        Robot::resetLiftEnc();
        Robot::getInstance().resetLiftMotorPID();
        Robot::getInstance().setLiftMotorSpeed(0);
        std::cout << "ResetLiftMotorCommand end!" << std::endl;
    }
    bool ResetLiftMotorCommand::isFinished() {
        cout << "liftLimit: " << liftLimit->read() << endl;
        if (!liftLimit->read()) {
            Robot::getInstance().setLiftMotorSpeed(0);
        }
        return Robot::getStopSignal() || isFinished_;
    }
    ICommand::ptr ResetLiftMotorCommand::create(int32_t speed) {
        return std::make_shared<ResetLiftMotorCommand>(speed)->withTimer(100);
    }

    ResetTurnMotorCommand::ResetTurnMotorCommand(int32_t speed) {
        isFinished_ = false;
        speed_ = speed;
        maxCounter_ = 10;
        counter_ = 0;
    }
    void ResetTurnMotorCommand::execute() {
        Robot::getInstance().setTurnMotorSpeed(speed_);
        static bool finished = false;
        if (!finished) {
            std::cout << "turningLimit->read() " << turningLimit->read() << std::endl;
            finished = turningLimit->read() > 1.0;
        }

        if (finished) {
            counter_++;
            speed_ = 0;
        } else {
            counter_ = 0;
        }
        isFinished_ = counter_ > maxCounter_;
        if (isFinished_) {
            finished = false;
        }
    }
    void ResetTurnMotorCommand::end() {
        Robot::getInstance().setTurnMotorSpeed(0);
        Robot::resetTurnEnc();
        Robot::getInstance().resetTurnMotorPID();
        std::cout << "ResetTurnMotorCommand end!" <<std::endl;
    }
    bool ResetTurnMotorCommand::isFinished() {
        return Robot::getStopSignal() || isFinished_;
    }
    ICommand::ptr ResetTurnMotorCommand::create(int32_t speed) {
        return std::make_shared<ResetTurnMotorCommand>(speed)->withTimer(100);
    }
} // namespace robot

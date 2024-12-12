#include "command/motor/MotorDistancePIDCommand.h"
#include "system/Robot.h"

namespace robot {
    LiftMotorDistancePIDCommand::LiftMotorDistancePIDCommand(int32_t distance, double distanceError, uint8_t verificationTimes) {
        counter_ = 0;
        distance_ = distance;
        distanceError_ = distanceError;
        verificationTimes_ = verificationTimes;
    }
    void LiftMotorDistancePIDCommand::execute() {
        Robot::getInstance().LiftMotorDistancePID(distance_);
    }
    void LiftMotorDistancePIDCommand::end() {
        ICommand::end();
        Robot::getInstance().setLiftMotorSpeed(0);
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
            LOG(WARNING) << "升降电机输入不可为正数";
        }
        distance = static_cast<int32_t>(distance * (1000 / 10.7 / 2));
        return std::make_shared<LiftMotorDistancePIDCommand>(distance, distanceError, verificationTimes)->withTimer(20);
    }

    TurnMotorDistancePIDCommand::TurnMotorDistancePIDCommand(int32_t distance, double distanceError, uint8_t verificationTimes) {
        counter_ = 0;
        distance_ = distance;
        distanceError_ = distanceError;
        verificationTimes_ = verificationTimes;
    }
    void TurnMotorDistancePIDCommand::execute() {
        Robot::getInstance().setTurnMotorDistance(distance_);
    }
    void TurnMotorDistancePIDCommand::end() {
        ICommand::end();
        Robot::getInstance().setTurnMotorSpeed(0);
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
            LOG(WARNING) << "旋转电机输入不可大于270°，小于-270°!";
        }
        angle = static_cast<int32_t>(angle * (1670.0 / 90.0) - 2);
        return std::make_shared<TurnMotorDistancePIDCommand>(angle, distanceError, verificationTimes)->withTimer(20);
    }

// *------------------------------------ 下面是 Reset 命令 ------------------------------------* //

    ResetLiftMotorCommand::ResetLiftMotorCommand(int32_t speed) {
        speed_ = speed;
        maxCounter_ = 10;
        counter_ = 0;
    }
    void ResetLiftMotorCommand::execute() {
        Robot::getInstance().setLiftMotorSpeed(speed_);

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
            LOG(INFO) << "升降限位按钮被按下";
            finished = false;
        }
    }
    void ResetLiftMotorCommand::end() {
        ICommand::end();
        Robot::resetLiftEnc();
        Robot::getInstance().resetLiftMotorPID();
        Robot::getInstance().setLiftMotorSpeed(0);
    }
    ICommand::ptr ResetLiftMotorCommand::create(int32_t speed) {
        return std::make_shared<ResetLiftMotorCommand>(speed)->withTimer(20);
    }

    ResetTurnMotorCommand::ResetTurnMotorCommand(int32_t speed) {
        speed_ = speed;
        maxCounter_ = 10;
        counter_ = 0;
    }
    void ResetTurnMotorCommand::execute() {
        Robot::getInstance().setTurnMotorSpeed(speed_);
        static bool finished = false;
        if (!finished) {
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
            LOG(INFO) << "旋转限位按钮被按下";
            finished = false;
        }
    }
    void ResetTurnMotorCommand::end() {
        ICommand::end();

        Robot::getInstance().setTurnMotorSpeed(0);
        Robot::resetTurnEnc();
        Robot::getInstance().resetTurnMotorPID();
    }
    ICommand::ptr ResetTurnMotorCommand::create(int32_t speed) {
        return std::make_shared<ResetTurnMotorCommand>(speed)->withTimer(20);
    }
} // namespace robot

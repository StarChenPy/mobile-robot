#ifndef MOBILE_ROBOT_MOTORDISTANCEPIDCOMMAND_H
#define MOBILE_ROBOT_MOTORDISTANCEPIDCOMMAND_H

#include "command/CommandBase.h"
#include "util/params.h"
namespace robot {
    class LiftMotorDistancePIDCommand : public CommandBase {
    public:
        typedef std::shared_ptr<LiftMotorDistancePIDCommand> ptr;
        /**
         * 升降电机 PID 控制命令
         * @param distance 升降距离
         * @param distanceError 误差范围
         * @param verificationTimes 重复验证次数
         */
        LiftMotorDistancePIDCommand(int32_t distance, double distanceError, uint8_t verificationTimes);

        void initialize() override;
        void execute() override;
        void end() override;
        bool isFinished() override;

        static Command::ptr create(double h, double distanceError = LIFT_MOTOR_DISTANCE_ERROR, uint8_t verificationTimes = LIFT_MOTOR_DISTANCE_COUNTER);

    private:
        bool isFinished_;
        int32_t counter_;
        int32_t distance_;
        double distanceError_;
        uint8_t verificationTimes_;
    };

    class TurnMotorDistancePIDCommand : public CommandBase {
    public:
        typedef std::shared_ptr<TurnMotorDistancePIDCommand> ptr;
        /**
         * 旋转电机 PID 控制命令
         * @param distance 旋转角度
         * @param distanceError 误差范围
         * @param verificationTimes 重复验证次数
         */
        TurnMotorDistancePIDCommand(int32_t distance, double distanceError, uint8_t verificationTimes);

        void execute() override;
        void end() override;
        bool isFinished() override;

        static Command::ptr create(double angle, double distanceError = TURN_MOTOR_DISTANCE_ERROR, uint8_t verificationTimes = TURN_MOTOR_DISTANCE_COUNTER);

    private:
        bool isFinished_;
        int32_t counter_;
        int32_t distance_;
        double distanceError_;
        uint8_t verificationTimes_;
    };

// *------------------------------------ 下面是 Reset 命令 ------------------------------------* //

    class ResetLiftMotorCommand : public CommandBase {
    public:
        typedef std::shared_ptr<ResetLiftMotorCommand> ptr;
        /**
         * 升降电机 PID 控制复位命令
         * @param speed 速度
         */
        explicit ResetLiftMotorCommand(int32_t speed);

        void execute() override;
        void end() override;
        bool isFinished() override;

        static Command::ptr create(int32_t speed = 10);

    private:
        bool isFinished_ = false;
        int32_t speed_ = 0;
        int32_t maxCounter_ = 10;
        int32_t counter_ = 0;
    };

    class ResetTurnMotorCommand : public CommandBase {
    public:
        typedef std::shared_ptr<ResetTurnMotorCommand> ptr;
        /**
         * 旋转电机 PID 控制复位命令
         * @param speed 速度
         */
        explicit ResetTurnMotorCommand(int32_t speed);

        void execute() override;
        void end() override;
        bool isFinished() override;

        static Command::ptr create(int32_t speed = 10);

    private:
        bool isFinished_ = false;
        int32_t speed_ = 0;
        int32_t maxCounter_ = 10;
        int32_t counter_ = 0;
    };
} // namespace robot

#endif // MOBILE_ROBOT_MOTORDISTANCEPIDCOMMAND_H

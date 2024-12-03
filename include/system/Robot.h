
#pragma once
#include "system/FuncClass/FuncHead.h"
#include "util/RobotCfg.h"
#include "util/params.h"

namespace robot {
    class Robot {
    public:
        typedef std::shared_ptr<Robot> ptr;

        Robot();

        static int32_t readLeftEnc();
        static int32_t readRightEnc();
        static int32_t readTurnEnc();
        static int32_t readLiftEnc();
        static void resetLeftEnc();
        static void resetRightEnc();
        static void resetTurnEnc();
        static void resetLiftEnc();
        static void resetAllEnc();

        static Robot &getInstance();

    private:
        uint32_t seed_counter_ = 0;
        uint32_t seed_stop_time_ = 0;
        uint32_t plant_counter_ = 0;
        uint32_t plant_stop_time_ = 0;

        uint32_t mixing_counter_ = 0;
        uint32_t mixing_time_ = 0;
        uint32_t mixing_stop_time_ = 0;

    public:
        // // 电机速度PID控制
        void controlLeftMotor();
        void controlRightMotor();
        void controlTurnMotor();
        void controlLiftMotor();
        void resetMotorsPID();
        void resetLeftMotorPID();
        void resetRightMotorPID();
        void resetTurnMotorPID();
        void resetLiftMotorPID();
        void setLeftMotorSpeed(double speed);
        void setRightMotorSpeed(double speed);
        void setTurnMotorSpeed(double speed);
        void setLiftMotorSpeed(double speed);
        void setLeftMotorSpeedWithoutPID() const;
        void setRightMotorSpeedWithoutPID() const;
        void setTurnMotorSpeedWithoutPID() const;
        void setLiftMotorSpeedWithoutPID() const;
        void setLeftMotorProcess(int32_t process);
        void setRightMotorProcess(int32_t process);
        void setTurnMotorProcess(int32_t process);
        void setLiftMotorProcess(int32_t process);
        void setLeftMotorLastENCCounter(int32_t last_enc_counter);
        void setRightMotorLastENCCounter(int32_t last_enc_counter);
        void setTurnMotorLastENCCounter(int32_t last_enc_counter);
        void setLiftMotorLastENCCounter(int32_t last_enc_counter);
        void LiftMotorDistancePID(int32_t setpoint);
        void setTurnMotorDistance(int32_t setpoint);

    public:
        PID::ptr left_pid_;
        PID::ptr right_pid_;
        PID::ptr turn_pid_;
        PID::ptr lift_pid_;
        PID::ptr lift_distance_pid_;
        PID::ptr turn_distance_pid_;
        double left_speed_without_PID_ = 0;
        double right_speed_without_PID_ = 0;
        double turn_speed_without_PID_ = 0;
        double lift_speed_without_PID_ = 0;
        void setLeftMotorSpeedWithoutPID(double speed) { left_speed_without_PID_ = speed; }

        void setRightMotorSpeedWithoutPID(double speed) { right_speed_without_PID_ = speed; }

        void setTurnMotorSpeedWithoutPID(double speed) { turn_speed_without_PID_ = speed; }

        void setLiftMotorSpeedWithoutPID(double speed) { lift_speed_without_PID_ = speed; }

        struct PidTemp {
            int32_t last_enc_counter = 0; // 用于速度PID
        };
        PidTemp left_pid_temp_;
        PidTemp right_pid_temp_;
        PidTemp turn_pid_temp_;
        PidTemp lift_pid_temp_;
        // 读取传感器信号
        static bool getStopSignal();

    public:
        bool calibrationIR(double distance, double angle_error = 0.5, double distance_error = 0.5, double left_right_e = 2,
                           double offset = 0);
        PID::ptr ir_angle_pid_;
        PID::ptr ir_distance_pid_;

        bool calibrationUS(double distance, double angle_error = 0.5, double distance_error = 0.5, double left_right_e = 2,
                           double offset = 0);
        PID::ptr us_angle_pid_;
        PID::ptr us_distance_pid_;

        bool calibrationSingleIR(double distance, double hold_phi, double angle_error, double distance_error);
        PID::ptr single_ir_angle_pid_;
        PID::ptr single_ir_distance_pid_;

    public:
        PIDParams leftPidParam;
        PIDOutputLimits leftOutputLimits;
        PIDParams rightPidParam;
        PIDOutputLimits rightOutputLimits;
        PIDParams turnPidParam;
        PIDOutputLimits turnOutputLimits;
        PIDParams liftPidParam;
        PIDOutputLimits liftOutputLimits;
        PIDParams liftDistancePidParam;
        PIDOutputLimits liftDistanceLimits;
        PIDParams turnDistancePidParam;
        PIDOutputLimits turnDistanceLimits;
        PIDParams irAnglePidParam;
        PIDOutputLimits irAngleLimits;
        PIDParams irDistancePidParam;
        PIDOutputLimits irDistanceLimits;
        PIDParams usAnglePidParam;
        PIDOutputLimits usAngleLimits;
        PIDParams usDistancePidParam;
        PIDOutputLimits usDistanceLimits;

        PIDParams singleIrAnglePidParam;
        PIDOutputLimits singleIrAngleLimits;
        PIDParams singleIrDistancePidParam;
        PIDOutputLimits singleIrDistanceLimits;

    public:
        UpdateOdom::Ptr odom = std::make_shared<UpdateOdom>();
        ChassisCtrl::Ptr chassis_ctrl = std::make_shared<ChassisCtrl>();
        LidarRead::Ptr lidar_read = std::make_shared<LidarRead>(LIDAR_INIT_ANGLE);
        LidarCalibrate::Ptr lidar_calib = std::make_shared<LidarCalibrate>();
        MovingAverageFilter::Ptr ir_right_filter = std::make_shared<MovingAverageFilter>(5);
        MovingAverageFilter::Ptr ir_left_filter = std::make_shared<MovingAverageFilter>(5);
        MovingAverageFilter::Ptr us_right_filter = std::make_shared<MovingAverageFilter>(5);
        MovingAverageFilter::Ptr us_left_filter = std::make_shared<MovingAverageFilter>(5);
    };
}

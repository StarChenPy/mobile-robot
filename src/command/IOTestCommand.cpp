#include "command/IOTestCommand.h"

//读取指令状态
void IOTestCommand::updataShareMemory() {
    LABVIEW::WriteIOShareAddress->read(writrIO);
    LABVIEW::ReadIOShareAddress->write(readIO);
}

// enc
void IOTestCommand::readENC() {
    readIO.enc.LeftENC = LeftENC->read();
    readIO.enc.RightENC = RightENC->read();
    readIO.enc.LiftENC = LiftENC->read();
    readIO.enc.TurnENC = TurnENC->read();
}
//按钮
void IOTestCommand::readButton() {
    int limit_s[4] = {0, 2, 3};
    Button->read();
    readIO.button.StartButton = Button->get(limit_s[1]);
    readIO.button.ResetButton = Button->get(limit_s[0]);
    readIO.button.StopButton = Button->get(limit_s[2]);
    readIO.button.EStopButton = StopLimit->read();
    std::cout << "Start: " << readIO.button.StartButton << std::endl;
    std::cout << "Reset: " << readIO.button.ResetButton << std::endl;
    std::cout << "Stop: " << readIO.button.StopButton << std::endl;
    std::cout << "Estoplimit: " << readIO.button.EStopButton << std::endl;
}
double IOTestCommand::IRDistance(double input) {
    return 101.012 - 166.289 * input + 141.969 * std::pow(input, 2) - 66.134 * std::pow(input, 3) +
           15.868 * std::pow(input, 4) - 1.532 * std::pow(input, 5);
}
//
void IOTestCommand::readSensor() {
    Leftus->trig();
    Rightus->trig();

    readIO.sensor.UltrasoundLeft = Leftus->echo() * 0.017;
    readIO.sensor.UltrasoundRight = Rightus->echo() * 0.017;
    readIO.sensor.IRLeft = IRDistance(IR_left->read());
    readIO.sensor.IRRight = IRDistance(IR_right->read());

    std::cout << "UltrasoundLeft: " << readIO.sensor.UltrasoundLeft << std::endl;
    std::cout << "UltrasoundRight: " << readIO.sensor.UltrasoundRight << std::endl;
    std::cout << "IR_left: " << readIO.sensor.IRLeft << std::endl;
    std::cout << "IR_right: " << readIO.sensor.IRRight << std::endl;
}
//
void IOTestCommand::readLimit() {
    bool Turn_s = TurningLimit->read() > 1.0;
    bool value_up = !LiftLimit->read();
    bool value_down = LiftDownLimit->read() > 1.0;

    readIO.limit.LiftUpLimit = value_up;
    readIO.limit.LiftDownLimit = value_down;
    readIO.limit.TurnLimit = Turn_s;
    std::cout << "LiftUp_limit: " << value_up << std::endl;
    std::cout << "LiftDown_limit: " << value_down << std::endl;
    std::cout << "Turn_limit: " << Turn_s << std::endl;
}
//
void IOTestCommand::readIMU() {
    readIO.imu_data = VMX::getYaw();
    std::cout << "IMU: " << readIO.imu_data << std::endl;
}

void IOTestCommand::readLidar() {
    static int cnt;
    if (cnt % 5 == 0) {
        double angle[360];
        double r[360];
        float intensity[360];
        std::vector<LaserPoint> datas = lidar->read();
        int i = 0;
        for (auto data : datas) {
            angle[i] = data.angle;
            r[i] = data.range;
            intensity[i] = data.intensity;
            // std::cout << "angle: " << data.angle << " distance: " <<
            // data.range << std::endl;
            i++;
            if (i >= 360) {
                break;
            }
        }
        LABVIEW::LidarAngleDataShareAddress->write(angle);
        LABVIEW::LidarRangeDataShareAddress->write(r);
        LABVIEW::LidarIntensityDataShareAddress->write(intensity);
    }
}

//
void IOTestCommand::readData() {
    readENC();
    readButton();
    readSensor();
    readLimit();
    readIMU();
    readLidar();
}

//
void IOTestCommand::ctrlMotor() {
    if (writrIO.motor.Left.start_flag == 1) {
        uint8_t speed = writrIO.motor.Left.speed;
        if (writrIO.motor.Left.dir == 0) {
            LeftMotor->setSpeedAndDir(speed, true, false);
        } else {
            LeftMotor->setSpeedAndDir(speed, false, true);
        }
        std::cout << "LeftENC: " << LeftENC->get() << std::endl;
    } else {
        LeftMotor->setSpeedAndDir(0, false, true);
    }
    if (writrIO.motor.Right.start_flag == 1) {
        uint8_t speed = writrIO.motor.Right.speed;
        if (writrIO.motor.Right.dir == 0) {
            RightMotor->setSpeedAndDir(speed, true, false);
        } else {
            RightMotor->setSpeedAndDir(speed, false, true);
        }
        std::cout << "RightENC: " << RightENC->get() << std::endl;
    } else {
        RightMotor->setSpeedAndDir(0, false, true);
    }
    if (writrIO.motor.Lift.start_flag == 1) {
        uint8_t speed = writrIO.motor.Lift.speed;
        if (writrIO.motor.Lift.dir == 0) {
            LiftMotor->setSpeedAndDir(speed, true, false);
        } else {
            LiftMotor->setSpeedAndDir(speed, false, true);
        }
        std::cout << "LiftENC: " << LiftENC->get() << std::endl;
    } else {
        LiftMotor->setSpeedAndDir(0, false, true);
    }
    if (writrIO.motor.Turn.start_flag == 1) {
        uint8_t speed = writrIO.motor.Turn.speed;
        if (writrIO.motor.Turn.dir == 0) {
            TurnMotor->setSpeedAndDir(speed, true, false);
        } else {
            TurnMotor->setSpeedAndDir(speed, false, true);
        }
        std::cout << "TurnENC: " << TurnENC->get() << std::endl;
    } else {
        TurnMotor->setSpeedAndDir(0, false, true);
    }
}
//
void IOTestCommand::ctrlServo() {
    if (writrIO.servo.Clamp.start_flag == 1) { //夹手舵机
        double val = writrIO.servo.Clamp.val;
        ClampServo->setDutyCycle(val);
        std::cout << "ClampServo Val: " << val << std::endl;
    }
    if (writrIO.servo.Raise.start_flag == 1) { // 抬手舵机
        double val = writrIO.servo.Raise.val;
        RaiseServo->setDutyCycle(val);
        std::cout << "RaiseServo Val: " << val << std::endl;
    }
    if (writrIO.servo.Telescopic.start_flag == 1) { // 伸缩舵机
        double val = writrIO.servo.Telescopic.val;
        TelescopicServo->setDutyCycle(val);
        std::cout << "TelescopicServo Val: " << val << std::endl;
    }
    if (writrIO.servo.Rotating.start_flag == 1) { // 旋转舵机
        double val = writrIO.servo.Rotating.val;
        RotatingServo->setDutyCycle(val);
        std::cout << "RotatingServo Val: " << val << std::endl;
    }
}
//
void IOTestCommand::ctrlLED() {
    if (writrIO.led.StartLED == 1) { //
        StartLed->setDutyCycle(1);
        std::cout << "StartLed" << std::endl;
    } else {
        StartLed->setDutyCycle(0);
    }
    if (writrIO.led.ResetLED == 1) { //
        ResetLed->setDutyCycle(1);
        std::cout << "ResetLed" << std::endl;
    } else {
        ResetLed->setDutyCycle(0);
    }
    if (writrIO.led.StopLED == 1) { //
        StopLed->setDutyCycle(1);
        std::cout << "StopLed" << std::endl;
    } else {
        StopLed->setDutyCycle(0);
    }
}
//
void IOTestCommand::ResetIMU() {
    if (writrIO.imu_reset == 1) { //
        zeroYaw();
        std::cout << "ResetIMU" << std::endl;
    }
}
//
void IOTestCommand::ResetENC() {
    if (writrIO.enc_reset.ResetLeftENC == 1) { //
        LeftENC->reset();
        std::cout << "LeftENC: " << LeftENC->get() << std::endl;
    }
    if (writrIO.enc_reset.ResetRightENC == 1) { //
        RightENC->reset();
        std::cout << "RightENC: " << RightENC->get() << std::endl;
    }
    if (writrIO.enc_reset.ResetLiftENC == 1) { //
        LiftENC->reset();
        std::cout << "LiftENC: " << LiftENC->get() << std::endl;
    }
    if (writrIO.enc_reset.ResetTurnENC == 1) { //
        TurnENC->reset();
        std::cout << "TurnENC: " << TurnENC->get() << std::endl;
    }
}

//
void IOTestCommand::ctrlIO() {
    ctrlMotor();
    ctrlServo();
    ctrlLED();
    ResetIMU();
    ResetENC();
}

void IOTestCommand::initialize() {
    uint8_t updata_status = COMMEND_WAIT;
    LABVIEW::TestIOStatusShareAddress->write(updata_status);

    std::cout << "IOTestCommand initialize!" << std::endl;
    is_finished = false;

    LeftMotor->setEnable();
    RightMotor->setEnable();
    LiftMotor->setEnable();
    TurnMotor->setEnable();
    Button->setEnable();
}
void IOTestCommand::execute() {
    int time = RobotGenius::getCurrentMs();
    int dt = time - m_last_time;
    m_last_time = time;

    readData();
    updataShareMemory();
    ctrlIO();

    // std::cout << "IOTestCommand execute dt = " << static_cast<double>(dt) <<
    // std::endl; is_finished = true;
}
void IOTestCommand::end() {
    std::cout << "IOTestCommand end!" << std::endl;
    uint8_t updata_status = COMMEND_END;
    LABVIEW::TestIOStatusShareAddress->write(updata_status);
}
bool IOTestCommand::isFinished() {
    uint8_t command_status;
    LABVIEW::TestIOStatusShareAddress->read(command_status);
    if (command_status == COMMEND_CANCEL) {
        is_finished = true;
    }
    // if(Robot::instance().getStopSignal()) {
    //   stopAll();
    // }
    return is_finished;
}

Command::ptr createIOTestCommand() { return std::make_shared<IOTestCommand>()->withTimer(100); }

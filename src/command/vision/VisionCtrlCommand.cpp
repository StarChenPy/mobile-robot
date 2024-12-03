#include "command/vision/VisionCtrlCommand.h"

void VisionCtrlCommand::initialize() {
    isFinished_ = false;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
    Vision::instance().clearBoxes();
    Vision::instance().clearResult();
}
void VisionCtrlCommand::execute() {
    Vision::instance().runMnn(true,
                              "/home/pi/Pick/result.jpg"); //注意路径*********
    std::vector<BoxInfo> boxs = Vision::instance().getBoxes();
    Vision::instance().print();

    double R_setpoint = 0;
    double L_setpoint = 0;
    if (!boxs.empty()) {
        int cap_cx = Vision::instance().getCapCx();
        for (auto & box : boxs) {
            if (box.label == fruit_label) {
                int fruit_cx = (box.x1 + box.x2) / 2;
                isFinished_ = Robot::getInstance().chassis_ctrl->VisionCtrlTask(fruit_cx, cap_cx);
                // Robot::getInstance().chassis_ctrl->VisionCtrlTask(fruit_cx,
                // cap_cx_);
                R_setpoint = Robot::getInstance().chassis_ctrl->get_R_setpoint();
                L_setpoint = Robot::getInstance().chassis_ctrl->get_L_setpoint();
                std::cout << "fruit class: " << Class_names[box.label] << std::endl;
                break;
            }
        }
    } else {
        std::cout << "水果丢失" << std::endl;
    }

    Robot::getInstance().setRightMotorSpeed(R_setpoint);
    Robot::getInstance().setLeftMotorSpeed(L_setpoint);
}
void VisionCtrlCommand::end() {
    std::cout << "VisionCtrlCommand end!" << std::endl;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
}

ICommand::ptr createVisionCtrlCommand(int label) { return std::make_shared<VisionCtrlCommand>(label)->withTimer(200); }

void VisionIdentifyCommand::initialize() {
    isFinished_ = false;
    Vision::instance().clearBoxes();
    Vision::instance().clearResult();
}
void VisionIdentifyCommand::execute() {
    Vision::instance().runMnn(true,
                              "/home/pi/Pick/result.jpg"); //注意路径*********
    std::vector<BoxInfo> boxs = Vision::instance().getBoxes();
    if (!boxs.empty()) {
        Vision::instance().getFruitXh(boxs);
    }
    Vision::instance().print();

    if (!boxs.empty()) {
        for (auto & box : boxs) {
            if (box.label == fruit_label) {
            }
        }
    } else {
        std::cout << "!!!No fruit: " << Class_names[fruit_label] << std::endl;
    }
}
void VisionIdentifyCommand::end() { std::cout << "VisionIdentifyCommand end!" << std::endl; }

ICommand::ptr createVisionIdentifyCommand(int label) {
    return std::make_shared<VisionIdentifyCommand>(label)->withTimer(200);
}

void VisionMoveCommand::initialize() {
    isFinished_ = false;
    InitPose = Robot::getInstance().odom->getPose();
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);

    std::vector<BoxInfo> boxs = Vision::instance().getBoxes();
    std::vector<cv::Point2f> FruitPoints = Vision::instance().getFruitXh(boxs);
    Vision::instance().print();

    double dx = 0;
    for (int i = 0; i < boxs.size(); i++) {
        if (boxs[i].label == fruit_label) {
            dx = FruitPoints[i].x;
            break;
        }
    }
    std::cout << "Fruit dx = " << dx << std::endl;
    target = {InitPose.x_ + dx, InitPose.y_, InitPose.theta_};
}
void VisionMoveCommand::execute() {
    Pose cur = Robot::getInstance().odom->getPose();
    isFinished_ = Robot::getInstance().chassis_ctrl->TrackingPointTask(target, cur);
    double R_setpoint = Robot::getInstance().chassis_ctrl->get_R_setpoint();
    double L_setpoint = Robot::getInstance().chassis_ctrl->get_L_setpoint();

    Robot::getInstance().setRightMotorSpeed(R_setpoint);
    Robot::getInstance().setLeftMotorSpeed(L_setpoint);
}
void VisionMoveCommand::end() {
    std::cout << "VisionMoveCommand end!" << std::endl;
    Robot::getInstance().setRightMotorSpeed(0);
    Robot::getInstance().setLeftMotorSpeed(0);
}

ICommand::ptr createVisionMoveCommand(int label) { return std::make_shared<VisionMoveCommand>(label)->withTimer(100); }

void VisionHeightCtrlCommand::initialize() {
    isFinished_ = false;
    Robot::getInstance().setLiftMotorSpeed(0);
    std::vector<BoxInfo> boxs = Vision::instance().getBoxes();
    std::vector<cv::Point2f> FruitPoints = Vision::instance().getFruitXh(boxs);
    Vision::instance().print();

    double dy = 0;
    for (int i = 0; i < boxs.size(); i++) {
        if (boxs[i].label == fruit_label) {
            dy = FruitPoints[i].y;
            break;
        }
    }
    std::cout << "Fruit dy = " << dy << std::endl;

    double high = 10;
    double low = 4;
    if (dy > high) {
        Height_target = -45;
    } else if (dy < high && dy > low) {
        Height_target = -52;
    } else if (dy < low) {
        Height_target = -58;
    }
    m_setpoint = static_cast<int32_t>(Height_target * (1000 / 10.7 / 2));
}
void VisionHeightCtrlCommand::execute() {
    Robot::getInstance().LiftMotorDistancePID(m_setpoint);
    std::cout << "Lift ENC: " << liftEnc->get() << " Lift set_point_: " << m_setpoint << std::endl;
}
void VisionHeightCtrlCommand::end() {
    std::cout << "VisionHeightCtrlCommand end!" << std::endl;
    Robot::getInstance().setLiftMotorSpeed(0);
}
bool VisionHeightCtrlCommand::isFinished() {
    if (abs(m_setpoint - liftEnc->get()) < LIFT_MOTOR_DISTANCE_ERROR) {
        m_conter++;
    } else {
        m_conter = 0;
    }
    return Robot::getStopSignal() || m_conter > LIFT_MOTOR_DISTANCE_COUNTER;
}

ICommand::ptr createVisionHeightCtrlCommand(int label) {
    return std::make_shared<VisionHeightCtrlCommand>(label)->withTimer(100);
}
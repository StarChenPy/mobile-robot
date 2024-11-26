#pragma once
#include "RobotGenius.h"
#include <iostream>
#include <memory>

#include <cstdio>
#include <ctime>
#include <iostream>
#include <string>

#include <MNN/Interpreter.hpp>
#include <MNN/MNNDefine.h>
#include <MNN/MNNForwardType.h>

#include "command/Vision/utils.h"

using namespace std;
using namespace RobotGenius;

class Vision {
  public:
    Vision(int index, int frame_width = 640, int frame_height = 480);
    bool saveImage(cv::Mat &cv_mat, std::string filename = "result.jpg");
    bool readFrame();
    std::vector<BoxInfo> mnnDecode(cv::Mat &cv_mat, std::shared_ptr<MNN::Interpreter> &net, MNN::Session *session,
                                   int INPUT_SIZE);
    bool mnnInit(std::string model_path, int numThread = 4);
    std::vector<BoxInfo> runMnn(bool save = false, std::string filename = "result.jpg");
    static Vision &instance(int index = 0, int frame_width = 640, int frame_height = 480);
    std::vector<BoxInfo> getBoxes() { return box_collection_; }
    void clearBoxes() { box_collection_.clear(); }
    void setResult(BoxInfo result) { mnn_result_ = result; }
    BoxInfo getResult() { return mnn_result_; }
    void clearResult() { mnn_result_ = BoxInfo(); }
    void print();
    void updateIdentifyDataShare();
    int getCapCx() const { return cap_cx_; }
    cv::Point2f getXh(double cap_x, double cap_y);

    uint8_t *imageToArray(const cv::Mat &image);
    cv::Mat arrayToImage(uint8_t *array, int rows, int cols, int channels);
    void saveImage(const std::string &filename, const cv::Mat &image);

    std::vector<cv::Point2f> getFruitXh(std::vector<BoxInfo> &boxes);

  private:
    cv::VideoCapture capture_;
    cv::Mat frame_;
    int cap_cx_ = 320 / 2;
    int cap_cy_ = 320 / 2;

    // MNN网络相关
  private:
    MNN::ScheduleConfig config_;
    MNN::Session *session_;
    cv::Mat mnn_image_;
    MatInfo mat_objection_;
    std::vector<BoxInfo> box_collection_;
    std::vector<cv::Point2f> fruit_xh_;
    std::shared_ptr<MNN::Interpreter> net_;
    BoxInfo mnn_result_;

    const char *kClassNames[50] = {"apple",        "banana",    "bell peppers", "chili pepper", "fig",
                                   "mangosteen",   "kugua",     "watermelon",   "potato",       "egg",
                                   "red egg",      "green egg", "green grape",  "stop sign",    "purple grape",
                                   "yellow grape", "lianwuguo"}; //只供打印用，改变这个数组并不会改变输出结果label
};

class IdentifyFruitCommand : public CommandBase {
  public:
    typedef std::shared_ptr<IdentifyFruitCommand> Ptr;
    void initialize() override;
    void execute() override;
    void end() override;
    bool isFinished() override;
    BoxInfo findHighestLabelWithThreshold(const std::vector<BoxInfo> &boxes, float threshold = 0.5);
    std::vector<BoxInfo> findLabelsWithThreshold(const std::vector<BoxInfo> &boxes, float threshold = 0.5);

  private:
    bool is_finished_ = false;
};

Command::ptr createIdentifyFruitCommand();
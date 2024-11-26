#pragma once
#include <iostream>
#include <memory>
#include "RobotGenius.h"


#include <iostream>
#include <string>
#include <ctime>
#include <cstdio>
#include <omp.h>


#include <MNN/MNNDefine.h>
#include <MNN/MNNForwardType.h>
#include <MNN/Interpreter.hpp>

#include "command/Vision/utils.h"

using namespace std;
using namespace RobotGenius;

class Vision {
 public:
  Vision(int index, int frame_width = 640, int frame_height = 480);
  bool saveImg(cv::Mat &cv_mat, std::string filename = "result.jpg");
  bool readFrame();
  std::vector<BoxInfo> mnn_decode(cv::Mat &cv_mat, std::shared_ptr<MNN::Interpreter> &net, MNN::Session *session, int INPUT_SIZE);
  bool MNNInit(std::string model_path,int numThread = 4);
  std::vector<BoxInfo> runMNN(bool save = false,std::string filename = "result.jpg");
  static Vision& GetInstance(int index = 0, int frame_width = 640, int frame_height = 480);
  std::vector<BoxInfo> getBoxs() { return m_bbox_collection; }
  void clearBoxs() { m_bbox_collection.clear(); }
  void setResult(BoxInfo result) { m_mnn_result = result; }
  BoxInfo getResult() { return m_mnn_result; }
  void clearResult() { m_mnn_result = BoxInfo(); }
  void print();
  void updataIdentifyDataShare();
  int get_cap_cx() {return cap_cx;}
  cv::Point2f getXH(double cap_x, double cap_y);

  uint8_t* imageToArray(const cv::Mat& image);
  cv::Mat arrayToImage(uint8_t* array, int rows, int cols, int channels);
  void saveImage(const std::string& filename, const cv::Mat& image);


  std::vector<cv::Point2f> getFruitXH(std::vector<BoxInfo> &boxs);

 private:
  cv::VideoCapture m_capture;
  cv::Mat m_frame_;
  int cap_cx = 320 / 2;
  int cap_cy = 320 / 2;

// MNN网络相关
 private:
  MNN::ScheduleConfig m_config_;
  MNN::Session *m_session_;
  cv::Mat m_mnn_image;
  MatInfo m_mmat_objection_;
  std::vector<BoxInfo> m_bbox_collection;
  std::vector<cv::Point2f> fruit_XH;
  std::shared_ptr<MNN::Interpreter> m_net_;
  BoxInfo m_mnn_result;

  const char *Class_names[50] = {
        "apple","banana","bell peppers","chili pepper","fig","mangosteen","kugua","watermelon",
        "potato","egg","red egg","green egg","green grape","stop sign","purple grape","yellow grape","lianwuguo"};   //只供打印用，改变这个数组并不会改变输出结果label
};


class IdentifyFruitCommand:public CommandBase {
 public:
  typedef std::shared_ptr<IdentifyFruitCommand> Ptr;
  void initialize() override;
  void execute()override;
  void end() override;
  bool isFinished() override;
  BoxInfo findHighestLabelWithThreshold(const std::vector<BoxInfo>& boxes, float threshold = 0.5);
  std::vector<BoxInfo> findLabelsWithThreshold(const std::vector<BoxInfo>& boxes, float threshold = 0.5);

 private:
  bool is_finished_ = false;

};


Command::Ptr createIdentifyFruitCommand();
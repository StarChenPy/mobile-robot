#include "command/Vision/VisionCommand.h"
#include "share.h"
#include "system/SysParams.h"

Vision::Vision(int index, int frame_width, int frame_height) {
  VisionCalInit();
  m_capture.open(index);
  MNNInit("/home/pi/Pick/last.mnn");      //注意路径是否正确
  // MNNInit("/../../../last.mnn");
  m_capture.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
  m_capture.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
  if (!m_capture.isOpened()) {
    std::cerr << "Error: Camera " << index << " is not opened." << std::endl;
    LABVIEW::FruitsInfo vision_err;
    vision_err.num = 0xFF;
    LABVIEW::FruitsDataShareAddress->write(vision_err);
  }
}

bool Vision::saveImg(cv::Mat &cv_mat, std::string filename) {
  if (cv_mat.empty()) {
    std::cerr << "Error: cv_mat is empty." << std::endl;
    return false;
  }
  return cv::imwrite(filename, cv_mat);
}
bool Vision::readFrame() {
  if (!m_capture.isOpened()) {
    std::cerr << "Camera is not Opened!" << std::endl;
    LABVIEW::FruitsInfo vision_err;
    vision_err.num = 0xFF;
    LABVIEW::FruitsDataShareAddress->write(vision_err);
    return false;
  }
  m_capture >> m_frame_;
  if (m_frame_.empty()) {
    return false;
  }
  return true;
}
std::vector<BoxInfo> Vision::mnn_decode(cv::Mat &cv_mat, std::shared_ptr<MNN::Interpreter> &net, MNN::Session *session, int INPUT_SIZE) {
  std::vector<int> dims{1, INPUT_SIZE, INPUT_SIZE, 3};
  auto nhwc_Tensor = MNN::Tensor::create<float>(dims, NULL, MNN::Tensor::TENSORFLOW);
  auto nhwc_data = nhwc_Tensor->host<float>();
  auto nhwc_size = nhwc_Tensor->size();
  std::memcpy(nhwc_data, cv_mat.data, nhwc_size);

  auto inputTensor = net->getSessionInput(session, nullptr);
  inputTensor->copyFromHostTensor(nhwc_Tensor);

  net->runSession(session);
  MNN::Tensor *tensor_scores = net->getSessionOutput(session, "outputs");
  MNN::Tensor tensor_scores_host(tensor_scores, tensor_scores->getDimensionType());
  tensor_scores->copyToHostTensor(&tensor_scores_host);
  auto pred_dims = tensor_scores_host.shape();

  const unsigned int num_proposals = pred_dims.at(1);
  const unsigned int num_classes = pred_dims.at(2) - 5;
  std::vector<BoxInfo> bbox_collection;

  for (unsigned int i = 0; i < num_proposals; ++i)
  {
    const float *offset_obj_cls_ptr = tensor_scores_host.host<float>() + (i * (num_classes + 5)); // row ptr
    float obj_conf = offset_obj_cls_ptr[4];
    if (obj_conf < 0.5)
      continue;

    float cls_conf = offset_obj_cls_ptr[5];
    unsigned int label = 0;
    for (unsigned int j = 0; j < num_classes; ++j)
    {
      float tmp_conf = offset_obj_cls_ptr[j + 5];
      if (tmp_conf > cls_conf)
      {
        cls_conf = tmp_conf;
        label = j;
      }
    }

    float conf = obj_conf * cls_conf; 
    if (conf < 0.5)
      continue;

    float cx = offset_obj_cls_ptr[0];
    float cy = offset_obj_cls_ptr[1];
    float w = offset_obj_cls_ptr[2];
    float h = offset_obj_cls_ptr[3];

    float x1 = (cx - w / 2.f);
    float y1 = (cy - h / 2.f);
    float x2 = (cx + w / 2.f);
    float y2 = (cy + h / 2.f);

    BoxInfo box;
    box.x1 = std::max(0.f, x1);
    box.y1 = std::max(0.f, y1);
    box.x2 = std::min(x2, (float)INPUT_SIZE - 1.f);
    box.y2 = std::min(y2, (float)INPUT_SIZE - 1.f);
    box.score = conf;
    box.label = label;
    bbox_collection.push_back(box);
  }

  delete nhwc_Tensor;
  return bbox_collection;
}

Vision& Vision::GetInstance(int index, int frame_width, int frame_height) {
  static Vision instance(index, frame_width, frame_height);
  return instance;
}

bool Vision::MNNInit(std::string model_path, int numThread) {
  m_net_ = std::shared_ptr<MNN::Interpreter>(MNN::Interpreter::createFromFile(model_path.c_str()));
  if (nullptr == m_net_) {
    return false;
  }
  m_config_.numThread = numThread;
  m_config_.type = static_cast<MNNForwardType>(MNN_FORWARD_CPU);
  MNN::BackendConfig backendConfig;
  // backendConfig.precision = (MNN::BackendConfig::PrecisionMode)1;
  backendConfig.precision = MNN::BackendConfig::Precision_Low_BF16;
  m_config_.backendConfig = &backendConfig;
  m_session_ = m_net_->createSession(m_config_);
  
  m_mmat_objection_.inpSize = 320;
  return true;
}

std::vector<BoxInfo> Vision::runMNN(bool save,std::string filename) {
  m_bbox_collection.clear();
  struct timespec begin, end;
  long time;
  clock_gettime(CLOCK_MONOTONIC, &begin);
  if (!readFrame()) {
    return m_bbox_collection;
  }
  cv::Mat m_mnn_image = m_frame_.clone();
  // saveImg(m_mnn_image, filename);
  cv::Mat pimg = preprocess(m_mnn_image, m_mmat_objection_);


  // std::cout << "image cols: " << m_mnn_image.cols << " image rows: " << m_mnn_image.rows << " image channels: "  << m_mnn_image.channels()<< std::endl;
  
  uint8_t* imageArray = imageToArray(m_mnn_image);
  if (imageArray == nullptr) {
    std::cerr << "数组分配失败！" << std::endl;
  }else{
    LABVIEW::ImageDataShareAddress->write(imageArray);
    // 将数组还原为图像
    // uint8_t* imageOut = new uint8_t[480 * 640 * 3];
    // LABVIEW::ImageDataShareAddress->read(imageOut);
    // cv::Mat restoredImage = arrayToImage(imageOut, 480, 640, 3);
    // saveImage("imageOut.jpg", restoredImage);
    
    // cv::Mat restoredImage = arrayToImage(imageArray, m_mnn_image.rows, m_mnn_image.cols, m_mnn_image.channels());
    // // 保存还原的图像
    // saveImage("restored_image.jpg", restoredImage);
  }
  
  
  m_bbox_collection = mnn_decode(pimg, m_net_, m_session_, m_mmat_objection_.inpSize);
  if(!m_bbox_collection.empty()){
    nms(m_bbox_collection, 0.50);
    draw_box(m_mnn_image, m_bbox_collection, m_mmat_objection_);
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  time = (end.tv_sec - begin.tv_sec) + (end.tv_nsec - begin.tv_nsec);
  if(time > 0) printf(">> Time : %lf ms\n", (double)time / 1000000);
  if (save && !m_bbox_collection.empty()) {
    saveImg(m_mnn_image, filename);
  }
  return m_bbox_collection;
}

std::vector<cv::Point2f> Vision::getFruitXH(std::vector<BoxInfo> &boxs){
  fruit_XH.clear();
  // 从文件中读取 homography 矩阵
  // cv::FileStorage fs("homography.xml", cv::FileStorage::READ);
  // cv::Mat homography;
  // fs["homography"] >> homography;
  // fs.release();

  // 图像中的四个点（像素坐标）
  std::vector<cv::Point2f> imagePoints;
  imagePoints.push_back(cv::Point2f(ImgCalData.Image.P1.x, ImgCalData.Image.P1.y));
  imagePoints.push_back(cv::Point2f(ImgCalData.Image.P2.x, ImgCalData.Image.P2.y));
  imagePoints.push_back(cv::Point2f(ImgCalData.Image.P3.x, ImgCalData.Image.P3.y));
  imagePoints.push_back(cv::Point2f(ImgCalData.Image.P4.x, ImgCalData.Image.P4.y));

  // 这些点在实际世界中的对应坐标
  std::vector<cv::Point2f> objectPoints;
  objectPoints.push_back(cv::Point2f(ImgCalData.Object.P1.x, ImgCalData.Object.P1.y));
  objectPoints.push_back(cv::Point2f(ImgCalData.Object.P2.x, ImgCalData.Object.P2.y));  //对应实际坐标
  objectPoints.push_back(cv::Point2f(ImgCalData.Object.P3.x, ImgCalData.Object.P3.y));
  objectPoints.push_back(cv::Point2f(ImgCalData.Object.P4.x, ImgCalData.Object.P4.y));
  // std::vector<cv::Point2f> imagePoints;
  // imagePoints.push_back(cv::Point2f(55.375, 140));
  // imagePoints.push_back(cv::Point2f(264, 137));
  // imagePoints.push_back(cv::Point2f(241, 235));
  // imagePoints.push_back(cv::Point2f(88, 236));

  // // 这些点在实际世界中的对应坐标
  // std::vector<cv::Point2f> objectPoints;
  // objectPoints.push_back(cv::Point2f(7.0f, 14.0f));
  // objectPoints.push_back(cv::Point2f(-10.0f, 14.0f));  //对应实际坐标
  // objectPoints.push_back(cv::Point2f(10.0f, 0.0f));
  // objectPoints.push_back(cv::Point2f(7.0f, 0.0f));

  // 计算单应性矩阵
  cv::Mat homography = cv::findHomography(imagePoints, objectPoints);

  std::vector<cv::Point2f> newImagePoints;
  for(int i = 0; i < boxs.size(); i++){
    int cx = (boxs[i].x1 + boxs[i].x2) / 2;
    int cy = (boxs[i].y1 + boxs[i].y2) / 2;
    newImagePoints.push_back(cv::Point2f(cx, cy));  // 新的图像坐标点
  }
  if(!newImagePoints.empty()){
    cv::perspectiveTransform(newImagePoints, fruit_XH, homography);
  }
  return fruit_XH;
}

void Vision::print(){
  std::vector<BoxInfo> boxs = getBoxs();
  std::vector<cv::Point2f> real = getFruitXH(boxs);
  for(int i = 0; i < boxs.size(); i++){
    double cx = (boxs[i].x1 + boxs[i].x2) / 2;
    double cy = (boxs[i].y1 + boxs[i].y2) / 2;
    std::cout << "i = " << i << " class: " << Class_names[boxs[i].label] << " score: " << boxs[i].score << " cx = " << cx << " cy = " << cy << std::endl;
    std::cout << "real_X = " << real[i].x << " real_H = " << real[i].y << std::endl;
  }
}

void Vision::updataIdentifyDataShare(){
  std::vector<BoxInfo> boxs = getBoxs();
  std::vector<cv::Point2f> real = getFruitXH(boxs);
  int index = 0;
  float s = 0.0;
  LABVIEW::IdentifyInfo info;
  LABVIEW::FruitsInfo fruits;
  
  fruits.num = boxs.size();
  for(int i = 0; i < boxs.size(); i++){
    if(boxs[i].score > s){
      s = boxs[i].score;
      index = i;
    }
    if(i == 0){
      // fruits.fruits_1(boxs[i],real[i]);
      fruits.fruits_1.box_info.x1 = boxs[i].x1;
      fruits.fruits_1.box_info.y1 = boxs[i].y1;
      fruits.fruits_1.box_info.x2 = boxs[i].x2;
      fruits.fruits_1.box_info.y2 = boxs[i].y2;
      fruits.fruits_1.box_info.score = boxs[i].score;
      fruits.fruits_1.box_info.label = boxs[i].label;
      fruits.fruits_1.XH.X = real[i].x;
      fruits.fruits_1.XH.H = real[i].y;
    }else if(i == 1){
      fruits.fruits_2.box_info.x1 = boxs[i].x1;
      fruits.fruits_2.box_info.y1 = boxs[i].y1;
      fruits.fruits_2.box_info.x2 = boxs[i].x2;
      fruits.fruits_2.box_info.y2 = boxs[i].y2;
      fruits.fruits_2.box_info.score = boxs[i].score;
      fruits.fruits_2.box_info.label = boxs[i].label;
      fruits.fruits_2.XH.X = real[i].x;
      fruits.fruits_2.XH.H = real[i].y;
    }else if(i == 2){
      fruits.fruits_3.box_info.x1 = boxs[i].x1;
      fruits.fruits_3.box_info.y1 = boxs[i].y1;
      fruits.fruits_3.box_info.x2 = boxs[i].x2;
      fruits.fruits_3.box_info.y2 = boxs[i].y2;
      fruits.fruits_3.box_info.score = boxs[i].score;
      fruits.fruits_3.box_info.label = boxs[i].label;
      fruits.fruits_3.XH.X = real[i].x;
      fruits.fruits_3.XH.H = real[i].y;
    }else if(i == 3){
      fruits.fruits_4.box_info.x1 = boxs[i].x1;
      fruits.fruits_4.box_info.y1 = boxs[i].y1;
      fruits.fruits_4.box_info.x2 = boxs[i].x2;
      fruits.fruits_4.box_info.y2 = boxs[i].y2;
      fruits.fruits_4.box_info.score = boxs[i].score;
      fruits.fruits_4.box_info.label = boxs[i].label;
      fruits.fruits_4.XH.X = real[i].x;
      fruits.fruits_4.XH.H = real[i].y;
    }else if(i == 4){
      fruits.fruits_5.box_info.x1 = boxs[i].x1;
      fruits.fruits_5.box_info.y1 = boxs[i].y1;
      fruits.fruits_5.box_info.x2 = boxs[i].x2;
      fruits.fruits_5.box_info.y2 = boxs[i].y2;
      fruits.fruits_5.box_info.score = boxs[i].score;
      fruits.fruits_5.box_info.label = boxs[i].label;
      fruits.fruits_5.XH.X = real[i].x;
      fruits.fruits_5.XH.H = real[i].y;
    }
  }
  LABVIEW::FruitsDataShareAddress->write(fruits);
  
  info.box_info.x1 = boxs[index].x1;
  info.box_info.y1 = boxs[index].y1;
  info.box_info.x2 = boxs[index].x2;
  info.box_info.y2 = boxs[index].y2;
  info.box_info.score = boxs[index].score;
  info.box_info.label = boxs[index].label;
  info.XH.X = real[index].x;
  info.XH.H = real[index].y;
  LABVIEW::IdentifyDataShareAddress->write(info);
  
}


//测试
cv::Point2f Vision::getXH(double cap_x, double cap_y){
  // 从文件中读取 homography 矩阵
  cv::FileStorage fs("homography.xml", cv::FileStorage::READ);
  cv::Mat homography;
  fs["homography"] >> homography;
  fs.release();

  // 透射变换应用到新的点上
  std::vector<cv::Point2f> newImagePoints;
  newImagePoints.push_back(cv::Point2f(cap_x, cap_y));  // 新的图像坐标点

  std::vector<cv::Point2f> transformedPoints;
  cv::perspectiveTransform(newImagePoints, transformedPoints, homography);

  return transformedPoints[0];
}


// 将图像转换为数组
uint8_t* Vision::imageToArray(const cv::Mat& image) {
    int rows = image.rows;
    int cols = image.cols;
    int channels = image.channels();
    uint8_t* imageArray = new uint8_t[rows * cols * channels];
    std::memcpy(imageArray, image.data, rows * cols * channels * sizeof(uint8_t));
    return imageArray;
}

// 将数组还原为图像
cv::Mat Vision::arrayToImage(uint8_t* array, int rows, int cols, int channels) {
    return cv::Mat(rows, cols, channels == 3 ? CV_8UC3 : CV_8UC1, array).clone();
}

// 保存图像
void Vision::saveImage(const std::string& filename, const cv::Mat& image) {
    if (!cv::imwrite(filename, image)) {
        std::cerr << "保存图像失败！" << std::endl;
    } else {
        // std::cout << "图像已保存为: " << filename << std::endl;
    }
}




void IdentifyFruitCommand::initialize() {
  uint8_t updata_status = COMMEND_WAIT;
  LABVIEW::IdentifyStatusShareAddress->write(updata_status);

  Vision::GetInstance().clearBoxs();
  Vision::GetInstance().clearResult();
}
void IdentifyFruitCommand::execute() {
  static int lable_last = 0;
  static int counter = 0;
  Vision::GetInstance().runMNN(true, "/home/pi/Pick/result.jpg");      //注意路径*********
  if (Vision::GetInstance().getBoxs().empty()) return;
  BoxInfo result = findHighestLabelWithThreshold(Vision::GetInstance().getBoxs());
  if (result.label == lable_last) {
    // std::cout << "fruit class: " << Class_names[result[i].label] << std::endl;
    counter++;
  } else {
    counter = 0;
  }
  lable_last = result.label;
  if (counter > 0) {
    is_finished_ = true;
    Vision::GetInstance().setResult(result);
  }

  // 打印中心坐标
  Vision::GetInstance().print();
  Vision::GetInstance().updataIdentifyDataShare();

}
void IdentifyFruitCommand::end() {
  std::cout << "IdentifyFruitCommand end!!!" <<std::endl;
  uint8_t updata_status = COMMEND_END;
  LABVIEW::IdentifyStatusShareAddress->write(updata_status);
}
bool IdentifyFruitCommand::isFinished() {
  uint8_t command_status;
  LABVIEW::IdentifyStatusShareAddress->read(command_status);
  if(command_status == COMMEND_CANCEL){
    is_finished_ = true;
  }
  return is_finished_;
}

BoxInfo IdentifyFruitCommand::findHighestLabelWithThreshold(const std::vector<BoxInfo>& boxes, float threshold) {
  BoxInfo result;
  int maxLabel = std::numeric_limits<int>::min();  // 使用最小的整数初始化
  // bool found = false;

  for (const auto& box : boxes) {
    if (box.score > threshold && box.label > maxLabel) {
      maxLabel = box.label;
      result = box;
      // found = true;
    }
  }
  return result;
}

std::vector<BoxInfo> IdentifyFruitCommand::findLabelsWithThreshold(const std::vector<BoxInfo>& boxes, float threshold) {
  std::vector<BoxInfo> result;
  for (const auto& box : boxes) {
    if (box.score > threshold) {
      result.push_back(box);
    }
  }
  return result;
}


Command::Ptr createIdentifyFruitCommand(){
  return std::make_shared<IdentifyFruitCommand>()->withTimer(200);
}
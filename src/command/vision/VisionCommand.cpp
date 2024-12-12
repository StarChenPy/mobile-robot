#include "command/vision/VisionCommand.h"
#include "system/SysParams.h"

Vision::Vision(int index, int frame_width, int frame_height) {
    VisionCalInit();
    capture_.open(index);
    mnnInit("/home/pi/Pick/last.mnn"); //注意路径是否正确
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    if (!capture_.isOpened()) {
        std::cerr << "Error: Camera " << index << " is not opened." << std::endl;
    }
}

bool Vision::saveImage(cv::Mat &cv_mat, std::string filename) {
    if (cv_mat.empty()) {
        std::cerr << "Error: cv_mat is empty." << std::endl;
        return false;
    }
    return cv::imwrite(filename, cv_mat);
}
bool Vision::readFrame() {
    if (!capture_.isOpened()) {
        std::cerr << "Camera is not Opened!" << std::endl;
        return false;
    }
    capture_ >> frame_;
    if (frame_.empty()) {
        return false;
    }
    return true;
}
std::vector<BoxInfo> Vision::mnnDecode(cv::Mat &cv_mat, std::shared_ptr<MNN::Interpreter> &net, MNN::Session *session,
                                       int INPUT_SIZE) {
    std::vector<int> dims{1, INPUT_SIZE, INPUT_SIZE, 3};
    auto nhwc_Tensor = MNN::Tensor::create<float>(dims, nullptr, MNN::Tensor::TENSORFLOW);
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

    for (unsigned int i = 0; i < num_proposals; ++i) {
        const float *offset_obj_cls_ptr = tensor_scores_host.host<float>() + (i * (num_classes + 5)); // row ptr
        float obj_conf = offset_obj_cls_ptr[4];
        if (obj_conf < 0.5)
            continue;

        float cls_conf = offset_obj_cls_ptr[5];
        unsigned int label = 0;
        for (unsigned int j = 0; j < num_classes; ++j) {
            float tmp_conf = offset_obj_cls_ptr[j + 5];
            if (tmp_conf > cls_conf) {
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

Vision &Vision::instance(int index, int frame_width, int frame_height) {
    static Vision instance(index, frame_width, frame_height);
    return instance;
}

bool Vision::mnnInit(std::string model_path, int numThread) {
    net_ = std::shared_ptr<MNN::Interpreter>(MNN::Interpreter::createFromFile(model_path.c_str()));
    if (nullptr == net_) {
        return false;
    }
    config_.numThread = numThread;
    config_.type = static_cast<MNNForwardType>(MNN_FORWARD_CPU);
    MNN::BackendConfig backendConfig;
    // backendConfig.precision = (MNN::BackendConfig::PrecisionMode)1;
    backendConfig.precision = MNN::BackendConfig::Precision_Low_BF16;
    config_.backendConfig = &backendConfig;
    session_ = net_->createSession(config_);

    mat_objection_.inpSize = 320;
    return true;
}

std::vector<BoxInfo> Vision::runMnn(bool save, std::string filename) {
    box_collection_.clear();
    struct timespec begin, end;
    long time;
    clock_gettime(CLOCK_MONOTONIC, &begin);
    if (!readFrame()) {
        return box_collection_;
    }
    cv::Mat m_mnn_image = frame_.clone();
    // saveImage(mnn_image_, filename);
    cv::Mat pimg = preprocess(m_mnn_image, mat_objection_);

    box_collection_ = mnnDecode(pimg, net_, session_, mat_objection_.inpSize);
    if (!box_collection_.empty()) {
        nms(box_collection_, 0.50);
        draw_box(m_mnn_image, box_collection_, mat_objection_);
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    time = (end.tv_sec - begin.tv_sec) + (end.tv_nsec - begin.tv_nsec);
    if (time > 0)
        printf(">> Time : %lf ms\n", (double)time / 1000000);
    if (save && !box_collection_.empty()) {
        saveImage(m_mnn_image, filename);
    }
    return box_collection_;
}

std::vector<cv::Point2f> Vision::getFruitXh(std::vector<BoxInfo> &boxes) {
    fruit_xh_.clear();

    // 图像中的四个点（像素坐标）
    std::vector<cv::Point2f> imagePoints;
    imagePoints.emplace_back(ImgCalData.Image.P1.x, ImgCalData.Image.P1.y);
    imagePoints.emplace_back(ImgCalData.Image.P2.x, ImgCalData.Image.P2.y);
    imagePoints.emplace_back(ImgCalData.Image.P3.x, ImgCalData.Image.P3.y);
    imagePoints.emplace_back(ImgCalData.Image.P4.x, ImgCalData.Image.P4.y);

    // 这些点在实际世界中的对应坐标
    std::vector<cv::Point2f> objectPoints;
    objectPoints.emplace_back(ImgCalData.Object.P1.x, ImgCalData.Object.P1.y);
    objectPoints.emplace_back(ImgCalData.Object.P2.x, ImgCalData.Object.P2.y); //对应实际坐标
    objectPoints.emplace_back(ImgCalData.Object.P3.x, ImgCalData.Object.P3.y);
    objectPoints.emplace_back(ImgCalData.Object.P4.x, ImgCalData.Object.P4.y);

    // 计算单应性矩阵
    cv::Mat homography = cv::findHomography(imagePoints, objectPoints);

    std::vector<cv::Point2f> newImagePoints;
    for (int i = 0; i < boxes.size(); i++) {
        int cx = (boxes[i].x1 + boxes[i].x2) / 2;
        int cy = (boxes[i].y1 + boxes[i].y2) / 2;
        newImagePoints.push_back(cv::Point2f(cx, cy)); // 新的图像坐标点
    }
    if (!newImagePoints.empty()) {
        cv::perspectiveTransform(newImagePoints, fruit_xh_, homography);
    }
    return fruit_xh_;
}

void Vision::print() {
    std::vector<BoxInfo> boxs = getBoxes();
    std::vector<cv::Point2f> real = getFruitXh(boxs);
    for (int i = 0; i < boxs.size(); i++) {
        double cx = (boxs[i].x1 + boxs[i].x2) / 2;
        double cy = (boxs[i].y1 + boxs[i].y2) / 2;
        std::cout << "i = " << i << " class: " << kClassNames[boxs[i].label] << " score: " << boxs[i].score
                  << " cx = " << cx << " cy = " << cy << std::endl;
        std::cout << "real_X = " << real[i].x << " real_H = " << real[i].y << std::endl;
    }
}

//测试
cv::Point2f Vision::getXh(double cap_x, double cap_y) {
    // 从文件中读取 homography 矩阵
    cv::FileStorage fs("homography.xml", cv::FileStorage::READ);
    cv::Mat homography;
    fs["homography"] >> homography;
    fs.release();

    // 透射变换应用到新的点上
    std::vector<cv::Point2f> newImagePoints;
    newImagePoints.push_back(cv::Point2f(cap_x, cap_y)); // 新的图像坐标点

    std::vector<cv::Point2f> transformedPoints;
    cv::perspectiveTransform(newImagePoints, transformedPoints, homography);

    return transformedPoints[0];
}

// 将图像转换为数组
uint8_t *Vision::imageToArray(const cv::Mat &image) {
    int rows = image.rows;
    int cols = image.cols;
    int channels = image.channels();
    uint8_t *imageArray = new uint8_t[rows * cols * channels];
    std::memcpy(imageArray, image.data, rows * cols * channels * sizeof(uint8_t));
    return imageArray;
}

// 将数组还原为图像
cv::Mat Vision::arrayToImage(uint8_t *array, int rows, int cols, int channels) {
    return cv::Mat(rows, cols, channels == 3 ? CV_8UC3 : CV_8UC1, array).clone();
}

// 保存图像
void Vision::saveImage(const std::string &filename, const cv::Mat &image) {
    if (!cv::imwrite(filename, image)) {
        std::cerr << "保存图像失败！" << std::endl;
    } else {
        // std::cout << "图像已保存为: " << filename << std::endl;
    }
}

void IdentifyFruitCommand::initialize() {
    Vision::instance().clearBoxes();
    Vision::instance().clearResult();
}
void IdentifyFruitCommand::execute() {
    static int label_last = 0;
    static int counter = 0;
    Vision::instance().runMnn(true,
                              "/home/pi/Pick/result.jpg"); //注意路径*********
    if (Vision::instance().getBoxes().empty())
        return;
    BoxInfo result = findHighestLabelWithThreshold(Vision::instance().getBoxes());
    if (result.label == label_last) {
        counter++;
    } else {
        counter = 0;
    }
    label_last = result.label;
    if (counter > 0) {
        isFinished_ = true;
        Vision::instance().setResult(result);
    }

    // 打印中心坐标
    Vision::instance().print();
}
void IdentifyFruitCommand::end() {
    std::cout << "IdentifyFruitCommand end!!!" << std::endl;
}
bool IdentifyFruitCommand::isFinished() {
    return isFinished_;
}

BoxInfo IdentifyFruitCommand::findHighestLabelWithThreshold(const std::vector<BoxInfo> &boxes, float threshold) {
    BoxInfo result;
    int maxLabel = std::numeric_limits<int>::min(); // 使用最小的整数初始化

    for (const auto &box : boxes) {
        if (box.score > threshold && box.label > maxLabel) {
            maxLabel = box.label;
            result = box;
            // found = true;
        }
    }
    return result;
}

std::vector<BoxInfo> IdentifyFruitCommand::findLabelsWithThreshold(const std::vector<BoxInfo> &boxes, float threshold) {
    std::vector<BoxInfo> result;
    for (const auto &box : boxes) {
        if (box.score > threshold) {
            result.push_back(box);
        }
    }
    return result;
}

ICommand::ptr createIdentifyFruitCommand() { return std::make_shared<IdentifyFruitCommand>()->withTimer(200); }
#include "command/Vision/utils.h"

#define x86_64 0
#define use_camera 0

using namespace MNN;

// // 将图像转换为数组
// uint8_t* imageToArray(const cv::Mat& image) {
//     int rows = image.rows;
//     int cols = image.cols;
//     int channels = image.channels();
//     uint8_t* imageArray = new uint8_t[rows * cols * channels];
//     std::memcpy(imageArray, image.data, rows * cols * channels * sizeof(uint8_t));
//     return imageArray;
// }

// // 将数组还原为图像
// cv::Mat arrayToImage(uint8_t* array, int rows, int cols, int channels) {
//     return cv::Mat(rows, cols, channels == 3 ? CV_8UC3 : CV_8UC1, array).clone();
// }

// // 保存图像
// void saveImage(const std::string& filename, const cv::Mat& image) {
//     if (!cv::imwrite(filename, image)) {
//         std::cerr << "保存图像失败！" << std::endl;
//     } else {
//         std::cout << "图像已保存为: " << filename << std::endl;
//     }
// }


cv::Mat preprocess(cv::Mat &cv_mat, MatInfo &mmat_objection)
{
    cv::Mat img, dstimg;
    cv::cvtColor(cv_mat, dstimg, cv::COLOR_BGR2RGB);
    mmat_objection.maxSide = cv_mat.rows > cv_mat.cols ? cv_mat.rows : cv_mat.cols;
    mmat_objection.ratio = float(mmat_objection.inpSize) / float(mmat_objection.maxSide);
    int fx = int(cv_mat.cols * mmat_objection.ratio);
    int fy = int(cv_mat.rows * mmat_objection.ratio);
    mmat_objection.Padw = int((mmat_objection.inpSize - fx) * 0.5);
    mmat_objection.Padh = int((mmat_objection.inpSize - fy) * 0.5);
    cv::resize(dstimg, img, cv::Size(fx, fy));
    cv::copyMakeBorder(img, img, mmat_objection.Padh, mmat_objection.Padh, mmat_objection.Padw, mmat_objection.Padw, cv::BORDER_CONSTANT, cv::Scalar::all(127));

    img.convertTo(img, CV_32FC3);
    img = img / 255.0f;
    return img;
}

// void nms(std::vector<BoxInfo> &input_boxes, float NMS_THRESH)
// {
//     std::sort(input_boxes.begin(), input_boxes.end(), [](BoxInfo a, BoxInfo b)
//               { return a.score > b.score; });
//     std::vector<float> vArea(input_boxes.size());
//     for (int i = 0; i < int(input_boxes.size()); ++i)
//     {
//         vArea[i] = (input_boxes.at(i).x2 - input_boxes.at(i).x1 + 1) * (input_boxes.at(i).y2 - input_boxes.at(i).y1 + 1);
//     }
// #pragma omp parallel num_threads(2)
// #pragma omp parallel for
//     for (int i = 0; i < int(input_boxes.size()); ++i)
//     {
//         for (int j = i + 1; j < int(input_boxes.size());)
//         {
//             float xx1 = std::max(input_boxes[i].x1, input_boxes[j].x1);
//             float yy1 = std::max(input_boxes[i].y1, input_boxes[j].y1);
//             float xx2 = std::min(input_boxes[i].x2, input_boxes[j].x2);
//             float yy2 = std::min(input_boxes[i].y2, input_boxes[j].y2);
//             float w = std::max(float(0), xx2 - xx1 + 1);
//             float h = std::max(float(0), yy2 - yy1 + 1);
//             float inter = w * h;
//             float ovr = inter / (vArea[i] + vArea[j] - inter);
//             if (ovr >= NMS_THRESH)
//             {
//                 if((input_boxes.begin() + j) == input_boxes.end()) {
//                     j++;
//                     continue;
//                 }
//                 input_boxes.erase(input_boxes.begin() + j);
//                 vArea.erase(vArea.begin() + j);
//             }
//             else
//                 j++;
//         }
//     }
//     vArea.clear();
// }

void nms(std::vector<BoxInfo> &input_boxes, float NMS_THRESH)
{
    std::sort(input_boxes.begin(), input_boxes.end(), [](BoxInfo a, BoxInfo b)
              { return a.score > b.score; });
    std::vector<float> vArea(input_boxes.size());
    for (int i = 0; i < int(input_boxes.size()); ++i)
    {
        vArea[i] = (input_boxes.at(i).x2 - input_boxes.at(i).x1 + 1) * (input_boxes.at(i).y2 - input_boxes.at(i).y1 + 1);
    }

    std::vector<bool> to_delete(input_boxes.size(), false);

#pragma omp parallel for
    for (int i = 0; i < int(input_boxes.size()); ++i)
    {
        for (int j = i + 1; j < int(input_boxes.size()); ++j)
        {
            if (to_delete[j])
                continue;

            float xx1 = std::max(input_boxes[i].x1, input_boxes[j].x1);
            float yy1 = std::max(input_boxes[i].y1, input_boxes[j].y1);
            float xx2 = std::min(input_boxes[i].x2, input_boxes[j].x2);
            float yy2 = std::min(input_boxes[i].y2, input_boxes[j].y2);
            float w = std::max(float(0), xx2 - xx1 + 1);
            float h = std::max(float(0), yy2 - yy1 + 1);
            float inter = w * h;
            float ovr = inter / (vArea[i] + vArea[j] - inter);
            if (ovr >= NMS_THRESH)
            {
                to_delete[j] = true;
            }
        }
    }

    // 删除标记为删除的元素
    for (int i = int(input_boxes.size()) - 1; i >= 0; --i)
    {
        if (to_delete[i])
        {
            input_boxes.erase(input_boxes.begin() + i);
            vArea.erase(vArea.begin() + i);
        }
    }
    vArea.clear();
    // vArea.resize(vArea.size());
}

void draw_box(cv::Mat &cv_mat, std::vector<BoxInfo> &boxes, MatInfo &mmat_objection)
{
    static const char *class_names[] = {
        "apple","banana","bell peppers","chili pepper","fig","mangosteen","kugua","watermelon","potato",
        "egg","red egg","green egg","green grape","stop sign","purple grape","yellow grape","lianwuguo" };

    char text[256];
    int x1, y1, objw, objh;
    for (auto box : boxes)
    {
        x1 = int(box.x1 / mmat_objection.ratio) - int(mmat_objection.Padw / mmat_objection.ratio);
        y1 = int(box.y1 / mmat_objection.ratio) - int(mmat_objection.Padh / mmat_objection.ratio);
        objw = int((box.x2 - box.x1) / mmat_objection.ratio);
        objh = int((box.y2 - box.y1) / mmat_objection.ratio);
        cv::Point pos = cv::Point(x1, y1 - 5);
        // printf("%f, %f, %f, %f, %f\n", box.x1, box.y1, box.x2, box.y2, box.score);
        cv::Rect rect = cv::Rect(x1, y1, objw, objh);
        cv::rectangle(cv_mat, rect, cv::Scalar(0, 255, 0));
        // std::cout << box.label << std::endl;
        sprintf(text, "%s %.1f%%", class_names[box.label], box.score * 100);
        cv::putText(cv_mat, text, pos, cv::FONT_HERSHEY_SIMPLEX, (float)objh *10 / (float)mmat_objection.maxSide,
                    cv::Scalar(0, 0, 255), 1);
    }
// #if use_camera
//     cv::imshow("Fourcc", cv_mat);
//     cv::waitKey(1);
// #else
    cv::imwrite("result.jpg", cv_mat);
// #endif
}

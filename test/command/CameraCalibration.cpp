#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main() {
    // 图像中的四个点（像素坐标）
    std::vector<cv::Point2f> imagePoints;
    imagePoints.push_back(cv::Point2f(55.375, 140));
    imagePoints.push_back(cv::Point2f(264, 137));
    imagePoints.push_back(cv::Point2f(241, 235));
    imagePoints.push_back(cv::Point2f(88, 236));

    // 这些点在实际世界中的对应坐标
    std::vector<cv::Point2f> objectPoints;
    objectPoints.push_back(cv::Point2f(7.0f, 14.0f));
    objectPoints.push_back(cv::Point2f(-10.0f, 14.0f));  // 对应实际坐标 (1, 0)
    objectPoints.push_back(cv::Point2f(10.0f, 0.0f));  // 对应实际坐标 (0, 1)
    objectPoints.push_back(cv::Point2f(7.0f, 0.0f));  // 对应实际坐标 (1, 1)

    // 计算单应性矩阵
    cv::Mat homography = cv::findHomography(imagePoints, objectPoints);
    
    // 将 homography 矩阵保存到文件
    cv::FileStorage fs("homography.xml", cv::FileStorage::WRITE);
    fs << "homography" << homography;
    fs.release();

    // 从文件中读取 homography 矩阵
    // cv::FileStorage fs("homography.xml", cv::FileStorage::READ);
    // cv::Mat homography;
    // fs["homography"] >> homography;
    // fs.release();

    // 透射变换应用到新的点上
    std::vector<cv::Point2f> newImagePoints;
    newImagePoints.push_back(cv::Point2f(150, 150));  // 新的图像坐标点

    std::vector<cv::Point2f> transformedPoints;
    cv::perspectiveTransform(newImagePoints, transformedPoints, homography);

    // 输出结果
    for (size_t i = 0; i < newImagePoints.size(); i++) {
        std::cout << "图像点: " << newImagePoints[i] << " 对应的世界坐标: " << transformedPoints[i] << std::endl;
    }

    return 0;
}


// // 将 homography 矩阵保存到文件
// cv::FileStorage fs("homography.xml", cv::FileStorage::WRITE);
// fs << "homography" << homography;
// fs.release();

// // 从文件中读取 homography 矩阵
// cv::FileStorage fs("homography.xml", cv::FileStorage::READ);
// cv::Mat homography;
// fs["homography"] >> homography;
// fs.release();


// void saveHomography(const cv::Mat& homography) {
//     cv::FileStorage fs("homography.xml", cv::FileStorage::WRITE);
//     fs << "homography" << homography;
//     fs.release();
// }

// cv::Mat loadHomography() {
//     cv::Mat homography;
//     cv::FileStorage fs("homography.xml", cv::FileStorage::READ);
//     fs["homography"] >> homography;
//     fs.release();
//     return homography;
// }
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>



// 将图像转换为数组
uint8_t* imageToArray(const cv::Mat& image) {
    int rows = image.rows;
    int cols = image.cols;
    int channels = image.channels();
    uint8_t* imageArray = new uint8_t[rows * cols * channels];
    std::memcpy(imageArray, image.data, rows * cols * channels * sizeof(uint8_t));
    return imageArray;
}

// 将数组还原为图像
cv::Mat arrayToImage(uint8_t* array, int rows, int cols, int channels) {
    return cv::Mat(rows, cols, channels == 3 ? CV_8UC3 : CV_8UC1, array).clone();
}

// 保存图像
void saveImage(const std::string& filename, const cv::Mat& image) {
    if (!cv::imwrite(filename, image)) {
        std::cerr << "保存图像失败！" << std::endl;
    } else {
        std::cout << "图像已保存为: " << filename << std::endl;
    }
}

int main() {
    // 读取图像
    cv::Mat image = cv::imread("/home/pi/Pick/result.jpg", cv::IMREAD_COLOR);
    
    // 检查图像是否成功加载
    if (image.empty()) {
        std::cerr << "无法加载图像！请检查路径。" << std::endl;
        return -1;
    }

    // 将图像转换为数组
    uint8_t* imageArray = imageToArray(image);
    if (imageArray == nullptr) {
        std::cerr << "数组分配失败！" << std::endl;
        return -1;
    }


    // 将数组还原为图像
    cv::Mat restoredImage = arrayToImage(imageArray, image.rows, image.cols, image.channels());

    // 保存还原的图像
    saveImage("restored_image.jpg", restoredImage);

    // 释放动态分配的内存
    delete[] imageArray;

    return 0;
}

// void convertMatToUint32Array(const cv::Mat& image, std::vector<uint32_t>& outputArray) {
//     if (image.type() != CV_8UC3) {
//         std::cerr << "Input image must be in RGB format (CV_8UC3)." << std::endl;
//         return;
//     }

//     outputArray.resize(image.rows * image.cols);

//     for (int y = 0; y < image.rows; ++y) {
//         for (int x = 0; x < image.cols; ++x) {
//             cv::Vec3b bgr = image.at<cv::Vec3b>(y, x);
//             outputArray[y * image.cols + x] = (0xFF << 24) | (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
//         }
//     }
// }

// void convertUint32ArrayToMat(const std::vector<uint32_t>& inputArray, cv::Mat& image, int width, int height) {
//     image.create(height, width, CV_8UC3);

//     for (int y = 0; y < height; ++y) {
//         for (int x = 0; x < width; ++x) {
//             uint32_t pixel = inputArray[y * width + x];
//             cv::Vec3b& bgr = image.at<cv::Vec3b>(y, x);
//             bgr[0] = pixel & 0xFF;         // B
//             bgr[1] = (pixel >> 8) & 0xFF;  // G
//             bgr[2] = (pixel >> 16) & 0xFF; // R
//         }
//     }
// }

// int main() {
//     cv::Mat image = cv::imread("/home/pi/Pick/result.jpg");

//     if (image.empty()) {
//         std::cerr << "Could not open or find the image." << std::endl;
//         return -1;
//     }

//     std::vector<uint32_t> outputArray;
//     convertMatToUint32Array(image, outputArray);

//     // 进行回转转换
//     cv::Mat convertedImage;
//     convertUint32ArrayToMat(outputArray, convertedImage, image.cols, image.rows);

//     // 保存转换后的图像
//     if (!cv::imwrite("converted_image.jpg", convertedImage)) {
//         std::cerr << "Could not save the image." << std::endl;
//         return -1;
//     }

//     std::cout << "Converted image saved as 'converted_image.jpg'." << std::endl;

//     return 0;
// }





// void convertMatToU8Array(const cv::Mat& image, std::vector<uint8_t>& outputArray) {
//     if (image.type() != CV_8UC3) {
//         std::cerr << "Input image must be in RGB format (CV_8UC3)." << std::endl;
//         return;
//     }

//     outputArray.resize(image.rows * image.cols * 3); // 三个通道

//     // 获取图像的指针
//     const uint8_t* imgData = image.data;

//     for (int i = 0; i < image.rows * image.cols; ++i) {
//         uint8_t r = imgData[i * 3 + 2]; // R
//         uint8_t g = imgData[i * 3 + 1]; // G
//         uint8_t b = imgData[i * 3];     // B

//         // 示例：将 RGB 通道值存储到输出数组中
//         outputArray[i * 3] = b;         // B
//         outputArray[i * 3 + 1] = g;     // G
//         outputArray[i * 3 + 2] = r;     // R
//     }
// }

// void convertU8ArrayToMat(const std::vector<uint8_t>& inputArray, cv::Mat& image, int width, int height) {
//     image.create(height, width, CV_8UC3);

//     uint8_t* imgData = image.data;

//     for (int i = 0; i < width * height; ++i) {
//         // 从输入数组中提取 R、G、B 值
//         imgData[i * 3] = inputArray[i * 3];         // B
//         imgData[i * 3 + 1] = inputArray[i * 3 + 1]; // G
//         imgData[i * 3 + 2] = inputArray[i * 3 + 2]; // R
//     }
// }

// int main() {
//     cv::Mat image = cv::imread("/home/pi/Pick/result.jpg");

//     if (image.empty()) {
//         std::cerr << "Could not open or find the image." << std::endl;
//         return -1;
//     }

//     std::vector<uint8_t> outputArray;
//     convertMatToU8Array(image, outputArray);

//     // 进行回转转换
//     cv::Mat convertedImage;
//     convertU8ArrayToMat(outputArray, convertedImage, image.cols, image.rows);

//     // 保存转换后的图像
//     if (!cv::imwrite("converted_image.jpg", convertedImage)) {
//         std::cerr << "Could not save the image." << std::endl;
//         return -1;
//     }

//     std::cout << "Converted image saved as 'converted_image.jpg'." << std::endl;

//     return 0;
// }



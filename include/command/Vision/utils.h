#pragma once
#include <MNN/ImageProcess.hpp>
#include <MNN/Interpreter.hpp>
#include <MNN/MNNDefine.h>
#include <MNN/MNNForwardType.h>
#include <opencv2/opencv.hpp>

#include <MNN/expr/Executor.hpp>
#include <MNN/expr/ExprCreator.hpp>
#include <MNN/expr/Module.hpp>
#include <sys/time.h>

typedef struct {
    float x1, y1, x2, y2, score;
    int label;
} BoxInfo;

typedef struct {
    int inpSize, maxSide, Padw, Padh;
    float ratio;
} MatInfo;

cv::Mat preprocess(cv::Mat &cv_mat, MatInfo &mmat_objection);

void nms(std::vector<BoxInfo> &result, float nms_threshold);

void draw_box(cv::Mat &cv_mat, std::vector<BoxInfo> &boxes, MatInfo &mmat_objection);

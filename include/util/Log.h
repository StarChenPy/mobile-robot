#pragma once
#ifndef MOBILE_ROBOT_LOG_H
#define MOBILE_ROBOT_LOG_H

#include <memory>
#include <glog/logging.h>

class Log {
public:
    static void init(char *projectName, const std::string& path = "./logs");
    static void shutdown();
};


#endif //MOBILE_ROBOT_LOG_H

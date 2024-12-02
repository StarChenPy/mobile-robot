#include "util/Log.h"

void Log::init(char *projectName) {
    google::InitGoogleLogging(projectName);
    google::SetStderrLogging(google::INFO); //设置级别高于 google::INFO 的日志同时输出到屏幕
    google::SetLogDestination(google::INFO,"./log/INFO/INFO_"); //设置 google::INFO 级别的日志存储路径和文件名前缀
    google::SetLogDestination(google::WARNING,"./log/WARNING/WARNING_");   //设置 google::WARNING 级别的日志存储路径和文件名前缀
    google::SetLogDestination(google::ERROR,"./log/ERROR/ERROR_");   //设置 google::ERROR 级别的日志存储路径和文件名前缀
    google::SetLogFilenameExtension("Qt_");     //设置文件名扩展

    FLAGS_colorlogtostderr = true;    //设置输出到屏幕的日志显示相应颜色
    FLAGS_logbufsecs = 0;        //缓冲日志输出，默认为30秒，此处改为立即输出
    FLAGS_max_log_size = 100;    //最大日志大小为 100MB
    FLAGS_stop_logging_if_full_disk = true;     //当磁盘被写满时，停止日志输出
}

void Log::shutdown() {
    google::ShutdownGoogleLogging();
}

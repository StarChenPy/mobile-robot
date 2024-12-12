#include "util/Log.h"
#include <filesystem>

inline void existsDirectory(const std::string& folderPath) {
    if (!std::filesystem::exists(folderPath)) {
        LOG(INFO) << "日志文件夹不存在，正在创建...";

        // 尝试创建文件夹
        if (std::filesystem::create_directory(folderPath)) {
            LOG(INFO) << "日志文件夹创建成功: " << folderPath;
        } else {
            LOG(ERROR) << "日志文件夹创建失败: " << folderPath;
        }
    }
}

void logFormatter(std::ostream &s, const google::LogMessage &m, void *data){
    s << "[" << google::GetLogSeverityName(m.severity()) << ' ' << m.thread_id() << "]"
      << ' '
      << std::setw(4) << 1900 + m.time().year() << "-"
      << std::setw(2) << 1 + m.time().month() << "-"
      << std::setw(2) << m.time().day()
      << ' '
      << std::setw(2) << m.time().hour() << ':'
      << std::setw(2) << m.time().min() << ':'
      << std::setw(2) << m.time().sec()
      << ' '
      << "[" << m.basename() << ':' << m.line() << "]";
}

void logFileFormatter(const std::string& path) {
    // 获取当前日期
    time_t now = time(nullptr);
    struct tm *local_time = localtime(&now);
    // 从tm结构体中获取年、月、日、时、分、秒
    int year = local_time->tm_year + 1900; // tm_year是以1900年为基的
    int month = local_time->tm_mon + 1;    // tm_mon是以0为1月的
    int day = local_time->tm_mday;         // tm_mday是月份中的哪一天
    int hour = local_time->tm_hour;
    int minute = local_time->tm_min;
    int second = local_time->tm_sec;
    std::stringstream ss;
    ss << year << "-" << month << "-" << day << " " << hour << ":" << minute << ":" << second;
    std::string current_date = ss.str();
    std::string info_log_path = path + "/INFO_" + current_date;
    std::string warn_log_path = path + "/WARNING_" + current_date;
    std::string error_log_path = path + "/ERROR_" + current_date;

    google::SetLogDestination(google::GLOG_INFO, info_log_path.c_str());    // 设置google::GLOG_INFO级别的日志存储路径和文件名前缀
    google::SetLogDestination(google::GLOG_WARNING, warn_log_path.c_str()); // 设置google::GLOG_WARNING级别的日志存储路径和文件名前缀
    google::SetLogDestination(google::GLOG_ERROR, error_log_path.c_str());  // 设置google::GLOG_ERROR级别的日志存储路径和文件名前缀
}

void Log::init(char *projectName, const std::string& path) {
    google::InitGoogleLogging(projectName);
    google::SetStderrLogging(google::INFO); //设置级别高于 google::INFO 的日志同时输出到屏幕
    google::SetLogFilenameExtension(".log");
    google::InstallPrefixFormatter(logFormatter, nullptr);

    FLAGS_log_dir = path;
    FLAGS_colorlogtostderr = true;    //设置输出到屏幕的日志显示相应颜色
    FLAGS_logbufsecs = 0;        //缓冲日志输出，默认为30秒，此处改为立即输出
    FLAGS_max_log_size = 100;    //最大日志大小为 100MB
    FLAGS_stop_logging_if_full_disk = true;     //当磁盘被写满时，停止日志输出
    FLAGS_timestamp_in_logfile_name = false;   // 日志文件名取消时间戳

    logFileFormatter(path);
    existsDirectory(FLAGS_log_dir);
}

void Log::shutdown() {
    google::ShutdownGoogleLogging();
}

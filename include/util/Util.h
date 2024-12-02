/**
 * @file util.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 常用模块
 * @version 0.1
 * @date 2022-09-06
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <boost/lexical_cast.hpp>
#include <cstdint>
#include <cstdio>
#include <cxxabi.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <pthread.h>
#include <string>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>
#include <utility>
#include <vector>
#define millisecond int64_t

namespace robot {

/**
 * @brief 获取线程id
 *
 * @return pid_t
 */
pid_t getThreadId();

///  获取时间
uint64_t getCurrentMs();
uint64_t getCurrentUs();

template <class T> const char *typeToName() {
    static const char *s_name = abi::__cxa_demangle(typeid(T).name(), nullptr, nullptr, nullptr);
    return s_name;
}

class FSUtil {
  public:
    static void listAllFile(std::vector<std::string> &files, const std::string &path, const std::string &subfix);
    static bool mkdir(const std::string &dirname);
    static bool isRunningPidFile(const std::string &pidFile);
    static bool rm(const std::string &path);
    static bool mv(const std::string &from, const std::string &to);
    static bool realpath(const std::string &path, std::string &rpath);
    static bool symlink(const std::string &frm, const std::string &to);
    static bool unlink(const std::string &filename, bool exist = false);
    static std::string dirname(const std::string &filename);
    static std::string basename(const std::string &filename);
    static bool openForRead(std::ifstream &ifs, const std::string &filename, std::ios_base::openmode mode);
    static bool openForWrite(std::ofstream &ofs, const std::string &filename, std::ios_base::openmode mode);
};

} //  namespace robot

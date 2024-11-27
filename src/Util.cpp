/**
 * @file util.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 常用模块
 * @version 0.1
 * @date 2022-09-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <csignal>
#include <cstring>
#include <dirent.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include "Util.h"

namespace robot {

pid_t getThreadId() { return syscall(SYS_gettid); }

uint64_t getCurrentMs() {
    struct timeval tv {};
    gettimeofday(&tv, nullptr);
    return tv.tv_sec * 1000ul + tv.tv_usec / 1000;
}

uint64_t getCurrentUs() {
    struct timeval tv {};
    gettimeofday(&tv, nullptr);
    return tv.tv_sec * 1000 * 1000ul + tv.tv_usec;
}

void FSUtil::listAllFile(std::vector<std::string> &files, const std::string &path, const std::string &subfix) {
    if (access(path.c_str(), 0) != 0) {
        return;
    }
    DIR *dir = opendir(path.c_str());
    if (dir == nullptr) {
        return;
    }
    struct dirent *dp = nullptr;
    while ((dp = readdir(dir)) != nullptr) {
        if (dp->d_type == DT_DIR) {
            if (!strcmp(dp->d_name, ".") || !strcmp(dp->d_name, "..")) {
                continue;
            }
            listAllFile(files, path + "/" + dp->d_name, subfix);
        } else if (dp->d_type == DT_REG) {
            std::string filename(dp->d_name);
            if (subfix.empty()) {
                files.push_back(path + "/" + filename);
            } else {
                if (filename.size() < subfix.size()) {
                    continue;
                }
                if (filename.substr(filename.length() - subfix.size()) == subfix) {
                    files.push_back(path + "/" + filename);
                }
            }
        }
    }
    closedir(dir);
}

static int _lstat(const char *file, struct stat *st = nullptr) {
    struct stat lst {};
    int ret = lstat(file, &lst);
    if (st) {
        *st = lst;
    }
    return ret;
}

static int _mkdir(const char *dirname) {
    if (access(dirname, F_OK) == 0) {
        return 0;
    }
    return mkdir(dirname, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

bool FSUtil::mkdir(const std::string &dirname) {
    if (_lstat(dirname.c_str()) == 0) {
        return true;
    }
    char *path = strdup(dirname.c_str());
    char *ptr = strchr(path + 1, '/');
    do {
        for (; ptr; *ptr = '/', ptr = strchr(ptr + 1, '/')) {
            *ptr = '\0';
            if (_mkdir(path) != 0) {
                break;
            }
        }
        if (ptr != nullptr) {
            break;
        } else if (_mkdir(path) != 0) {
            break;
        }
        free(path);
        return true;
    } while (0);
    free(path);
    return false;
}

bool FSUtil::isRunningPidFile(const std::string &pidFile) {
    if (_lstat(pidFile.c_str()) != 0) {
        return false;
    }
    std::ifstream ifs(pidFile);
    std::string line;
    if (!ifs || !std::getline(ifs, line)) {
        return false;
    }
    if (line.empty()) {
        return false;
    }
    pid_t pid = atoi(line.c_str());
    if (pid <= 1) {
        return false;
    }
    if (kill(pid, 0) != 0) {
        return false;
    }
    return true;
}

bool FSUtil::unlink(const std::string &filename, bool exist) {
    if (!exist && _lstat(filename.c_str())) {
        return true;
    }
    return ::unlink(filename.c_str()) == 0;
}

bool FSUtil::rm(const std::string &path) {
    struct stat st {};
    if (lstat(path.c_str(), &st)) {
        return true;
    }
    if (!(st.st_mode & S_IFDIR)) {
        return unlink(path);
    }

    DIR *dir = opendir(path.c_str());
    if (!dir) {
        return false;
    }

    bool ret = true;
    struct dirent *dp;
    while ((dp = readdir(dir))) {
        if (!strcmp(dp->d_name, ".") || !strcmp(dp->d_name, "..")) {
            continue;
        }
        std::string dirname = path + "/" + dp->d_name;
        ret = rm(dirname);
    }
    closedir(dir);
    if (::rmdir(path.c_str())) {
        ret = false;
    }
    return ret;
}

bool FSUtil::mv(const std::string &from, const std::string &to) {
    if (!rm(to)) {
        return false;
    }
    return rename(from.c_str(), to.c_str()) == 0;
}

bool FSUtil::realpath(const std::string &path, std::string &rpath) {
    if (_lstat(path.c_str())) {
        return false;
    }
    char *ptr = ::realpath(path.c_str(), nullptr);
    if (nullptr == ptr) {
        return false;
    }
    std::string(ptr).swap(rpath);
    free(ptr);
    return true;
}

bool FSUtil::symlink(const std::string &frm, const std::string &to) {
    if (!rm(to)) {
        return false;
    }
    return ::symlink(frm.c_str(), to.c_str()) == 0;
}

std::string FSUtil::dirname(const std::string &filename) {
    if (filename.empty()) {
        return ".";
    }
    auto pos = filename.rfind('/');
    if (pos == 0) {
        return "/";
    } else if (pos == std::string::npos) {
        return ".";
    } else {
        return filename.substr(0, pos);
    }
}

std::string FSUtil::basename(const std::string &filename) {
    if (filename.empty()) {
        return filename;
    }
    auto pos = filename.rfind('/');
    if (pos == std::string::npos) {
        return filename;
    } else {
        return filename.substr(pos + 1);
    }
}

bool FSUtil::openForRead(std::ifstream &ifs, const std::string &filename, std::ios_base::openmode mode) {
    ifs.open(filename.c_str(), mode);
    return ifs.is_open();
}

bool FSUtil::openForWrite(std::ofstream &ofs, const std::string &filename, std::ios_base::openmode mode) {
    ofs.open(filename.c_str(), mode);
    if (!ofs.is_open()) {
        std::string dir = dirname(filename);
        mkdir(dir);
        ofs.open(filename.c_str(), mode);
    }
    return ofs.is_open();
}

} //  namespace robot

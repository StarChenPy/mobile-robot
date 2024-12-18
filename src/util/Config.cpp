#include "util/Config.h"
#include "util/Util.h"
#include <sys/stat.h>

namespace robot {

ConfigVarBase::ptr Config::LookupBase(const std::string &name) {
    RWMutexType::ReadLock lock(GetMutex());
    auto it = GetDatas().find(name);
    return it == GetDatas().end() ? nullptr : it->second;
}

static void listAllMember(const std::string &prefix, const YAML::Node &node,
                          std::list<std::pair<std::string, const YAML::Node>> &output) {
    if (prefix.find_first_not_of("abcdefghikjlmnopqrstuvwxyz._012345678") != std::string::npos) {
        std::cout << "Config invalid name: " << prefix << " : " << node << std::endl;
        return;
    }
    output.emplace_back(prefix, node);
    if (node.IsMap()) {
        for (auto it = node.begin(); it != node.end(); ++it) {
            listAllMember(prefix.empty() ? it->first.Scalar() : prefix + "." + it->first.Scalar(), it->second, output);
        }
    }
}

void Config::loadFromYaml(const YAML::Node &root) {
    std::list<std::pair<std::string, const YAML::Node>> all_nodes;
    listAllMember("", root, all_nodes);

    for (auto &i : all_nodes) {
        std::string key = i.first;
        if (key.empty()) {
            continue;
        }

        std::transform(key.begin(), key.end(), key.begin(), ::tolower);
        ConfigVarBase::ptr var = LookupBase(key);

        if (var) {
            if (i.second.IsScalar()) {
                var->fromString(i.second.Scalar());
            } else {
                std::stringstream ss;
                ss << i.second;
                var->fromString(ss.str());
            }
        }
    }
}

static std::map<std::string, uint64_t> s_file2modifytime;
static robot::Mutex s_mutex;

void Config::loadFromConfDir(const std::string &path, bool force) {
    std::string absoulte_path;
    std::vector<std::string> files;
    FSUtil::listAllFile(files, absoulte_path, ".yml");

    for (auto &i : files) {
        {
            struct stat st{};
            lstat(i.c_str(), &st);
            robot::Mutex::Lock lock(s_mutex);
            if (!force && s_file2modifytime[i] == (uint64_t)st.st_mtime) {
                continue;
            }
            s_file2modifytime[i] = st.st_mtime;
        }
        try {
            YAML::Node root = YAML::LoadFile(i);
            loadFromYaml(root);
            std::cout << "LoadConfFile file=" << i << " ok" << std::endl;
        } catch (...) {
            std::cout << "LoadConfFile file=" << i << " failed" << std::endl;
        }
    }
}

void Config::visit(std::function<void(ConfigVarBase::ptr)> cb) {
    RWMutexType::ReadLock lock(GetMutex());
    ConfigVarMap &m = GetDatas();
    for (auto & it : m) {
        cb(it.second);
    }
}

} // namespace robot

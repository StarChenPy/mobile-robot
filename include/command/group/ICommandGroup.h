#pragma once
#include "command/ICommand.h"
#include <iostream>
#include <memory>
#include <vector>

namespace robot {
    class ICommandGroup: public ICommand {
    public:
        typedef std::shared_ptr<ICommandGroup> ptr;

        ICommandGroup();

        /**
         * 向命令组添加命令
         * @tparam ICommand 命令类型
         * @param commands  命令数组
         */
        template <typename... ICommand> void addCommand(ICommand... commands) {
            // 递归调用展开参数
            addCommand(commands...);
        }
        void addCommand(const std::vector<ICommand::ptr>& commands);
        void addCommand(const ICommand::ptr& command);
    protected:
        std::vector<ICommand::ptr> commands_;
        ICommand::ptr nextCommand_;
    };
} // namespace robot
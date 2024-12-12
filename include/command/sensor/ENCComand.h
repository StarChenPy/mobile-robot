#pragma once

#include "system/RobotCfg.h"
#include "RobotGenius.h"
#include "util/params.h"

// 定义一个命令类，用于读取左编码器数据
class readLeftENCCommand : public ICommand {
public:
    typedef std::shared_ptr<readLeftENCCommand> ptr;

    void execute() override; // 执行命令
};

// 定义一个命令类，用于读取右编码器数据
class readRightENCCommand : public ICommand {
public:
    typedef std::shared_ptr<readRightENCCommand> ptr;

    void execute() override;
};

// 定义一个命令类，用于读取转向编码器数据
class readTurnENCCommand : public ICommand {
public:
    typedef std::shared_ptr<readTurnENCCommand> ptr;

    void execute() override;
};

// 定义一个命令类，用于读取升降编码器数据
class readLiftENCCommand : public ICommand {
public:
    typedef std::shared_ptr<readLiftENCCommand> ptr;

    void execute() override;
};

// 定义一个命令类，用于读取里程计编码器数据
class readOdomENCCommand : public ICommand {
public:
    typedef std::shared_ptr<readOdomENCCommand> ptr;

    void execute() override;
};

// 定义一个命令类，用于读取所有编码器数据
class readAllENCCommand : public ICommand {
public:
    typedef std::shared_ptr<readAllENCCommand> ptr;

    void execute() override;
};

// 定义一个命令类，用于重置左编码器
class resetLeftENCCommand : public ICommand {
public:
    typedef std::shared_ptr<resetLeftENCCommand> ptr;

    void execute() override;
};

// 定义一个命令类，用于重置右编码器
class resetRightENCCommand : public ICommand {
public:
    typedef std::shared_ptr<resetRightENCCommand> ptr;

    void execute() override;
};

// 定义一个命令类，用于重置转向编码器
class resetTurnENCCommand : public ICommand {
public:
    typedef std::shared_ptr<resetTurnENCCommand> ptr;

    void execute() override;
};

// 定义一个命令类，用于重置升降编码器
class resetLiftENCCommand : public ICommand {
public:
    typedef std::shared_ptr<resetLiftENCCommand> ptr;

    void execute() override;
};

// 定义一个命令类，用于重置里程计编码器
class resetOdomENCCommand : public ICommand {
public:
    typedef std::shared_ptr<resetOdomENCCommand> ptr;

    void execute() override;
};

// 定义一个命令类，用于重置所有编码器
class resetAllENCCommand : public ICommand {
public:
    typedef std::shared_ptr<resetAllENCCommand> ptr;

    void execute() override;
};

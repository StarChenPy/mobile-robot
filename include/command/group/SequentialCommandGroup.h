/**
 * @file SequentialCommandGroup.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 顺序结构，命令组按照顺序运行命令
 * @version 0.1
 * @date 2024-04-14
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "ICommandGroup.h"
#include <memory>
#include <vector>

namespace robot {

class SequentialCommandGroup : public ICommandGroup {
  public:
    typedef std::shared_ptr<SequentialCommandGroup> ptr;
    /**
     * 顺序执行命令组
     * 最后一个命令结束后结束
     */
    SequentialCommandGroup() = default;
    ~SequentialCommandGroup() override = default;

  public:
    void initialize() override;
    void execute() override;
    void end() override;

    static ICommandGroup::ptr create();
  protected:
    ICommand::ptr currentCommand_;
    uint64_t currentCommandIndex_ = 0;
};
} // namespace robot

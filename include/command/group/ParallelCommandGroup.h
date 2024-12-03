#pragma once

#include "ICommandGroup.h"
#include "util/Scheduler.h"
#include <memory>
#include <vector>

namespace robot {

    class ParallelCommandGroup : public ICommandGroup {
    public:
        typedef std::shared_ptr<ParallelCommandGroup> ptr;
        /**
         * 并行执行命令组
         * 在内部命令全部结束后结束
         */
        ParallelCommandGroup() = default;
        ~ParallelCommandGroup() override = default;

    public:
        void initialize() override;
        void execute() override;
        void end() override;
        bool isFinished() override;

        static ICommandGroup::ptr create();
    };

} // namespace robot
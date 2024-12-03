#pragma once
#include "ICommandGroup.h"
#include <memory>
#include <vector>

namespace robot {
    class ParallelRaceGroup : public ICommandGroup {
    public:
        typedef std::shared_ptr<ParallelRaceGroup> ptr;
        /**
         * 并行执行命令组
         * 在任一命令结束后结束
         */
        ParallelRaceGroup() = default;;
        ~ParallelRaceGroup() override = default;

        void initialize() override;
        void execute() override;
        void end() override;
        bool isFinished() override;

        static ICommandGroup::ptr create();
    public:
        uint64_t stopCommandIndex_ = 0;
    };
} // namespace robot

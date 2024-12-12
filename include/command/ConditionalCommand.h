#pragma once
#include "ICommand.h"
#include <functional>

namespace robot {
    class ConditionalCommand : public ICommand {
    public:
        ConditionalCommand(std::function<bool()> condition, ICommand::ptr onTrueCommand, ICommand::ptr onFalseCommand);
        ~ConditionalCommand() override = default;

    public:
        void initialize() override;
        void execute() override;
        void end() override;
        bool isFinished() override;

        static ICommand::ptr create(const std::function<bool()>& condition, const ICommand::ptr& onTrueCommand, const ICommand::ptr& onFalseCommand);

    private:
        std::function<bool()> condition_;
        ICommand::ptr onTrueCommand_;
        ICommand::ptr onFalseCommand_;
    };
} // namespace robot
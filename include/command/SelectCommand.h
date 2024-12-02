/**
 * @file SelectCommand.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief
 * @version 0.1
 * @date 2024-04-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "Command.h"
#include <memory>
#include <map>

namespace robot {
template <typename T> class SelectCommand : public CommandBase {
  public:
    typedef std::shared_ptr<SelectCommand> ptr;
    // SelectCommand(std::function<T()> selector, std::pair<T, Command::ptr>...
    // commands_ptr) : m_selector_(std::move(selector)) {
    //   (commands_.insert(std::move(commands_ptr)), ...);
    // }
    SelectCommand(std::function<T()> selector, std::vector<std::pair<T, std::unique_ptr<Command>>> &&commands)
        : m_selector_(std::move(selector)) {
        for (auto &command : commands) {
            m_commands_.insert(std::move(command));
        }
    }
    virtual ~SelectCommand() = default;

    // No copy constructors for command groups
    SelectCommand(const SelectCommand &other) = delete;

    // Prevent template expansion from emulating copy ctor
    SelectCommand(SelectCommand &) = delete;

  public:
    void initialize() override;
    void execute() override { workCommand_->execute(); }
    void end() override { workCommand_->end(); }

    bool isFinished() override { return workCommand_->isFinished(); }

  private:
    std::function<T()> m_selector_;
    std::map<T, Command::ptr> m_commands_;
};
template <typename T> void SelectCommand<T>::initialize() {
    auto key = m_selector_();
    auto it = m_commands_.find(key);
    if (it != m_commands_.end()) {
        workCommand_ = it->second;
        workCommand_->initialize();
    } else {
        workCommand_ = nullptr;
        throw std::runtime_error("SelectCommand: No command found for key " + std::to_string(key));
    }
}

} // namespace robot

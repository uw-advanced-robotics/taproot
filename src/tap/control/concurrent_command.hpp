#ifndef TAPROOT_CONCURRENT_COMMAND_HPP_
#define TAPROOT_CONCURRENT_COMMAND_HPP_

#include <functional>
#include <vector>

#include "command_scheduler_types.hpp"
#include "command.hpp"

namespace tap
{
namespace control
{
template <bool RACE>
class ConcurrentTemplateCommand : public Command
{
public:
    ConcurrentTemplateCommand(
        std::vector<Command*> commands,
        const char* name,
        Command* deadlineCommand = nullptr);

    const char* getName() const override;
    bool isReady() override;
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;
    void addCommand(Command* command) override;

private:
    std::vector<Command*> commands;
    Command* deadlineCommand;
    const char* name;
    command_scheduler_bitmap_t finishedCommands;
    command_scheduler_bitmap_t allCommands;
};

using ConcurrentCommand = ConcurrentTemplateCommand<false>;
using ConcurrentRaceCommand = ConcurrentTemplateCommand<true>;
using ConcurrentDeadlineCommand = ConcurrentTemplateCommand<false>;

}  // namespace control
}  // namespace tap

#endif  // TAPROOT_CONCURRENT_COMMAND_HPP_

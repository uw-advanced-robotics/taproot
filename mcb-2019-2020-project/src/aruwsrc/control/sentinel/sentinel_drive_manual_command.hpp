#ifndef __SENTINEL_DRIVE_MANUAL_COMMAND_HPP__
#define __SENTINEL_DRIVE_MANUAL_COMMAND_HPP__

#include <aruwlib/control/command.hpp>

#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
class SentinelDriveSubsystem;

class SentinelDriveManualCommand : public aruwlib::control::Command
{
public:
    explicit SentinelDriveManualCommand(SentinelDriveSubsystem* subsystem);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "sentinel drive manual command"; }

private:
    SentinelDriveSubsystem* subsystemSentinelDrive;
};

}  // namespace control

}  // namespace aruwsrc

#endif

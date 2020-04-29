#ifndef __SENTINEL_DRIVE_MANUAL_COMMAND_HPP__
#define __SENTINEL_DRIVE_MANUAL_COMMAND_HPP__

#include <aruwlib/control/command.hpp>
#include "sentinel_drive_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class SentinelDriveSubsystem;

class SentinelDriveManualCommand : public Command
{
 public:
    explicit SentinelDriveManualCommand(SentinelDriveSubsystem* subsystem);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

 private:
    SentinelDriveSubsystem* subsystemSentinelDrive;
};

}  // namespace control

}  // namespace aruwsrc

#endif
